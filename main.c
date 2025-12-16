#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/socket.h>

#include "bme280_driver.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define TAG "ROVER_WIFI"

#define ENA_GPIO 25
#define IN1_GPIO 26
#define IN2_GPIO 27

#define ENB_GPIO 13
#define IN3_GPIO 14
#define IN4_GPIO 12

#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_CHANNEL_A LEDC_CHANNEL_0
#define LEDC_CHANNEL_B LEDC_CHANNEL_1
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY 20000

#define ROVER_TCP_PORT 3333
#define SPEED_DUTY 255

static void motors_gpio_init(void) {
  gpio_config_t io_conf = {
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = (1ULL << IN1_GPIO) | (1ULL << IN2_GPIO) | (1ULL << IN3_GPIO) |
                      (1ULL << IN4_GPIO),
      .pull_up_en = 0,
      .pull_down_en = 0,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&io_conf);

  gpio_set_level(IN1_GPIO, 0);
  gpio_set_level(IN2_GPIO, 0);
  gpio_set_level(IN3_GPIO, 0);
  gpio_set_level(IN4_GPIO, 0);
}

static void motors_pwm_init(void) {
  ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_MODE,
      .timer_num = LEDC_TIMER,
      .duty_resolution = LEDC_DUTY_RES,
      .freq_hz = LEDC_FREQUENCY,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ch_a = {
      .gpio_num = ENA_GPIO,
      .speed_mode = LEDC_MODE,
      .channel = LEDC_CHANNEL_A,
      .timer_sel = LEDC_TIMER,
      .duty = 0,
      .hpoint = 0,
  };
  ledc_channel_config(&ch_a);

  ledc_channel_config_t ch_b = {
      .gpio_num = ENB_GPIO,
      .speed_mode = LEDC_MODE,
      .channel = LEDC_CHANNEL_B,
      .timer_sel = LEDC_TIMER,
      .duty = 0,
      .hpoint = 0,
  };
  ledc_channel_config(&ch_b);
}

static void motorA_drive(int8_t dir, uint8_t duty) {
  uint32_t d = (dir == 0) ? 0U : (uint32_t)duty;

  if (dir > 0) {
    gpio_set_level(IN1_GPIO, 1);
    gpio_set_level(IN2_GPIO, 0);
  } else if (dir < 0) {
    gpio_set_level(IN1_GPIO, 0);
    gpio_set_level(IN2_GPIO, 1);
  } else {
    gpio_set_level(IN1_GPIO, 0);
    gpio_set_level(IN2_GPIO, 0);
  }

  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_A, d);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_A);
}

static void motorB_drive(int8_t dir, uint8_t duty) {
  uint32_t d = (dir == 0) ? 0U : (uint32_t)duty;

  if (dir > 0) {
    gpio_set_level(IN3_GPIO, 1);
    gpio_set_level(IN4_GPIO, 0);
  } else if (dir < 0) {
    gpio_set_level(IN3_GPIO, 0);
    gpio_set_level(IN4_GPIO, 1);
  } else {
    gpio_set_level(IN3_GPIO, 0);
    gpio_set_level(IN4_GPIO, 0);
  }

  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, d);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B);
}

static void motors_stop_all(void) {
  motorA_drive(0, 0);
  motorB_drive(0, 0);
}

static void rover_forward(uint8_t duty) {
  ESP_LOGI(TAG, "FORWARD");
  motorA_drive(+1, duty);
  motorB_drive(+1, duty);
}

static void rover_backward(uint8_t duty) {
  ESP_LOGI(TAG, "BACKWARD");
  motorA_drive(-1, duty);
  motorB_drive(-1, duty);
}

static void rover_left(uint8_t duty) {
  ESP_LOGI(TAG, "LEFT");
  motorA_drive(-1, duty);
  motorB_drive(+1, duty);
}

static void rover_right(uint8_t duty) {
  ESP_LOGI(TAG, "RIGHT");
  motorA_drive(+1, duty);
  motorB_drive(-1, duty);
}

static void rover_stop(void) {
  ESP_LOGI(TAG, "STOP");
  motors_stop_all();
}

static void wifi_init_softap(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  wifi_config_t wifi_config = {
      .ap =
          {
              .ssid = "ESP32_ROVER",
              .ssid_len = 0,
              .channel = 1,
              .password = "aguacate",
              .max_connection = 1,
              .authmode = WIFI_AUTH_WPA_WPA2_PSK,
          },
  };

  if (strlen((char *)wifi_config.ap.password) == 0) {
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "SoftAP init done. SSID: %s  PASS: %s", wifi_config.ap.ssid,
           wifi_config.ap.password);
}

static void tcp_server_task(void *pvParameters) {
  int listen_sock, sock;
  struct sockaddr_in destAddr = {0};
  destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  destAddr.sin_family = AF_INET;
  destAddr.sin_port = htons(ROVER_TCP_PORT);

  listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (listen_sock < 0) {
    ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    vTaskDelete(NULL);
    return;
  }

  int opt = 1;
  setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  int err = bind(listen_sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
  if (err != 0) {
    ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    close(listen_sock);
    vTaskDelete(NULL);
    return;
  }

  err = listen(listen_sock, 1);
  if (err != 0) {
    ESP_LOGE(TAG, "Error during listen: errno %d", errno);
    close(listen_sock);
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(TAG, "TCP server listening on port %d", ROVER_TCP_PORT);

  while (1) {
    ESP_LOGI(TAG, "Waiting for a new client...");
    struct sockaddr_in6 sourceAddr;
    socklen_t addrLen = sizeof(sourceAddr);

    sock = accept(listen_sock, (struct sockaddr *)&sourceAddr, &addrLen);
    if (sock < 0) {
      ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
      continue;
    }

    ESP_LOGI(TAG, "Client connected. Use WASD + X over TCP.");

    char rx_buffer[64];
    while (1) {
      int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
      if (len < 0) {
        ESP_LOGE(TAG, "recv failed: errno %d", errno);
        break;
      }
      if (len == 0) {
        ESP_LOGI(TAG, "Client disconnected");
        break;
      }

      for (int i = 0; i < len; i++) {
        char c = rx_buffer[i];
        switch (c) {
          case 'w':
          case 'W':
            rover_forward(SPEED_DUTY);
            break;
          case 's':
          case 'S':
            rover_backward(SPEED_DUTY);
            break;
          case 'a':
          case 'A':
            rover_left(SPEED_DUTY);
            break;
          case 'd':
          case 'D':
            rover_right(SPEED_DUTY);
            break;
          case 'x':
          case 'X':
            rover_stop();
            break;
          case '\n':
          case '\r':
            break;
          case 't':
          case 'T': {
            float t = bme280_get_temperature();
            float h = bme280_get_humidity();
            float p = bme280_get_pressure();
            float alt = bme280_get_altitude();
            char msg[96];
            int n = snprintf(msg, sizeof(msg),
                             "T=%.2f C  H=%.2f %%  P=%.2f hPa  Alt=%.2f m\r\n",
                             t, h, p, alt);
            send(sock, msg, n, 0);
          } break;
          default:
            ESP_LOGI(TAG, "Tecla desconocida: %c", c);
            break;
        }
      }
    }

    rover_stop();
    shutdown(sock, 0);
    close(sock);
  }
}

void app_main(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  }

  motors_gpio_init();
  motors_pwm_init();
  motors_stop_all();

  wifi_init_softap();
  bme280_start_task();

  xTaskCreate(tcp_server_task, "tcp_server_task", 4096, NULL, 5, NULL);
}
