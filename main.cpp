#include "esp_camera.h"
#include <WebServer.h>
#include <WiFi.h>

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5

#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

const char *ap_ssid = "ESP32_ROVER_CAM";
const char *ap_pass = "12345678";

WebServer server(80);

void handle_root() {
  String html =
      "<html><body style='background:#111;color:#eee;font-family:sans-serif;text-align:center'>"
      "<h2>ESP32-ROVER Stream</h2>"
      "<p>Conectado a la red ESP32_ROVER</p>"
      "<img src=\"/stream\" style='width:90%;max-width:480px;'/>"
      "</body></html>";
  server.send(200, "text/html", html);
}

void handle_stream() {
  WiFiClient client = server.client();

  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);

  while (client.connected()) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      break;
    }

    client.print("--frame\r\n");
    client.print("Content-Type: image/jpeg\r\n\r\n");
    client.write(fb->buf, fb->len);
    client.print("\r\n");

    esp_camera_fb_return(fb);
    delay(0);
  }

  Serial.println("Stream ended");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\nBooting");

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    Serial.println("PSRAM OK");
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 25;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    Serial.println("PSRAM NOT FOUND!");
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 30;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed 0x%x\n", err);
    return;
  }

  WiFi.mode(WIFI_AP);
  bool ap_ok = WiFi.softAP(ap_ssid, ap_pass);
  if (!ap_ok) {
    Serial.println("Failed to start AP!");
    return;
  }
  WiFi.setSleep(false);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP started. SSID: ");
  Serial.println(ap_ssid);
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.on("/", handle_root);
  server.on("/stream", handle_stream);
  server.begin();
  Serial.println("Stream server started. Open the IP in your browser.");
}

void loop() { server.handleClient(); }
