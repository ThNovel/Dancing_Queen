#include "bme280_driver.h"

#include <math.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "BME280"

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

#define BME280_ADDR 0x76

#define BME280_REG_ID 0xD0
#define BME280_REG_RESET 0xE0
#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG 0xF5

#define BME280_REG_PRESS_MSB 0xF7
#define BME280_REG_CALIB00 0x88
#define BME280_REG_CALIB26 0xA1
#define BME280_REG_CALIB_HUM 0xE1

typedef struct {
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;

  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;

  uint8_t dig_H1;
  int16_t dig_H2;
  uint8_t dig_H3;
  int16_t dig_H4;
  int16_t dig_H5;
  int8_t dig_H6;
} bme280_calib_t;

static bme280_calib_t s_calib;
static int32_t s_t_fine;

static float s_last_temperature = 0.0f;
static float s_last_humidity = 0.0f;
static float s_last_pressure = 0.0f;

static esp_err_t i2c_master_init(void) {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };
  ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
  return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t write_reg(uint8_t reg, uint8_t value) {
  uint8_t data[2] = {reg, value};
  return i2c_master_write_to_device(I2C_MASTER_NUM, BME280_ADDR, data, sizeof(data),
                                    pdMS_TO_TICKS(100));
}

static esp_err_t read_regs(uint8_t start_reg, uint8_t *buf, size_t len) {
  return i2c_master_write_read_device(I2C_MASTER_NUM, BME280_ADDR, &start_reg, 1, buf,
                                      len, pdMS_TO_TICKS(100));
}

static esp_err_t read_calibration(void) {
  esp_err_t err;
  uint8_t buf[32];

  err = read_regs(BME280_REG_CALIB00, buf, 24);
  if (err != ESP_OK) return err;

  s_calib.dig_T1 = (uint16_t)(buf[1] << 8) | buf[0];
  s_calib.dig_T2 = (int16_t)((buf[3] << 8) | buf[2]);
  s_calib.dig_T3 = (int16_t)((buf[5] << 8) | buf[4]);

  s_calib.dig_P1 = (uint16_t)(buf[7] << 8) | buf[6];
  s_calib.dig_P2 = (int16_t)((buf[9] << 8) | buf[8]);
  s_calib.dig_P3 = (int16_t)((buf[11] << 8) | buf[10]);
  s_calib.dig_P4 = (int16_t)((buf[13] << 8) | buf[12]);
  s_calib.dig_P5 = (int16_t)((buf[15] << 8) | buf[14]);
  s_calib.dig_P6 = (int16_t)((buf[17] << 8) | buf[16]);
  s_calib.dig_P7 = (int16_t)((buf[19] << 8) | buf[18]);
  s_calib.dig_P8 = (int16_t)((buf[21] << 8) | buf[20]);
  s_calib.dig_P9 = (int16_t)((buf[23] << 8) | buf[22]);

  err = read_regs(BME280_REG_CALIB26, &s_calib.dig_H1, 1);
  if (err != ESP_OK) return err;

  err = read_regs(BME280_REG_CALIB_HUM, buf, 7);
  if (err != ESP_OK) return err;

  s_calib.dig_H2 = (int16_t)((buf[1] << 8) | buf[0]);
  s_calib.dig_H3 = buf[2];
  s_calib.dig_H4 = (int16_t)((buf[3] << 4) | (buf[4] & 0x0F));
  s_calib.dig_H5 = (int16_t)((buf[5] << 4) | (buf[4] >> 4));
  s_calib.dig_H6 = (int8_t)buf[6];

  return ESP_OK;
}

static esp_err_t bme280_init(void) {
  uint8_t id = 0;
  ESP_ERROR_CHECK(read_regs(BME280_REG_ID, &id, 1));
  ESP_LOGI(TAG, "BME280 ID = 0x%02X", id);

  if (id != 0x60) {
    ESP_LOGW(TAG, "Unexpected ID (expected 0x60), continuing anyway");
  }

  ESP_ERROR_CHECK(write_reg(BME280_REG_RESET, 0xB6));
  vTaskDelay(pdMS_TO_TICKS(10));

  ESP_ERROR_CHECK(read_calibration());
  ESP_ERROR_CHECK(write_reg(BME280_REG_CTRL_HUM, 0x01));
  ESP_ERROR_CHECK(write_reg(BME280_REG_CTRL_MEAS, 0x27));

  uint8_t config = (4 << 5);
  ESP_ERROR_CHECK(write_reg(BME280_REG_CONFIG, config));

  ESP_LOGI(TAG, "BME280 initialized");
  return ESP_OK;
}

static esp_err_t read_raw(int32_t *raw_temp, int32_t *raw_press, int32_t *raw_hum) {
  uint8_t data[8];
  esp_err_t err = read_regs(BME280_REG_PRESS_MSB, data, sizeof(data));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Raw read failed: %s", esp_err_to_name(err));
    return err;
  }

  int32_t adc_P = (int32_t)(((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) |
                            ((uint32_t)data[2] >> 4));
  int32_t adc_T = (int32_t)(((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) |
                            ((uint32_t)data[5] >> 4));
  int32_t adc_H = (int32_t)(((uint32_t)data[6] << 8) | (uint32_t)data[7]);

  *raw_temp = adc_T;
  *raw_press = adc_P;
  *raw_hum = adc_H;

  return ESP_OK;
}

static float compensate_temperature(int32_t adc_T) {
  float var1 = (((float)adc_T) / 16384.0f - ((float)s_calib.dig_T1) / 1024.0f) *
               (float)s_calib.dig_T2;
  float var2 =
      ((((float)adc_T) / 131072.0f - ((float)s_calib.dig_T1) / 8192.0f) *
       (((float)adc_T) / 131072.0f - ((float)s_calib.dig_T1) / 8192.0f)) *
      (float)s_calib.dig_T3;

  s_t_fine = (int32_t)(var1 + var2);
  return (var1 + var2) / 5120.0f;
}

static float compensate_pressure(int32_t adc_P) {
  float var1, var2, p;

  var1 = ((float)s_t_fine / 2.0f) - 64000.0f;
  var2 = var1 * var1 * ((float)s_calib.dig_P6) / 32768.0f;
  var2 = var2 + var1 * ((float)s_calib.dig_P5) * 2.0f;
  var2 = (var2 / 4.0f) + ((float)s_calib.dig_P4) * 65536.0f;
  var1 = (((float)s_calib.dig_P3) * var1 * var1 / 524288.0f +
          ((float)s_calib.dig_P2) * var1) /
         524288.0f;
  var1 = (1.0f + var1 / 32768.0f) * (float)s_calib.dig_P1;

  if (var1 == 0.0f) return 0.0f;

  p = 1048576.0f - (float)adc_P;
  p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
  var1 = ((float)s_calib.dig_P9) * p * p / 2147483648.0f;
  var2 = p * ((float)s_calib.dig_P8) / 32768.0f;
  p = p + (var1 + var2 + (float)s_calib.dig_P7) / 16.0f;

  return p / 100.0f;
}

static float compensate_humidity(int32_t adc_H) {
  float var_H = ((float)s_t_fine) - 76800.0f;
  if (var_H == 0.0f) return 0.0f;

  var_H = (adc_H -
           (((float)s_calib.dig_H4) * 64.0f + ((float)s_calib.dig_H5) / 16384.0f * var_H)) *
          (((float)s_calib.dig_H2) / 65536.0f *
           (1.0f +
            ((float)s_calib.dig_H6) / 67108864.0f * var_H *
                (1.0f + ((float)s_calib.dig_H3) / 67108864.0f * var_H)));

  var_H = var_H * (1.0f - ((float)s_calib.dig_H1) * var_H / 524288.0f);
  if (var_H > 100.0f) var_H = 100.0f;
  if (var_H < 0.0f) var_H = 0.0f;
  return var_H;
}

static esp_err_t read_all(float *temperature_c, float *humidity_rh, float *pressure_hpa) {
  int32_t raw_T, raw_P, raw_H;
  esp_err_t err = read_raw(&raw_T, &raw_P, &raw_H);
  if (err != ESP_OK) return err;

  float T = compensate_temperature(raw_T);
  float P = compensate_pressure(raw_P);
  float H = compensate_humidity(raw_H);

  *temperature_c = T;
  *pressure_hpa = P;
  *humidity_rh = H;

  return ESP_OK;
}

static void bme_task(void *arg) {
  ESP_LOGI(TAG, "bme_task started");
  ESP_ERROR_CHECK(i2c_master_init());
  ESP_ERROR_CHECK(bme280_init());

  while (1) {
    float t, h, p;
    esp_err_t err = read_all(&t, &h, &p);
    if (err == ESP_OK) {
      s_last_temperature = t;
      s_last_humidity = h;
      s_last_pressure = p;
      ESP_LOGI(TAG, "T=%.2f C  H=%.2f %%  P=%.2f hPa", t, h, p);
    } else {
      ESP_LOGW(TAG, "BME280 read failed: %s", esp_err_to_name(err));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void bme280_start_task(void) {
  xTaskCreate(bme_task, "bme_task", 4096, NULL, 4, NULL);
}

float bme280_get_temperature(void) { return s_last_temperature; }
float bme280_get_humidity(void) { return s_last_humidity; }
float bme280_get_pressure(void) { return s_last_pressure; }

float bme280_get_altitude(void) {
  float p = s_last_pressure;
  const float p0 = 1013.25f;
  return 44330.0f * (1.0f - powf(p / p0, 0.1903f));
}
