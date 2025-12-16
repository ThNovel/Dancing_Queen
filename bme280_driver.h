#ifndef BME280_DRIVER_H
#define BME280_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

void bme280_start_task(void);
float bme280_get_temperature(void);
float bme280_get_humidity(void);
float bme280_get_pressure(void);
float bme280_get_altitude(void);

#ifdef __cplusplus
}
#endif

#endif
