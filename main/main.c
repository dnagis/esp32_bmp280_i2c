/* 
 * Novembre 2018:
 * Code pour taper dans les capteur bmx280 pompé sur https://github.com/openairproject/sensor-esp32.git
 * basé sur  
 * Hello World Example
 * 
 * Pour compiler, une fois cloné aller à la racine du dir et CC=gcc make, rien à ajouter ou modifier
 * 
 * 
 * Les numéros des pins: dans bmx280.h
 * 
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "esp_sleep.h"

#include "i2c_bme280.h"
#include "bmx280.h"

static const char *TAG = "vvnx";

//--------- ENV -----------

typedef struct {
	env_data_t env_data;
	long timestamp;
} env_data_record_t;

static env_data_record_t last_env_data[2];
static bmx280_config_t bmx280_config[2];

static void env_sensor_callback(env_data_t* env_data) {
	if (env_data->sensor_idx <= 1) {
		ESP_LOGI(TAG,"env (%d): temp : %.2f C, pressure: %.2f hPa, humidity: %.2f %%", env_data->sensor_idx, env_data->temp, env_data->pressure, env_data->humidity);
		env_data_record_t* r = last_env_data + env_data->sensor_idx;
		r->timestamp = oap_epoch_sec();
		memcpy(&last_env_data->env_data, env_data, sizeof(env_data_t));
	} else {
		ESP_LOGE(TAG, "env (%d) - invalid sensor", env_data->sensor_idx);
	}
}

static void env_sensors_init() {
	memset(&last_env_data, 0, sizeof(env_data_record_t)*2);
	memset(bmx280_config, 0, sizeof(bmx280_config_t)*2);

	if (bmx280_set_hardware_config(&bmx280_config[0], 0) == ESP_OK) {
		bmx280_config[0].interval = 5000;
		bmx280_config[0].callback = &env_sensor_callback;

		if (bmx280_init(&bmx280_config[0]) != ESP_OK) {
			ESP_LOGE(TAG, "couldn't initialise bmx280 sensor %d", 0);
		}
	}

	if (bmx280_set_hardware_config(&bmx280_config[1], 1) == ESP_OK) {
		bmx280_config[1].interval = 5000;
		bmx280_config[1].callback = &env_sensor_callback;

		if (bmx280_init(&bmx280_config[1]) != ESP_OK) {
			ESP_LOGE(TAG, "couldn't initialise bmx280 sensor %d", 1);
		}
	}
}


void app_main()
{
    printf("Début de main...\n");

            
    
    env_sensors_init(); 
    
            

    for (int i = 10; i >= 0; i--) {
        printf("Fin des haricots in %d  seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Dodo...\n");
    fflush(stdout);
    
    //esp_restart();
    
    esp_sleep_enable_timer_wakeup(20 * 1000000);
	esp_deep_sleep_start();
}
