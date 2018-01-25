/**
 * Lecture bmp280 via i2c sur l'esp32. 
 * 
 * Sur plaque de dev, VDD et GND et Clk sur SCL_PIN (voir les defines ici), Data (SDI) sur SDA_PIN
 * 
 * --> il faut ABSOLUMENT faire un MIX des deux ci dessous sinon ça passe pas...
 * https://github.com/BoschSensortec/BMP280_driver
 * 	|- bmp280.[c,h] doivent être à côté pour pouvoir compiler
 * https://github.com/yanbe/bme280-esp-idf-i2c/blob/master/main/main.c
 * 
 * 
 * Attention flash pas possible si le bmp280 a VDD et GND branché à celui de l'esp32, mais au runtime 
 * 	il faut qu'ils soient branchés sinon com marche pas... Donc débrancher le 3v3 de l'i2c bmp280 le temps du flash (peut 
 *  être qu'avec 18 & 19 il n'y a pas ce problème???)
 * 
 * interessant si tu n'arrives pas à savoir si tu es bien connecté en i2c:
 * 	 https://github.com/nkolban/esp32-snippets/blob/master/i2c/scanner/i2cscanner.c
 * 
 * 
 * ToDo -> je ne crois pas avoir la bonne température, je ne lance pas la séquence où tu lui écris dans la gueule, ou alors il faut reset?, 
 * 	la température a l'air d'être toujours la même, et pas cohérente...
 * 
 * 
 * 
 * **/





#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"

#include "sdkconfig.h" 
#include "bmp280.h"

#define BMP280_API

#define SDA_PIN GPIO_NUM_18 //15 et 2 marche aussi, MAIS avec 15 et 2 j'ai l'impression que flash bloque quand VDD branché...
#define SCL_PIN GPIO_NUM_19


#define TAG_BMP280 "BMP280"

#define	I2C_BUFFER_LEN 8
#define BUFFER_LENGTH	0xFF
#define BMP280_DATA_INDEX	1
#define BMP280_ADDRESS_INDEX	2

struct bmp280_t bmp280;



s8  BMP280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMP280_INIT_VALUE;
	//vient de https://github.com/yanbe/bme280-esp-idf-i2c/blob/master/main/main.c, dans l'api de Bosch c'est incomplet (weird, I know...)
	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, reg_addr, true);
	i2c_master_write(cmd, reg_data, cnt, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		iError = SUCCESS;
	} else {
		iError = FAIL;
	}
	i2c_cmd_link_delete(cmd);
	return (s8)iError;
}

s8  BMP280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMP280_INIT_VALUE;
	//vient de https://github.com/yanbe/bme280-esp-idf-i2c/blob/master/main/main.c, dans l'api de Bosch c'est incomplet (weird, I know...)
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

	if (cnt > 1) {
		i2c_master_read(cmd, reg_data, cnt-1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, reg_data+cnt-1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		iError = SUCCESS;
	} else {
		iError = FAIL;
	}

	i2c_cmd_link_delete(cmd);
	

	
	
	return (s8)iError;
}



void i2c_master_init()
{
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.scl_io_num = SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000
	};
	i2c_param_config(I2C_NUM_0, &i2c_config);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

void  BMP280_delay_msek(u32 msek)
{
	vTaskDelay(msek/portTICK_PERIOD_MS);
}

s8 I2C_routine(void) {
	bmp280.bus_write = BMP280_I2C_bus_write;
	bmp280.bus_read = BMP280_I2C_bus_read;
	bmp280.dev_addr = BMP280_I2C_ADDRESS2;
	bmp280.delay_msec = BMP280_delay_msek;

	return BMP280_INIT_VALUE;
}


void app_main(void)
{
	s32 com_rslt = ERROR;
	s32 v_data_uncomp_tem_s32, v_actual_temp_s32;
	
	ESP_LOGI(TAG_BMP280, "Démarrage...");
	
	i2c_master_init();
	
	I2C_routine();
	
	com_rslt = bmp280_init(&bmp280);
	com_rslt += bmp280_set_power_mode(BMP280_NORMAL_MODE);
	com_rslt += bmp280_set_work_mode(BMP280_ULTRA_LOW_POWER_MODE);
	com_rslt += bmp280_set_standby_durn(BMP280_STANDBY_TIME_1_MS);
	
	com_rslt += bmp280_read_uncomp_temperature(&v_data_uncomp_tem_s32);
	v_actual_temp_s32 = bmp280_compensate_temperature_int32(v_data_uncomp_tem_s32);
	
	if (com_rslt != SUCCESS)
		ESP_LOGE(TAG_BMP280, "erreur...code: %d", com_rslt);
		
	if (com_rslt == SUCCESS)
		ESP_LOGI(TAG_BMP280, "%d degC", v_actual_temp_s32);
		
	com_rslt += bmp280_set_power_mode(BMP280_SLEEP_MODE);
	

}
