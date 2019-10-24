
#include "wire_config.h"
#include "wire.h"

#define I2C_MASTER_TX_BUF_DISABLE 0     /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0     /*!< I2C master doesn't need buffer */

wire_t wire0 = 
{
	.i2c_num = 0,
	.scl_io_num = CONFIG_WIRE_PORT0_I2C_SCL_GPIO,
	.sda_io_num = CONFIG_WIRE_PORT0_I2C_SDA_GPIO,
	.clk_speed = CONFIG_WIRE_PORT0_CLOCK_SPEED
};

wire_t wire1 =
{
	.i2c_num = 1,
	.scl_io_num = CONFIG_WIRE_PORT1_I2C_SCL_GPIO,
	.sda_io_num = CONFIG_WIRE_PORT1_I2C_SDA_GPIO,
	.clk_speed = CONFIG_WIRE_PORT0_CLOCK_SPEED
};

esp_err_t 
wire_init(wire_t* wire)
{
    i2c_config_t conf = 
    {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = wire->sda_io_num,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = wire->scl_io_num,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = wire->clk_speed
    };

    esp_err_t err = i2c_param_config(wire->i2c_num, &conf);

	if( err != ESP_OK ) 
	{
		return err;
	}

    return i2c_driver_install(wire->i2c_num, conf.mode,
				I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


uint8_t
wire_read(wire_t *p_wire, uint8_t device_addr, uint8_t reg_addr)
{
	uint8_t rd_data = 0;
	wire_read_bytes( p_wire, device_addr, reg_addr, &rd_data, 1);
	return rd_data;
}

uint8_t 
wire_read_bytes(wire_t *p_wire, uint8_t device_addr, uint8_t reg_addr, 
				uint8_t *p_buff_rd, uint8_t length)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	esp_err_t ret;

	if(length == 0)
	{
		return -1;
	}

	wire_begin_transmission(p_wire, device_addr, reg_addr);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (length > 1) {
        i2c_master_read(cmd, p_buff_rd, length - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, p_buff_rd + length - 1, NACK_VAL);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(p_wire->i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

uint8_t 
wire_write(wire_t *p_wire, uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	esp_err_t ret;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(p_wire->i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

uint8_t 
wire_write_bytes(wire_t *p_wire, uint8_t device_addr, uint8_t reg_addr, 
				 uint8_t *p_buff_wr, uint8_t length)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	esp_err_t ret;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write(cmd, p_buff_wr, length, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(p_wire->i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

uint8_t 
wire_begin_transmission(wire_t *p_wire, uint8_t device_addr, uint8_t reg_addr)
{
	i2c_cmd_handle_t cmd;

	cmd = i2c_cmd_link_create();

	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, 1));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_addr, 1));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(p_wire->i2c_num, cmd, 1000/portTICK_PERIOD_MS));

	i2c_cmd_link_delete(cmd);

	return 0;
}
