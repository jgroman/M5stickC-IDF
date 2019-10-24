#ifndef __WIRE_H__
#define __WIRE_H__

#include <stdio.h>

#include "driver/i2c.h"

#define ACK_CHECK_EN  0x1       /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0       /*!< I2C master will not check ack from slave */
#define ACK_VAL       0x0       /*!< I2C ack value */
#define NACK_VAL      0x1       /*!< I2C nack value */

typedef struct wire_s {
	i2c_port_t i2c_num;
	gpio_num_t scl_io_num;
	gpio_num_t sda_io_num;
	uint32_t   clk_speed;
} wire_t;

extern wire_t wire0;	// I2C Port 0
extern wire_t wire1;	// I2C Port 1

esp_err_t 
wire_init(wire_t* wire);

// Returns Wire struct of a given port
wire_t
wire_get_port_data(i2c_port_t i2c_num);

uint8_t
wire_read(wire_t *p_wire, uint8_t device_addr, uint8_t reg_addr);

uint8_t 
wire_read_bytes(wire_t *p_wire, uint8_t device_addr, uint8_t reg_addr, 
				uint8_t *p_buff_rd, uint8_t length);

uint8_t 
wire_write(wire_t *p_wire, uint8_t device_addr, uint8_t reg_addr, uint8_t value);

uint8_t 
wire_write_bytes(wire_t *p_wire, uint8_t device_addr, uint8_t reg_addr, 
				 uint8_t *p_buff_wr, uint8_t length);

uint8_t 
wire_begin_transmission(wire_t *p_wire, uint8_t device_addr, uint8_t reg_addr);

#endif