#include "wire.h" 

#include "mpu6886_config.h"
#include "mpu6886.h"

enum Gscale Gyscale = GFS_2000DPS;
enum Ascale Acscale = AFS_8G;

static void
reg_read(uint8_t reg_addr, uint8_t length, uint8_t *p_buf_rd);

static void
reg_write(uint8_t reg_addr, uint8_t length, uint8_t *p_buf_wr);


int 
mpu6886_init(void)
{
    uint8_t tempdata[1];
    uint8_t regdata;

    reg_read(MPU6886_REG_WHOAMI, 1, tempdata);

    if (tempdata[0] != MPU6886_WHOAMI_VALUE)
    {
        return -1;
    }

    vTaskDelay(5 / portTICK_PERIOD_MS);

    regdata = 0x00;
    reg_write(MPU6886_PWR_MGMT_1, 1, &regdata);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    regdata = (0x01 << 7);
    reg_write(MPU6886_PWR_MGMT_1, 1, &regdata);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    regdata = (0x01 << 0);
    reg_write(MPU6886_PWR_MGMT_1, 1, &regdata);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    regdata = 0x10;
    reg_write(MPU6886_ACCEL_CONFIG, 1, &regdata);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    regdata = 0x18;
    reg_write(MPU6886_GYRO_CONFIG, 1, &regdata);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    regdata = 0x01;
    reg_write(MPU6886_REG_CONFIG, 1, &regdata);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    regdata = 0x05;
    reg_write(MPU6886_SMPLRT_DIV, 1, &regdata);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    regdata = 0x00;
    reg_write(MPU6886_INT_ENABLE, 1, &regdata);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    regdata = 0x00;
    reg_write(MPU6886_ACCEL_CONFIG2, 1, &regdata);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    regdata = 0x00;
    reg_write(MPU6886_USER_CTRL, 1, &regdata);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    regdata = 0x00;
    reg_write(MPU6886_FIFO_ENABLE, 1, &regdata);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    regdata = 0x22;
    reg_write(MPU6886_INT_PIN_CFG, 1, &regdata);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    regdata = 0x01;
    reg_write(MPU6886_INT_ENABLE, 1, &regdata);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    MPU6886getGres();
    MPU6886getAres();
    
    return 0;
}

void MPU6886getAccelAdc(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];
    reg_read(MPU6886_ACCEL_XOUT_H, 6, buf);

    *ax = ((int16_t)buf[0] << 8) | buf[1];
    *ay = ((int16_t)buf[2] << 8) | buf[3];
    *az = ((int16_t)buf[4] << 8) | buf[5];
}
void MPU6886getGyroAdc(int16_t *gx, int16_t *gy, int16_t *gz)
{

    uint8_t buf[6];
    reg_read(MPU6886_GYRO_XOUT_H, 6, buf);

    *gx = ((uint16_t)buf[0] << 8) | buf[1];
    *gy = ((uint16_t)buf[2] << 8) | buf[3];
    *gz = ((uint16_t)buf[4] << 8) | buf[5];
}

void MPU6886getTempAdc(int16_t *t)
{
    uint8_t buf[2];
    reg_read(MPU6886_TEMP_OUT_H, 2, buf);

    *t = ((uint16_t)buf[0] << 8) | buf[1];
}

void MPU6886getGres()
{
    switch (Gyscale)
    {
        // Possible gyro scales (and their register bit settings) are:
    case GFS_250DPS:
        gRes = 250.0 / 32768.0;
        break;
    case GFS_500DPS:
        gRes = 500.0 / 32768.0;
        break;
    case GFS_1000DPS:
        gRes = 1000.0 / 32768.0;
        break;
    case GFS_2000DPS:
        gRes = 2000.0 / 32768.0;
        break;
    }
}

void MPU6886getAres()
{
    switch (Acscale)
    {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
        aRes = 2.0 / 32768.0;
        break;
    case AFS_4G:
        aRes = 4.0 / 32768.0;
        break;
    case AFS_8G:
        aRes = 8.0 / 32768.0;
        break;
    case AFS_16G:
        aRes = 16.0 / 32768.0;
        break;
    }
}

void MPU6886setFIFOEnable(bool enableflag)
{
    uint8_t regdata = 0;
    if (enableflag)
    {
        regdata = 0x0c;
        reg_write(MPU6886_FIFO_ENABLE, 1, &regdata);
        regdata = 0x40;
        reg_write(MPU6886_USER_CTRL, 1, &regdata);
        //MPU6886_FIFO_ENABLE = 0x0C
        //MPU6886_USER_CTRL = 0x40
    }
    else
    {
        regdata = 0x00;
        reg_write(MPU6886_FIFO_ENABLE, 1, &regdata);
        regdata = 0x00;
        reg_write(MPU6886_USER_CTRL, 1, &regdata);
    }
}

uint8_t MPU6886ReadFIFO()
{
    uint8_t ReData = 0;
    reg_read(MPU6886_FIFO_R_W, 1, &ReData);
    return ReData;
}

void MPU6886ReadFIFOBuff(uint8_t *DataBuff, uint16_t Length)
{
    reg_read(MPU6886_FIFO_R_W, Length, DataBuff);
}

uint16_t MPU6886ReadFIFOCount()
{
    uint8_t Buff[2];
    uint16_t ReData = 0;
    reg_read(MPU6886_FIFO_CONUTH, 2, Buff);
    ReData = Buff[0];
    ReData <<= 8;
    ReData |= Buff[1];
    return ReData;
}

void MPU6886SetGyroFsr(enum Gscale scale)
{
    //return IIC_Write_Byte(MPU_GYRO_CFG_REG,scale<<3);//设置陀螺仪满量程范围
    unsigned char regdata;
    regdata = (scale << 3);
    reg_write(MPU6886_GYRO_CONFIG, 1, &regdata);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    Gyscale = scale;
    MPU6886getGres();
}

void MPU6886SetAccelFsr(enum Ascale scale)
{
    unsigned char regdata;
    regdata = (scale << 3);
    reg_write(MPU6886_ACCEL_CONFIG, 1, &regdata);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    Acscale = scale;
    MPU6886getAres();
}

void MPU6886getAccelData(float *ax, float *ay, float *az)
{
    int16_t accX = 0;
    int16_t accY = 0;
    int16_t accZ = 0;
    MPU6886getAccelAdc(&accX, &accY, &accZ);

    *ax = (float)accX * aRes;
    *ay = (float)accY * aRes;
    *az = (float)accZ * aRes;
}

void MPU6886getGyroData(float *gx, float *gy, float *gz)
{
    int16_t gyroX = 0;
    int16_t gyroY = 0;
    int16_t gyroZ = 0;
    MPU6886getGyroAdc(&gyroX, &gyroY, &gyroZ);

    *gx = (float)gyroX * gRes;
    *gy = (float)gyroY * gRes;
    *gz = (float)gyroZ * gRes;
}

void MPU6886getTempData(float *t)
{

    int16_t temp = 0;
    MPU6886getTempAdc(&temp);

    *t = (float)temp / 326.8 + 25.0;
}

static void
reg_read(uint8_t reg_addr, uint8_t length, uint8_t *p_buf_rd)
{
    wire_t wire_port = wire_get_port_data(CONFIG_MPU6886_I2C_PORT);
    wire_read_bytes(&wire_port, CONFIG_MPU6886_I2C_ADDRESS, reg_addr, p_buf_rd, length);
}

static void
reg_write(uint8_t reg_addr, uint8_t length, uint8_t *p_buf_wr)
{
    wire_t wire_port = wire_get_port_data(CONFIG_MPU6886_I2C_PORT);
    wire_write_bytes(&wire_port, CONFIG_MPU6886_I2C_ADDRESS, reg_addr, p_buf_wr, length);
}
