#include <stdio.h>
#include "as3953.h"
#include "ams_spi.h"

void as3953_get_info_register(as3953_register_t *dev)
{
    uint8_t i;
    uint8_t buffer[0x12];

    for (i = 0x00; i < 0x12; i++)
    {
        if (i != 0x03 || i != 0x06 | i != 0x07 | i != 0x0e | i != 0x0f)
        	ams_read_register(i, &buffer[i]);
    }

    dev->io_config = buffer[0x00];
    dev->mode_define = buffer[0x01];
    dev->bit_rate_define = buffer[0x02];
    dev->rfid_status = buffer[0x04];
    dev->rats = buffer[0x05];
    dev->mask_main_int = buffer[0x08];
    dev->mask_aux_int = buffer[0x09];
    dev->main_int = buffer[0x0A];
    dev->aux_int = buffer[0x0B];
    dev->fifo_status1 = buffer[0x0C];
    dev->fifo_status2 = buffer[0x0D];
    dev->num_tran_bytes1 = buffer[0x10];
    dev->num_tran_bytes2 = buffer[0x11];
}

void as3953_show_info_register(as3953_register_t *dev)
{
    printf("AS3953 device:\r\n");
    printf("0x00 io_config: %02x\r\n", dev->io_config);
    printf("0x01 mode_define: %02x\r\n", dev->mode_define);
    printf("0x02 it_rate_define: %02x\r\n", dev->bit_rate_define);
    printf("0x04 rfid_status: %02x\r\n", dev->rfid_status);
    printf("0x05 rats: %02x\r\n", dev->rats);
    printf("0x08 mask_main_int: %02x\r\n", dev->mask_main_int);
    printf("0x09 mask_aux_int: %02x\r\n", dev->mask_aux_int);
    printf("0x0a main_int: %02x\r\n", dev->main_int);
    printf("0x0b aux_int: %02x\r\n", dev->aux_int);
    printf("0x0c fifo_status1: %02x\r\n", dev->fifo_status1);
    printf("0x0d fifo_status2: %02x\r\n", dev->fifo_status2);
    printf("0x10 num_tran_bytes1: %02x\r\n", dev->num_tran_bytes1);
    printf("0x11 num_tran_bytes2: %02x\r\n", dev->num_tran_bytes2);
}

/**
 * read register
 **/

void as3953_read_io_config_reg(uint8_t *data)
{
    ams_read_register(AS3953_IO_CONFIG_REG, data);
}

void as3953_read_mode_define_reg(uint8_t *data)
{
    ams_read_register(AS3953_MODE_DEFINE_REG, data);
}

void as3953_read_bit_rate_reg(uint8_t *data)
{
    ams_read_register(AS3953_BIT_RATE_REG, data);
}

void as3953_read_rfid_status_reg(uint8_t *data)
{
    ams_read_register(AS3953_RFID_STATUS_REG, data);
}

void as3953_read_rats_reg(uint8_t *data)
{
    ams_read_register(AS3953_RATS_REG, data);
}

void as3953_read_mask_main_int_reg(uint8_t *data)
{
    ams_read_register(AS3953_MASK_MAIN_INT_REG, data);
}

void as3953_read_mask_aux_int_reg(uint8_t *data)
{
    ams_read_register(AS3953_MASK_AUX_INT_REG, data);
}

void as3953_read_main_int_reg(uint8_t *data)
{
    ams_read_register(AS3953_MAIN_INT_REG, data);
}

void as3953_read_aux_int_reg(uint8_t *data)
{
    ams_read_register(AS3953_AUX_INT_REG, data);
}

void as3953_read_fifo_status1_reg(uint8_t *data)
{
    ams_read_register(AS3953_FIFO_STAT_1_REG, data);
}

void as3953_read_fifo_status2_reg(uint8_t *data)
{
    ams_read_register(AS3953_FIFO_STAT_2_REG, data);
}

void as3953_read_num_tran_bytes1_reg(uint8_t *data)
{
    ams_read_register(AS3953_NUM_TX_BYTE_1_REG, data);
}

void as3953_read_num_tran_bytes2_reg(uint8_t *data)
{
    ams_read_register(AS3953_NUM_TX_BYTE_2_REG, data);
}

/**
 * write register
 **/

void as3953_write_io_config_reg(uint8_t *data)
{
    ams_read_register(AS3953_IO_CONFIG_REG, data);
}

void as3953_write_mode_define_reg(uint8_t *data)
{
    ams_read_register(AS3953_MODE_DEFINE_REG, data);
}

void as3953_write_bit_rate_reg(uint8_t *data)
{
    ams_read_register(AS3953_BIT_RATE_REG, data);
}

void as3953_write_mask_main_int_reg(uint8_t *data)
{
    ams_read_register(AS3953_MASK_MAIN_INT_REG, data);
}

void as3953_write_mask_aux_int_reg(uint8_t *data)
{
    ams_read_register(AS3953_MASK_AUX_INT_REG, data);
}

void as3953_write_num_tran_bytes1_reg(uint8_t *data)
{
    ams_read_register(AS3953_NUM_TX_BYTE_1_REG, data);
}

void as3953_write_num_tran_bytes2_reg(uint8_t *data)
{
    ams_read_register(AS3953_NUM_TX_BYTE_2_REG, data);
}

void as3953_set_bit_rate(ui)
{
    
}
/************************ (C) COPYRIGHT Kien Minh Co.,Ltd *****END OF FILE****/
