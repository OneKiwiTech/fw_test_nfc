#ifndef AS3953_H_
#define AS3953_H_

#include "fsl_spi.h"
#include "common.h"

#define AS3953_IO_CONFIG_REG            0x00
#define AS3953_MODE_DEFINE_REG          0x01
#define AS3953_BIT_RATE_REG             0x02
#define AS3953_RFID_STATUS_REG          0x04
#define AS3953_RATS_REG                 0x05
#define AS3953_MASK_MAIN_INT_REG        0x08
#define AS3953_MASK_AUX_INT_REG         0x09
#define AS3953_MAIN_INT_REG             0x0A
#define AS3953_AUX_INT_REG              0x0B
#define AS3953_FIFO_STAT_1_REG          0x0C
#define AS3953_FIFO_STAT_2_REG          0x0D
#define AS3953_NUM_TX_BYTE_1_REG        0x10
#define AS3953_NUM_TX_BYTE_2_REG        0x11

typedef struct
{
    uint8_t io_config;          // 0x00 IO Configuration Register
    uint8_t mode_define;        // 0x01 Mode Definition Register
    uint8_t bit_rate_define;    // 0x02 Bit Rate Definition Register
    uint8_t rfid_status;        // 0x04 RFID Status Display Register
    uint8_t rats;               // 0x05 RATS Register
    uint8_t mask_main_int;      // 0x08 Mask Main Interrupt Register
    uint8_t mask_aux_int;       // 0x09 Mask Auxiliary Interrupt Register
    uint8_t main_int;           // 0x0A Main Interrupt Register
    uint8_t aux_int;            // 0x0B Auxiliary Interrupt Register
    uint8_t fifo_status1;       // 0x0C FIFO Status Register 1
    uint8_t fifo_status2;       // 0x0D FIFO Status Register 2
    uint8_t num_tran_bytes1;    // 0x10 Number of Transmitted Bytes Register 1
    uint8_t num_tran_bytes2;    // 0x11 Number of Transmitted Bytes Register 2
} __attribute__((packed)) as3953_register_t;


typedef struct
{
    uint8_t deselect_command_reception_irq;
    uint8_t framing_frror_irq;
    uint8_t parity_error_irg;
    uint8_t CRC_Error_IRQ;
    uint8_t FIFO_Error_IRQ;
    uint8_t EEPROM_Successful_Termination;
    uint8_t EEPROM_Programming_Error_IRQ;
    uint8_t EEPROM_Access_Due_To_PICC_Activation;
} as3953_aux_interrupt_t;

void as3953_get_info_register(as3953_register_t *dev);
void as3953_show_info_register(as3953_register_t *dev);
void as3953_read_register_block(as3953_register_t *dev);

#endif /* AS3953_H_ */

/**
 * https://github.com/DanielKalicki/ModularElectronics/blob/master/Code/NfcModule/src/icDrivers/AS3953.h
 * https://github.com/FlexCOS/code/blob/master/src/main/mod_rtos/as3953.c
 */

/************************ (C) COPYRIGHT Kien Minh Co.,Ltd *****END OF FILE****/
