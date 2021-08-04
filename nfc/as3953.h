#ifndef AS3953_H_
#define AS3953_H_

#include "fsl_spi.h"
#include "common.h"

#define SPI_CS_PORT         0U
#define SPI_CS_PIN          4U
#define SPI_CS_SET()        GPIO_PinWrite(GPIO, SPI_CS_PORT, SPI_CS_PIN, 1)
#define SPI_CS_CLR()        GPIO_PinWrite(GPIO, SPI_CS_PORT, SPI_CS_PIN, 0)


#define UNSELECT()          SPI_CS_SET()
#define SELECT()            SPI_CS_CLR()

#define DRV_SPI             SPI3
#define DRV_SPI_IRQ         FLEXCOMM3_IRQn
#define DRV_SPI_CLK_FREQ    CLOCK_GetFlexCommClkFreq(3U)

#define AMS_REG_IO_CONF                 0x00
#define AMS_REG_IC_CONF0                0x01
#define AMS_REG_IC_CONF1                0x02
#define AMS_REG_IC_CONF2                0x03
    #define AMS_RFCFG_EN                0x80
    #define AMS_TUN_MOD                 0x40
#define AMS_REG_RFID_STATUS             0x04
    #define AMS_HF_PON                  0x80
    #define AMS_STATE_MASK              0x78
    #define AMS_STATE_INVALID           0x04
    #define AMS_STATE_OFF               (0 << 3)
    #define AMS_STATE_SENSE             (1 << 3)
    #define AMS_STATE_RESOLUTION        (3 << 3)
    #define AMS_STATE_RESOLUTION_L2     (2 << 3)
    #define AMS_STATE_SELECTED          (6 << 3)
    #define AMS_STATE_SECTOR2           (7 << 3)
    #define AMS_STATE_SECTORX_2         (0xf << 3)
    #define AMS_STATE_SELECTEDX         (0xe << 3)
    #define AMS_STATE_SENSEX_L2         (0xa << 3)
    #define AMS_STATE_SENSEX            (0xb << 3)
    #define AMS_STATE_SLEEP             (0x9 << 3)
// ... //
#define AMS_REG_MASK_INT0               0x08
    #define AMS_MASK0_PU                (1<<7)  // power up
    #define AMS_MASK0_WU_A              (1<<6)  // selected INT
    #define AMS_MASK0_SLP               (1<<5)
    #define AMS_MASK0_EEW_RF            (1<<4)
    #define AMS_MASK0_EER_RF            (1<<3)
    #define AMS_MASK0_RXE               (1<<2)
    #define AMS_MASK0_TXE               (1<<1)
    #define AMS_MASK0_XRF               (1<<0)
#define AMS_REG_MASK_INT1               0x09
#define AMS_REG_INT0                    0x0a
    #define AMS_INT_XRF                 (1<<0)
    #define AMS_INT_TXE                 (1<<1)
    #define AMS_INT_RXE                 (1<<2)
    #define AMS_INT_EER_RF              (1<<3)
    #define AMS_INT_EEW_RF              (1<<4)
    #define AMS_INT_SLP                 (1<<5)
    #define AMS_INT_WU_A                (1<<6)
    #define AMS_INT_INIT                (1<<7)
#define AMS_REG_INT1                    0x0b
    #define AMS_INT_ACC_ERR             (1<<0)
    #define AMS_INT_EEAC_ERR            (1<<1)
    #define AMS_INT_IO_EEWR             (1<<2)
    #define AMS_INT_BF_ERR              (1<<3)
    #define AMS_INT_CRC_ERR             (1<<4)
    #define AMS_INT_PAR_ERR             (1<<5)
    #define AMS_INT_FRM_ERR             (1<<6)
    #define AMS_INT_RXS                 (1<<7)
#define AMS_REG_BUF2                    0x0c
    #define AMS_BUF_LEN_MASK            0x1f
    #define AMS_BUF_INVALID             0x80
#define AMS_REG_BUF1                    0x0d
// ... //
#define AMS_REG_PRODUCT_TYPE            0x1c
#define AMS_REG_PRODUCT_SUBTYPE         0x1d
#define AMS_REG_VERSION_MAJOR           0x1e
#define AMS_REG_VERSION_MINOR           0x1f

#define AMS_CONFIG_UID_ADDR				0x00
#define AMS_CONFIG_BLOCK0_ADDR          0x7e
#define AMS_CONFIG_BLOCK1_ADDR          0x7f

#define AMS_CFG1_VOLTAGE_LEVEL_1V9      (0x00<<2)
#define AMS_CFG1_VOLTAGE_LEVEL_2V0      (0x01<<2)
#define AMS_CFG1_VOLTAGE_LEVEL_2V1      (0x02<<2)
#define AMS_CFG1_VOLTAGE_LEVEL_2V2      (0x03<<2)
#define AMS_CFG1_VOLTAGE_LEVEL_2V3      (0x04<<2)
#define AMS_CFG1_VOLTAGE_LEVEL_2V4      (0x05<<2)
#define AMS_CFG1_VOLTAGE_LEVEL_2V5      (0x06<<2)
#define AMS_CFG1_VOLTAGE_LEVEL_2V6      (0x07<<2)
#define AMS_CFG1_VOLTAGE_LEVEL_2V7      (0x08<<2)
#define AMS_CFG1_VOLTAGE_LEVEL_2V8      (0x09<<2)
#define AMS_CFG1_VOLTAGE_LEVEL_2V9      (0x0a<<2)
#define AMS_CFG1_VOLTAGE_LEVEL_3V0      (0x0b<<2)

#define AMS_CFG1_OUTPUT_RESISTANCE_ZZ   0x00
#define AMS_CFG1_OUTPUT_RESISTANCE_100  0x01
#define AMS_CFG1_OUTPUT_RESISTANCE_50   0x02
#define AMS_CFG1_OUTPUT_RESISTANCE_25   0x03

#define AMS_CFG2_RFCFG_EN               (1<<7)
#define AMS_CFG2_TUN_MOD                (1<<6)

#define AMS_CMD_DEFAULT                 0x02
#define AMS_CMD_CLEAR_BUFFER            0x04
#define AMS_CMD_RESTART_TRANSCEIVER     0x06
#define AMS_CMD_DIS_EN_TRANSCEIVER      0x07
#define AMS_CMD_TRANSMIT_BUFFER         0x08
#define AMS_CMD_TRANSMIT_ACK            0x09
#define AMS_CMD_TRANSMIT_NACK0          0x0A
#define AMS_CMD_TRANSMIT_NACK1          0x0B
#define AMS_CMD_TRANSMIT_NACK4          0x0D
#define AMS_CMD_TRANSMIT_NACK5          0x0C
#define AMS_CMD_SLEEP                   0x10
#define AMS_CMD_SENSE                   0x11
#define AMS_CMD_SENSE_SLEEP             0x12


typedef union
{
    uint8_t buffer[0x20];
    struct {
        uint8_t io_config;          // 0x00 IO Configuration Register
        uint8_t mode_define;        // 0x01 Mode Definition Register
        uint8_t bit_rate_define;    // 0x02 Bit Rate Definition Register
        uint8_t _nc1;               // 0x03
        uint8_t rfid_status;        // 0x04 RFID Status Display Register
        uint8_t rats;               // 0x05 RATS Register
        uint8_t _nc2[2];            // 0x06 - 0x07
        uint8_t mask_main_int;      // 0x08 Mask Main Interrupt Register
        uint8_t mask_aux_int;       // 0x09 Mask Auxiliary Interrupt Register
        uint8_t main_int;           // 0x0A Main Interrupt Register
        uint8_t aux_int;            // 0x0B Auxiliary Interrupt Register
        uint8_t fifo_status1;       // 0x0C FIFO Status Register 1
        uint8_t fifo_status2;       // 0x0D FIFO Status Register 2
        uint8_t _nc3[2];            // 0x0E - 0x0F
        uint8_t num_tran_bytes1;    // 0x10 Number of Transmitted Bytes Register 1
        uint8_t num_tran_bytes2;    // 0x11 Number of Transmitted Bytes Register 2
    } regs;
} __attribute__((packed)) ams_device_t;


status_t as3953_read_register(uint8_t address, uint8_t *data);
status_t as3953_write_register(uint8_t address, uint8_t data);
status_t as3953_read_buffer(uint8_t *data, uint16_t len);
status_t as3953_write_buffer(uint8_t *data, uint16_t len);
status_t as3953_read_eeprom(uint8_t address, uint8_t *data, uint16_t len);
status_t as3953_write_eeprom(uint8_t address, uint8_t *data);
status_t as3953_write_command(uint8_t cmd);

uint8_t as3953_init(void);
void as3953_get_info_register(ams_device_t *dev);
void as3953_show_info_register(ams_device_t *dev);

#endif /* AS3953_H_ */

/************************ (C) COPYRIGHT Kien Minh Co.,Ltd *****END OF FILE****/
