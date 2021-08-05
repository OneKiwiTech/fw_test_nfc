#ifndef AMS_SPI_H_
#define AMS_SPI_H_

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

void spi_init(void);
status_t ams_write_register(uint8_t address, uint8_t data);
status_t ams_read_register(uint8_t address, uint8_t *data);
status_t ams_eeprom_write(uint8_t address, uint8_t *data, uint16_t len);
status_t ams_eeprom_read(uint8_t address, uint8_t *data, uint16_t len);
status_t ams_fifo_write(uint8_t *data, uint8_t len);
status_t ams_fifo_read(uint8_t *data, uint8_t len);
status_t ams_command(uint8_t cmd);

#endif /* AMS_SPI_H_ */

/**
 * https://github.com/DanielKalicki/ModularElectronics/blob/master/Code/NfcModule/src/icDrivers/AS3953.h
 * https://github.com/FlexCOS/code/blob/master/src/main/mod_rtos/as3953.c
 */

/************************ (C) COPYRIGHT Kien Minh Co.,Ltd *****END OF FILE****/
