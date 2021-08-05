#include <stdio.h>
#include "fsl_common.h"
#include "fsl_iocon.h"
#include "fsl_spi.h"
#include "fsl_gpio.h"
#include "ams_spi.h"

void spi_pins_init(void)
{
    const uint32_t port0_pin0_config = (
        IOCON_PIO_FUNC2 |               /* Pin is configured as FC3_SCK */
        IOCON_PIO_MODE_PULLUP |         /* Selects pull-up function */
        IOCON_PIO_SLEW_STANDARD |       /* Standard mode, output slew rate control is enabled */
        IOCON_PIO_INV_DI |              /* Input function is not inverted */
        IOCON_PIO_DIGITAL_EN |          /* Enables digital function */
        IOCON_PIO_OPENDRAIN_DI          /* Open drain is disabled */
    );

    const uint32_t port0_pin2_config = (
        IOCON_PIO_FUNC1 |               /* Pin is configured as FC3_TXD_SCL_MISO_WS */
        IOCON_PIO_MODE_PULLUP |         /* Selects pull-up function */
        IOCON_PIO_SLEW_STANDARD |       /* Standard mode, output slew rate control is enabled */
        IOCON_PIO_INV_DI |              /* Input function is not inverted */
        IOCON_PIO_DIGITAL_EN |          /* Enables digital function */
        IOCON_PIO_OPENDRAIN_DI          /* Open drain is disabled */
    );

    const uint32_t port0_pin3_config = (
        IOCON_PIO_FUNC1 |               /* Pin is configured as FC3_RXD_SDA_MOSI_DATA */
        IOCON_PIO_MODE_PULLUP |         /* Selects pull-up function */
        IOCON_PIO_SLEW_STANDARD |       /* Standard mode, output slew rate control is enabled */
        IOCON_PIO_INV_DI |              /* Input function is not inverted */
        IOCON_PIO_DIGITAL_EN |          /* Enables digital function */
        IOCON_PIO_OPENDRAIN_DI          /* Open drain is disabled */
    );

    IOCON_PinMuxSet(IOCON, 0U, 0U, port0_pin0_config);
    IOCON_PinMuxSet(IOCON, 0U, 2U, port0_pin2_config);
    IOCON_PinMuxSet(IOCON, 0U, 3U, port0_pin3_config);

    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t spi_cs_config = {
        kGPIO_DigitalOutput, 0,
    };

    /* Init output LED GPIO. */
    GPIO_PinInit(GPIO, SPI_CS_PORT, SPI_CS_PIN, &spi_cs_config);
    GPIO_PinWrite(GPIO, SPI_CS_PORT, SPI_CS_PIN, 1);
}

void spi_init(void)
{
    spi_master_config_t spi_config = {0};

    spi_pins_init();

    /* attach 12 MHz clock to SPI2 */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM3);
    /* reset FLEXCOMM for SPI */
    RESET_PeripheralReset(kFC3_RST_SHIFT_RSTn);

    /* Init SPI master */
    /*
    * userConfig.enableLoopback = false;
    * userConfig.enableMaster = true;
    * userConfig.polarity = kSPI_ClockPolarityActiveHigh;
    * userConfig.phase = kSPI_ClockPhaseFirstEdge;
    * userConfig.direction = kSPI_MsbFirst;
    * userConfig.baudRate_Bps = 500000U;
    */
    SPI_MasterGetDefaultConfig(&spi_config);
    spi_config.polarity = kSPI_ClockPolarityActiveLow;
    spi_config.phase = kSPI_ClockPhaseSecondEdge;
    spi_config.direction = kSPI_MsbFirst;
    spi_config.baudRate_Bps = 4000000U;
    SPI_MasterInit(DRV_SPI, &spi_config, DRV_SPI_CLK_FREQ);
}

status_t spi_transfer(uint8_t *tx_data, uint8_t *rx_data, uint8_t size)
{
    spi_transfer_t spi_config = {0};

    spi_config.txData   = tx_data;
    spi_config.rxData   = rx_data;
    spi_config.dataSize = size;
    spi_config.configFlags = kSPI_FrameAssert;

    return SPI_MasterTransferBlocking(DRV_SPI, &spi_config);
}

status_t ams_write_register(uint8_t address, uint8_t data)
{
    uint8_t tx_data[2] = {address & 0x1f, data};
    return spi_transfer(tx_data, NULL, sizeof(tx_data));
}

status_t ams_read_register(uint8_t address, uint8_t *data)
{
    status_t status = 0;
    uint8_t rx_buffer[32] = {0x00};
    uint8_t tx_data[2] = {0x20 | (address & 0x1f), 0x00};
    uint8_t len = 0;
    len = sizeof(tx_data);

    status = spi_transfer(tx_data, rx_buffer, len);
    UNSELECT();
    SELECT();
    *data = rx_buffer[len - 1];

    return status;
}

status_t ams_eeprom_write(uint8_t address, uint8_t *data, uint16_t len)
{
    uint8_t tx_data[2] = {0x40, address << 1};

    spi_transfer(tx_data, NULL, sizeof(tx_data));
    return spi_transfer(data, NULL, len);
}

status_t ams_eeprom_read(uint8_t address, uint8_t *data, uint16_t len)
{
    status_t status = 0;
    uint8_t rx_buffer[32] = {0x00};
    uint8_t tx_data[2] = {0x7f, address << 1};
    spi_transfer(tx_data, NULL, sizeof(tx_data));

    status = spi_transfer(NULL, rx_buffer, len);
    UNSELECT();
    SELECT();

    memcpy(data, rx_buffer, len);
    return status;
}

status_t ams_fifo_write(uint8_t *data, uint8_t len)
{
	status_t status = 0;
	uint8_t fifo_write = 0x80;
	//mask it fifo write
	spi_transfer(&fifo_write, NULL, 1);
	status = spi_transfer(data, NULL, len);
	return status;
}

status_t ams_fifo_read(uint8_t *data, uint8_t len)
{
	status_t status = 0;
	uint8_t fifo_read = 0xbf;
	uint8_t rx_buffer[32] = {0x00};

	spi_transfer(&fifo_read, NULL, 1);
	status = spi_transfer(NULL, rx_buffer, len);
	memcpy(data, rx_buffer, len);
	return status;
}

status_t ams_command(uint8_t cmd)
{
    uint8_t tx_data = 0xc0 | cmd;
    return spi_transfer(&tx_data, NULL, 1);
}

/************************ (C) COPYRIGHT Kien Minh Co.,Ltd *****END OF FILE****/
