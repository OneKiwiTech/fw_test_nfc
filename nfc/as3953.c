#include <stdio.h>
#include "fsl_common.h"
#include "fsl_iocon.h"
#include "fsl_spi.h"
#include "fsl_gpio.h"
#include "as3953.h"

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

status_t as3953_write_register(uint8_t address, uint8_t data)
{
    uint8_t tx_data[2] = {address & 0x1f, data};
    return spi_transfer(tx_data, NULL, sizeof(tx_data));
}

status_t as3953_read_register(uint8_t address, uint8_t *data)
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

status_t as3953_read_buffer(uint8_t *data, uint16_t len)
{
    status_t status = 0;
    uint8_t tx_data = 0xa0;
    uint8_t rx_buffer[32] = {0x00};

    spi_transfer(&tx_data, NULL, 1);
    status = spi_transfer(NULL, rx_buffer, len);
    memcpy(data, rx_buffer, len);

    return status;
}

status_t as3953_write_buffer(uint8_t *data, uint16_t len)
{
    uint8_t tx_data = 0xa0;
    spi_transfer(&tx_data, NULL, 1);
    return spi_transfer(data, NULL, len);
}

status_t as3953_read_eeprom(uint8_t address, uint8_t *data, uint16_t len)
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

status_t as3953_write_eeprom(uint8_t address, uint8_t *data)
{
    uint8_t tx_data[2] = {0x40, address << 1};

    spi_transfer(tx_data, NULL, sizeof(tx_data));
    return spi_transfer(data, NULL, sizeof(data));
}

status_t as3953_write_command(uint8_t cmd)
{
    uint8_t tx_data = 0xc0 | cmd;
    return spi_transfer(&tx_data, NULL, 1);
}


uint8_t as3953_init(void)
{
	spi_init();

	uint8_t productType = 0;
	as3953_read_register(AMS_REG_PRODUCT_TYPE, &productType);
	if (productType == 0x14)
	{
		return 1;
	}

	return 0;
}

void as3953_get_info_register(ams_device_t *dev)
{
	uint8_t i;
	uint8_t buffer[0x12];

	for (i = 0x00; i < 0x12; i++)
	{
		as3953_read_register(i, &buffer[i]);
	}

    dev->regs.io_config = buffer[0x00];
    dev->regs.mode_define = buffer[0x01];
    dev->regs.bit_rate_define = buffer[0x02];
    dev->regs.rfid_status = buffer[0x04];
    dev->regs.rats = buffer[0x05];
    dev->regs.mask_main_int = buffer[0x08];
    dev->regs.mask_aux_int = buffer[0x09];
    dev->regs.main_int = buffer[0x0A];
    dev->regs.aux_int = buffer[0x0B];
    dev->regs.fifo_status1 = buffer[0x0C];
    dev->regs.fifo_status2 = buffer[0x0D];
    dev->regs.num_tran_bytes1 = buffer[0x10];
    dev->regs.num_tran_bytes2 = buffer[0x11];
}

void as3953_show_info_register(ams_device_t *dev)
{
	printf("AS3953 device:\r\n");
	printf("0x00 io_config: %02x\r\n", dev->regs.io_config);
	printf("0x01 mode_define: %02x\r\n", dev->regs.mode_define);
	printf("0x02 it_rate_define: %02x\r\n", dev->regs.bit_rate_define);
	printf("0x04 rfid_status: %02x\r\n", dev->regs.rfid_status);
	printf("0x05 rats: %02x\r\n", dev->regs.rats);
	printf("0x08 mask_main_int: %02x\r\n", dev->regs.mask_main_int);
	printf("0x09 mask_aux_int: %02x\r\n", dev->regs.mask_aux_int);
	printf("0x0a main_int: %02x\r\n", dev->regs.main_int);
	printf("0x0b aux_int: %02x\r\n", dev->regs.aux_int);
	printf("0x0c fifo_status1: %02x\r\n", dev->regs.fifo_status1);
	printf("0x0d fifo_status2: %02x\r\n", dev->regs.fifo_status2);
	printf("0x10 num_tran_bytes1: %02x\r\n", dev->regs.num_tran_bytes1);
	printf("0x11 num_tran_bytes2: %02x\r\n", dev->regs.num_tran_bytes2);
}

/************************ (C) COPYRIGHT Kien Minh Co.,Ltd *****END OF FILE****/
