#include "ams.h"

#include <stdio.h>
#include "fsl_common.h"
#include "fsl_iocon.h"
#include "fsl_spi.h"
#include "fsl_gpio.h"

#define  SPI_RX_BUF_SIZE     64U
static   uint8_t  g_SpiRxBuf[SPI_RX_BUF_SIZE] = { 0 };

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
	/*
	send_recv(0x00| addr);
	send_recv(tx);

	UNSELECT();
	SELECT();
	*/
	uint8_t tx_data[2] = {address & 0x1f, data};
	return spi_transfer(tx_data, NULL, sizeof(tx_data));
}

status_t ams_read_register(uint8_t address, uint8_t *rx_data)
{
	status_t xRet = 0;
	uint16_t len = 0;

	/*
	send_recv(0x20| (addr & 0x1f));
	uint8_t data = send_recv(0);
	UNSELECT();
	SELECT();
	return data;
	*/
	uint8_t tx_data[2] = {0x20 | (address & 0x1f), 0x00};
	len = sizeof(tx_data);
	xRet = spi_transfer(tx_data, &g_SpiRxBuf[0], len);

	UNSELECT();
	SELECT();
	*rx_data = g_SpiRxBuf[len - 1];

	return xRet;
}

void ams_read_info_register(AMS_DEVICE * dev)
{
	uint8_t i;
	uint8_t data_regs[0x1f];

	for (i = 0x00; i < 0x0d; i++)
	{
		ams_read_register(i, &data_regs[i]);
	}

	dev->regs.io_conf = data_regs[0x00];
	dev->regs.ic_conf0 = data_regs[0x01];
	dev->regs.ic_conf1 = data_regs[0x02];
	dev->regs.ic_conf2 = data_regs[0x03];
	dev->regs.rfid_status = data_regs[0x04];
	dev->regs.ic_status = data_regs[0x05];
	dev->regs.mask_int0 = data_regs[0x08];
	dev->regs.mask_int1 = data_regs[0x09];
	dev->regs.int0 = data_regs[0x0a];
	dev->regs.int1 = data_regs[0x0b];
	dev->regs.buffer_status2 = data_regs[0x0c];
	dev->regs.buffer_status1 = data_regs[0x0d];
	dev->regs.last_nfc_addr = data_regs[0x0e];
	dev->regs.product_type = data_regs[0x1c];
	dev->regs.product_subtype = data_regs[0x1d];
	dev->regs.version_maj = data_regs[0x1e];
	dev->regs.version_min = data_regs[0x1f];
}

void ams_print_device(AMS_DEVICE * dev)
{
    printf("AMS_DEVICE:\r\n");
    printf("    io_conf:        %02x\r\n",dev->regs.io_conf);
    printf("    ic_conf0:       %02x\r\n",dev->regs.ic_conf0);
    printf("    ic_conf1:       %02x\r\n",dev->regs.ic_conf1);
    printf("    ic_conf2:       %02x\r\n",dev->regs.ic_conf2);
    printf("    rfid_status:    %02x\r\n",dev->regs.rfid_status);
    printf("    ic_status:      %02x\r\n",dev->regs.ic_status);
    printf("    mask_int0:      %02x\r\n",dev->regs.mask_int0);
    printf("    mask_int1:      %02x\r\n",dev->regs.mask_int1);
    printf("    int0:           %02x\r\n",dev->regs.int0);
    printf("    int1:           %02x\r\n",dev->regs.int1);
    printf("    buffer_status2: %02x\r\n",dev->regs.buffer_status2);
    printf("    buffer_status1: %02x\r\n",dev->regs.buffer_status1);
    printf("    last_nfc_addr:  %02x\r\n",dev->regs.last_nfc_addr);
    printf("    product_type:   %02x\r\n",dev->regs.product_type);
    printf("    product_subtype:%02x\r\n",dev->regs.product_subtype);
    printf("    version_maj:    %02x\r\n",dev->regs.version_maj);
    printf("    version_min:    %02x\r\n",dev->regs.version_min);
}

status_t ams_read_buffer(uint8_t *data, int rx_size)
{
	status_t xRet = 0;

	/*
	send_recv(0xa0);
	while(len--)
	{
		*data++ = send_recv(0x00);
	}

	UNSELECT();
	SELECT();
	*/

	uint8_t tx_data = 0xa0;
	spi_transfer(&tx_data, NULL, 1);

	xRet = spi_transfer(NULL, g_SpiRxBuf, rx_size);
	memcpy(data, g_SpiRxBuf, rx_size);

	return xRet;
}

status_t ams_write_buffer(uint8_t *data, int size)
{
	/*
	send_recv(0x80);
	while(len--)
	{
		send_recv(*data++);
	}

	UNSELECT();
	SELECT();
	*/
	uint8_t tx_data = 0xa0;
	spi_transfer(&tx_data, NULL, 1);
	return spi_transfer(data, NULL, size);
}

status_t ams_read_eeprom(uint8_t address, uint8_t *rx_data, uint16_t rx_size)
{
	status_t ret = 0;
	/*
	send_recv(0x7f);
	send_recv(block << 1);

	data[0] = send_recv(0);
	data[1] = send_recv(0);
	data[2] = send_recv(0);
	data[3] = send_recv(0);

	UNSELECT();
	SELECT();
	*/
	uint8_t tx_data[2] = {0x7f, address << 1};
	spi_transfer(tx_data, NULL, sizeof(tx_data));

	ret = spi_transfer(NULL, &g_SpiRxBuf[0], rx_size);
	UNSELECT();
	SELECT();

	memcpy(rx_data, g_SpiRxBuf, rx_size);

	return ret;
}

status_t ams_write_eeprom(uint8_t address, uint8_t *data)
{
	/*
	send_recv(0x40);
	send_recv(block << 1);

	send_recv(data[0]);
	send_recv(data[1]);
	send_recv(data[2]);
	send_recv(data[3]);

	UNSELECT();
	SELECT();
	*/
	uint8_t tx_data[2] = {0x40, address << 1};
	spi_transfer(tx_data, NULL, sizeof(tx_data));

	return spi_transfer(data, NULL, sizeof(data));
}

status_t ams_write_command(uint8_t cmd)
{
	/*
	send_recv(0xc0 | cmd);
	UNSELECT();
	SELECT();
	*/
	uint8_t tx_data = 0xc0 | cmd;
	return spi_transfer(&tx_data, NULL, 1);
}

int ams_init(void)
{

	spi_init();

    uint8_t productType = 0;
    ams_read_register(AMS_REG_PRODUCT_TYPE, &productType);
    if (productType == 0x14)
    {
        return 1;
    }

    return 0;
}

void ams_test_spi(void)
{
	uint8_t productType = 0;
	ams_read_register(AMS_REG_PRODUCT_TYPE, &productType);
	if (productType == 0x14)
	{
		asm("NOP");
	}
}
/************************ (C) COPYRIGHT Kien Minh Co.,Ltd *****END OF FILE****/
