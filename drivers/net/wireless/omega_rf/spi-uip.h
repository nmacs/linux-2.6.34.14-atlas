/**
 * \file	spi-uip.h
 **/

#ifndef OMEGA_SPI_UIP_H
#define OMEGA_SPI_UIP_H

#define SPI_CMD_NOP        0
#define SPI_CMD_READ       1
#define SPI_CMD_WRITE      2
#define SPI_CMD_SETHWADDR  3
#define SPI_CMD_IRQACK     4

#define SPI_IRQ_RX         1
#define SPI_IRQ_TX         2

#define SPI_STATUS_MAGIC   0xEE
#define SPI_CMD_MAGIC      0xAA

struct omega_rf_hdr
{
	uint8_t srcaddr[8];
	uint8_t destaddr[8];
	int8_t rssi;
	uint8_t reserved[7];
};

struct spi_command_t
{
	uint8_t magic;
	uint8_t cmd;
	union {
		struct {
			uint16_t size;
		} write;
		struct {
			uint16_t size;
		} read;
		struct {
			uint8_t addr[8];
		} setaddr;
		uint8_t u8[14];
	};
	
};

struct spi_status_t
{
	uint8_t magic;
	uint8_t pending_irqs;
	uint16_t rx_packet_size;
	uint16_t tx_queue_freespace;
	uint8_t reserved[10];
};

#if CONTIKI

void spi_uip_init(void);
void set_hw_address(rimeaddr_t *addr);

#endif

#endif /* OMEGA_SPI_UIP_H */
