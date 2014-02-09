#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <linux/spi/flash.h>
#include <linux/mtd/partitions.h>
#include <asm/mach-types.h>
#include <asm/hardware/nvic.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>
#include <mach/hardware.h>
#include <mach/uart.h>
#include <mach/spi.h>
#include <mach/leds.h>
#include <mach/memory.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/clock.h>
#include <mach/dma.h>
#include <mach/sram.h>
#include <mach/pins.h>
#include <mach/cpu.h>
#include <net/telit_he910.h>

/***************************************************************************/

static struct resource board_spi_resources0[] = {
  {
         .start = STLR_SSI0_BASE,
         .end = STLR_SSI0_BASE + SZ_1K - 1,
         .flags = IORESOURCE_MEM,
  }, {
         .start = STLR_SSI0_IRQ,
         .end = STLR_SSI0_IRQ,
         .flags = IORESOURCE_IRQ,
  },
};

static struct resource board_spi_resources1[] = {
  {
         .start = STLR_SSI1_BASE,
         .end = STLR_SSI1_BASE + SZ_1K - 1,
         .flags = IORESOURCE_MEM,
  }, {
         .start = STLR_SSI1_IRQ,
         .end = STLR_SSI1_IRQ,
         .flags = IORESOURCE_IRQ,
  },
};

static int board_spi0_cs[] = {GPIO_SSI_CS_SF, GPIO_SSI_CS_EE};

#ifdef CONFIG_STELLARIS_DMA
char __sramdata ssi0_dma_rx_buffer[DMA_MAX_TRANSFER_SIZE];
char __sramdata ssi0_dma_tx_buffer[DMA_MAX_TRANSFER_SIZE];
#endif

static struct spi_stellaris_master board_spi_0_data = {
  .chipselect = board_spi0_cs,
  .num_chipselect = ARRAY_SIZE(board_spi0_cs),
#ifdef CONFIG_STELLARIS_DMA
	.dma_rx_channel = DMA_CHANNEL_SSI0_RX,
	.dma_tx_channel = DMA_CHANNEL_SSI0_TX,
	.dma_rx_buffer = ssi0_dma_rx_buffer,
	.dma_tx_buffer = ssi0_dma_tx_buffer,
#endif
};

static int board_spi1_cs[] = {GPIO_SSI_CS_ETH};

#ifdef CONFIG_STELLARIS_DMA
char __sramdata ssi1_dma_rx_buffer[DMA_MAX_TRANSFER_SIZE];
char __sramdata ssi1_dma_tx_buffer[DMA_MAX_TRANSFER_SIZE];
#endif

static struct spi_stellaris_master board_spi_1_data = {
  .chipselect = board_spi1_cs,
  .num_chipselect = ARRAY_SIZE(board_spi1_cs),
#ifdef CONFIG_STELLARIS_DMA
	.dma_rx_channel = DMA_CHANNEL_SSI1_RX,
	.dma_tx_channel = DMA_CHANNEL_SSI1_TX,
	.dma_rx_buffer = ssi1_dma_rx_buffer,
	.dma_tx_buffer = ssi1_dma_tx_buffer,
#endif
};

struct platform_device board_spi_device0 = {
  .name = "stellaris-spi",
  .id = 0,
  .num_resources = ARRAY_SIZE(board_spi_resources0),
  .resource = board_spi_resources0,
  .dev.platform_data = &board_spi_0_data,
};

struct platform_device board_spi_device1 = {
  .name = "stellaris-spi",
  .id = 1,
  .num_resources = ARRAY_SIZE(board_spi_resources1),
  .resource = board_spi_resources1,
  .dev.platform_data = &board_spi_1_data,
};

static struct mtd_partition flash_partitions[] = {
  {
    .name = "root",
    .size = MTDPART_SIZ_FULL,
    .offset = 0,
  },
};

static struct flash_platform_data flash_chip = {
  .name     = "flash",
  .parts    = flash_partitions,
  .nr_parts = ARRAY_SIZE(flash_partitions),
  .type     = "w25q64",
};

static struct spi_eeprom eeprom_chip = {
  .name   = "eeprom",
  .byte_len = 64 * 1024,
  .page_size  = 128,
  .flags    = EE_ADDR2,
};

static struct spi_board_info spi_devices[] = {
  {
    .modalias      = "w25q64",
    .max_speed_hz  = 10 * 1000000,
    .bus_num       = 0,
    .chip_select   = 0,
    .platform_data = &flash_chip,
  },
  {
    .modalias      = "at25",
    .max_speed_hz  = 5 * 1000000,
    .bus_num       = 0,
    .chip_select   = 1,
    .platform_data = &eeprom_chip,
  },
  {
    .modalias      = "ks8851",
    .max_speed_hz  = 5 * 1000000,
    .bus_num       = 1,
    .chip_select   = 0,
    .irq           = STLR_GPIOF_IRQ, // ETH IRQ on PG5
  }
};

/***************************************************************************/

static struct platform_device uart_device = {
  .name     = "uart-stellaris",
  .id     = 0,
};

/***************************************************************************/

static struct platform_device wdt_device = {
	.name		= "stellaris-wdt",
	.id		= -1,
	.num_resources	= 0,
};

/***************************************************************************/

#ifdef CONFIG_STELLARIS_ADC
struct platform_device adc_device = {
	.name		  = "stellaris-adc",
	.id		  = -1,
};
#endif

/***************************************************************************/

#ifdef CONFIG_ATLAS_CPU_LED
static struct stellaris_led_platdata cpu_led_pdata = {
	.name           = "cpu-led",
	.gpio           = GPIO_CPU_LED,
	.flags          = 0,
	.def_trigger    = "cpuidle",
};

static struct platform_device cpu_led = {
	.name               = "stellaris-led",
	.id                 = 1,
	.dev.platform_data  = &cpu_led_pdata,
};
#endif

/***************************************************************************/

#ifdef CONFIG_ATLAS_POWER
static struct platform_device power_device = {
	.name           = "atlas-power",
	.id             = -1,
};
#endif

/***************************************************************************/

static struct telit_platform_data telit_pdata = {
	.pwr_mon_gpio  = GPIO_TL_PWRMON,
	.pwr_on_gpio   = GPIO_TL_PWR_ON,
	.shutdown_gpio = GPIO_TL_SHUTDOWN,
	.if_en_gpio    = GPIO_TL_IF_EN,
};

static struct platform_device telit_device = {
	.name              = "telit_he910",
	.id                = 1,
	.dev.platform_data = &telit_pdata,
};

/***************************************************************************/

static struct platform_device *devices[] = {
  &uart_device,
  &board_spi_device0,
	&board_spi_device1,
	&wdt_device,
#ifdef CONFIG_ATLAS_CPU_LED
	&cpu_led,
#endif
#ifdef CONFIG_STELLARIS_ADC
	&adc_device,
#endif
#ifdef CONFIG_ATLAS_POWER
	&power_device,
#endif
	&telit_device
};

/***************************************************************************/

#ifdef CONFIG_COPY_TO_SRAM
extern unsigned int __sram_start[], __sram_end[], __sram_load_address[];
#endif

/***************************************************************************/

extern void tm4c_clock_init(void);

static void __init board_init(void)
{
	tm4c_clock_init();
#ifdef CONFIG_COPY_TO_SRAM
	memcpy(__sram_start, __sram_load_address,
	       (unsigned long)__sram_end - (unsigned long)__sram_start);
#endif
	platform_add_devices(devices, ARRAY_SIZE(devices));
	spi_register_board_info(spi_devices, ARRAY_SIZE(spi_devices));

	gpioirqenable(GPIO_ETH_INTRN);
}

/***************************************************************************/

static void __init board_init_irq(void)
{
  nvic_init();
}

/***************************************************************************/

static void __init board_map_io(void)
{
}

/***************************************************************************/
/***************************************************************************/

MACHINE_START(ATLAS, "atlas")
  .phys_io  = STLR_PERIPH_BASE,
  .io_pg_offst  = (IO_ADDRESS(0) >> 18) & 0xfffc,
  .boot_params  = PHYS_OFFSET + 0x100,
  .map_io   = board_map_io,
  .init_irq = board_init_irq,
  .timer    = &stellaris_timer,
  .init_machine = board_init,
MACHINE_END
