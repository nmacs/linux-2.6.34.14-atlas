#ifndef __TELIT_HE910_H
#define __TELIT_HE910_H

struct telit_platform_data {
	int pwr_mon_gpio;
	int pwr_on_gpio;
	int shutdown_gpio;
	int if_en_gpio;
	int spi_srdy_gpio;
};

#endif /* __TELIT_HE910_H */