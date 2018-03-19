#include <linux/switch.h>

#define QUARTZ_MAX3218_DRIVER_NAME "max3218"	

struct max3218_data {
	struct platform_device *max_pdev;
	struct device *dev;

	struct mutex lock;
	struct switch_dev detect_switch;
	bool plugged;

	int irq_det_n;
	int rs232_shdn_n;
	int rs232_en;
	int rs232_det_n;
};
