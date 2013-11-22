#define DEBUG
#define VERBOSE_DEBUG 1

#include <linux/module.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#include <mach/hardware.h>
#include <mach/pins.h>

#include <net/telit_he910.h>

#define POWER_PROC_ENTRY "driver/telit_pwr"
#define MAX_PROC_SIZE 100

enum telit_state {
	switched_off,
	switching_off,
	switching_on,
	switched_on
};

struct telit_modem {
	struct platform_device *pdev;
	struct proc_dir_entry *power_proc_entry;
	struct workqueue_struct *work_queue;
	struct work_struct work_power_on;
	struct work_struct work_power_off;
	int pwr_mon_gpio;
	int pwr_on_gpio;
	int shutdown_gpio;
	int if_en_gpio;
	int spi_srdy_gpio;
	enum telit_state state;
};

static inline int get_power_status(struct telit_modem *priv)
{
	return gpioread(priv->pwr_mon_gpio, 0);
}

static inline void interface_control(struct telit_modem *priv, int ctrl)
{
	gpiowrite(priv->if_en_gpio, ctrl);
}

static inline int is_modem_ready(struct telit_modem *priv)
{
	return !gpioread(priv->spi_srdy_gpio, 0);
}

#ifdef CONFIG_PROC_FS
static int read_proc_pwr(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
	struct telit_modem *priv = data;
	int len;
	char *state = 0;
	
	if (offset)
		return 0;
	
	switch(priv->state)
	{
		case switched_off:
			state = "off";
			break;
		case switching_off:
			state = "switching off";
			break;
		case switching_on:
			state = "switching on";
			break;
		case switched_on:
			state = "on";
			break;
	};
	len = sprintf(buf, "%s", state);
	*eof = 1;
	
	return len;
}

static int write_proc_pwr(struct file *file, const char __user *buf, unsigned long count, void *data)
{
	struct telit_modem *priv = data;
	char proc_data[MAX_PROC_SIZE];
	
	if(count > MAX_PROC_SIZE)
		count = MAX_PROC_SIZE;
	
	if(copy_from_user(proc_data, buf, count))
		return -EFAULT;
	
	if( strncmp(proc_data, "on", 2) == 0 )
		return queue_work(priv->work_queue, &priv->work_power_on);
	else if( strncmp(proc_data, "off", 2) == 0 )
		return queue_work(priv->work_queue, &priv->work_power_off);

	return count;
}

static int __devinit init_procfs(struct telit_modem *priv)
{
	priv->power_proc_entry = create_proc_entry(POWER_PROC_ENTRY, 0666, NULL);
	if (!priv->power_proc_entry) {
		printk(KERN_ERR "Error creating proc entry: " POWER_PROC_ENTRY);
		return -ENOMEM;
	}

	priv->power_proc_entry->read_proc = read_proc_pwr;
	priv->power_proc_entry->write_proc = write_proc_pwr;
	priv->power_proc_entry->data = priv;

	dev_dbg(&priv->pdev->dev, "init_procfs: done\n");

	return 0;
}
#endif

static void hw_shutdown_unconditional(struct telit_modem *priv)
{
	dev_dbg(&priv->pdev->dev, "Shutdown unconditional...\n");
	interface_control(priv, 0);
	gpiowrite(priv->shutdown_gpio, 0);
	msleep(200);
	gpiowrite(priv->shutdown_gpio, 1);
}

static int modem_on_proc(struct telit_modem *priv)
{
	int power_status = 0;

	//interface_control(priv, 1);
	//power_status = get_power_status(priv);
	if (power_status)
		return power_status;

	dev_dbg(&priv->pdev->dev, "Switching modem on...\n");
	priv->state = switching_on;

	gpiowrite(priv->pwr_on_gpio, 0);
	dev_vdbg(&priv->pdev->dev, "Hold PWR_ON\n");
	msleep(5000);
	gpiowrite(priv->pwr_on_gpio, 1);
	dev_vdbg(&priv->pdev->dev, "Release PWR_ON\n");

	msleep(200);
	//interface_control(priv, 1);
	//power_status = get_power_status(priv);
	power_status = 1;
	dev_dbg(&priv->pdev->dev, "Power: %s\n", power_status ? "ON" : "OFF");
	
	if (!power_status)
		hw_shutdown_unconditional(priv);
	else
		msleep(1000);
	
	return power_status;
}

static int modem_off_proc(struct telit_modem *priv)
{
	int i;
	int power_status;

	dev_dbg(&priv->pdev->dev, "Switching modem off...\n");

	//interface_control(priv, 0);

	gpiowrite(priv->pwr_on_gpio, 0);
	dev_vdbg(&priv->pdev->dev, "Hold PWR_ON\n");
	msleep(3000);
	gpiowrite(priv->pwr_on_gpio, 1);
	dev_vdbg(&priv->pdev->dev, "Release PWR_ON\n");

	msleep(200);

	for (i = 0; i < 15; i++ )
	{
		//interface_control(priv, 1);
		//power_status = get_power_status(priv);
		//interface_control(priv, 0);
		power_status = 0;
		if (!power_status)
			break;
		msleep(1000);
	}
	
	if (power_status)
		hw_shutdown_unconditional(priv);
	
	//interface_control(priv, 1);
	//power_status = get_power_status(priv);
	//interface_control(priv, 0);

	if (power_status)
		dev_err(&priv->pdev->dev, "Fail to power off\n");
	
	return power_status;
}

static int power_on(struct telit_modem *priv)
{
	int power_status = 0;
	int tries = 0;
	const int max_tries = 5;
	
	priv->state = switching_on;
	
	while (1) {
		power_status = modem_on_proc(priv);
		if (power_status || tries >= max_tries) break;
		tries++;
	}

	priv->state = power_status ? switched_on : switched_off;
	
	return power_status;
}

static int power_off(struct telit_modem *priv)
{
	priv->state = switching_off;
	modem_off_proc(priv);
	priv->state = switched_off;
	return 0;
}

static void telit_power_on(struct work_struct *work)
{
	struct telit_modem *priv = container_of(work, struct telit_modem, work_power_on);
	power_on(priv);
}

static void telit_power_off(struct work_struct *work)
{
	struct telit_modem *priv = container_of(work, struct telit_modem, work_power_off);
	power_off(priv);
}

static int __devinit telit_probe(struct platform_device *pdev)
{
	struct telit_modem *priv;
	int ret;
	struct telit_platform_data *plat = pdev->dev.platform_data;

	priv = kzalloc(sizeof(struct telit_modem), GFP_KERNEL);
	if (priv == 0)
		return -ENOMEM;

	priv->pdev = pdev;
	priv->pwr_mon_gpio = plat->pwr_mon_gpio;
	priv->pwr_on_gpio = plat->pwr_on_gpio;
	priv->shutdown_gpio = plat->shutdown_gpio;
	priv->if_en_gpio = plat->if_en_gpio;
	priv->spi_srdy_gpio = plat->spi_srdy_gpio;
	priv->state = switched_off;

	dev_set_drvdata(&pdev->dev, priv);
	
	priv->work_queue = create_workqueue("telit");
	if (priv->work_queue == 0)
	{
		printk(KERN_ERR "Fail to create workqueue\n");
		kfree(priv);
		return -ENOMEM;
	}
	
	INIT_WORK(&priv->work_power_on, telit_power_on);
	INIT_WORK(&priv->work_power_off, telit_power_off);

#ifdef CONFIG_PROC_FS
	ret = init_procfs(priv);
	if( ret )
	{
		destroy_workqueue(priv->work_queue);
		kfree(priv);
		return ret;
	}
#endif

#ifdef CONFIG_TELIT_HE910_POWER_ON
	if (!queue_work(priv->work_queue, &priv->work_power_on))
		dev_err(&pdev->dev, "fail to power on\n");
#endif

	dev_dbg(&pdev->dev, "probed\n");

	return 0;
}

static int __devexit telit_remove(struct platform_device *pdev)
{
	struct telit_modem *priv = dev_get_drvdata(&pdev->dev);
	dev_info(&pdev->dev, "remove\n");
	kfree(priv);
	return 0;
}

static struct platform_driver telit_driver = {
	.driver = {
		.name = "telit_he910",
		.owner = THIS_MODULE,
	},
	.probe = telit_probe,
	.remove = __devexit_p(telit_remove),
	//.suspend = telit_suspend,
	//.resume = telit_resume,
};

static int __init telit_init(void)
{
	return platform_driver_register(&telit_driver);
}

static void __exit telit_exit(void)
{
	platform_driver_unregister(&telit_driver);
}


module_init(telit_init);
module_exit(telit_exit);

MODULE_DESCRIPTION("Telit Modem HE910 driver");
MODULE_AUTHOR("Max Nekludov <Max.Nekludov@elster.com>");
MODULE_LICENSE("GPL");