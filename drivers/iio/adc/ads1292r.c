
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h> 
#include <linux/err.h> 
#include <linux/sched.h>
#include <linux/delay.h> 
#include <linux/module.h> 
#include <linux/of_gpio.h> 
#include <linux/iio/iio.h> 
#include <linux/iio/sysfs.h> 
#include <linux/iio/buffer.h> 
#include <linux/iio/trigger.h> 
#include <linux/iio/trigger_consumer.h> 
#include <linux/iio/triggered_buffer.h> 
#include <linux/iio/adc/ad_sigma_delta.h> 

//registers
#define ID_REG		0X00
#define CONFIG1_REG	0X01
#define CONFIG2_REG	0x01
#define LOFF		0x03
#define CH1SET		0x04
#define CH2SET		0x05
#define RLD_SENS	0x06
#define LOFF_SENS 	0x07
#define LOFF_STAT	0x08
#define RESP1		0X09
#define RESP2		0x0a
#define GPIO 		0x0b

//commands
#define WAKEUP		0x02
#define STANDBY	        0x04
#define RESET 		0x06
#define START		0x08
#define STOP		0X0A
#define OFFSETCAL	0x1A
#define RDATAC		0X10
#define SDATAC  	0X11
#define RDATA		0x12

#define ADS1292R_ID              1
#define ADS1292R_CH_1            1

volatile bool conversion_complete = false;

enum ads1292r_supported_device_ids {
	ID_ADS1292R
};


static int send_command(uint8_t command,struct ad_sigma_delta *sigma_delta)
{
	uint8_t data;
	struct spi_transfer t = {
		.tx_buf		= &data,
		.len		= 1,
		.cs_change	= sigma_delta->bus_locked,
	};
	struct spi_message m;
	int ret;
	
	data = command;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	if (sigma_delta->bus_locked)
		ret = spi_sync_locked(sigma_delta->spi, &m);
	else
		ret = spi_sync(sigma_delta->spi, &m);
printk(KERN_ALERT "command = %d  ret = %d ",command,ret);
		return ret;
}

static int send_data(uint8_t *data, struct ad_sigma_delta *sigma_delta)
{
	struct spi_transfer t = {
		.tx_buf		= data,
		.len		= 3,
		.cs_change	= sigma_delta->bus_locked,
	};
	struct spi_message m;
	int ret;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	conversion_complete = false;
	if (sigma_delta->bus_locked)
		ret = spi_sync_locked(sigma_delta->spi, &m);
	else
		ret = spi_sync(sigma_delta->spi, &m);

printk(KERN_ALERT "send data ret = %d ",ret);
	return ret;
}

static int receive_data(uint8_t * data, struct ad_sigma_delta *sigma_delta)
{
	struct spi_transfer r[] = {
		{
			.tx_buf = NULL,
	//		.len = 0,
		}, {
			.rx_buf = data,
			.len = 9,
			.cs_change = sigma_delta->bus_locked,
		},
	};
	struct spi_message m;
	int ret;
	spi_message_init(&m);
	spi_message_add_tail(&r[0], &m);
	spi_message_add_tail(&r[1], &m);
	if (sigma_delta->bus_locked)
		ret = spi_sync_locked(sigma_delta->spi, &m);
	else
		ret = spi_sync(sigma_delta->spi, &m);
printk(KERN_ALERT "receive data  ret = %d ",ret);
	return ret;
}



static struct ads1292r_state *ad_sigma_delta_to_ads1292r(struct ad_sigma_delta *sd)
{
	return container_of(sd, struct ads1292r_state, sd);
}

void ads1292r_event_handler(int irq, void *data)
{
	struct iio_dev *indio_dev = data;
	uint8_t value[9];
	struct ads1292r_state *st = iio_priv(indio_dev);
        struct ad_sigma_delta *sigma_delta = iio_device_get_drvdata(indio_dev);
	int result;
	int ret;
	printk(KERN_ALERT "**indio dev id %s ",indio_dev->name);
	printk(KERN_ALERT " ***in the irq");
	send_command(RDATA,sigma_delta);
	receive_data(&value,sigma_delta);
	printk(KERN_ALERT "the output continous value is %d %d %d %d %d %d %d %d %d ",value[0],value[1],value[2],value[3],value[4],value[5],value[6],value[7],value[8]);
	memcpy(&st->data_buffer,value,9);
}


static int ads1292r_set_mode(struct ad_sigma_delta *sd,
			   enum ad_sigma_delta_mode mode)
{
	struct ads1292r_state *st = ad_sigma_delta_to_ads1292r(sd);
	uint8_t data[3];
	switch(mode)
	{
		case AD_SD_MODE_CONTINUOUS:	send_command(WAKEUP,sd);
						data[0] = CONFIG1_REG | (1 << 6);
						data[1] = 0x00;
						data[2] &= ~(1<<7);
						send_data(data,sd);
						send_command(START,sd);
						break;
						
						
		case AD_SD_MODE_IDLE:
						send_command(STOP,sd);
						send_command(STANDBY,sd);
						break;

		case AD_SD_MODE_POWERDOWN:
						send_command(STANDBY,sd);
						break;
	}
}


static const struct ad_sigma_delta_info ads1292r_sigma_delta_info = {
	.set_channel = NULL,
	.set_mode = ads1292r_set_mode,
	.has_registers = true,
	.addr_shift = 0,
};


static int ads1292r_setup(struct iio_dev *indio_dev,
	unsigned int vref_mv)
{
	struct ads1292r_state *st = iio_priv(indio_dev);
	int ret =0;

	/* reset the serial interface */
	ret = spi_write(st->sd.spi, (u8 *)&ret, sizeof(ret));
	if (ret < 0)
		goto out;
	usleep_range(500, 2000); /* Wait for at least 500us */


	gpiod_set_value(st->ste_gpio, 1);
	udelay(800);
	gpiod_set_value(st->ste_gpio, 0);


	udelay(800);
	return 0;
out:
	dev_err(&st->sd.spi->dev, "setup failed\n");
	return ret;
}

static const u16 ads1292r_sample_freq_avail[16] = {8, 16, 32, 64, 128, 250, 475,
					860};

static ssize_t ads1292r_read_frequency(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1292r_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n",
	       st->chip_info->sample_freq_avail[st->freq_mode]);
}

static ssize_t ads1292r_write_frequency(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1292r_state *st = iio_priv(indio_dev);
	long lval;
	int i, ret;

	ret = kstrtol(buf, 10, &lval);
	if (ret)
		return ret;

	if (lval == 0)
		return -EINVAL;

	for (i = 0; i < 8; i++)
		if (lval == st->chip_info->sample_freq_avail[i])
			break;
	if (i == 8)
		return -EINVAL;

	//struct iio_dev *indio_dev1 = iio_device_get(indio_dev);
	//if (indio_dev1)
	//	return EBUSY;

	return 0;
}

static IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO,
		ads1292r_read_frequency,
		ads1292r_write_frequency);

static IIO_CONST_ATTR_SAMP_FREQ_AVAIL(
	"125 250 500 1000 2000 4000 8000 00");

static IIO_CONST_ATTR_NAMED(sampling_frequency_available_ads1292r,
	sampling_frequency_available, "125 250 500 1000 2000 4000 8000 00");


static struct attribute *ads1292r_attributes[] = {
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	NULL
};

static const struct attribute_group ads1292r_attribute_group = {
	.attrs = ads1292r_attributes,
};



static int ads1292r_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct ads1292r_state *st = iio_priv(indio_dev);
	struct ad_sigma_delta *sigma_delta;
	uint8_t value[9];
	uint8_t data[3];
	sigma_delta = &st->sd;
	int ret;
	unsigned long;
	gpiod_set_value(st->ste_gpio, 1);
	udelay(800);
	gpiod_set_value(st->ste_gpio, 0);

	udelay(800);
	send_command(WAKEUP,sigma_delta);
	
	udelay(800);
	data[0] = CONFIG1_REG | (1 << 6);
	data[1] = 0x00;
	data[2] = (1<<7);
	conversion_complete = false;
	send_data(data,sigma_delta);
	udelay(800);
	sigma_delta->irq_dis = false;
	enable_irq(sigma_delta->spi->irq);
	//gpiod_set_value(st->start_gpio, 1);
	send_command(START,sigma_delta);
	while(!conversion_complete);
	
	udelay(800);
	send_command(RDATA,sigma_delta);
	conversion_complete = false;
	udelay(800);
	receive_data(&value,sigma_delta);
//	gpiod_set_value(st->start_gpio, 0);
	printk(KERN_ALERT "the output value is \n  %x %x %x %x %x %x %x %x %x \n",value[0],value[1],value[2],value[3],value[4],value[5],value[6],value[7],value[8]);


		return IIO_VAL_INT;
}



static int ads1292r_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct ads1292r_state *st = iio_priv(indio_dev);
	int ret, i;
	unsigned int tmp;

	//struct iio_dev *indio_dev1 = iio_device_get(indio_dev);
	//if (indio_dev1)
	//	return ret;


	return ret;
}

static int ads1292r_write_raw_get_fmt(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       long mask)
{
	return IIO_VAL_INT_PLUS_NANO;
}

static const struct iio_info ads1292r_info = {
	.read_raw = &ads1292r_read_raw,
	.write_raw = &ads1292r_write_raw,
	.write_raw_get_fmt = &ads1292r_write_raw_get_fmt,
	.attrs = &ads1292r_attribute_group,
	.validate_trigger = ad_sd_validate_trigger,
	.driver_module = THIS_MODULE,
};

#define DECLARE_ADS1292R_CHANNELS(_name, _b, _sb, _s) \
const struct iio_chan_spec _name##_channels[] = { \
	AD_SD_DIFF_CHANNEL(0, 0, 0, ADS1292R_CH_1, (_b), (_sb), (_s)), \
	IIO_CHAN_SOFT_TIMESTAMP(6), \
}


static DECLARE_ADS1292R_CHANNELS(ads1292r, 24, 24, 0);


static const struct ads1292r_chip_info ads1292r_chip_info_tbl[] = {
	[ID_ADS1292R] = {
		.id = ADS1292R_ID,
		.channels = ads1292r_channels,
		.num_channels = ARRAY_SIZE(ads1292r_channels),
		.iio_info = &ads1292r_info,
		.sample_freq_avail = ads1292r_sample_freq_avail,
	}
};

static int ads1292r_probe(struct spi_device *spi)
{
	struct ads1292r_state *st;
	struct iio_dev *indio_dev;
	int ret, vref_mv = 0;
	printk(KERN_INFO " ********************");
	
	

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;
	
	 printk(KERN_INFO " probing started");
	st = iio_priv(indio_dev);

	st->ste_gpio = devm_gpiod_get(&spi->dev, "ste");
	if (IS_ERR(st->ste_gpio)) {
		ret = PTR_ERR(st->ste_gpio);
		if (ret != -ENOENT && ret != -ENOSYS) {
			dev_err(&spi->dev, "Failed to allocate spi gpio\n");
			return ret;
		}
		st->ste_gpio = NULL;
	} else {
		gpiod_direction_output(st->ste_gpio, 0);
	}

	st->start_gpio = devm_gpiod_get(&spi->dev, "start");
	if (IS_ERR(st->start_gpio)) {
		ret = PTR_ERR(st->start_gpio);
		if (ret != -ENOENT && ret != -ENOSYS) {
			dev_err(&spi->dev, "Failed to allocate spi gpio\n");
			return ret;
		}
		st->start_gpio = NULL;
	} else {
		gpiod_direction_output(st->start_gpio, 0);
	}



	st->reset_gpio = devm_gpiod_get(&spi->dev, "reset");
	if (IS_ERR(st->reset_gpio)) {
		ret = PTR_ERR(st->reset_gpio);
		if (ret != -ENOENT && ret != -ENOSYS) {
			dev_err(&spi->dev, "Failed to allocate reset gpio\n");
			return ret;
		}
		st->reset_gpio = NULL;
	} else {
		gpiod_direction_output(st->reset_gpio, 1);
	}


	ad_sd_init(&st->sd, indio_dev, spi, &ads1292r_sigma_delta_info);
		st->reg = devm_regulator_get(&spi->dev, "sw2_3v3");
		if (IS_ERR(st->reg))
			return PTR_ERR(st->reg);

		ret = regulator_enable(st->reg);
		if (ret)
			return ret;

		vref_mv = regulator_get_voltage(st->reg);
		if (vref_mv < 0) {
			ret = vref_mv;
			return ret;
		}

		vref_mv /= 1000;
	st->chip_info =
		&ads1292r_chip_info_tbl[0];

	spi_set_drvdata(spi, indio_dev);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->dev.of_node = spi->dev.of_node;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;
	indio_dev->info = st->chip_info->iio_info;

	ret = ad_sd_setup_buffer_and_trigger(indio_dev);
	
	ret = ads1292r_setup(indio_dev, vref_mv);
	if (ret)
		goto error_remove_trigger;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_remove_trigger;
	printk(KERN_ALERT "i am up");
	return 0;
error_remove_trigger:
	ad_sd_cleanup_buffer_and_trigger(indio_dev);

	return ret;
}

static int ads1292r_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ads1292r_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	ad_sd_cleanup_buffer_and_trigger(indio_dev);
	printk(KERN_INFO "i am down");
	regulator_disable(st->reg);
	return 0;
}

static const struct spi_device_id ads1292r_id[] = {
	{"ads1292r", ID_ADS1292R},
	{}
};
MODULE_DEVICE_TABLE(spi, ads1292r_id);

static struct spi_driver ads1292r_driver = {
	.driver = {
		.name	= "ti,ads1292r",
	},
	.probe		= ads1292r_probe,
	.remove		= ads1292r_remove,
	.id_table	= ads1292r_id,
};
module_spi_driver(ads1292r_driver);

MODULE_AUTHOR("Unnikrishnan <unnikrishnan.s_i@gadgeon.com>");
MODULE_DESCRIPTION("Gadgeon ADS1292R DEVICE DRIVER");
MODULE_LICENSE("GPL");


