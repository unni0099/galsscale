
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
#define ADC_DATA_REESULT       0X00 
#define ADC_CONTROL_REGISTER1  0X01 
#define MISC_REGISTER1         0X02 
#define MISC_REGISTER2         0X03 
#define DEVICE_CONTROL1        0X09
#define DEVICE_CONTROL2        0X0F
#define ISW_MUX                0X0A
#define WEIGHT_SCALE_CONTROL   0X0D
#define ADC_CONTROL_REGISTER2  0X10
#define MISC_REGISTER3         0X1A

#define ADC_CONV_MODE_SINGLE_SHOT          (1<<15)
#define ADC_MEAS_MODE_SINGLE_ENDED         (1<<11) 
#define ADC_PDN                            (1<<7)

//Device Control 1 register setup         
#define WIGHT_SCALE_CONF_CONTROL1           0X6005
#define DEVICE_CONTROL1_REG_SLEEP           0X600C
#define DEVICE_CONTROL1_REG_OFF             0X6000

#define DEVICE_GAIN(X)                      (X<<13)
#define BRIDGE_SELECT(X)                    (X<<1)

#define DEVICE_CONTROL1        0X09
#define AFE4300_CONF_CHAN_MASK	0x06
#define AFE4300_CONF_CHAN(x)	((x) & 0xf) /* Channel select */
#define AFE4300_ID              1
#define AFE4300_CH_1            1
#define AFE4300_CH_2            2
#define AFE4300_CH_3            3
#define AFE4300_CH_4            4

bool single_convertion_complete = false;

struct afe4300_chip_info {
	unsigned int id;
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
	unsigned int flags;

	const struct iio_info *iio_info;
	const u16 *sample_freq_avail;
};

struct afe4300_state {
	const struct afe4300_chip_info	*chip_info;
	struct regulator		*reg;
	u16				int_vref_mv;
	u16				mode;
	u16				conf;
	u32				scale_avail[8][2];
	u16				freq_mode;
	u16                             gain;
	struct gpio_desc 		*ste_gpio;
	struct gpio_desc 		*reset_gpio;	
	struct gpio_desc 		*pwr_en;	
	struct gpio_desc 		*data_gpio;	
	struct ad_sigma_delta		sd;
	int                             irq;

};

enum afe4300_supported_device_ids {
	ID_AFE4300
};

static struct afe4300_state *ad_sigma_delta_to_afe4300(struct ad_sigma_delta *sd)
{
	return container_of(sd, struct afe4300_state, sd);
}

static int afe4300_set_channel(struct ad_sigma_delta *sd, unsigned int channel)
{
	struct afe4300_state *st = ad_sigma_delta_to_afe4300(sd);
	st->conf = 0x0000;
	st->conf &= ~AFE4300_CONF_CHAN_MASK;
	st->conf |= BRIDGE_SELECT((channel - 1));

	gpiod_set_value(st->ste_gpio, 1);
	udelay(800);
	gpiod_set_value(st->ste_gpio, 0);
	
	printk(KERN_ALERT "setting the channel %d \n",channel);
	return ad_sd_write_reg(&st->sd, DEVICE_CONTROL2, 2, st->conf);
}

static int afe4300_set_mode(struct ad_sigma_delta *sd,
			   enum ad_sigma_delta_mode mode)
{
	struct afe4300_state *st = ad_sigma_delta_to_afe4300(sd);
	ad_sd_write_reg(&st->sd, ADC_CONTROL_REGISTER2, 2, 0x0000);

	gpiod_set_value(st->ste_gpio, 1);
	udelay(800);
	gpiod_set_value(st->ste_gpio, 0);
	printk(KERN_ALERT "setting the mode %d \n",mode);
	switch(mode)
	{
		case AD_SD_MODE_CONTINUOUS:
						st->mode = 0x6005;	
						ad_sd_write_reg(&st->sd, DEVICE_CONTROL1, 2, st->mode);	
						st->mode = 0x4170;
						st->mode &= ~ADC_PDN;
						return ad_sd_write_reg(&st->sd, ADC_CONTROL_REGISTER1, 2, st->mode);
						
		case AD_SD_MODE_SINGLE:
						st->mode = 0x6005;	
						ad_sd_write_reg(&st->sd, DEVICE_CONTROL1, 2, st->mode);	
						st->mode = 0x4970;	
						printk(KERN_ALERT "doing the single mode convertion %d \n",mode);
						st->mode |= ADC_PDN | ADC_CONV_MODE_SINGLE_SHOT;
						return ad_sd_write_reg(&st->sd, ADC_CONTROL_REGISTER1, 2, st->mode);
						
						
		case AD_SD_MODE_IDLE:
						st->mode = 0x600c;	
						ad_sd_write_reg(&st->sd, DEVICE_CONTROL1, 2, st->mode);	
						st->mode = 0x4120;
						st->mode |= ADC_PDN;
						return ad_sd_write_reg(&st->sd, ADC_CONTROL_REGISTER1, 2, st->mode);

		case AD_SD_MODE_POWERDOWN:
						st->mode = 0x6000;	
						ad_sd_write_reg(&st->sd, DEVICE_CONTROL1, 2, st->mode);	
						st->mode = 0x4120;
						st->mode |= ADC_PDN;
						return ad_sd_write_reg(&st->sd, ADC_CONTROL_REGISTER1, 2, st->mode);
	}
}



static const struct ad_sigma_delta_info afe4300_sigma_delta_info = {
	.set_channel = afe4300_set_channel,
	.set_mode = afe4300_set_mode,
	.has_registers = true,
	.addr_shift = 0,
};


static int afe4300_setup(struct iio_dev *indio_dev,
	unsigned int vref_mv)
{
	struct afe4300_state *st = iio_priv(indio_dev);
	int ret =0;

	/* reset the serial interface */
	ret = spi_write(st->sd.spi, (u8 *)&ret, sizeof(ret));
	if (ret < 0)
		goto out;
	usleep_range(500, 2000); /* Wait for at least 500us */


	gpiod_set_value(st->reset_gpio, 0);
	udelay(800);
	gpiod_set_value(st->reset_gpio, 1);

	gpiod_set_value(st->ste_gpio, 1);
	udelay(800);
	gpiod_set_value(st->ste_gpio, 0);


	udelay(800);
	ad_sd_write_reg(&st->sd,ADC_CONTROL_REGISTER1, 2, 0x51c0);
	udelay(800);
	ad_sd_write_reg(&st->sd, MISC_REGISTER1, 2, 0x0000);
	udelay(800);
	ad_sd_write_reg(&st->sd, MISC_REGISTER2, 2, 0xffff);
	udelay(800);
        ad_sd_write_reg(&st->sd, DEVICE_CONTROL1, 2, 0x0004);	
	udelay(800);
	ad_sd_write_reg(&st->sd,WEIGHT_SCALE_CONTROL, 2, 0x0000);
	udelay(800);
	ad_sd_write_reg(&st->sd,DEVICE_CONTROL2, 2, 0x0000);
	udelay(800);
	ad_sd_write_reg(&st->sd, ADC_CONTROL_REGISTER2, 2, 0x0011);
	udelay(800);
	ad_sd_write_reg(&st->sd, MISC_REGISTER3, 2, 0x00C0);
	udelay(800);
	st->mode = 0x5140;
	st->conf = 0;


        printk(KERN_ALERT "**************device setup finished*****************\n");

	ret = afe4300_set_mode(&st->sd, AD_SD_MODE_IDLE);
	if (ret)
		goto out;

	ret = afe4300_set_channel(&st->sd, 0);
	if (ret)
		goto out;

	return 0;
out:
	dev_err(&st->sd.spi->dev, "setup failed\n");
	return ret;
}

static const u16 afe4300_sample_freq_avail[16] = {8, 16, 32, 64, 128, 250, 475,
					860};

static ssize_t afe4300_read_frequency(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct afe4300_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n",
	       st->chip_info->sample_freq_avail[st->freq_mode]);
}

static ssize_t afe4300_write_frequency(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct afe4300_state *st = iio_priv(indio_dev);
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
	st->freq_mode = i;
	st->mode &= ~(7<<4);
	st->mode |= i<<4;
	ad_sd_write_reg(&st->sd, ADC_CONTROL_REGISTER1, sizeof(st->mode), st->mode);

	return len;
}

static IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO,
		afe4300_read_frequency,
		afe4300_write_frequency);

static IIO_CONST_ATTR_SAMP_FREQ_AVAIL(
	"8 16 32 64 128 250 475 860");

static IIO_CONST_ATTR_NAMED(sampling_frequency_available_afe4300,
	sampling_frequency_available, "8 16 32 64 128 250 475 860");


static struct attribute *afe4300_attributes[] = {
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	NULL
};

static const struct attribute_group afe4300_attribute_group = {
	.attrs = afe4300_attributes,
};



static int afe4300_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct afe4300_state *st = iio_priv(indio_dev);
	int ret;
	unsigned long;
	gpiod_set_value(st->ste_gpio, 1);
	udelay(800);
	gpiod_set_value(st->ste_gpio, 0);

	udelay(800);
		ret = ad_sigma_delta_single_conversion(indio_dev, chan, val);
		if (ret < 0)
			return ret;
		return IIO_VAL_INT;
}

static int afe4300_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct afe4300_state *st = iio_priv(indio_dev);
	int ret, i;
	unsigned int tmp;

	//struct iio_dev *indio_dev1 = iio_device_get(indio_dev);
	//if (indio_dev1)
	//	return ret;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
				ret = 0;
				tmp = st->gain;
				st->gain &= val2;
				if (tmp == st->gain)
					break;

				ad_sd_write_reg(&st->sd, WEIGHT_SCALE_CONTROL,
						sizeof(st->gain), (st->gain<<13));
				break;
		
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int afe4300_write_raw_get_fmt(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       long mask)
{
	return IIO_VAL_INT_PLUS_NANO;
}


static const struct iio_info afe4300_info = {
	.read_raw = &afe4300_read_raw,
	.write_raw = &afe4300_write_raw,
	.write_raw_get_fmt = &afe4300_write_raw_get_fmt,
	.attrs = &afe4300_attribute_group,
	//.validate_trigger = ad_sd_validate_trigger,
	.driver_module = THIS_MODULE,
};

#define DECLARE_AFE4300_CHANNELS(_name, _b, _sb, _s) \
const struct iio_chan_spec _name##_channels[] = { \
	AD_SD_DIFF_CHANNEL(0, 0, 0, AFE4300_CH_1, (_b), (_sb), (_s)), \
	AD_SD_DIFF_CHANNEL(1, 1, 1, AFE4300_CH_2, (_b), (_sb), (_s)), \
	AD_SD_DIFF_CHANNEL(2, 2, 2, AFE4300_CH_3, (_b), (_sb), (_s)), \
	AD_SD_DIFF_CHANNEL(3, 3, 3, AFE4300_CH_4, (_b), (_sb), (_s)), \
	IIO_CHAN_SOFT_TIMESTAMP(6), \
}


static DECLARE_AFE4300_CHANNELS(afe4300, 16, 16, 0);


static const struct afe4300_chip_info afe4300_chip_info_tbl[] = {
	[ID_AFE4300] = {
		.id = AFE4300_ID,
		.channels = afe4300_channels,
		.num_channels = ARRAY_SIZE(afe4300_channels),
		.iio_info = &afe4300_info,
		.sample_freq_avail = afe4300_sample_freq_avail,
	}
};

static int afe4300_probe(struct spi_device *spi)
{
	struct afe4300_state *st;
	struct iio_dev *indio_dev;
	int ret, vref_mv = 0;
	if (!spi->irq) {
		dev_err(&spi->dev, "no IRQ?\n");
		return -ENODEV;
	}

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;
	
	 printk(KERN_INFO " probing of afe4300 started");
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


	st->reset_gpio = devm_gpiod_get(&spi->dev, "reset");
	if (IS_ERR(st->reset_gpio)) {
		ret = PTR_ERR(st->reset_gpio);
		if (ret != -ENOENT && ret != -ENOSYS) {
			dev_err(&spi->dev, "Failed to allocate reset gpio\n");
			return ret;
		}
		st->reset_gpio = NULL;
	} else {
		gpiod_direction_output(st->reset_gpio, 0);
	}


	ad_sd_init(&st->sd, indio_dev, spi, &afe4300_sigma_delta_info);
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
		&afe4300_chip_info_tbl[0];

	spi_set_drvdata(spi, indio_dev);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->dev.of_node = spi->dev.of_node;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;
	indio_dev->info = st->chip_info->iio_info;

	ret = ad_sd_setup_buffer_and_trigger(indio_dev);
	
	ret = afe4300_setup(indio_dev, vref_mv);
	if (ret)
		goto error_remove_trigger;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_remove_trigger;
	printk(KERN_ALERT "afe4300 is up");
	return 0;
error_remove_trigger:
	ad_sd_cleanup_buffer_and_trigger(indio_dev);

	return ret;
}

static int afe4300_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct afe4300_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	ad_sd_cleanup_buffer_and_trigger(indio_dev);
	printk(KERN_INFO "i am down");
	regulator_disable(st->reg);
	return 0;
}

static const struct spi_device_id afe4300_id[] = {
	{"afe4300", ID_AFE4300},
	{}
};
MODULE_DEVICE_TABLE(spi, afe4300_id);

static struct spi_driver afe4300_driver = {
	.driver = {
		.name	= "ti,afe4300",
	},
	.probe		= afe4300_probe,
	.remove		= afe4300_remove,
	.id_table	= afe4300_id,
};
module_spi_driver(afe4300_driver);

MODULE_AUTHOR("Unnikrishnan <unnikrishnan.s_i@gadgeon.com>");
MODULE_DESCRIPTION("Gadgeon AFE4300 DEVICE DIVER");
MODULE_LICENSE("GPL");


