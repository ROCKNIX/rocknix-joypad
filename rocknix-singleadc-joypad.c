/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ROCKNIX singleadc joypad driver
 *
 * Copyright (C) 2024 ROCKNIX (https://github.com/ROCKNIX)
 */

/*----------------------------------------------------------------------------*/
#include <linux/input-polldev.h>
#include <linux/platform_device.h>
#include <linux/iio/consumer.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0))
#include <linux/of_gpio.h>
#else
#include <linux/of_gpio_legacy.h>
#endif
#include <linux/delay.h>

/*----------------------------------------------------------------------------*/
#define DRV_NAME "rocknix-singleadc-joypad"
#define __LEFT_JOYSTICK_INVERT__
/*----------------------------------------------------------------------------*/
#define	ADC_MAX_VOLTAGE		1800
#define	ADC_DATA_TUNING(x, p)	((x * p) / 100)
#define	ADC_TUNING_DEFAULT	180

struct bt_adc {
	/* report value (mV) */
	int value;
	/* report type */
	int report_type;
	/* input device init value (mV) */
	int max, min;
	/* calibrated adc value */
	int cal;
	/*  adc scale value */
	int scale;
	/* invert report */
	bool invert;
	/* amux channel */
	int amux_ch;
	/* adc data tuning value([percent), p = positive, n = negative */
	int tuning_p, tuning_n;
};

struct analog_mux {
	/* IIO ADC Channel : amux connect channel */
	struct iio_channel *iio_ch;
	/* analog mux select(a,b) gpio */
	int sel_a_gpio, sel_b_gpio;
	/* analog mux enable gpio */
	int en_gpio;
};

struct bt_gpio {
	/* GPIO Request label */
	const char *label;
	/* GPIO Number */
	int num;
	/* report type */
	int report_type;
	/* report linux code */
	int linux_code;
	/* prev button value */
	bool old_value;
	/* button press level */
	bool active_level;
};

struct joypad {
	struct device *dev;
	struct input_polled_dev	*poll_dev;
	int poll_interval;

	/* report enable/disable */
	bool enable;

	/* analog mux & joystick control */
	struct analog_mux *amux;
	/* analog mux max count */
	int amux_count;
	/* analog button */
	struct bt_adc *adcs;

	/* report interval (ms) */
	int bt_gpio_count;
	struct bt_gpio *gpios;

	/* button auto repeat */
	int auto_repeat;

	/* report threshold (mV) */
	int bt_adc_fuzz, bt_adc_flat;
	/* adc read value scale */
	int bt_adc_scale;
	/* joystick deadzone control */
	int bt_adc_deadzone;

	struct mutex lock;
};

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int joypad_amux_select(struct analog_mux *amux, int channel)
{
	/* select mux channel */
	gpio_set_value(amux->en_gpio, 0);

	switch(channel) {
		case 0:	/* EVENT (ABS_RY) */
			gpio_set_value(amux->sel_a_gpio, 0);
			gpio_set_value(amux->sel_b_gpio, 0);
			break;
		case 1:	/* EVENT (ABS_RX) */
			gpio_set_value(amux->sel_a_gpio, 0);
			gpio_set_value(amux->sel_b_gpio, 1);
			break;
		case 2:	/* EVENT (ABS_Y) */
			gpio_set_value(amux->sel_a_gpio, 1);
			gpio_set_value(amux->sel_b_gpio, 0);
			break;
		case 3:	/* EVENT (ABS_X) */
			gpio_set_value(amux->sel_a_gpio, 1);
			gpio_set_value(amux->sel_b_gpio, 1);
			break;
		default:
			/* amux disanle */
			gpio_set_value(amux->en_gpio, 1);
			return -1;
	}
	/* mux swtiching speed : 35ns(on) / 9ns(off) */
	usleep_range(10, 20);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int joypad_adc_read(struct analog_mux *amux, struct bt_adc *adc)
{
	int value;


	if (joypad_amux_select(amux, adc->amux_ch))
		return 0;

	iio_read_channel_raw(amux->iio_ch, &value);

	value *= adc->scale;
#ifdef __LEFT_JOYSTICK_INVERT__
	return value;
#else
	return (adc->invert ? (adc->max - value) : value);
#endif
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/rocknix-singleadc-joypad/poll_interval [rw]
 */
/*----------------------------------------------------------------------------*/
static ssize_t joypad_store_poll_interval(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	mutex_lock(&joypad->lock);
	joypad->poll_interval = simple_strtoul(buf, NULL, 10);
	mutex_unlock(&joypad->lock);

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t joypad_show_poll_interval(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", joypad->poll_interval);
}

/*----------------------------------------------------------------------------*/
static DEVICE_ATTR(poll_interval, S_IWUSR | S_IRUGO,
		   joypad_show_poll_interval,
		   joypad_store_poll_interval);

/*----------------------------------------------------------------------------*/
/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/rocknix-singleadc-joypad/adc_fuzz [rw]
 */
/*----------------------------------------------------------------------------*/
static ssize_t joypad_store_adc_fuzz(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	mutex_lock(&joypad->lock);
	joypad->bt_adc_fuzz = simple_strtoul(buf, NULL, 10);
	mutex_unlock(&joypad->lock);

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t joypad_show_adc_fuzz(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", joypad->bt_adc_fuzz);
}

/*----------------------------------------------------------------------------*/
static DEVICE_ATTR(adc_fuzz, S_IWUSR | S_IRUGO,
		   joypad_show_adc_fuzz,
		   joypad_store_adc_fuzz);

/*----------------------------------------------------------------------------*/
/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/rocknix-singleadc-joypad/adc_flat [rw]
 */
/*----------------------------------------------------------------------------*/
static ssize_t joypad_store_adc_flat(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	mutex_lock(&joypad->lock);
	joypad->bt_adc_flat = simple_strtoul(buf, NULL, 10);
	mutex_unlock(&joypad->lock);

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t joypad_show_adc_flat(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", joypad->bt_adc_flat);
}

/*----------------------------------------------------------------------------*/
static DEVICE_ATTR(adc_flat, S_IWUSR | S_IRUGO,
		   joypad_show_adc_flat,
		   joypad_store_adc_flat);

/*----------------------------------------------------------------------------*/
/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/rocknix-singleadc-joypad/adc_scale [rw]
 */
/*----------------------------------------------------------------------------*/
static ssize_t joypad_store_adc_scale(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	mutex_lock(&joypad->lock);
	joypad->bt_adc_scale = simple_strtoul(buf, NULL, 10);
	mutex_unlock(&joypad->lock);

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t joypad_show_adc_scale(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", joypad->bt_adc_scale);
}

/*----------------------------------------------------------------------------*/
static DEVICE_ATTR(adc_scale, S_IWUSR | S_IRUGO,
		   joypad_show_adc_scale,
		   joypad_store_adc_scale);

/*----------------------------------------------------------------------------*/
/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/rocknix-singleadc-joypad/adc_deadzone [rw]
 */
/*----------------------------------------------------------------------------*/
static ssize_t joypad_store_adc_deadzone(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	mutex_lock(&joypad->lock);
	joypad->bt_adc_deadzone = simple_strtoul(buf, NULL, 10);
	mutex_unlock(&joypad->lock);

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t joypad_show_adc_deadzone(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", joypad->bt_adc_deadzone);
}

/*----------------------------------------------------------------------------*/
static DEVICE_ATTR(adc_deadzone, S_IWUSR | S_IRUGO,
		   joypad_show_adc_deadzone,
		   joypad_store_adc_deadzone);

/*----------------------------------------------------------------------------*/
/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/rocknix-singleadc-joypad/enable [rw]
 */
/*----------------------------------------------------------------------------*/
static ssize_t joypad_store_enable(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	mutex_lock(&joypad->lock);
	joypad->enable = simple_strtoul(buf, NULL, 10);
	mutex_unlock(&joypad->lock);

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t joypad_show_enable(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", joypad->enable);
}

/*----------------------------------------------------------------------------*/
static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO,
		   joypad_show_enable,
		   joypad_store_enable);

/*----------------------------------------------------------------------------*/
/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/rocknix-singleadc-joypad/adc_cal [rw]
 */
/*----------------------------------------------------------------------------*/
static ssize_t joypad_store_adc_cal(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);
	bool calibration;

	calibration = simple_strtoul(buf, NULL, 10);

	if (calibration) {
		int nbtn;

		mutex_lock(&joypad->lock);
		for (nbtn = 0; nbtn < joypad->amux_count; nbtn++) {
			struct bt_adc *adc = &joypad->adcs[nbtn];

			adc->value = joypad_adc_read(joypad->amux, adc);
			if (!adc->value) {
				dev_err(joypad->dev, "%s : saradc channels[%d]!\n",
					__func__, nbtn);
				continue;
			}
			adc->cal = adc->value;
		}
		mutex_unlock(&joypad->lock);
	}
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t joypad_show_adc_cal(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);
	int nbtn;
	ssize_t pos;

	for (nbtn = 0, pos = 0; nbtn < joypad->amux_count; nbtn++) {
		struct bt_adc *adc = &joypad->adcs[nbtn];
		pos += sprintf(&buf[pos], "adc[%d]->cal = %d\n",
				nbtn, adc->cal);
	}
	pos += sprintf(&buf[pos], "adc scale = %d\n", joypad->bt_adc_scale);
	return pos;
}

/*----------------------------------------------------------------------------*/
static DEVICE_ATTR(adc_cal, S_IWUSR | S_IRUGO,
		   joypad_show_adc_cal,
		   joypad_store_adc_cal);

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static struct attribute *joypad_attrs[] = {
	&dev_attr_poll_interval.attr,
	&dev_attr_adc_fuzz.attr,
	&dev_attr_adc_flat.attr,
	&dev_attr_adc_scale.attr,
	&dev_attr_adc_deadzone.attr,
	&dev_attr_enable.attr,
	&dev_attr_adc_cal.attr,
	NULL,
};

static struct attribute_group joypad_attr_group = {
	.attrs = joypad_attrs,
};

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static void joypad_gpio_check(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;
	int nbtn, value;

	for (nbtn = 0; nbtn < joypad->bt_gpio_count; nbtn++) {
		struct bt_gpio *gpio = &joypad->gpios[nbtn];

		if (gpio_get_value_cansleep(gpio->num) < 0) {
			dev_err(joypad->dev, "failed to get gpio state\n");
			continue;
		}
		value = gpio_get_value(gpio->num);
		if (value != gpio->old_value) {
			input_event(poll_dev->input,
				gpio->report_type,
				gpio->linux_code,
				(value == gpio->active_level) ? 1 : 0);
			gpio->old_value = value;
		}
	}
	input_sync(poll_dev->input);
}

/*----------------------------------------------------------------------------*/
static void joypad_adc_check(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;
	int nbtn;
	int mag;

	/* Assumes an even number of axes and that joystick axis pairs are sequential */
	/* e.g. left stick Y immediately follows left stick X */
	for (nbtn = 0; nbtn < joypad->amux_count; nbtn += 2) {
		struct bt_adc *adcx = &joypad->adcs[nbtn];
		struct bt_adc *adcy = &joypad->adcs[nbtn + 1];

		/* Read first joystick axis */
		adcx->value = joypad_adc_read(joypad->amux, adcx);
		if (!adcx->value) {
			//dev_err(joypad->dev, "%s : saradc channels[%d]! adc->value : %d\n",__func__, nbtn, adc->value);
			continue;
		}
		adcx->value = adcx->value - adcx->cal;

		/* Read second joystick axis */
		adcy->value = joypad_adc_read(joypad->amux, adcy);
		if (!adcy->value) {
			//dev_err(joypad->dev, "%s : saradc channels[%d]! adc->value : %d\n",__func__, nbtn, adc->value);
			continue;
		}
		adcy->value = adcy->value - adcy->cal;

		/* Scaled Radial Deadzone */
		/* https://web.archive.org/web/20190129113357/http://www.third-helix.com/2013/04/12/doing-thumbstick-dead-zones-right.html */
		mag = int_sqrt((adcx->value * adcx->value) + (adcy->value * adcy->value));
		if (joypad->bt_adc_deadzone) {
			if (mag <= joypad->bt_adc_deadzone) {
				adcx->value = 0;
				adcy->value = 0;
			}
			else {
				/* Assumes adcx->max == -adcx->min == adcy->max == -adcy->min */
				/* Order of operations is critical to avoid integer overflow */
				adcx->value = (((adcx->max * adcx->value) / mag) * (mag - joypad->bt_adc_deadzone)) / (adcx->max - joypad->bt_adc_deadzone);
				adcy->value = (((adcy->max * adcy->value) / mag) * (mag - joypad->bt_adc_deadzone)) / (adcy->max - joypad->bt_adc_deadzone);
			}
		}

		/* adc data tuning */
		if (adcx->tuning_n && adcx->value < 0)
			adcx->value = ADC_DATA_TUNING(adcx->value, adcx->tuning_n);
		if (adcx->tuning_p && adcx->value > 0)
			adcx->value = ADC_DATA_TUNING(adcx->value, adcx->tuning_p);
		if (adcy->tuning_n && adcy->value < 0)
			adcy->value = ADC_DATA_TUNING(adcy->value, adcy->tuning_n);
		if (adcy->tuning_p && adcy->value > 0)
			adcy->value = ADC_DATA_TUNING(adcy->value, adcy->tuning_p);

		/* Clamp to [min, max] */
		adcx->value = adcx->value > adcx->max ? adcx->max : adcx->value;
		adcx->value = adcx->value < adcx->min ? adcx->min : adcx->value;
		adcy->value = adcy->value > adcy->max ? adcy->max : adcy->value;
		adcy->value = adcy->value < adcy->min ? adcy->min : adcy->value;

		input_report_abs(poll_dev->input,
			adcx->report_type,
			adcx->invert ? adcx->value * (-1) : adcx->value);
		input_report_abs(poll_dev->input,
			adcy->report_type,
			adcy->invert ? adcy->value * (-1) : adcy->value);
	}
	input_sync(poll_dev->input);
}

/*----------------------------------------------------------------------------*/
static void joypad_poll(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;

	if (joypad->enable) {
		joypad_adc_check(poll_dev);
		joypad_gpio_check(poll_dev);
	}
	if (poll_dev->poll_interval != joypad->poll_interval) {
		mutex_lock(&joypad->lock);
		poll_dev->poll_interval = joypad->poll_interval;
		mutex_unlock(&joypad->lock);
	}
}

/*----------------------------------------------------------------------------*/
static void joypad_open(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;
	int nbtn;

	for (nbtn = 0; nbtn < joypad->bt_gpio_count; nbtn++) {
		struct bt_gpio *gpio = &joypad->gpios[nbtn];
		gpio->old_value = gpio->active_level ? 0 : 1;
	}
	for (nbtn = 0; nbtn < joypad->amux_count; nbtn++) {
		struct bt_adc *adc = &joypad->adcs[nbtn];

		adc->value = joypad_adc_read(joypad->amux, adc);
		if (!adc->value) {
			dev_err(joypad->dev, "%s : saradc channels[%d]!\n",
				__func__, nbtn);
			continue;
		}
		adc->cal = adc->value;
		dev_info(joypad->dev, "%s : adc[%d] adc->cal = %d\n",
			__func__, nbtn, adc->cal);
	}
	/* buttons status sync */
	joypad_adc_check(poll_dev);
	joypad_gpio_check(poll_dev);

	/* button report enable */
	mutex_lock(&joypad->lock);
	joypad->enable = true;
	mutex_unlock(&joypad->lock);

	dev_info(joypad->dev, "%s : opened\n", __func__);
}

/*----------------------------------------------------------------------------*/
static void joypad_close(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;

	/* button report disable */
	mutex_lock(&joypad->lock);
	joypad->enable = false;
	mutex_unlock(&joypad->lock);

	dev_info(joypad->dev, "%s : closed\n", __func__);
}

/*----------------------------------------------------------------------------*/
static int joypad_amux_setup(struct device *dev, struct joypad *joypad)
{
	struct analog_mux *amux;
	enum iio_chan_type type;
	enum of_gpio_flags flags;
	int ret;

	/* analog mux control struct init */
	joypad->amux = devm_kzalloc(dev, sizeof(struct analog_mux),
					GFP_KERNEL);
	if (!joypad->amux) {
		dev_err(dev, "%s amux devm_kzmalloc error!", __func__);
		return -ENOMEM;
	}
	amux = joypad->amux;
	amux->iio_ch = devm_iio_channel_get(dev, "amux_adc");
	if (PTR_ERR(amux->iio_ch) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (IS_ERR(amux->iio_ch)) {
		dev_err(dev, "iio channel get error\n");
		return -EINVAL;
	}
	if (!amux->iio_ch->indio_dev)
		return -ENXIO;

	if (iio_get_channel_type(amux->iio_ch, &type))
		return -EINVAL;

	if (type != IIO_VOLTAGE) {
		dev_err(dev, "Incompatible channel type %d\n", type);
		return -EINVAL;
	}

	amux->sel_a_gpio = of_get_named_gpio_flags(dev->of_node,
				"amux-a-gpios", 0, &flags);
	if (gpio_is_valid(amux->sel_a_gpio)) {
		ret = devm_gpio_request(dev, amux->sel_a_gpio, "amux-sel-a");
		if (ret < 0) {
			dev_err(dev, "%s : failed to request amux-sel-a %d\n",
				__func__, amux->sel_a_gpio);
			goto err_out;
		}
		ret = gpio_direction_output(amux->sel_a_gpio, 0);
		if (ret < 0)
			goto err_out;
	}

	amux->sel_b_gpio = of_get_named_gpio_flags(dev->of_node,
				"amux-b-gpios", 0, &flags);
	if (gpio_is_valid(amux->sel_b_gpio)) {
		ret = devm_gpio_request(dev, amux->sel_b_gpio, "amux-sel-b");
		if (ret < 0) {
			dev_err(dev, "%s : failed to request amux-sel-b %d\n",
				__func__, amux->sel_b_gpio);
			goto err_out;
		}
		ret = gpio_direction_output(amux->sel_b_gpio, 0);
		if (ret < 0)
			goto err_out;
	}

	amux->en_gpio = of_get_named_gpio_flags(dev->of_node,
				"amux-en-gpios", 0, &flags);
	if (gpio_is_valid(amux->en_gpio)) {
		ret = devm_gpio_request(dev, amux->en_gpio, "amux-en");
		if (ret < 0) {
			dev_err(dev, "%s : failed to request amux-en %d\n",
				__func__, amux->en_gpio);
			goto err_out;
		}
		ret = gpio_direction_output(amux->en_gpio, 0);
		if (ret < 0)
			goto err_out;
	}
	return	0;
err_out:
	return ret;
}

/*----------------------------------------------------------------------------*/
static int joypad_adc_setup(struct device *dev, struct joypad *joypad)
{
	int nbtn;

	/* adc button struct init */
	joypad->adcs = devm_kzalloc(dev, joypad->amux_count *
				sizeof(struct bt_adc), GFP_KERNEL);
	if (!joypad->adcs) {
		dev_err(dev, "%s devm_kzmalloc error!", __func__);
		return -ENOMEM;
	}

	for (nbtn = 0; nbtn < joypad->amux_count; nbtn++) {
		struct bt_adc *adc = &joypad->adcs[nbtn];

		adc->scale = joypad->bt_adc_scale;

		adc->max = (ADC_MAX_VOLTAGE / 2);
		adc->min = (ADC_MAX_VOLTAGE / 2) * (-1);
		if (adc->scale) {
			adc->max *= adc->scale;
			adc->min *= adc->scale;
		}
		adc->amux_ch = nbtn;
		adc->invert = false;

		switch (nbtn) {
			case 0:
				adc->report_type = ABS_RY;
				if (device_property_read_u32(dev,
					"abs_ry-p-tuning",
					&adc->tuning_p))
					adc->tuning_p = ADC_TUNING_DEFAULT;
				if (device_property_read_u32(dev,
					"abs_ry-n-tuning",
					&adc->tuning_n))
					adc->tuning_n = ADC_TUNING_DEFAULT;
				break;
			case 1:
				adc->report_type = ABS_RX;
				if (device_property_read_u32(dev,
					"abs_rx-p-tuning",
					&adc->tuning_p))
					adc->tuning_p = ADC_TUNING_DEFAULT;
				if (device_property_read_u32(dev,
					"abs_rx-n-tuning",
					&adc->tuning_n))
					adc->tuning_n = ADC_TUNING_DEFAULT;
				break;
			case 2:
			#ifdef __LEFT_JOYSTICK_INVERT__
				adc->invert = true;
			#endif
				adc->report_type = ABS_Y;
				if (device_property_read_u32(dev,
					"abs_y-p-tuning",
					&adc->tuning_p))
					adc->tuning_p = ADC_TUNING_DEFAULT;
				if (device_property_read_u32(dev,
					"abs_y-n-tuning",
					&adc->tuning_n))
					adc->tuning_n = ADC_TUNING_DEFAULT;
				break;
			case 3:
			#ifdef __LEFT_JOYSTICK_INVERT__
				adc->invert = true;
			#endif
				adc->report_type = ABS_X;
				if (device_property_read_u32(dev,
					"abs_x-p-tuning",
					&adc->tuning_p))
					adc->tuning_p = ADC_TUNING_DEFAULT;
				if (device_property_read_u32(dev,
					"abs_x-n-tuning",
					&adc->tuning_n))
					adc->tuning_n = ADC_TUNING_DEFAULT;
				break;
			default :
				dev_err(dev, "%s amux count(%d) error!",
					__func__, nbtn);
				return -EINVAL;
		}
	}
	return	0;
}

/*----------------------------------------------------------------------------*/
static int joypad_gpio_setup(struct device *dev, struct joypad *joypad)
{
	struct device_node *node, *pp;
	int nbtn;

	node = dev->of_node;
	if (!node)
		return -ENODEV;

	joypad->gpios = devm_kzalloc(dev, joypad->bt_gpio_count *
				sizeof(struct bt_gpio), GFP_KERNEL);

	if (!joypad->gpios) {
		dev_err(dev, "%s devm_kzmalloc error!", __func__);
		return -ENOMEM;
	}

	nbtn = 0;
	for_each_child_of_node(node, pp) {
		enum of_gpio_flags flags;
		struct bt_gpio *gpio = &joypad->gpios[nbtn++];
		int error;

		gpio->num = of_get_gpio_flags(pp, 0, &flags);
		if (gpio->num < 0) {
			error = gpio->num;
			dev_err(dev, "Failed to get gpio flags, error: %d\n",
				error);
			return error;
		}

		/* gpio active level(key press level) */
		gpio->active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;

		gpio->label = of_get_property(pp, "label", NULL);

		if (gpio_is_valid(gpio->num)) {
			error = devm_gpio_request_one(dev, gpio->num,
						      GPIOF_IN, gpio->label);
			if (error < 0) {
				dev_err(dev,
					"Failed to request GPIO %d, error %d\n",
					gpio->num, error);
				return error;
			}
		}
		if (of_property_read_u32(pp, "linux,code", &gpio->linux_code)) {
			dev_err(dev, "Button without keycode: 0x%x\n",
				gpio->num);
			return -EINVAL;
		}
		if (of_property_read_u32(pp, "linux,input-type",
				&gpio->report_type))
			gpio->report_type = EV_KEY;
	}
	if (nbtn == 0)
		return -EINVAL;

	return	0;
}

/*----------------------------------------------------------------------------*/
struct input_dev * joypad_input_g;

void rk_send_key_f_key_up(void)
{
	if (!joypad_input_g)
		return;

	input_report_key(joypad_input_g, BTN_MODE, 1);
	input_sync(joypad_input_g);
}
EXPORT_SYMBOL(rk_send_key_f_key_up);

void rk_send_key_f_key_down(void)
{
	if (!joypad_input_g)
		return;

	input_report_key(joypad_input_g, BTN_MODE, 0);
	input_sync(joypad_input_g);
}
EXPORT_SYMBOL(rk_send_key_f_key_down);

static int joypad_input_setup(struct device *dev, struct joypad *joypad)
{
	struct input_polled_dev *poll_dev;
	struct input_dev *input;
	int nbtn, error;
	u32 joypad_vendor = 0;
	u32 joypad_revision = 0;
	u32 joypad_product = 0;

	poll_dev = devm_input_allocate_polled_device(dev);
	if (!poll_dev) {
		dev_err(dev, "no memory for polled device\n");
		return -ENOMEM;
	}

	poll_dev->private	= joypad;
	poll_dev->poll		= joypad_poll;
	poll_dev->poll_interval	= joypad->poll_interval;
	poll_dev->open		= joypad_open;
	poll_dev->close		= joypad_close;

	input = poll_dev->input;

	input->name = DRV_NAME;

	joypad_input_g=input;

	device_property_read_string(dev, "joypad-name", &input->name);
	input->phys = DRV_NAME"/input0";

	device_property_read_u32(dev, "joypad-vendor", &joypad_vendor);
	device_property_read_u32(dev, "joypad-revision", &joypad_revision);
	device_property_read_u32(dev, "joypad-product", &joypad_product);
	input->id.bustype = BUS_HOST;
	//input->id.bustype = BUS_USB;
	input->id.vendor  = (u16)joypad_vendor;
	input->id.product = (u16)joypad_product;
	input->id.version = (u16)joypad_revision;

	/* IIO ADC key setup (0 mv ~ 1800 mv) * adc->scale */
	__set_bit(EV_ABS, input->evbit);

	// Set mapped ones on dt
	for(nbtn = 0; nbtn < joypad->amux_count; nbtn++) {
		struct bt_adc *adc = &joypad->adcs[nbtn];
		input_set_abs_params(input, adc->report_type,
				adc->min, adc->max,
				joypad->bt_adc_fuzz,
				joypad->bt_adc_flat);
		dev_info(dev,
			"%s : SCALE = %d, ABS min = %d, max = %d,"
			" fuzz = %d, flat = %d, deadzone = %d\n",
			__func__, adc->scale, adc->min, adc->max,
			joypad->bt_adc_fuzz, joypad->bt_adc_flat,
			joypad->bt_adc_deadzone);
		dev_info(dev,
			"%s : adc tuning_p = %d, adc_tuning_n = %d\n\n",
			__func__, adc->tuning_p, adc->tuning_n);
	}

	/* GPIO key setup */
	__set_bit(EV_KEY, input->evbit);
	for(nbtn = 0; nbtn < joypad->bt_gpio_count; nbtn++) {
		struct bt_gpio *gpio = &joypad->gpios[nbtn];
		input_set_capability(input, gpio->report_type,
				gpio->linux_code);
	}

	if (joypad->auto_repeat)
		__set_bit(EV_REP, input->evbit);

	joypad->dev = dev;
	joypad->poll_dev = poll_dev;

	error = input_register_polled_device(poll_dev);
	if (error) {
		dev_err(dev, "unable to register polled device, err=%d\n",
			error);
		return error;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int joypad_dt_parse(struct device *dev, struct joypad *joypad)
{
	int error = 0;

	/* initialize values from device-tree */
	device_property_read_u32(dev, "button-adc-fuzz",
				&joypad->bt_adc_fuzz);
	device_property_read_u32(dev, "button-adc-flat",
				&joypad->bt_adc_flat);
	device_property_read_u32(dev, "button-adc-scale",
				&joypad->bt_adc_scale);
	device_property_read_u32(dev, "button-adc-deadzone",
				&joypad->bt_adc_deadzone);

	device_property_read_u32(dev, "amux-count",
				&joypad->amux_count);

	device_property_read_u32(dev, "poll-interval",
				&joypad->poll_interval);

	joypad->auto_repeat = device_property_present(dev, "autorepeat");

	joypad->bt_gpio_count = device_get_child_node_count(dev);

	if ((joypad->amux_count == 0) || (joypad->bt_gpio_count == 0)) {
		dev_err(dev, "adc key = %d, gpio key = %d error!",
			joypad->amux_count, joypad->bt_gpio_count);
		return -EINVAL;
	}

	error = joypad_adc_setup(dev, joypad);
	if (error)
		return error;

	error = joypad_amux_setup(dev, joypad);
	if (error)
		return error;

	error = joypad_gpio_setup(dev, joypad);
	if (error)
		return error;

	dev_info(dev, "%s : adc key cnt = %d, gpio key cnt = %d\n",
			__func__, joypad->amux_count, joypad->bt_gpio_count);

	return error;
}

/*----------------------------------------------------------------------------*/
static int joypad_probe(struct platform_device *pdev)
{
	struct joypad *joypad;
	struct device *dev = &pdev->dev;
	int error;

	joypad = devm_kzalloc(dev, sizeof(struct joypad), GFP_KERNEL);
	if (!joypad) {
		dev_err(dev, "joypad devm_kzmalloc error!");
		return -ENOMEM;
	}

	/* device tree data parse */
	error = joypad_dt_parse(dev, joypad);
	if (error) {
		dev_err(dev, "dt parse error!(err = %d)\n", error);
		return error;
	}

	mutex_init(&joypad->lock);
	platform_set_drvdata(pdev, joypad);

	error = sysfs_create_group(&pdev->dev.kobj, &joypad_attr_group);
	if (error) {
		dev_err(dev, "create sysfs group fail, error: %d\n",
			error);
		return error;
	}

	/* poll input device setup */
	error = joypad_input_setup(dev, joypad);
	if (error) {
		dev_err(dev, "input setup failed!(err = %d)\n", error);
		return error;
	}
	dev_info(dev, "%s : probe success\n", __func__);
	return 0;
}

static void joypad_shutdown(struct platform_device *pdev)
{
	struct joypad *joypad = platform_get_drvdata(pdev);
	input_unregister_polled_device(joypad->poll_dev);
}
/*----------------------------------------------------------------------------*/
static const struct of_device_id joypad_of_match[] = {
	{ .compatible = "rocknix-singleadc-joypad", },
	{},
};

MODULE_DEVICE_TABLE(of, joypad_of_match);

/*----------------------------------------------------------------------------*/
static struct platform_driver joypad_driver = {
	.probe = joypad_probe,
	.shutdown = joypad_shutdown,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(joypad_of_match),
	},
};

/*----------------------------------------------------------------------------*/
static int __init joypad_init(void)
{
	return platform_driver_register(&joypad_driver);
}

/*----------------------------------------------------------------------------*/
static void __exit joypad_exit(void)
{
	platform_driver_unregister(&joypad_driver);
}

/*----------------------------------------------------------------------------*/
late_initcall(joypad_init);
module_exit(joypad_exit);

/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("ROCKNIX");
MODULE_DESCRIPTION("ROCKNIX singleadc joypad driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_INFO(intree, "Y");

/*----------------------------------------------------------------------------*/
