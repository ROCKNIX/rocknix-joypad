/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ROCKNIX singleadc joypad driver
 *
 * Copyright (C) 2024 ROCKNIX (https://github.com/ROCKNIX)
 */

/*----------------------------------------------------------------------------*/
#include <linux/module.h>
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
#include <linux/pwm.h>
#include "rocknix-joypad.h"

#include <linux/kthread.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/fs.h>
#include <linux/termios.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>

/*----------------------------------------------------------------------------*/
#define DRV_NAME "rocknix-singleadc-joypad"
/*----------------------------------------------------------------------------*/
#define	ADC_MAX_VOLTAGE		1800
#define	ADC_DATA_TUNING(x, p)	((x * p) / 100)
#define	ADC_TUNING_DEFAULT	180
#define	CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

/* We add a 5s inactivity timeout for re-opening ttyS1 */
#define MIYOO_SERIAL_INACTIVITY_TIMEOUT (5 * HZ)

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

/* Replicate the calibration and constants from the userland code. */
#define MIYOO_FRAME_SIZE        6
#define MIYOO_MAGIC_START       0xFF
#define MIYOO_MAGIC_END         0xFE

/* For calibrating final ranges to [-32760..32760] */
#define MIYOO_AXIS_MIN          (-32760)
#define MIYOO_AXIS_MAX          ( 32760)

/* We'll gather 50 frames for auto-cal */
#define MIYOO_CALIBRATION_FRAMES    50

/* We'll use a default 10% radial deadzone ratio for example */
static float miyoo_deadzone_ratio = 0.10f;

struct miyoo_cal {
	int x_min;
	int x_max;
	int x_zero;
	int y_min;
	int y_max;
	int y_zero;
};

/*
 * We'll store the "left stick" and "right stick" calibrations
 * plus live data for the Miyoo serial approach
 */
struct miyoo_serial_stick {
	struct miyoo_cal left_cal;
	struct miyoo_cal right_cal;

	/* We keep the last parsed values for x,y each stick */
	int left_x;
	int left_y;
	int right_x;
	int right_y;
};

/* A forward-declare for our kernel thread function */
static int miyoo_serial_threadfn(void *data);

/*
 * We'll define a helper that directly manipulates the tty struct
 * instead of doing an ioctl() with a kernel pointer.
 */
static void set_tty_9600_8N1(struct file *filp)
{
	struct tty_file_private *file_priv;
	struct tty_struct *tty;
	struct ktermios newt;

	if (!filp || !filp->private_data)
		return;

	/* filp->private_data is typically a struct tty_file_private if this is a TTY */
	file_priv = filp->private_data;
	if (!file_priv || !file_priv->tty)
		return;

	tty = file_priv->tty;

	/* Copy existing termios and modify it in-kernel. */
	newt = tty->termios;
	newt.c_iflag = 0;
	newt.c_oflag = 0;
	newt.c_cflag = B9600 | CS8 | CREAD | CLOCAL;
	newt.c_lflag = 0;
	memset(newt.c_cc, 0, NCCS);
	newt.c_cc[VMIN]  = 1;
	newt.c_cc[VTIME] = 0;

	/*
	 * This bypasses the userland copy checks, because we are
	 * operating in kernel space.
	 */
	tty_set_termios(tty, &newt);
}

struct joypad {
	struct device *dev;
	int poll_interval;

	/* report enable/disable */
	bool enable;

	/* analog mux & joystick control */
	struct analog_mux *amux;
	/* analog mux max count */
	int amux_count;
	/* analog button */
	struct bt_adc *adcs;

	/* report reference point */
	bool invert_absx;
	bool invert_absy;
	bool invert_absrx;
	bool invert_absry;

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

	/* pwm device for rumble*/
	struct input_dev *input;
	struct pwm_device *pwm;
	struct work_struct play_work;
	u16 level;
	u16 boost_weak;
	u16 boost_strong;
	bool has_rumble;
	bool rumble_enabled; /* to turn rumble on/off if a device has it */


	/* New property to override logic with Miyoo serial approach */
	bool use_miyoo_serial;
	/*
	 * We store everything for the Miyoo logic:
	 *   - calibration data
	 *   - kernel thread
	 *   - file pointer, etc.
	 */
	struct miyoo_serial_stick miyoo;
	struct task_struct *miyoo_thread;
	struct file *miyoo_filp;

	/*
	 * We'll do a delayed work item to do the TTY initialization
	 * after the driver has finished loading. This ensures we
	 * don't block the kernel boot.
	 */
	struct delayed_work miyoo_init_work;

	/* For detecting inactivity on the Miyoo TTY */
	unsigned long last_data_jiffies;

};

extern struct input_dev * joypad_input_g;

static int pwm_vibrator_start(struct joypad *joypad)
{
	struct pwm_state state;
	int err;

	pwm_get_state(joypad->pwm, &state);
	pwm_set_relative_duty_cycle(&state, joypad->level, 0xffff);
	state.enabled = true;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0))
	err = pwm_apply_state(joypad->pwm, &state);
#else
	err = pwm_apply_might_sleep(joypad->pwm, &state);
#endif
	if (err) {
		 dev_err(joypad->dev, "failed to apply pwm state: %d", err);
		 return err;
	}

	return 0;
}

static void pwm_vibrator_stop(struct joypad *joypad)
{
	pwm_disable(joypad->pwm);
}

static void pwm_vibrator_play_work(struct work_struct *work)
{
	struct joypad *joypad = container_of(work,
					    struct joypad, play_work);
	mutex_lock(&joypad->lock);
	if (!joypad->rumble_enabled) {
		pwm_vibrator_stop(joypad);
		mutex_unlock(&joypad->lock);
		return;
	}

	if (joypad->level)
		 pwm_vibrator_start(joypad);
	else
		 pwm_vibrator_stop(joypad);

	mutex_unlock(&joypad->lock);
}

/*----------------------------------------------------------------------------*/
static int joypad_amux_select(struct analog_mux *amux, int channel)
{
	/* select mux channel */
	gpio_set_value_cansleep(amux->en_gpio, 0);

	switch(channel) {
		case 0:	/* EVENT (ABS_RY) */
			gpio_set_value_cansleep(amux->sel_a_gpio, 0);
			gpio_set_value_cansleep(amux->sel_b_gpio, 0);
			break;
		case 1:	/* EVENT (ABS_RX) */
			gpio_set_value_cansleep(amux->sel_a_gpio, 0);
			gpio_set_value_cansleep(amux->sel_b_gpio, 1);
			break;
		case 2:	/* EVENT (ABS_Y) */
			gpio_set_value_cansleep(amux->sel_a_gpio, 1);
			gpio_set_value_cansleep(amux->sel_b_gpio, 0);
			break;
		case 3:	/* EVENT (ABS_X) */
			gpio_set_value_cansleep(amux->sel_a_gpio, 1);
			gpio_set_value_cansleep(amux->sel_b_gpio, 1);
			break;
		default:
			/* amux disanle */
			gpio_set_value_cansleep(amux->en_gpio, 1);
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

	return value;
}

/*----------------------------------------------------------------------------*/
/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/rocknix-singleadc-joypad/rumble_enable [rw]
 */
/*----------------------------------------------------------------------------*/
static ssize_t joypad_store_rumble_enable(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf,
					  size_t count)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);
	bool enable = simple_strtoul(buf, NULL, 10);

	mutex_lock(&joypad->lock);
	if (enable && !joypad->rumble_enabled) {
		joypad->rumble_enabled = true;
	} else if (!enable && joypad->rumble_enabled) {
		joypad->rumble_enabled = false;
		joypad->level = 0;
		cancel_work_sync(&joypad->play_work);
		pwm_vibrator_stop(joypad);
	}
	mutex_unlock(&joypad->lock);
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t joypad_show_rumble_enable(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);
	return sprintf(buf, "%d\n", joypad->rumble_enabled ? 1 : 0);
}

/*----------------------------------------------------------------------------*/
static DEVICE_ATTR(rumble_enable, S_IWUSR | S_IRUGO,
		   joypad_show_rumble_enable,
		   joypad_store_rumble_enable);

/*----------------------------------------------------------------------------*/
static struct attribute *joypad_rumble_attrs[] = {
	&dev_attr_rumble_enable.attr,
	NULL,
};

static struct attribute_group joypad_rumble_attr_group = {
	.attrs = joypad_rumble_attrs,
};

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
		value = gpio_get_value_cansleep(gpio->num);
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
		if (joypad->use_miyoo_serial) {
			/*
			 * Instead of ADC, we rely on the data read in
			 * our miyoo_serial_threadfn(). We'll just do
			 * input_report_abs here from the stored values.
			 */
			input_report_abs(poll_dev->input, ABS_X,  joypad->miyoo.left_x);
			input_report_abs(poll_dev->input, ABS_Y,  joypad->miyoo.left_y);
			input_report_abs(poll_dev->input, ABS_RX, joypad->miyoo.right_x);
			input_report_abs(poll_dev->input, ABS_RY, joypad->miyoo.right_y);
			input_sync(poll_dev->input);

			joypad_gpio_check(poll_dev);
		} else {
			joypad_adc_check(poll_dev);
			joypad_gpio_check(poll_dev);
		}
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
		int val = gpio_get_value_cansleep(gpio->num);
		if (val < 0)
			val = gpio->active_level ? 0 : 1;
		gpio->old_value = val;

		// Immediately report the current state
		input_event(poll_dev->input, gpio->report_type,
					gpio->linux_code,
					(val == gpio->active_level) ? 1 : 0);
	}
	input_sync(poll_dev->input);

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
	/* buttons status sync */
	joypad_adc_check(poll_dev);
	joypad_gpio_check(poll_dev);

	/* button report enable */
	mutex_lock(&joypad->lock);
	joypad->enable = true;
	mutex_unlock(&joypad->lock);
}

/*----------------------------------------------------------------------------*/
static void joypad_close(struct input_polled_dev *poll_dev)
{
	struct joypad *joypad = poll_dev->private;

	if (joypad->has_rumble) {
		cancel_work_sync(&joypad->play_work);
		pwm_vibrator_stop(joypad);
	}

	/* button report disable */
	mutex_lock(&joypad->lock);
	joypad->enable = false;
	mutex_unlock(&joypad->lock);
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
		ret = devm_gpio_request_one(dev, amux->sel_a_gpio, GPIOF_IN, "amux-sel-a");
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
		ret = devm_gpio_request_one(dev, amux->sel_b_gpio, GPIOF_IN, "amux-sel-b");
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
		ret = devm_gpio_request_one(dev, amux->en_gpio, GPIOF_IN, "amux-en");
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
	u32 channel_mapping[] = {0, 1, 2, 3};

	if (device_property_present(dev, "amux-channel-mapping")) {
		int ret;
		ret = of_property_read_u32_array(dev->of_node,
				"amux-channel-mapping", channel_mapping, 4);
		if (ret < 0) {
			dev_err(dev, "invalid channel mapping\n");
			return -EINVAL;
		}
	}

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
		adc->invert = false;

		switch (nbtn) {
			case 0:
				if (joypad->invert_absry)
					adc->invert = true;
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
				if (joypad->invert_absrx)
					adc->invert = true;
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
				if (joypad->invert_absy)
					adc->invert = true;
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
				if (joypad->invert_absx)
					adc->invert = true;
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
		adc->amux_ch = channel_mapping[nbtn];
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
static int rumble_play_effect(struct input_dev *dev, void *data, struct ff_effect *effect)
{
	struct joypad *joypad = data;
	u32 boosted_level;
	if (effect->type != FF_RUMBLE)
		 return 0;

	mutex_lock(&joypad->lock);
	if (!joypad->rumble_enabled) {
		mutex_unlock(&joypad->lock);
		return 0; // Ignore rumble effects if disabled
	}

	if (effect->u.rumble.strong_magnitude)
		boosted_level = effect->u.rumble.strong_magnitude + joypad->boost_strong;
	else
		boosted_level = effect->u.rumble.weak_magnitude + joypad->boost_weak;

	joypad->level = (u16)CLAMP(boosted_level, 0, 0xffff);

	mutex_unlock(&joypad->lock);

	schedule_work(&joypad->play_work);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int joypad_rumble_setup(struct device *dev, struct joypad *joypad)
{
	int error;
	struct pwm_state state;

	joypad->pwm = devm_pwm_get(dev, "enable");
	if (IS_ERR(joypad->pwm)) {
		dev_err(dev, "rumble get error\n");
		return -EINVAL;
	}

	INIT_WORK(&joypad->play_work, pwm_vibrator_play_work);

	/* Sync up PWM state and ensure it is off. */
	pwm_init_state(joypad->pwm, &state);
	state.enabled = false;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0))
	error = pwm_apply_state(joypad->pwm, &state);
#else
	error = pwm_apply_might_sleep(joypad->pwm, &state);
#endif
	if (error) {
		 dev_err(dev, "failed to apply initial PWM state: %d",
			 error);
		 return error;
	}

	dev_info(dev, "rumble setup success!\n");
	return 0;
}

/*----------------------------------------------------------------------------*/
static int joypad_input_setup(struct device *dev, struct joypad *joypad)
{
	struct input_polled_dev *poll_dev;
	struct input_dev *input;
	int nbtn, error;
	u32 joypad_bustype = BUS_HOST;
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

	device_property_read_u32(dev, "joypad-bustype", &joypad_bustype);
	device_property_read_u32(dev, "joypad-vendor", &joypad_vendor);
	device_property_read_u32(dev, "joypad-revision", &joypad_revision);
	device_property_read_u32(dev, "joypad-product", &joypad_product);
	input->id.bustype = (u16)joypad_bustype;
	input->id.vendor  = (u16)joypad_vendor;
	input->id.product = (u16)joypad_product;
	input->id.version = (u16)joypad_revision;

	/* IIO ADC key setup (0 mv ~ 1800 mv) * adc->scale */
	/*
	 * We always want 4 axes available if using Miyoo
	 * or if we have actual amux_count > 0
	 */
	if (joypad->amux_count > 0 || joypad->use_miyoo_serial) {
		__set_bit(EV_ABS, input->evbit);
	}

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

	/*
	 * For the Miyoo serial approach, we’ll also ensure that
	 * ABS_X, ABS_Y, ABS_RX, ABS_RY are defined ([-32760..32760]) if used:
	 */
	if (joypad->use_miyoo_serial) {
		input_set_abs_params(input, ABS_X,
				     MIYOO_AXIS_MIN, MIYOO_AXIS_MAX, 16, 16);
		input_set_abs_params(input, ABS_Y,
				     MIYOO_AXIS_MIN, MIYOO_AXIS_MAX, 16, 16);
		input_set_abs_params(input, ABS_RX,
				     MIYOO_AXIS_MIN, MIYOO_AXIS_MAX, 16, 16);
		input_set_abs_params(input, ABS_RY,
				     MIYOO_AXIS_MIN, MIYOO_AXIS_MAX, 16, 16);
	}

	/* Rumble setup */
	if (joypad->has_rumble) {
		u32 boost_weak = 0;
		u32 boost_strong = 0;
		device_property_read_u32(dev, "rumble-boost-weak", &boost_weak);
		device_property_read_u32(dev, "rumble-boost-strong", &boost_strong);
		joypad->boost_weak = boost_weak;
		joypad->boost_strong = boost_strong;
		dev_info(dev, "Boost = %d, %d", boost_weak, boost_strong);
		input_set_capability(input, EV_FF, FF_RUMBLE);
		error = input_ff_create_memless(input, joypad, rumble_play_effect);
		if (error) {
			dev_err(dev, "unable to register rumble, err=%d\n",
				error);
			return error;
		}
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

	error = input_register_polled_device(poll_dev);
	if (error) {
		dev_err(dev, "unable to register polled device, err=%d\n",
			error);
		return error;
	}

	return 0;
}

/*
 * Utility for calibration & applying radial deadzone 
 * (mirroring the userland logic from miyoostick).
 */
static int miyoo_calibrate_axis_x(const struct miyoo_cal *cal, int raw)
{
    int rangePos, rangeNeg, result;

    if (!cal)
        return 0;

    rangePos = cal->x_max - cal->x_zero;
    rangeNeg = cal->x_zero - cal->x_min;
    result = 0;

	if (raw > cal->x_zero) {
		int diff = raw - cal->x_zero;
		if (rangePos != 0) {
			result = (diff * MIYOO_AXIS_MAX) / rangePos;
			if (result > MIYOO_AXIS_MAX) result = MIYOO_AXIS_MAX;
		}
	} else if (raw < cal->x_zero) {
		int diff = cal->x_zero - raw;
		if (rangeNeg != 0) {
			result = -(diff * (-MIYOO_AXIS_MIN)) / rangeNeg;
			if (result < MIYOO_AXIS_MIN) result = MIYOO_AXIS_MIN;
		}
	}
	return result;
}

static int miyoo_calibrate_axis_y(const struct miyoo_cal *cal, int raw)
{
    int rangePos, rangeNeg, result;

    if (!cal)
        return 0;

    rangePos = cal->y_max - cal->y_zero;
    rangeNeg = cal->y_zero - cal->y_min;
    result = 0;

	if (raw > cal->y_zero) {
		int diff = raw - cal->y_zero;
		if (rangePos != 0) {
			result = (diff * MIYOO_AXIS_MAX) / rangePos;
			if (result > MIYOO_AXIS_MAX) result = MIYOO_AXIS_MAX;
		}
	} else if (raw < cal->y_zero) {
		int diff = cal->y_zero - raw;
		if (rangeNeg != 0) {
			result = -(diff * (-MIYOO_AXIS_MIN)) / rangeNeg;
			if (result < MIYOO_AXIS_MIN) result = MIYOO_AXIS_MIN;
		}
	}
	return result;
}

static void miyoo_apply_deadzone(int *px, int *py)
{
	int x = *px;
	int y = *py;
	long long magSq = (long long)x * x + (long long)y * y;
	long long dead = (long long)(miyoo_deadzone_ratio * MIYOO_AXIS_MAX);
	long long deadSq = dead * dead;

	if (magSq <= deadSq) {
		*px = 0;
		*py = 0;
	}
}
/*----------------------------------------------------------------------------*/
static int miyoo_autocal(struct joypad *joypad)
{
	long sumYL = 0, sumXL = 0, sumYR = 0, sumXR = 0;
	int framesRead = 0;
	unsigned char buf[MIYOO_FRAME_SIZE];

	dev_info(joypad->dev, "Auto-calibration: reading %d frames...", MIYOO_CALIBRATION_FRAMES);

	while (framesRead < MIYOO_CALIBRATION_FRAMES) {
		ssize_t n = kernel_read(joypad->miyoo_filp, buf, MIYOO_FRAME_SIZE, &joypad->miyoo_filp->f_pos);
		if (n < 0) {
			dev_err(joypad->dev, "Auto-cal read error: %zd\n", n);
			msleep(10);
			continue;
		}
		if (n < MIYOO_FRAME_SIZE) {
			msleep(10);
			continue;
		}
		if (buf[0] == MIYOO_MAGIC_START && buf[5] == MIYOO_MAGIC_END) {
			sumYL += buf[1];
			sumXL += buf[2];
			sumYR += buf[3];
			sumXR += buf[4];
			framesRead++;
		}
	}

	if (framesRead == 0) {
		dev_err(joypad->dev, "No calibration frames read!\n");
		return -EIO;
	}
	{
		int avgYL = sumYL / framesRead;
		int avgXL = sumXL / framesRead;
		int avgYR = sumYR / framesRead;
		int avgXR = sumXR / framesRead;
		dev_info(joypad->dev, "  Average YL=%d, XL=%d, YR=%d, XR=%d\n",
			 avgYL, avgXL, avgYR, avgXR);

		joypad->miyoo.left_cal.x_zero  = avgXL;
		joypad->miyoo.left_cal.y_zero  = avgYL;
		joypad->miyoo.right_cal.x_zero = avgXR;
		joypad->miyoo.right_cal.y_zero = avgYR;
	}
	dev_info(joypad->dev, "Auto-calibration complete.\n");
	return 0;
}

static void miyoo_parse_frame(struct joypad *joypad, unsigned char *frame)
{
	if (frame[0] != MIYOO_MAGIC_START || frame[5] != MIYOO_MAGIC_END) {
		return;
	}

	/* We got a valid frame, so reset our inactivity timer */
	joypad->last_data_jiffies = jiffies;

	{
		int rawYL = frame[1];
		int rawXL = frame[2];
		int rawYR = frame[3];
		int rawXR = frame[4];

		int lx = miyoo_calibrate_axis_x(&joypad->miyoo.left_cal, rawXL);
		int ly = miyoo_calibrate_axis_y(&joypad->miyoo.left_cal, rawYL);
		int rx = miyoo_calibrate_axis_x(&joypad->miyoo.right_cal, rawXR);
		int ry = miyoo_calibrate_axis_y(&joypad->miyoo.right_cal, rawYR);

		miyoo_apply_deadzone(&lx, &ly);
		miyoo_apply_deadzone(&rx, &ry);

		joypad->miyoo.left_x  = lx;
		joypad->miyoo.left_y  = ly;
		joypad->miyoo.right_x = rx;
		joypad->miyoo.right_y = ry;
	}
}

/* 
 * We'll factor out open/close so we can easily reopen after inactivity.
 */
static int miyoo_serial_open(struct joypad *joypad)
{
	struct file *filp;

	filp = filp_open("/dev/ttyS1", O_RDWR | O_NOCTTY, 0);
	if (IS_ERR(filp)) {
		dev_err(joypad->dev, "ERROR: open /dev/ttyS1 failed: %ld\n", PTR_ERR(filp));
		return PTR_ERR(filp);
	}
	joypad->miyoo_filp = filp;

	/* 
	 * Instead of doing an ioctl() with a kernel pointer, 
	 * we set 9600 8N1 by manipulating the underlying tty->termios directly.
	 */
	set_tty_9600_8N1(filp);

	dev_info(joypad->dev, "Opened serial port /dev/ttyS1 and set 9600 8N1 (in-kernel).\n");
	return 0;
}

static void miyoo_serial_close(struct joypad *joypad)
{
	if (joypad->miyoo_filp) {
		dev_info(joypad->dev, "Closing /dev/ttyS1.\n");
		filp_close(joypad->miyoo_filp, NULL);
		joypad->miyoo_filp = NULL;
	}
}

static int miyoo_serial_threadfn(void *data)
{
	struct joypad *joypad = data;
	unsigned char frame[MIYOO_FRAME_SIZE];

	dev_info(joypad->dev, "Starting Miyoo serial read loop.\n");

	while (!kthread_should_stop()) {
		ssize_t n = kernel_read(joypad->miyoo_filp, frame, MIYOO_FRAME_SIZE, &joypad->miyoo_filp->f_pos);
		if (n < 0) {
			dev_err(joypad->dev, "Serial read error: %zd\n", n);
			msleep(10);
			continue;
		}
		if (n < MIYOO_FRAME_SIZE) {
			msleep(10);
			continue;
		}
		miyoo_parse_frame(joypad, frame);

		/*
		 * Check for inactivity. If we haven't received a valid frame for
		 * MIYOO_SERIAL_INACTIVITY_TIMEOUT jiffies, re-init the port.
		 */
		if (time_after(jiffies, joypad->last_data_jiffies + MIYOO_SERIAL_INACTIVITY_TIMEOUT)) {
			dev_warn(joypad->dev, "No data from /dev/ttyS1 for 5s; re-opening TTY.\n");

			/* Step 1: close the old filp */
			miyoo_serial_close(joypad);

			/* Step 2: attempt to reopen the TTY */
			if (miyoo_serial_open(joypad) == 0) {
				/*
				 * We do NOT re-auto-cal on reconnection, per request.
				 * Just reset the timer so we won't keep re-opening.
				 */
				joypad->last_data_jiffies = jiffies;
				dev_info(joypad->dev, "TTY re-init success!\n");
			} else {
				dev_err(joypad->dev,
					"Failed to re-open /dev/ttyS1; will retry in loop.\n");
				/*
				 * If the open fails, the next iteration of the loop 
				 * will keep failing reads until a successful open.
				 */
			}
		}
	}

	dev_info(joypad->dev, "Miyoo serial thread exiting.\n");
	return 0;
}

static int miyoo_start_serial(struct joypad *joypad)
{
	int ret;
	
	dev_info(joypad->dev, "Starting /dev/ttyS1 init after driver load.\n");

	ret = miyoo_serial_open(joypad);
	if (ret < 0)
		return ret;

	/*
	 * We do an initial auto-cal here if desired. 
	 * This is the only time we'll calibrate, not on reconnection.
	 */
	ret = miyoo_autocal(joypad);
	if (ret < 0) {
		dev_err(joypad->dev, "Auto-calibration failed: %d\n", ret);
		/* We can continue, but the user won't have a correct zero. */
	}

	joypad->miyoo_thread = kthread_run(miyoo_serial_threadfn, joypad, "miyoo-serial-thread");
	if (IS_ERR(joypad->miyoo_thread)) {
		dev_err(joypad->dev, "Failed to create Miyoo serial thread.\n");
		miyoo_serial_close(joypad);
		return PTR_ERR(joypad->miyoo_thread);
	}

	joypad->last_data_jiffies = jiffies;

	dev_info(joypad->dev, "Miyoo serial logic started successfully!\n");
	return 0;
}

/*
 * We'll define a delayed work function that calls miyoo_start_serial()
 * after 10 seconds. This ensures the driver loads fully first.
 */
static void miyoo_delayed_init_workfn(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct joypad *joypad = container_of(dwork, struct joypad, miyoo_init_work);

	miyoo_start_serial(joypad);
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

	/* change the report reference point? (ADC MAX - read value) */
	joypad->invert_absx = device_property_present(dev, "invert-absx");
	joypad->invert_absy = device_property_present(dev, "invert-absy");
	joypad->invert_absrx = device_property_present(dev, "invert-absrx");
	joypad->invert_absry = device_property_present(dev, "invert-absry");
	dev_info(dev, "%s : invert-absx = %d, inveret-absy = %d, invert-absrx = %d, invert-absry = %d\n",
		__func__, joypad->invert_absx, joypad->invert_absy, joypad->invert_absrx, joypad->invert_absry);

	joypad->bt_gpio_count = device_get_child_node_count(dev);

	if ((joypad->amux_count == 0) && (joypad->bt_gpio_count == 0)) {
		dev_err(dev, "adc key = %d, gpio key = %d error!",
			joypad->amux_count, joypad->bt_gpio_count);
		/* not a fatal error if we are using Miyoo,
		   but let's keep it for safety (later we'll override). */
	}

	error = 0;
	if (joypad->amux_count > 0) {
		error = joypad_adc_setup(dev, joypad);
		if (error)
			return error;

		error = joypad_amux_setup(dev, joypad);
		if (error)
			return error;
	}

	error = joypad_gpio_setup(dev, joypad);
	if (error)
		return error;

	dev_info(dev, "%s : adc key cnt = %d, gpio key cnt = %d\n",
			__func__, joypad->amux_count, joypad->bt_gpio_count);

	joypad->has_rumble =
		device_property_present(dev, "pwm-names");
	if (joypad->has_rumble)
		dev_info(dev, "%s : has rumble\n", __func__);

	joypad->use_miyoo_serial =
		device_property_present(dev, "rocknix,use-miyoo-serial-joypad");
	if (joypad->use_miyoo_serial) {
		dev_info(dev, "%s : using Miyoo Serial Joypad logic\n", __func__);
		/*
		 * We'll define some default calibrations for raw [85..200]
		 * from your original userland code, so that x_zero=130, etc.
		 */
		joypad->miyoo.left_cal.x_min = 85;
		joypad->miyoo.left_cal.x_max = 200;
		joypad->miyoo.left_cal.x_zero= 130;
		joypad->miyoo.left_cal.y_min = 85;
		joypad->miyoo.left_cal.y_max = 200;
		joypad->miyoo.left_cal.y_zero= 130;

		joypad->miyoo.right_cal.x_min = 85;
		joypad->miyoo.right_cal.x_max = 200;
		joypad->miyoo.right_cal.x_zero= 130;
		joypad->miyoo.right_cal.y_min = 85;
		joypad->miyoo.right_cal.y_max = 200;
		joypad->miyoo.right_cal.y_zero= 130;

		joypad->miyoo.left_x  = 0;
		joypad->miyoo.left_y  = 0;
		joypad->miyoo.right_x = 0;
		joypad->miyoo.right_y = 0;
	}

	return error;
}

static int __maybe_unused joypad_suspend(struct device *dev)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);
	if (joypad->has_rumble) {
		cancel_work_sync(&joypad->play_work);
		if (joypad->level)
			 pwm_vibrator_stop(joypad);
	}
	return 0;
}

static int __maybe_unused joypad_resume(struct device *dev)
{
	struct platform_device *pdev  = to_platform_device(dev);
	struct joypad *joypad = platform_get_drvdata(pdev);
	if (joypad->has_rumble) {
		if (joypad->level)
			 pwm_vibrator_start(joypad);
	}
	return 0;
}

static SIMPLE_DEV_PM_OPS(joypad_pm_ops, joypad_suspend, joypad_resume);

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

	/* poll input device setup */
	error = joypad_input_setup(dev, joypad);
	if (error) {
		dev_err(dev, "input setup failed!(err = %d)\n", error);
		return error;
	}

        if (joypad->has_rumble) {
		/* rumble setup */
		error = sysfs_create_group(&pdev->dev.kobj, &joypad_rumble_attr_group);
		if (error) {
			dev_err(dev, "create sysfs group fail, error: %d\n",
				error);
			return error;
		}

		error = joypad_rumble_setup(dev, joypad);
		if (error) {
			 dev_err(dev, "rumble setup failed!(err = %d)\n", error);
			 return error;
		}

		joypad->rumble_enabled = true;
	}

	if (joypad->use_miyoo_serial) {
		/*
		 * Instead of blocking in probe or using a separate boot
		 * thread, we schedule a delayed work for 10 seconds.
		 */
		INIT_DELAYED_WORK(&joypad->miyoo_init_work, miyoo_delayed_init_workfn);
		schedule_delayed_work(&joypad->miyoo_init_work, 10 * HZ);
	}

	dev_info(dev, "%s : probe success\n", __func__);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int joypad_remove(struct platform_device *pdev)
{
	struct joypad *joypad = platform_get_drvdata(pdev);

	/* Stop the Miyoo serial thread and cleanup whatever is running */
	if (joypad->use_miyoo_serial) {
		cancel_delayed_work_sync(&joypad->miyoo_init_work);
		if (joypad->miyoo_thread) {
			kthread_stop(joypad->miyoo_thread);
			joypad->miyoo_thread = NULL;
		}
		miyoo_serial_close(joypad);
	}

	/* Cleanup rumble too */
	if (joypad->has_rumble) {
		cancel_work_sync(&joypad->play_work);
		pwm_vibrator_stop(joypad);
		sysfs_remove_group(&pdev->dev.kobj, &joypad_rumble_attr_group);
	}

	dev_info(&pdev->dev, "%s : removed\n", __func__);
	return 0;
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
	.remove = joypad_remove,
	.driver = {
		.name = DRV_NAME,
		.pm = &joypad_pm_ops,
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
