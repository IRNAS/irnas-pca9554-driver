/** @file gpio_pca9554.c
 *
 * @brief Driver for PCA9554 I2C-based GPIO driver.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2022 Irnas. All rights reserved.
 * Author: Marko Sagadin <marko@irnas.eu>
 */

#define DT_DRV_COMPAT irnas_pca9554

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <gpio_utils.h>

LOG_MODULE_REGISTER(gpio_pca9554, CONFIG_GPIO_LOG_LEVEL);

/* Register definitions */
#define REG_INPUT_PORT	 0x00
#define REG_OUTPUT_PORT	 0x01
/* Polarity inversion register is here only for completion, it is not actually used */
#define REG_POL_INV_PORT 0x02
#define REG_CONF_PORT	 0x03

/** Configuration data */
struct gpio_pca9554_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	struct i2c_dt_spec bus;
};

/** Runtime driver data */
struct gpio_pca9554_drv_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;

	/* reg_caches mirror registers values on the the actual PCA9554  */
	struct {
		uint8_t input;
		uint8_t output;
		uint8_t configuration;
	} reg_cache;

	struct k_sem lock;
};

/**
 * @brief Read register and save the read value into buffer.
 *
 * @param[in]  dev	Device struct of the PCA9554.
 * @param[in]  reg	Register to read.
 * @param[out] buf	Buffer to read data into.
 *
 * @return 0 if successful, failed otherwise.
 */
static int read_port_regs(const struct device *dev, uint8_t reg, uint8_t *buf)
{
	const struct gpio_pca9554_config *const config = dev->config;
	int ret;

	ret = i2c_burst_read_dt(&config->bus, reg, buf, sizeof(*buf));
	if (ret) {
		LOG_ERR("ADDR[0x%X]: error reading register 0x%X (%d)", config->bus.addr, reg, ret);
		return ret;
	}

	LOG_DBG("ADDR[0x%X]: read: REG[0x%X] = 0x%X", config->bus.addr, reg, *buf);

	return 0;
}

/**
 * @brief Write a value to a register
 *
 * @param[in]  dev	Device struct of the PCA9554.
 * @param[in]  reg	Register to write into.
 * @param[out] cache	Pointer to cache to be updated with the value after successful write.
 * @param[in]  value	New value to set.
 *
 * @return 0 if successful, failed otherwise.
 */
static int write_port_regs(const struct device *dev, uint8_t reg, uint8_t *cache, uint8_t value)
{
	const struct gpio_pca9554_config *const config = dev->config;

	LOG_DBG("ADDR[0x%X]: write: REG[0x%X] = 0x%02X", config->bus.addr, reg, value);

	uint8_t buf[] = {reg, value};
	int ret = i2c_write_dt(&config->bus, buf, sizeof(buf));

	if (ret) {
		LOG_ERR("ADDR[0x%X]: error writing to register 0x%X (err: %d)", config->bus.addr,
			reg, ret);
		return ret;
	}
	*cache = value;
	return ret;
}

/**
 * @brief Setup the pin direction (input or output)
 *
 * @param[in] dev	Device struct of the PCA9554.
 * @param[in] pin	The pin number.
 * @param[in] flags	Flags of pin or port.
 *
 * @return 0 if successful, failed otherwise.
 */
static int setup_pin_dir(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	struct gpio_pca9554_drv_data *const drv_data = dev->data;

	uint16_t reg_conf = drv_data->reg_cache.configuration;
	uint16_t reg_out = drv_data->reg_cache.output;
	int ret;

	/* For each pin, 0 == output, 1 == input */
	if (flags & GPIO_OUTPUT) {
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			reg_out |= BIT(pin);
		} else if (flags & GPIO_OUTPUT_INIT_LOW) {
			reg_out &= ~BIT(pin);
		}
		reg_conf &= ~BIT(pin);
	} else {
		reg_conf |= BIT(pin);
	}

	ret = write_port_regs(dev, REG_CONF_PORT, &drv_data->reg_cache.configuration, reg_conf);
	if (ret) {
		return ret;
	}
	return write_port_regs(dev, REG_OUTPUT_PORT, &drv_data->reg_cache.output, reg_out);
}

/**
 * @brief Configure pin or port
 *
 * @param[in] dev	Device struct of the PCA9554
 * @param[in] pin	The pin number
 * @param[in] flags	Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_pca9554_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	struct gpio_pca9554_drv_data *const drv_data = dev->data;
	const struct gpio_pca9554_config *const config = dev->config;
	int ret;

	/* Does not support disconnected pin */
	if ((flags & (GPIO_INPUT | GPIO_OUTPUT)) == GPIO_DISCONNECTED) {
		return -ENOTSUP;
	}

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	ret = setup_pin_dir(dev, pin, flags);
	if (ret) {
		LOG_ERR("ADDR[0x%X]: error setting pin direction (%d)", config->bus.addr, ret);
	}

	k_sem_give(&drv_data->lock);
	return ret;
}

/**
 * @brief Configure pin or port
 *
 * @param[in] dev	Device struct of the PCA9554
 * @param[in] pin	The pin number
 * @param[in] flags	Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_pca9554_port_get_raw(const struct device *dev, uint32_t *value)
{
	struct gpio_pca9554_drv_data *const drv_data = dev->data;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	ret = read_port_regs(dev, REG_INPUT_PORT, &drv_data->reg_cache.input);
	if (ret) {
		goto done;
	}

	*value = drv_data->reg_cache.input;

done:
	k_sem_give(&drv_data->lock);
	return ret;
}

static int gpio_pca9554_port_set_masked_raw(const struct device *dev, uint32_t mask, uint32_t value)
{
	struct gpio_pca9554_drv_data *const drv_data = dev->data;
	uint16_t reg_out;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	reg_out = drv_data->reg_cache.output;
	reg_out = (reg_out & ~mask) | (mask & value);

	ret = write_port_regs(dev, REG_OUTPUT_PORT, &drv_data->reg_cache.output, reg_out);

	k_sem_give(&drv_data->lock);

	return ret;
}

static int gpio_pca9554_port_set_bits_raw(const struct device *dev, uint32_t mask)
{
	return gpio_pca9554_port_set_masked_raw(dev, mask, mask);
}

static int gpio_pca9554_port_clear_bits_raw(const struct device *dev, uint32_t mask)
{
	return gpio_pca9554_port_set_masked_raw(dev, mask, 0);
}

static int gpio_pca9554_port_toggle_bits(const struct device *dev, uint32_t mask)
{
	struct gpio_pca9554_drv_data *const drv_data = dev->data;
	uint16_t reg_out;
	int ret;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	reg_out = drv_data->reg_cache.output;
	reg_out ^= mask;

	ret = write_port_regs(dev, REG_OUTPUT_PORT, &drv_data->reg_cache.output, reg_out);

	k_sem_give(&drv_data->lock);

	return ret;
}

static int gpio_pca9554_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
						enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	LOG_ERR("Interrupts are not supported");
	return 0;
}

static const struct gpio_driver_api gpio_pca9554_drv_api_funcs = {
	.pin_configure = gpio_pca9554_configure,
	.port_get_raw = gpio_pca9554_port_get_raw,
	.port_set_masked_raw = gpio_pca9554_port_set_masked_raw,
	.port_set_bits_raw = gpio_pca9554_port_set_bits_raw,
	.port_clear_bits_raw = gpio_pca9554_port_clear_bits_raw,
	.port_toggle_bits = gpio_pca9554_port_toggle_bits,
	.pin_interrupt_configure = gpio_pca9554_pin_interrupt_configure,
};

/**
 * @brief Initialization function of PCA9554
 *
 * @param[in] dev	Device struct.
 *
 * @return 0 if successful, failed otherwise.
 */
static int gpio_pca9554_init(const struct device *dev)
{
	const struct gpio_pca9554_config *const config = dev->config;
	struct gpio_pca9554_drv_data *const drv_data = dev->data;

	if (!device_is_ready(config->bus.bus)) {
		LOG_ERR("I2C device is not ready!");
		return -ENODEV;
	}

	k_sem_init(&drv_data->lock, 1, 1);

	return 0;
}

#define GPIO_PCA9554_DEVICE_INSTANCE(inst)                                                         \
	static const struct gpio_pca9554_config gpio_pca9554_##inst##_cfg = {                      \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(inst),            \
			},                                                                         \
		.bus = I2C_DT_SPEC_INST_GET(inst),                                                 \
	};                                                                                         \
                                                                                                   \
	static struct gpio_pca9554_drv_data gpio_pca9554_##inst##_drvdata = {                      \
		.reg_cache.input = 0xFF,                                                           \
		.reg_cache.output = 0xFF,                                                          \
		.reg_cache.configuration = 0xFF,                                                   \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, gpio_pca9554_init, NULL, &gpio_pca9554_##inst##_drvdata,       \
			      &gpio_pca9554_##inst##_cfg, POST_KERNEL,                             \
			      CONFIG_GPIO_PCA9554_INIT_PRIORITY, &gpio_pca9554_drv_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(GPIO_PCA9554_DEVICE_INSTANCE);
