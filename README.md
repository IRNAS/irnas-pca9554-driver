# PCA9554 8-bit I2C-bus and SMBus I/O port driver

This is a Zephyr driver for NXP's PCA9554/PCA9554A 8-bit I2C-bus and SMBus I/O
port chip.

It can be used with Zephyr's GPIO API.

**IMPORTANT**: This driver was tested with `nrf-sdk v2.1.0.`

**IMPORTANT**: This driver does not yet support interrupts, using functions like
`gpio_pin_interrupt_configure` will not do anything.

## Support

This driver supports both PCA9554 and PCA9554A devices. The PCA9554A is
identical to the PCA9554 except that the fixed I2C-bus address is different
allowing up to sixteen of these devices (eight of each) on the same
I2C-bus/SMBus.

## Usage

To use add below two snippets into your project's west.yml file and run
`west update`:

1. In `remotes` section, if not already added:

```yaml
- name: irnas
  url-base: https://github.com/irnas
```

2. In the `projects` section, select revision you need:

```yaml
- name: irnas-pca9554-driver
    repo-path: irnas-pca9554-driver
    path: irnas/irnas-pca9554-driver
    remote: irnas
    revision: v1.0.0
```

### Device tree

To enable this driver you need to add below snippet into your DTS or overlay
file.

```yaml
&i2c0 {
        pca9554: pca9554@27 {
        compatible = "irnas,pca9554";
        label = "pca9554";
        reg = <0x27>;
        gpio-controller;
        #gpio-cells = <2>;
        ngpios=<8>;
    };
};
```

### KConfig

See [KConfig.pca9554](./drivers/gpio/Kconfig.pca9554) for possible configuration
options.
