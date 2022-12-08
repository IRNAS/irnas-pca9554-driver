# COPYRIGHT NOTICE: (c) 2022 Irnas. All rights reserved.
#
# Author: Marko Sagadin <marko@irnas.eu>

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_GPIO_PCA9554 gpio_pca9554.c)
zephyr_library_include_directories($ENV{ZEPHYR_BASE}/drivers/gpio)
