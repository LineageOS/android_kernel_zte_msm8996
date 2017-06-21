#
# Makefile for the Synaptics DSX touchscreen driver.
#

# Each configuration option enables a list of files.

ifeq ($(ZTE_TS_FIRMWARE_UPDATE_IMPROVE), true)
ccflags-y += -DZTE_TS_FIRMWARE_UPDATE_IMPROVE
endif

obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C_ZTE) += synaptics_dsx_i2c.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_CORE_ZTE) += synaptics_dsx_core.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_RMI_DEV_ZTE) += synaptics_dsx_rmi_dev.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_FW_UPDATE_ZTE) += synaptics_dsx_fw_update.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_TEST_REPORTING_ZTE) += synaptics_dsx_test_reporting.o
