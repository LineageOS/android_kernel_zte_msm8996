# refer to build/core/product_config.mk for INTERNAL_VALID_VARIANTS := user userdebug eng
#ifeq ($(TARGET_BUILD_VARIANT),user)
#$(info zte_dload_user: $(TARGET_BUILD_VARIANT))
#ccflags-y	+=	-DZTE_RESTART_FLAG_USER_BUILD
#else ifeq ($(TARGET_BUILD_VARIANT),userdebug)
#$(info zte_dload_userdebug: $(TARGET_BUILD_VARIANT))
#ccflags-y	+=	-DZTE_RESTART_FLAG_USERDEBUG_BUILD
#else ifeq ($(TARGET_BUILD_VARIANT),eng)
#$(info zte_dload_eng: $(TARGET_BUILD_VARIANT))
#ccflags-y	+=	-DZTE_RESTART_FLAG_ENG_BUILD
#else
# zte_restart_unexpected_TARGET_BUILD_VARIANT, just ignore or skip here
#endif

obj-$(CONFIG_POWER_RESET_AS3722) += as3722-poweroff.o
obj-$(CONFIG_POWER_RESET_AT91_POWEROFF) += at91-poweroff.o
obj-$(CONFIG_POWER_RESET_AT91_RESET) += at91-reset.o
obj-$(CONFIG_POWER_RESET_AXXIA) += axxia-reset.o
obj-$(CONFIG_POWER_RESET_BRCMSTB) += brcmstb-reboot.o
obj-$(CONFIG_POWER_RESET_GPIO) += gpio-poweroff.o
obj-$(CONFIG_POWER_RESET_GPIO_RESTART) += gpio-restart.o
obj-$(CONFIG_POWER_RESET_HISI) += hisi-reboot.o
obj-$(CONFIG_POWER_RESET_MSM) += msm-poweroff.o
obj-$(CONFIG_POWER_RESET_LTC2952) += ltc2952-poweroff.o
obj-$(CONFIG_POWER_RESET_QNAP) += qnap-poweroff.o
obj-$(CONFIG_POWER_RESET_RESTART) += restart-poweroff.o
obj-$(CONFIG_POWER_RESET_SUN6I) += sun6i-reboot.o
obj-$(CONFIG_POWER_RESET_ST) += st-poweroff.o
obj-$(CONFIG_POWER_RESET_VERSATILE) += arm-versatile-reboot.o
obj-$(CONFIG_POWER_RESET_VEXPRESS) += vexpress-poweroff.o
obj-$(CONFIG_POWER_RESET_XGENE) += xgene-reboot.o
obj-$(CONFIG_POWER_RESET_KEYSTONE) += keystone-reset.o
obj-$(CONFIG_POWER_RESET_SYSCON) += syscon-reboot.o
