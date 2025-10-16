################################################################################
#
# ros2_zenoh_c
#
################################################################################

ROS2_ZENOH_C_VERSION = 0.1.0
ROS2_ZENOH_C_SITE = $(call github,your-repo,ros2_zenoh_c,$(ROS2_ZENOH_C_VERSION))
ROS2_ZENOH_C_LICENSE = Apache-2.0
ROS2_ZENOH_C_LICENSE_FILES = LICENSE

ROS2_ZENOH_C_DEPENDENCIES = 

ifeq ($(BR2_PACKAGE_ROS2_ZENOH_C_ZENOH),y)
ROS2_ZENOH_C_DEPENDENCIES += zenoh-c
ROS2_ZENOH_C_MAKE_OPTS += ZENOH_AVAILABLE=1
endif

ifeq ($(BR2_PACKAGE_ROS2_ZENOH_C_MICROCDR),y)
ROS2_ZENOH_C_DEPENDENCIES += micro-cdr
ROS2_ZENOH_C_MAKE_OPTS += MICROCDR_AVAILABLE=1
endif

define ROS2_ZENOH_C_BUILD_CMDS
	$(MAKE) $(TARGET_CONFIGURE_OPTS) -C $(@D) \
		$(ROS2_ZENOH_C_MAKE_OPTS) all
endef

define ROS2_ZENOH_C_INSTALL_TARGET_CMDS
	$(MAKE) -C $(@D) DESTDIR=$(TARGET_DIR) install
endef

define ROS2_ZENOH_C_INSTALL_STAGING_CMDS
	$(MAKE) -C $(@D) DESTDIR=$(STAGING_DIR) install
endef

$(eval $(generic-package))
