#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := bmx280_i2c
EXTRA_COMPONENT_DIRS := /initrd/mnt/dev_save/esp/esp-idf/components_vincent/esp-idf-lib/components
include $(IDF_PATH)/make/project.mk

