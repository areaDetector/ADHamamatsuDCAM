TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#STATIC_BUILD = NO

PROD_NAME = hamamatsuApp
PROD_IOC = $(PROD_NAME)

# <name>.dbd will be created from <name>Include.dbd
DBD += $(PROD_NAME).dbd
$(PROD_NAME)_DBD += hamamatsuSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
PROD_SRCS_DEFAULT += $(PROD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp

PROD_LIBS += hamamatsu
ifeq ($(OS_CLASS), Linux)
  PROD_SYS_LIBS += dcamapi
  dcamapi_DIR = /usr/local/lib
endif
ifeq ($(OS_CLASS), WIN32)
  PROD_LIBS += dcamapi
  dcamapi_DIR = C:/Hamamatsu/dcamsdk4/lib/win64
endif

include $(ADCORE)/ADApp/commonDriverMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
