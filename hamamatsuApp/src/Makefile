TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#======== DETECTOR LIBRARY ==============

# The following gets rid of the -fno-implicit-templates flag on vxWorks, 
# so we get automatic template instantiation.
# This is what we want for miscellaneous/asynPortDriver.cpp
ifeq (vxWorks,$(findstring vxWorks, $(T_A)))
CODE_CXXFLAGS=
endif

INC += hamamatsu.h

USR_CPPFLAGS += -I$(DCAM_API_DIR)/inc -I$(DCAM_API_DIR)/samples/cpp/misc
USR_CPPFLAGS_Linux += -DLINUX
USR_CPPFLAGS_WIN32 += -DWIN32

LIBRARY_IOC = hamamatsu
LIB_SRCS += hamamatsu.cpp
SRC_DIRS += $(DCAM_API_DIR)/samples/cpp/misc
LIB_SRCS += common.cpp

LIB_LIBS += dcamapi
ifeq ($(OS_CLASS), Linux)
  dcamapi_DIR = /usr/local/lib
  dcamapi_DIR += /usr/local/hamamatsu_dcam/api
endif
ifeq ($(OS_CLASS), WIN32)
  dcamapi_DIR = $(DCAM_API_DIR)/lib/win64
endif

DBD += hamamatsuSupport.dbd


include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
