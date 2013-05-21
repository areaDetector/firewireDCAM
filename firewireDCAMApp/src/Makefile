TOP=../..

include $(TOP)/configure/CONFIG

dc1394_DIR=$(DC1394)/lib
USR_CFLAGS_Linux += -std=gnu89

#--------------------------------------------------------
# Build a little tool to do various firewire bus checks
#--------------------------------------------------------
PROD_IOC += firewiretool
firewiretool_SRCS_DEFAULT += firewiretool.cpp
firewiretool_LIBS += dc1394
firewiretool_LIBS += $(EPICS_BASE_IOC_LIBS)

#PROD_IOC += testCams
#testCams_SRCS_DEFAULT += testCams.cpp
#testCams_LIBS += dc1394
#testCams_LIBS += $(EPICS_BASE_IOC_LIBS)


# -------------------------------
# Build a Diamond Support Module
# -------------------------------

LIBRARY_IOC += firewireDCAM

# xxxRecord.dbd will be installed into <top>/dbd
DBD += firewireDCAM.dbd
DBD += firewireDCAM_vdct.dbd
firewireDCAM_vdct_DBD += base.dbd
firewireDCAM_vdct_DBD += asyn.dbd
firewireDCAM_vdct_DBD += NDPluginSupport.dbd

# The following are compiled and added to the support library
firewireDCAM_SRCS += drvFirewireDCAM.c
firewireDCAM_SRCS += firewireDCAM.cpp

# linking against various basic support libraries
firewireDCAM_LIBS += asyn
firewireDCAM_LIBS += ADBase netCDF
firewireDCAM_LIBS += dc1394
firewireDCAM_LIBS += $(EPICS_BASE_IOC_LIBS)

#include $(AREA_DETECTOR)/ADApp/commonDriverMakefile

include $(TOP)/configure/RULES
