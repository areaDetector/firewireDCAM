TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
# ADD MACRO DEFINITIONS AFTER THIS LINE

dc1394_DIR=$(DC1394)
USR_CFLAGS_Linux += -std=gnu89

#--------------------------------------------------------
# Build a little tool to do various firewire bus checks
#--------------------------------------------------------
PROD_IOC_Linux += firewiretool
firewiretool_SRCS_DEFAULT += firewiretool.cpp
firewiretool_SYS_LIBS += dc1394
firewiretool_LIBS += $(EPICS_BASE_IOC_LIBS)

#PROD_IOC += testCams
#testCams_SRCS_DEFAULT += testCams.cpp
#testCams_LIBS += dc1394
#testCams_LIBS += $(EPICS_BASE_IOC_LIBS)

# -------------------------------
# Build the Support Module
# -------------------------------
LIBRARY_IOC_Linux += firewireDCAM
DBD += firewireDCAM.dbd

# This is to make the VDCT flatdb script happy
# VDCT/flatdb is used to expand the 'feature' template multiple times
# in the main firewireDCAM.template
DBD += firewireDCAM_vdct.dbd
firewireDCAM_vdct_DBD += base.dbd
firewireDCAM_vdct_DBD += asyn.dbd
firewireDCAM_vdct_DBD += NDPluginSupport.dbd

LIB_SRCS += drvFirewireDCAM.c
LIB_SRCS += firewireDCAM.cpp

#include $(ADCORE)/ADApp/commonLibraryMakefile
# (ulrik:) I think the commonLibraryMakefile include much more
# than it really need to, causing some artificial dependencies
# that may cause trouble later. Or maybe I've misunderstood something...
# In my case the following two lines are all that is required:
LIB_LIBS += ADBase
LIB_LIBS += asyn
LIB_LIBS += dc1394

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
# ADD RULES AFTER THIS LINE
