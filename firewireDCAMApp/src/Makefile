TOP=../..

include $(TOP)/configure/CONFIG

#--------------------------------------------------------
# Build a little tool to do various firewire bus checks
#--------------------------------------------------------
PROD_IOC += firewiretool
firewiretool_SRCS_DEFAULT += firewiretool.cpp
firewiretool_LIBS += dc1394
firewiretool_LIBS += $(EPICS_BASE_IOC_LIBS)

# -------------------------------
# Build a Diamond Support Module
# -------------------------------

LIBRARY_IOC += firewireDCAM

# xxxRecord.dbd will be installed into <top>/dbd
DBD += firewireDCAM.dbd
DBD += firewireDCAM_vdct.dbd
firewireDCAM_vdct_DBD += base.dbd
firewireDCAM_vdct_DBD += asyn.dbd

# This seems to be necessary to build against the dc1394 libraries...
# - if anyone knows how to build the dc1394/raw1394 libraries in a 
#   way that we can get rid of this statement, please let me know
#   (Ulrik Pedersen, DLS Nov 2008)
USR_CFLAGS_Linux += -std=gnu89

# The following are compiled and added to the support library
firewireDCAM_SRCS += drvFirewireDCAM.c
firewireDCAM_SRCS += firewireDCAM.cpp

# This line says that this IOC Application depends on the
# xxx Support Module
firewireDCAM_LIBS += asyn
firewireDCAM_LIBS += NDPlugin ADBase
#firewireDCAM_LIBS += calc sscan busy
firewireDCAM_LIBS += dc1394 raw1394

# We don't really need to link against the EPICS Base libraries
# when we are just compiling a support module. We just have to 
# remember to do that linking when we build the IOC! (but let's 
# do that in a seperate module to keep things clean and tidy)
#firewireDCAM_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
