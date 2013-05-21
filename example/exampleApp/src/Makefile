TOP=../..

include $(TOP)/configure/CONFIG


# ------------------------
# Build an IOC Application
# ------------------------

PROD_IOC = example

# example.dbd will be installed into <top>/dbd
DBD += example.dbd

# example.dbd will be created from these files
example_DBD += commonDriverSupport.dbd
example_DBD += firewireDCAM.dbd

# example_registerRecordDeviceDriver.cpp will be created
example_SRCS += example_registerRecordDeviceDriver.cpp

# These two lines are needed for non-vxWorks builds, such as Linux
example_SRCS_DEFAULT += exampleMain.cpp
example_SRCS_vxWorks += -nil-

# The following adds object code from base/src/vxWorks
example_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# This line says that this IOC Application depends on the
# xxx Support Module
example_LIBS += firewireDCAM

# We need to link this IOC Application against the EPICS Base libraries
example_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(AREADETECTOR)/ADApp/commonDriverMakefile

# ---------------------------------------------------

include $(TOP)/configure/RULES
