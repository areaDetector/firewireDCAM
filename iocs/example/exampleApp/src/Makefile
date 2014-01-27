TOP=../..

include $(TOP)/configure/CONFIG


# ------------------------
# Build an IOC Application
# ------------------------

PROD_IOC_Linux = example

# example.dbd will be installed into <top>/dbd
DBD += example.dbd

# example.dbd will be created from these files
example_DBD += commonDriverSupport.dbd
example_DBD += firewireDCAM.dbd

# example_registerRecordDeviceDriver.cpp will be created
example_SRCS += example_registerRecordDeviceDriver.cpp
example_SRCS_DEFAULT += exampleMain.cpp

# Library dependencies
example_LIBS += firewireDCAM

include $(ADCORE)/ADApp/commonDriverMakefile

# ---------------------------------------------------

include $(TOP)/configure/RULES
