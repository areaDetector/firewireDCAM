TOP=../..

include $(TOP)/configure/CONFIG


# ------------------------
# Build an IOC Application
# ------------------------

PROD_IOC = example

# example.dbd will be installed into <top>/dbd
DBD += example.dbd

# example.dbd will be created from these files
example_DBD += base.dbd
example_DBD += asyn.dbd
example_DBD += firewireDCAM.dbd
example_DBD += ADSupport.dbd
example_DBD += pluginSupport.dbd
example_DBD += calc.dbd
example_DBD += sscan.dbd


# example_registerRecordDeviceDriver.cpp will be created
# example.dbd
example_SRCS += example_registerRecordDeviceDriver.cpp

# These two lines are needed for non-vxWorks builds, such as Linux
example_SRCS_DEFAULT += exampleMain.cpp
example_SRCS_vxWorks += -nil-

# Add locally compiled object code
#example_SRCS +=

# The following adds object code from base/src/vxWorks
example_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Point to where we can find the libdc1394 library files and 
# include files. 
# Note: this is not necessary if we have build dc1394 to look
# like an EPICS module with lib/linux-x86/ and include/ directories...
#dc1394_DIR = /where/libdc1394/is/installed/lib
#USR_LIBS_Linux += dc1394
#USR_INCLUDES_Linux += /where/libdc1394/is/installed/includes

# This line says that this IOC Application depends on the
# xxx Support Module
example_LIBS += asyn
example_LIBS += firewireDCAM
example_LIBS += ADPlugin ADBase
example_LIBS += calc sscan
# Comment next line out if libdc1394 is *not* build as an EPICS module
example_LIBS += dc1394  

# We need to link this IOC Application against the EPICS Base libraries
example_LIBS += $(EPICS_BASE_IOC_LIBS)

# ---------------------------------------------------

include $(TOP)/configure/RULES
