TOP = ../..
include $(TOP)/configure/CONFIG

SCRIPTS+=stexample.boot
SCRIPTS+=stexample2cam.boot
SCRIPTS+=startgui.sh
SCRIPTS += envPaths

EXPAND_VARS += AD=$(AREA_DETECTOR)
EXPAND_VARS += ASYN=$(ASYN)
#EXPAND_VARS += FIREWIREDCAM=$(FIREWIREDCAM)
EXPAND_VARS += FIREWIREDCAM=$(shell perl $(TOOLS)/fullPathName.pl $(FIREWIREDCAM))
EXPAND_VARS += TOP=$(shell perl $(TOOLS)/fullPathName.pl $(TOP))
EXPAND_VARS += ARCH=$(T_A)
EXPAND += startgui.sh
EXPAND += stexample.boot
EXPAND += stexample2cam.boot


include $(TOP)/configure/RULES


envPaths:
	$(shell $(PERL) $(EPICS_BASE_HOST_BIN)/convertRelease.pl -a $(T_A) -T$(TOP) envPaths)
