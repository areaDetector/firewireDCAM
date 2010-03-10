TOP = ../..
include $(TOP)/configure/CONFIG

SCRIPTS+=stexample.boot
SCRIPTS+=stexamplesimple.boot
SCRIPTS+=stexample2cam.boot
SCRIPTS+=stJamesTest.boot
SCRIPTS+=startgui.sh

EXPAND_VARS += AD=$(AREA_DETECTOR)
EXPAND_VARS += ASYN=$(ASYN)
#EXPAND_VARS += FIREWIREDCAM=$(FIREWIREDCAM)
EXPAND_VARS += FIREWIREDCAM=$(shell perl $(TOOLS)/fullPathName.pl $(FIREWIREDCAM))
EXPAND_VARS += TOP=$(shell perl $(TOOLS)/fullPathName.pl $(TOP))
EXPAND += startgui.sh


include $(TOP)/configure/RULES
