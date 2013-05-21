TOP = ../..
include $(TOP)/configure/CONFIG

# set to -d to get debugging
DEBUG :=
#XMLBUILDER=dls-xml-iocbuilder.py
XMLBUILDER=/dls_sw/work/common/python/iocbuilder/xmlbuilder/xmlbuilder.py
MAKEDOC=dls-make-doxygen-documentation.py
#MAKEDOC=/dls_sw/work/common/python/dls_scripts/py_scripts/dls-make-doxygen-documentation.py

# this is the doxygen output dir
DOCDIR := $(TOP)/documentation/doxygen

# add the documentation files to the install target
install: $(DOCDIR)

# rule for creating the doxygen documentation
$(DOCDIR): config.src manual.src $(DOCDIR)/build_instructions_example
	mkdir -p $(DOCDIR)
	$(MAKEDOC) -o $(DOCDIR) config.src manual.src

# rule for generating build instructions from an xml file
$(DOCDIR)/build_instructions_%: $(TOP)/etc/makeIocs/%.xml
	mkdir -p $(DOCDIR)
	$(XMLBUILDER) --doc=$@ $(DEBUG) $^

# rule for generating build instructions from a py file
$(DOCDIR)/build_instructions_%: $(TOP)/etc/makeIocs/%.py
	mkdir -p $(DOCDIR)
	$^ --doc=$@ $(DEBUG) example

# don't do anything in clean
clean: ;

# Special rule to get rid of installed documentation dir
uninstall:
	$(RMDIR) $(DOCDIR)
