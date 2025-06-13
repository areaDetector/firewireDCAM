firewireDCAM
============

EPICS areaDetector driver for firewire IEEE1394 cameras using libdc1394.

This areaDetector driver was previously published and hosted here: http://controls.diamond.ac.uk/downloads/support/firewireDCAM/ up until release 1-8.

See the release notes [here](RELEASE_NOTES.md)

**This repository is no longer active and was archived on June 13, 2025.**

Introduction
============

This module is a Linux firewire (IEEE 1394) camera driver plug-in for another EPICS support module/framework: http://cars9.uchicago.edu/software/epics/areaDetector.html

Please note that for new applications, the author recommends going for the faster, more versatile and often cheaper GigE-enabled cameras. See the <a href="http://controls.diamond.ac.uk/downloads/support/aravisGigE">aravisGigE areaDetector driver.</a>

Firewire IEEE1394 Cameras
=========================
This module uses a set of open source libraries to control the cameras. The main library is <a href="http://damien.douxchamps.net/ieee1394/libdc1394">libdc1394</a>

libdc1394 is able to control firewire cameras that comply with the IIDC DCAM protocol. Please see the libdc1394 <a href="http://damien.douxchamps.net/ieee1394/cameras/">list of cameras</a> and the <a href="http://damien.douxchamps.net/ieee1394/libdc1394/v2.x/faq/">libdc1394 FAQ</a> for more details about which cameras can be used with libdc1394 (note that the developer of this module has only tested with a limited number of camera models: Point Grey Flea2 and AVR Pike)

Digital video recorders with tapes or other local memory typically does not comply with the IIDC DCAM protocol! Cameras that produce compressed images are not supported either. This module does not and is not planned to provide any compression or decompression features. If such feature is needed it is recommended to develop a compression plug-in to the areaDetector framework.

The module supports both 1394A [400Mb/s] and 1394B [800Mb/s] mode cameras.

Where to get it
---------------
Released (tagged) versions of the module can be downloaded as source code tarballs from areaDetector/firewireDCAM repository.

If you want to contribute code to the repository, please use the usual github forking workflow and issue pull requests with your developments.


Build Instructions
==================
These build instructions assume some knowledge of building EPICS modules. It is a requirement that <a href="http://www.aps.anl.gov/epics/base/index.php">EPICS base</a> has already been downloaded and successfully build with all the common environment variables set. 

Dependencies
------------

**EPICS base and modules**

* EPICS base <a href="http://www.aps.anl.gov/epics/base/R3-14/12.php">R3.14.12.3</a>
* <a href="http://www.aps.anl.gov/epics/modules/soft/asyn">asyn</a> 4-21
* <a href="http://cars9.uchicago.edu/software/epics/areaDetector.html">areaDetector</a> 1-9

Optional Dependencies
---------------------
The <a href="http://controls.diamond.ac.uk/downloads/support/ffmpegServer">ffmpegServer</a> module is recommended for new firewire (or any areaDetector) IOCs. This plugin provide mjpeg compression and with the <a href="http://controls.diamond.ac.uk/downloads/support/nullhttpd">nullhttpd</a> module also http streaming for client side image viewing. It also provides a QT/OpenGL based viewer application.

Firewire IIDC DCAM libraries
----------------------------
The only required Linux firewire camera support library is now: <a href="http://sourceforge.net/project/showfiles.php?group_id=8157">libdc1394</a> 2.2.1
libraw1394 is no longer required.

Kernel Modules
--------------
The libdc1394 libraries depend on a set of drivers/kernel modules that will need to be loaded. There are basically two options when selecting the firewire kernel modules: the traditional ohci1394 (and friends) or the newer 'Juju' stack. Most distributions now ship with the Juju firewire stack. The Juju stack removes the requirement to use the raw1394 kernel module.

When using the Juju firewire stack the following command can inform if the correct kernel modules have been loaded:
<pre>
/sbin/lsmod | grep firewire
firewire_ohci          24695  0 
firewire_core          50151  1 firewire_ohci
crc_itu_t               1717  1 firewire_core
</pre>

Device Permissions
------------------
Once the kernel modules are loaded you must ensure that the user that will run the IOC application has read+write permissions on the necessary device files. The files in question can be checked for permissions with the following commands:
<pre>
ls -l /dev/fw*
crw-rw-rw-. 1 root root  248, 0 Apr 18 13:24 /dev/fw0
crw-rw-rw-. 1 root video 248, 1 Apr 18 13:24 /dev/fw1
crw-rw-rw-. 1 root root  248, 2 Apr 18 13:24 /dev/fw2
</pre>


Building the 1394 libraries
---------------------------
Please check the INSTALL or README files provided with the 1394 libraries for installation notes. Libraw1394 need to be build before libdc1394 to get the dependencies right.

The basic build and installation is fairly simple (the last step possibly as root, depending on the installation location):
<pre>
./configure
make
make install
</pre>


User Manual
===========
Diagnostic command line utility
-------------------------------
A small commandline application is provided with the firewireDCAM module. This application can be used to diagnose connection issues with a bus of many cameras - and report information about the available modes of operation for each camera found on the bus. Finally it can be used to send a reset command to each camera.

The tool comes with online help which explain the options:
<pre>
bin/linux-x86_64/firewiretool --help

./bin/linux-x86_64/firewiretool - probe firewire camera for various parameters

Usage:
./bin/linux-x86_64/firewiretool [--verbose] [--camid=0xnnnnnnnnnnnn] [--reset] [--help]
    [--report[=level]]
    --verbose: Switch on verbose print out.
    --camid:   Search for and use a camera on the bus
               with the particular ID (12 digit hex value)
    --reset:   Reset the firewire bus using the first camera
               found on each chain (generation) of the bus
    --report   Request a report of the found cameras. Will be
               limited to one camera if --camid has been used.
               The optional 'level' specifies what type of report
    --help:    This help message
</pre>

The report mode provide extensive output of all available modes:
<pre>
bin/linux-x86_64/firewiretool --report 
Enumerated 1 camera(s) on bus
_________________________________________________________________________________________
               GUID |             Vendor |                    Model |    Node |    Gen. |
_________________________________________________________________________________________
 0x00B09D01008118C9 |Point Grey Research |          Flea2 FL2-08S2C |       0 |       8 |
_________________________________________________________________________________________

    Supported modes for camera: 0x00B09D01008118C9      Flea2 FL2-08S2C
________________________________________________________________________________________
 mode |    size WxH | scalable | color coding (id) | framerates (fps)                  |
________________________________________________________________________________________
   64 |  160 x  120 |     NO   |     YUV444 ( 355) |  7.50  15.00  30.00               |
   65 |  320 x  240 |     NO   |     YUV422 ( 354) |  1.88   3.75   7.50  15.00  30.00 |
   66 |  640 x  480 |     NO   |     YUV411 ( 353) |  1.88   3.75   7.50  15.00  30.00 |
   67 |  640 x  480 |     NO   |     YUV422 ( 354) |  1.88   3.75   7.50  15.00  30.00 |
   68 |  640 x  480 |     NO   |       RGB8 ( 356) |  1.88   3.75   7.50  15.00  30.00 |
   69 |  640 x  480 |     NO   |      MONO8 ( 352) |  1.88   3.75   7.50  15.00  30.00 |
   70 |  640 x  480 |     NO   |     MONO16 ( 357) |  1.88   3.75   7.50  15.00  30.00 |
   71 |  800 x  600 |     NO   |     YUV422 ( 354) |  3.75   7.50  15.00  30.00        |
   72 |  800 x  600 |     NO   |       RGB8 ( 356) |  7.50  15.00  30.00               |
   73 |  800 x  600 |     NO   |      MONO8 ( 352) |  7.50  15.00  30.00               |
   74 | 1024 x  768 |     NO   |     YUV422 ( 354) |  1.88   3.75   7.50  15.00  30.00 |
   75 | 1024 x  768 |     NO   |       RGB8 ( 356) |  1.88   3.75   7.50  15.00        |
   76 | 1024 x  768 |     NO   |      MONO8 ( 352) |  1.88   3.75   7.50  15.00  30.00 |
   77 |  800 x  600 |     NO   |     MONO16 ( 357) |  3.75   7.50  15.00  30.00        |
   78 | 1024 x  768 |     NO   |     MONO16 ( 357) |  1.88   3.75   7.50  15.00  30.00 |
   88 | 1032 x  776 |    YES   | MONO8 YUV411 YUV422 YUV444 RGB8 MONO16 RAW8 RAW16     |
   89 |  516 x  388 |    YES   | MONO8 YUV411 YUV422 YUV444 RGB8 MONO16                |
   90 | 1032 x  388 |    YES   | MONO8 YUV411 YUV422 YUV444 RGB8 MONO16                |
________________________________________________________________________________________
</pre>


EPICS areaDetector framework
============================
The firewireDCAM module is a plug-in to the areaDetector framework. It is recommended to get familiar with the framework when starting to use this plug-in. Excellent documentation can be found on the <a href="http://cars9.uchicago.edu/software/epics/areaDetector.html">areaDetector documentation</a> page.

AreaDetector provides a number of ready-made templates, MEDM screens and plug-ins that can be used for general control and data readout. These are all fairly generic to 2D detectors. 

Note as this is a plug-in module to a larger framework, it is not considered an end-user application. A number of things will need to be done to enable an end-user to operate cameras:
* An IOC binary need to be build with a startup script and possibly substitutions files to instantiate all the necessary or desired templates.
* Custom templates or databases may need to be developed to suit specific controls requirement in your application.
* End user client display of the images is not included in this package. The data can be streamed to disk using the <a href="http://cars9.uchicago.edu/software/epics/NDPluginFile.html">NDPluginFile plug-in</a> or grabbed by any Channel Access client through the PVs available in the <a href="http://cars9.uchicago.edu/software/epics/NDPluginStdArrays.html">NDPluginStdArrays</a>

The following sections describes the templates and configuration functions this plug-in module provides and how to use them.

Database Template
-----------------
One database template is supplied to enable control and read back of all camera features. See firewireDCAM.template for details regarding macros and what features and records are available.

Startup Script
--------------
A couple of startup scripts are supplied in the example/iocBoot/iocexample/ directory. These startup scripts have a number of macros that will need to be substituted before running them. The EPICS build system at DLS will automate this and place the substituted startup files in example/bin/linux-x86/stexample.boot but other sites may possibly not have the same build system rules...

Configuration Functions
-----------------------
A number of areaDetector plug-ins are used in the example startup scripts. Please see the relevant documentation from areaDetector and modify according to your needs. The plug-ins configuration function documentation can be found on the areaDetector website: <a href="http://cars9.uchicago.edu/software/epics/NDPluginStdArrays.html#Configuration">NDPluginStdArrays</a> and <a href="http://cars9.uchicago.edu/software/epics/NDPluginROI.html#Configuration">NDPluginROI</a>

In order to initialise the firewire bus and each individual camera, two configuration functions need to be called during startup to initialise the module: 
* FDC_InitBus(): This need to be called only once to initialise the bus before FDC_Config can be called. This function does not need any arguments.
* FDC_Config(): Initialise the individual camera. 
 * Arguments: Asyn port name, camera ID (hex number), bus speed (800/400), Number of frame buffers Max memory, color mode (0 or 1)
