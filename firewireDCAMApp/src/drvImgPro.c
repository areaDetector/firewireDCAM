/* For exporting the configuration functions to the ioc shell */

#include <iocsh.h>
#include <drvSup.h>
#include <epicsExport.h>

#include "imgPro.h"

/* EPICS iocsh shell commands */

static const iocshArg imgProConfigArg0 = {"Port name", iocshArgString};
static const iocshArg imgProConfigArg1 = {"Frame Queue Size", iocshArgInt};
static const iocshArg imgProConfigArg2 = {"Blocking Callbacks", iocshArgInt};
static const iocshArg imgProConfigArg3 = {"NDArray Port", iocshArgString};
static const iocshArg imgProConfigArg4 = {"NDArray Address", iocshArgInt};
static const iocshArg imgProConfigArg5 = {"Max Buffers", iocshArgInt};
static const iocshArg imgProConfigArg6 = {"Max Memory", iocshArgInt};
static const iocshArg * const imgProConfigArgs[] =  {&imgProConfigArg0,
                                                     &imgProConfigArg1,
                                                     &imgProConfigArg2,
                                                     &imgProConfigArg3,
                                                     &imgProConfigArg4,
                                                     &imgProConfigArg5,
                                                     &imgProConfigArg6};

static const iocshFuncDef imgProFuncDef = {"imgProConfigure", 7, imgProConfigArgs};

static void imgProCallFunc(const iocshArgBuf *args)
{
    imgProConfigure(args[0].sval, args[1].ival, args[2].ival, args[3].sval, args[4].ival, args[5].ival, args[6].ival);
}

/* Register imgPro::imgProConfigure for use on iocsh */

static void imgProRegister(void)
{
    iocshRegister(&imgProFuncDef, imgProCallFunc);
}

epicsExportRegistrar(imgProRegister);
