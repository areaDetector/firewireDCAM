/** \file drvFirewireDCAM.c
 * \brief Exporting the configuration functions to the ioc shell.
 */

#include <iocsh.h>
#include <drvSup.h>
#include <epicsExport.h>

#include "drvFirewireDCAM.h"

static const iocshArg firewireDCAMConfigArg0 = {"Port name", iocshArgString};
static const iocshArg firewireDCAMConfigArg1 = {"ID", iocshArgString};
static const iocshArg firewireDCAMConfigArg2 = {"Bus speed", iocshArgInt};
static const iocshArg firewireDCAMConfigArg3 = {"maxBuffers", iocshArgInt};
static const iocshArg firewireDCAMConfigArg4 = {"maxMemory", iocshArgInt};
static const iocshArg firewireDCAMConfigArg5 = {"disableScalable", iocshArgInt};
static const iocshArg * const firewireDCAMConfigArgs[] =  {&firewireDCAMConfigArg0,
                                                          &firewireDCAMConfigArg1,
                                                          &firewireDCAMConfigArg2,
                                                          &firewireDCAMConfigArg3,
                                                          &firewireDCAMConfigArg4,
                                                          &firewireDCAMConfigArg5};
static const iocshFuncDef configFirewireDCAM = {"FDC_Config", 6, firewireDCAMConfigArgs};
static void configfirewireDCAMCallFunc(const iocshArgBuf *args)
{
    FDC_Config(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival);
}


static const iocshFuncDef firewireDCAMInitBusFuncDef = {"FDC_InitBus", 0, NULL};
static void firewireDCAMInitBusCallFunc(const iocshArgBuf *args)
{
	FDC_InitBus();
}

static const iocshFuncDef firewireDCAMResetBusFuncDef = {"FDC_ResetBus", 0, NULL};
static void firewireDCAMResetBusCallFunc(const iocshArgBuf *args)
{
	FDC_ResetBus();
}


static void firewireDCAMRegister(void)
{

    iocshRegister(&configFirewireDCAM, configfirewireDCAMCallFunc);
    iocshRegister(&firewireDCAMInitBusFuncDef, firewireDCAMInitBusCallFunc);
    iocshRegister(&firewireDCAMResetBusFuncDef, firewireDCAMResetBusCallFunc);
}

epicsExportRegistrar(firewireDCAMRegister);
