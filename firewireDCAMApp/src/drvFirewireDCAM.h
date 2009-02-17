/** \file drvFirewireDCAM.h
 * \brief Header file for the firewireDCAM module. The only functions this module exports
 * are the FDC_InitBus() and FDC_Config() to enable use of them in the IOC shell.
 */

#ifndef DRV_FIREWIREDCAM_H
#define DRV_FIREWIREDCAM_H

#ifdef __cplusplus
extern "C" {
#endif

int FDC_InitBus(void);
int FDC_Config(const char *portName, const char* camid, int speed, int maxBuffers, size_t maxMemory, int colour);

#ifdef __cplusplus
}
#endif
#endif
