/*
 * Author: Ulrik Pedersen,
 *         Diamond Light Source, Copyright 2008
 *
 * License: This file is part of 'firewireDCAM'
 *
 * 'firewireDCAM' is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * 'firewireDCAM' is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with 'firewireDCAM'.  If not, see <http://www.gnu.org/licenses/>.
 */

/** \file firewireDCAM.cpp
 * \brief This is areaDetector plug-in support for firewire cameras that comply
 *  with the IIDC DCAM protocol. This implements the \ref FirewireDCAM class which
 *  inherits from the areaDetector ADDriver class.
 *
 *  The driver uses the Linux libraries dc1394 and raw1394.
 *
 *  Author:  Ulrik Kofoed Pedersen
 *           Diamond Light Source Ltd, UK.
 *  Created: November 2008
 *
 */

/* Standard includes... */
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* EPICS includes */
#include <epicsString.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsMutex.h>

/* Dependency support modules includes:
 * asyn, areaDetector, dc1394 */
#include <ADDriver.h>

/* libdc1394 includes */
#include <dc1394/dc1394.h>
/* Needed to set trigger mode, lives in offsets.h */
#define REG_CAMERA_TRIGGER_MODE             0x830U

/** Print an errorcode to stderr. Convenience macro to be used when an asynUser is not yet available. */
#define ERR(errCode) if (errCode != 0) fprintf(stderr, "ERROR [%s:%d]: dc1394 code: %d\n", __FILE__, __LINE__, errCode)
/** Convenience macro to be used inside the firewireDCAM class. */
#define PERR(pasynUser, errCode) this->err(pasynUser, errCode, __LINE__)
/** Number of image buffers the dc1394 library will use internally */
#define FDC_DC1394_NUM_BUFFERS 5

/** Only used for debugging/error messages to identify where the message comes from*/
static const char *driverName = "FirewireDCAM";
/** dc1394 handle to the firewire bus.  */
static dc1394_t * dc1394fwbus = NULL;
/** List of dc1394 camera handles. */
static dc1394camera_list_t * dc1394camList = NULL;
void reset_bus();
/** Global bus lock to ensure dc1394_capture_setup and dc1394_video_set_transmission in startCapture are done not interrupted */
static epicsMutexId setupLock = epicsMutexCreate();

/** Main driver class inherited from areaDetectors ADDriver class.
 * One instance of this class will control one firewire camera on the bus.
 */
class FirewireDCAM : public ADDriver
{
public:
	FirewireDCAM(	const char *portName, const char* camid, int speed,
					int maxBuffers, size_t maxMemory, int disableScalable );

	/* virtual methods to override from ADDriver */
	virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value);
	virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value);
	void report(FILE *fp, int details);

	/* Local methods to this class */
	asynStatus err( asynUser* asynUser, dc1394error_t dc1394_err, int errOriginLine);
	void imageGrabTask();
	int decodeFrame(dc1394video_frame_t * dc1394_frame);
	asynStatus startCapture(asynUser *pasynUser);
	asynStatus stopCapture(asynUser *pasynUser);
	asynStatus stopCaptureAndWait(asynUser *pasynUser);

	/* camera feature control functions */
	asynStatus initCamera(unsigned long long int camUID);
	asynStatus setFeatureValue(asynUser *pasynUser, int addr, epicsInt32 value, epicsInt32 *rbValue);
	asynStatus setFeatureAbsValue(asynUser *pasynUser, int addr, epicsFloat64 value, epicsFloat64 *rbValue);
	asynStatus setFeatureMode(asynUser *pasynUser, int addr, epicsInt32 value, epicsInt32 *rbValue);
	asynStatus checkFeature(asynUser *pasynUser, int addr, dc1394feature_info_t **featInfo, char** featureName, const char* functionName);
	asynStatus setFrameRate( asynUser *pasynUser, epicsInt32 iframerate);
	asynStatus setFrameRate( asynUser *pasynUser, epicsFloat64 dframerate);
	int getAllFeatures();

	asynStatus lookupColorMode(dc1394color_coding_t colorCoding, int *colorMode, int *dataType);
	dc1394color_coding_t lookupColorCoding(int colorMode, int dataType);
	dc1394video_mode_t lookupVideoMode(dc1394color_coding_t colorCoding);
	asynStatus setVideoMode(asynUser* pasynUser);
	asynStatus setRoi(asynUser *pasynUser);
	int setAcquireParam(int acquire);
	dc1394error_t captureDequeueTimeout(dc1394camera_t *camera, dc1394video_frame_t **frame, epicsFloat64 timeout);

	/* Data */
	NDArray *pRaw;
	dc1394camera_t *camera;
    epicsEventId startEventId;
    epicsEventId stopEventId;
    dc1394featureset_t features;
    int disableScalable;
    int busSpeed;
    unsigned long long camguid;

protected:
    int FDC_feat_val;                       /** Feature value (int32 read/write) addr: 0-17 */
    #define FIRST_FDC_PARAM FDC_feat_val
    int FDC_feat_val_max;                  /** Feature maximum boundry value (int32 read) addr: 0-17 */
    int FDC_feat_val_min;                  /** Feature minimum boundry value (int32 read)  addr: 0-17*/
    int FDC_feat_val_abs;                  /** Feature absolute value (float64 read/write) addr: 0-17 */
    int FDC_feat_val_abs_max;              /** Feature absolute maximum boundry value (float64 read) addr: 0-17 */
    int FDC_feat_val_abs_min;              /** Feature absolute minimum boundry value (float64 read) addr: 0-17 */
    int FDC_feat_mode;                     /** Feature control mode: 0:manual or 1:automatic (camera controlled) (int32 read/write)*/
    int FDC_feat_available;                /** Is a given featurea available in the camera 1=available 0=not available (int32, read) */
    int FDC_feat_absolute;                 /** Feature has absolute (floating point) controls available 1=available 0=not available (int32 read) */
	int FDC_framerate;                     /** Set and read back the frame rate (int32 (enums) read/write)*/
	int FDC_videomode;                     /** Set and read back the video mode (int32 (enums) read/write)*/
    int FDC_bandwidth;                     /** Read back the current bandwidth (int32, read)*/
    #define LAST_FDC_PARAM FDC_bandwidth

};
/* end of FirewireDCAM class description */



typedef struct camList_t {
	ELLNODE node;
	FirewireDCAM *fdc;
	dc1394camera_t *cam;
}camList_t;

/** A singleton class that keep a list of the (configured) cameras on the bus.
 *
 */
class BusManager
{
public:
	static BusManager* getInstance();
	void addDriverInstance(FirewireDCAM* fdc);
	void resetBus();
private:
	BusManager();
	static BusManager* instance;
	ELLLIST bus;
};
BusManager* BusManager::instance = NULL;


typedef struct camNode_t {
	ELLNODE node;
	uint32_t generation;
	uint64_t *newguid;
	dc1394camera_t *cam;
}camNode_t;

void reset_bus()
{
	dc1394_t * d;
	dc1394camera_list_t * list;
	dc1394camera_t *cam = NULL;
    uint32_t generation, latch=0;
    uint32_t node;
    ELLLIST camList;
    camNode_t *camListItem, *tmp;
    unsigned int i, newBus;

    d = dc1394_new ();
	ERR( dc1394_camera_enumerate (d, &list) );
	printf("Found %d cameras\n", list->num);

	// To reset a multi-bus system it is necessary to find a camera on each
	// individual bus and call the reset function with that camera.
	ellInit(&camList);

	// Get the 'generation' parameter for each camera. This parameter indicate
	// which bus the camera is located on.
	// For each specific 'generation' we add the camera handle to a list which
	// we can later use to reset each bus.
	for (i=0;i<list->num; i++)
	{
		printf("cam ID: 0x%16.16llX", list->ids[i].guid);
		fflush(stdout);
		cam = dc1394_camera_new (d, list->ids[i].guid);
		ERR( dc1394_camera_get_node(cam, &node, &generation) );
		printf("  busID=%d\n", generation);

		// Run through the collected list of cameras and check if anyone
		// has the same 'generation' parameter... (i.e. is on the same bus)
		tmp=(camNode_t*)ellFirst(&camList);
		newBus = 1;
		while(tmp!=NULL)
		{
			if (generation == tmp->generation)
			{
				newBus = 0;
				break;
			}
			tmp=(camNode_t*)ellNext((ELLNODE*)tmp);
		}

		// If we havent already listed a camera on this bus -or if this is the
		// first camera we check: add the camera handle to a list of cameras that
		// we want to use for resetting busses.
		// Else free up the camera handle as we won't use it until we instantiate
		// our driver plugin.
		if (newBus==1 || i==0)
		{
			camListItem = (camNode_t*)calloc(1, sizeof(camNode_t));
			camListItem->cam = cam;
			camListItem->generation = generation;
			ellAdd(&camList, (ELLNODE*)camListItem);
			latch = generation;
		} else
		{
			// if we dont need the camera handle to reset the bus
			// we might as well free it up
			dc1394_camera_free(cam);
		}
	}

	// Go through the list of cameras that have been identified to be
	// on separate physical busses. Call reset for each of them and free
	// up the camera handle
	camListItem = (camNode_t*)ellFirst(&camList);
	while(camListItem != NULL)
	{
		printf("Resetting bus: %3d using cam: 0x%16.16llX... ", camListItem->generation, camListItem->cam->guid);
		fflush(stdout);
		ERR( dc1394_reset_bus(camListItem->cam) );
		printf("Done\n");
		dc1394_camera_free(camListItem->cam);
		camListItem = (camNode_t*)ellNext((ELLNODE*)camListItem);
	}

	// Clear up after ourselves.
	ellFree(&camList);
    dc1394_camera_free_list (list);
    dc1394_free (d);
    printf("\n");
    return;
}    

/** \brief Initialise the firewire bus.
 *
 * This function need to be called only once to initialise the firewire bus before FDC_Config() can be called.
 * The bus will be first be reset, then scanned for cameras and the number of cameras and their hexadecimal
 * ID will be printed to stdout.
 */
extern "C" int FDC_InitBus(void)
{
    dc1394error_t err;

    if (dc1394camList!=NULL) dc1394_camera_free_list(dc1394camList);
    if (dc1394fwbus!=NULL) dc1394_free(dc1394fwbus);

    // First reset the bus
    reset_bus();

	/* initialise the bus */
	dc1394fwbus = dc1394_new ();
	/* scan the bus for all cameras */
	err = dc1394_camera_enumerate (dc1394fwbus, &dc1394camList);
	ERR( err );

	if (dc1394camList->num <= 0)
	{
		dc1394_log_error("No cameras found");
		return -1;
	}
	return 0;
}


BusManager::BusManager()
{
	ellInit( &(this->bus) );
}

/* Return always the same instance of the bus manager (singleton) */
BusManager* BusManager::getInstance()
{
	if (BusManager::instance == NULL)
	{
		BusManager::instance = new BusManager();
	}
	return BusManager::instance;
}

/* Add a FirewireDCAM areaDetector driver instance (one camera) to
 * the internal list of driver instances. */
void BusManager::addDriverInstance(FirewireDCAM* fdc)
{
	camList_t* item;
	printf("BusManager: adding driver instance: \'%s\'\n", fdc->portName);
	item = (camList_t*)calloc(1,sizeof(camList_t));
	item->fdc = fdc;
	ellAdd(&this->bus, (ELLNODE*)item);
}

/* Perform a bus reset while ensuring none of the areaDetector driver
 * instances will access any dc1394 camera functions.
 * After a bus reset the drivers internal camera handles will be
 * re-initialised.  */
void BusManager::resetBus()
{
	camList_t* item=NULL;
	//dc1394error_t err;
	epicsEventWaitStatus eventStatus;

	// Iterate through all the configured camera drivers on the bus
	// to stop them from talking to the cameras
	item = (camList_t*)ellFirst(&this->bus);
	while(item!=NULL)
	{
		// First set the drivers 'acquire' parameter to 0 so the image grabbing
		// thread will pause, waiting for a start event
		// if acquiring
		if (item->fdc->setAcquireParam(0)) {
			// Wait for a 'stopped' event to say the image thread has stopped
			printf("BusManager [%s]: waiting for 'stopEventId'\n", item->fdc->portName);
			eventStatus = epicsEventWaitWithTimeout(item->fdc->stopEventId, 2.0);

			/* Stop the actual transmission! */
			printf("BusManager [%s]: setting transmission off\n", item->fdc->portName);
			ERR(dc1394_video_set_transmission(item->fdc->camera, DC1394_OFF));
			printf("BusManager [%s]: setting capture stop\n", item->fdc->portName);
			ERR(dc1394_capture_stop(item->fdc->camera));
		}
		printf("BusManager [%s]: doing iso_release_all\n", item->fdc->portName);
		ERR(dc1394_iso_release_all(item->fdc->camera));
		printf("BusManager [%s]: freeing the camera\n", item->fdc->portName);
		dc1394_camera_free(item->fdc->camera);

		item=(camList_t*)ellNext((ELLNODE*)item);
	}

	// Re-initialise (and yet another bus reset)
	printf("BusManager: re-initialising the bus with FDC_InitBus()\n");
	FDC_InitBus();

	// Iterate through all configured drivers
	item = (camList_t*)ellFirst(&this->bus);
	while(item!=NULL)
	{
		item->fdc->initCamera(item->fdc->camguid);
		item=(camList_t*)ellNext((ELLNODE*)item);
	}
}


/** \brief Configuration function to configure one camera.
 *
 * This function need to be called once for each camera to be used by the IOC. A call to this
 * function instanciates one object from the FirewireDCAM class.
 * \param portName Asyn port name to assign to the camera.
 * \param camid The camera ID or serial number in a hexadecimal string. Lower case and
 *              upper case letters can be used. This is used to identify a specific camera
 *              on the bus. For instance: "0x00b09d01007139d0".
 * \param speed The bus speed to be used. This indicates whether to use the bus in 1394A or 1394B mode.
 *              Valid values are: 400 or 800. If an invalid value is entered the function will always
 *              default to 400 (legacy mode).
 * \param maxBuffers Maxiumum number of NDArray objects (image buffers) this driver is allowed to allocate.
 *                   This driver requires 2 buffers, and each queue element in a plugin can require one buffer
 *                   which will all need to be added up in this parameter.
 * \param maxMemory Maximum memory (in bytes) that this driver is allowed to allocate. So if max. size = 1024x768 (8bpp)
 *                  and maxBuffers is, say 14. maxMemory = 1024x768x14 = 11010048 bytes (~11MB)
 * \param disableScalable Disable scalable modes if this is 1
 */
extern "C" int FDC_Config(const char *portName, const char* camid, int speed, int maxBuffers, size_t maxMemory, int disableScalable)
{
	FirewireDCAM* fdc;
	BusManager* busman;
	fdc = new FirewireDCAM( portName, camid, speed, maxBuffers, maxMemory, disableScalable);
	busman = BusManager::getInstance();
	busman->addDriverInstance(fdc);
	return asynSuccess;
}

extern "C" void FDC_ResetBus()
{
	BusManager* busman;
	busman = BusManager::getInstance();
	busman->resetBus();
}

/** Specific asyn commands for this support module. These will be used and
 * managed by the parameter library (part of areaDetector). */
#define FDC_feat_valString           "FDC_FEAT_VAL"
#define FDC_feat_val_maxString       "FDC_FEAT_VAL_MAX"
#define FDC_feat_val_minString       "FDC_FEAT_VAL_MIN"
#define FDC_feat_val_absString       "FDC_FEAT_VAL_ABS"
#define FDC_feat_val_abs_maxString   "FDC_FEAT_VAL_ABS_MAX"
#define FDC_feat_val_abs_minString   "FDC_FEAT_VAL_ABS_MIN"
#define FDC_feat_modeString          "FDC_FEAT_MODE"
#define FDC_feat_availableString     "FDC_FEAT_AVAILABLE"
#define FDC_feat_absoluteString      "FDC_FEAT_ABSOLUTE"
#define FDC_framerateString          "FDC_FRAMERATE"
#define FDC_videomodeString          "FDC_VIDEOMODE"
#define FDC_bandwidthString          "FDC_BANDWIDTH"

/** Number of asyn parameters (asyn commands) this driver supports. */
#define NUM_FDC_PARAMS (&LAST_FDC_PARAM - &FIRST_FDC_PARAM + 1)

/** Feature mapping from DC1394 library enums to a local driver enum
 * The local driver identifies a feature based on the address of the asyn request.
 * The address range is [0..DC1394_FEATURE_NUM-1] and the dc1394 feature enum starts
 * at an offset of DC1394_FEATURE_MIN... */
#define FDC_DC1394_FEATOFFSET DC1394_FEATURE_MIN

static void imageGrabTaskC(void *drvPvt)
{
    FirewireDCAM *pPvt = (FirewireDCAM *)drvPvt;

    pPvt->imageGrabTask();
}

/** Constructor for the FirewireDCAM class
 * Initialises the camera object by setting all the default parameters and initializing
 * the camera hardware with it. This function also reads out the current settings of the
 * camera and prints out a selection of parameters to the shell.
 * \param portName The asyn port name to give the particular instance.
 * \param camid The unique ID stored in the camera.
 * \param speed The bus speed to use this camera at. Can be 800[Mb/s] for 1394B mode or 400[Mb/s] for 1394A mode.
 * \param maxBuffers The largest number of image buffers this driver can create.
 * \param maxMemory The maximum amount of memory in bytes that the driver can allocate for images.
 * \param disableScalable Disable scalable (format 7) modes if this is 1
 */
FirewireDCAM::FirewireDCAM(	const char *portName, const char* camid, int speed,
							int maxBuffers, size_t maxMemory, int disableScalable )
	: ADDriver(portName, DC1394_FEATURE_NUM, NUM_FDC_PARAMS, maxBuffers, maxMemory,
			0, 0, // interfacemask and interruptmask
			ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, // asynflags and autoconnect,
			0, 0),// thread priority and stack size
		pRaw(NULL)
{
	const char *functionName = "FirewireDCAM";
	unsigned long long int camUID = 0;
	unsigned int ret;
	dc1394error_t err;
	this->disableScalable = disableScalable;
	this->busSpeed = speed;

    createParam(FDC_feat_valString,             asynParamInt32,   &FDC_feat_val);
    createParam(FDC_feat_val_maxString,         asynParamInt32,   &FDC_feat_val_max);
    createParam(FDC_feat_val_minString,         asynParamInt32,   &FDC_feat_val_min);
    createParam(FDC_feat_val_absString,         asynParamFloat64, &FDC_feat_val_abs);
    createParam(FDC_feat_val_abs_maxString,     asynParamFloat64, &FDC_feat_val_abs_max);
    createParam(FDC_feat_val_abs_minString,     asynParamFloat64, &FDC_feat_val_abs_min);
    createParam(FDC_feat_modeString,            asynParamInt32,   &FDC_feat_mode);
    createParam(FDC_feat_availableString,       asynParamInt32,   &FDC_feat_available);
    createParam(FDC_feat_absoluteString,        asynParamInt32,   &FDC_feat_absolute);
    createParam(FDC_framerateString,            asynParamInt32,   &FDC_framerate);
    createParam(FDC_videomodeString,            asynParamInt32,   &FDC_videomode);
    createParam(FDC_bandwidthString,            asynParamFloat64, &FDC_bandwidth);

	/* Create the start and stop event that will be used to signal our
	 * image grabbing thread when to start/stop	 */
	printf("Creating epicsevents...                 ");
	this->startEventId = epicsEventCreate(epicsEventEmpty);
	this->stopEventId = epicsEventCreate(epicsEventEmpty);
	printf("OK\n");

	/* parse the string of hex-numbers that is the cameras unique ID */
	ret = sscanf(camid, "0x%16llX", &camUID);
	this->initCamera(camUID);

	/* Let dc1394 print out the camera info to stdout. This could possibly be
	 * located in the report function instead but I like to see the info at startup */
	err=dc1394_camera_print_info(this->camera, stdout);
	ERR( err );

    setIntegerParam(ADAcquire, 0);
	/* Start up acquisition thread */
    printf("Starting up image grabbing task...      ");
	fflush(stdout);
    if (epicsThreadCreate("imageGrabTask",
    		epicsThreadPriorityMedium,
    		epicsThreadGetStackSize(epicsThreadStackMedium),
    		(EPICSTHREADFUNC)imageGrabTaskC,
    		this) == NULL) {
    	printf("%s:%s epicsThreadCreate failure for image task\n",
    			driverName, functionName);
    	return;
    } else printf("OK\n");
    printf("Configuration complete!\n");
	return;
}

asynStatus FirewireDCAM::initCamera(unsigned long long int camUID) {
	const char *functionName = "initCamera";
	int dimensions[2];
	unsigned int sizeX, sizeY;
	unsigned int i, status=0;
    uint32_t value;
    dc1394operation_mode_t opMode;
	dc1394speed_t opSpeed;
	char chMode = 'A';
	dc1394video_mode_t videoMode;
	dc1394error_t err;

	/* configure the camera to the mode and so on... */
	for (i = 0; i < dc1394camList->num; i++)
	{
		/* See if we can find the camera on the bus with the specific ID */
		if (camUID == dc1394camList->ids[i].guid)
		{
			/* initialise the camera on the bus */
			this->camera = dc1394_camera_new (dc1394fwbus, camUID);
			this->camguid = camUID;
			/*printf("cameraInit: Using camera with GUID %llX\n", camera->guid);*/
			break;
		}
	}

	/* If we didn't find the camera with the specific ID we return... */
	if (this->camera == NULL)
	{
		fprintf(stderr,"### ERROR ### Did not find camera with GUID: 0x%16.16llX\n", camUID);
		return asynError;
	}

	/* Set a reasonable video mode */
	setIntegerParam(NDDataType, NDUInt8);
	setIntegerParam(NDColorMode, NDColorModeRGB1);
	videoMode = this->lookupVideoMode(lookupColorCoding(NDColorModeRGB1, NDUInt8));
	if (videoMode == 0) {
		// This must be a black and white camera
		setIntegerParam(NDColorMode, NDColorModeMono);
		videoMode = this->lookupVideoMode(lookupColorCoding(NDColorModeMono, NDUInt8));
	}
	if (videoMode == 0) {
		fprintf(stderr,"### ERROR ### Cannot set 8 bit mono or colour video mode\n");
		return asynError;
	}
	this->setVideoMode(this->pasynUserSelf);

	/* Get the image size from our current video mode */
	printf("Getting image dims from mode%3d...      ", (int) videoMode);
	fflush(stdout);
	err = dc1394_get_image_size_from_video_mode(this->camera, videoMode, &sizeX, &sizeY);
	dimensions[0] = (int)sizeX;
	dimensions[1] = (int)sizeY;
	ERR( err );
	printf("%dx%d\n", dimensions[0], dimensions[1]);

	if (this->busSpeed == 800)
	{
		opMode = DC1394_OPERATION_MODE_1394B;
		opSpeed = DC1394_ISO_SPEED_800;
		chMode = 'B';
	} else
	{
		opMode = DC1394_OPERATION_MODE_LEGACY;
		opSpeed = DC1394_ISO_SPEED_400;
		chMode = 'A';
		this->busSpeed = 400;
	}
	/* TODO: We probably need to add a bit more rigorous error checking after
	 * setting operation mode and iso speed: I'm not entirely sure what happens
	 * if we try to set 800Mb/s (B mode) on a camera that doesn't support it. Or
	 * what happens if we set B mode on a bus that has a mix of A and B mode cameras? */
	printf("Setting 1394%c mode...                   ", chMode);
	fflush(stdout);
	err=dc1394_video_set_operation_mode(this->camera, opMode);
	ERR( err );
	printf("OK\n");

	printf("Setting ISO speed to %dMb/s...         ", this->busSpeed);
	fflush(stdout);
	err=dc1394_video_set_iso_speed(this->camera, opSpeed);
	ERR( err );
	printf("OK\n");
/*
	printf("Setting framerate (7.5): %d...          ", DC1394_FRAMERATE_7_5);
	fflush(stdout);
	err=dc1394_video_set_framerate(this->camera, DC1394_FRAMERATE_7_5);
	ERR( err );
	printf("OK\n");
*/
/*	printf("Preparing capture...                    ");
	fflush(stdout);
	err=dc1394_capture_setup(this->camera,FDC_DC1394_NUM_BUFFERS, DC1394_CAPTURE_FLAGS_DEFAULT);
	ERR( err );
	printf("OK\n");
*/

	/* Getting all available features and their information from the camera */
	err = dc1394_feature_get_all (this->camera, &(this->features));
	ERR(err);

	printf("Getting Trigger mode...                 ");
	fflush(stdout);
	err = dc1394_get_control_register(camera, REG_CAMERA_TRIGGER_MODE, &value);
	ERR( err );
	if (value == 0x80100000) {
		status |= setIntegerParam(ADTriggerMode, 0);
		printf("0x%X (Internal)\n", value);
	} else if (value == 0x82100000) {
		status |= setIntegerParam(ADTriggerMode, 1);
		printf("0x%X (External)\n", value);
	} else {
		status |= setIntegerParam(ADTriggerMode, 1);
		printf("0x%X (Unknown)\n", value);
	}

	/* Set the parameters from the camera in our areaDetector param lib */
	printf("Setting the areaDetector parameters...  ");
	fflush(stdout);
	
    status |=  setStringParam (ADManufacturer, this->camera->vendor);
    status |= setStringParam (ADModel, this->camera->model);
    status |= setIntegerParam(ADMaxSizeX, dimensions[0]);
    status |= setIntegerParam(ADMaxSizeY, dimensions[1]);
    status |= setIntegerParam(ADSizeX, dimensions[0]);
    status |= setIntegerParam(ADSizeY, dimensions[1]);
    status |= setIntegerParam(ADImageMode, ADImageContinuous);
    status |= setIntegerParam(ADNumImages, 100);
    status |= setIntegerParam(ADBinX, 1);
    status |= setIntegerParam(ADBinY, 1);
    status |= setIntegerParam(ADReverseX, 0);
    status |= setIntegerParam(ADReverseY, 0);
    err= dc1394_get_control_register(camera, REG_CAMERA_TRIGGER_MODE, &value);
	status |= PERR( this->pasynUserSelf, err );
	/* Flea specific, see:
	 * http://www.ptgrey.com/support/downloads/documents/TAN2004004_Synchronizing_to_external_signal_DCAM1.31.pdf
	 */
    status |= this->getAllFeatures();
    if (status)
    {
         fprintf(stderr, "ERROR %s: unable to set camera parameters\n", functionName);
         return asynError;
    } else {
    	printf("OK\n");
    	return asynSuccess;
    }
    
}

/* Set acquire, and return 1 if a change was made */
int FirewireDCAM::setAcquireParam(int acquire)
{
	int old_acquire;
	getIntegerParam(ADAcquire, &old_acquire);
	setIntegerParam(ADAcquire, acquire);
	return old_acquire != acquire;
}

/** Run the dc1394_capture_dequeue() function in a polling loop until a frame is received or it times out.
 *
 */
dc1394error_t FirewireDCAM::captureDequeueTimeout(dc1394camera_t *camera, dc1394video_frame_t **frame, epicsFloat64 timeout)
{
	epicsTimeStamp start, now;
	dc1394error_t err = DC1394_SUCCESS;
	dc1394video_frame_t *fptr;
	epicsFloat64 dt = 0.0;
	const char* functionName = "captureDequeueTimeout";
	epicsTimeGetCurrent(&start);

	this->unlock();
	do
	{
		err = dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_POLL, &fptr);
		PERR(this->pasynUserSelf,err);
		epicsTimeGetCurrent(&now);
		dt = epicsTimeDiffInSeconds(&now, &start);
		epicsThreadSleep( epicsThreadSleepQuantum() );
		//asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s err=%d dt=%.3f (timeout=%.2f) fptr=%p\n",
		//		driverName, functionName,(int)err,dt,timeout,fptr);
	}while(fptr == NULL && dt <= timeout && err == DC1394_SUCCESS);
	this->lock();

	asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s err=%d dt=%.3f (timeout=%.3f) fptr=%p\n",
			  driverName, functionName,(int)err,dt,timeout,fptr);

	if (dt > timeout)
	{
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
				"%s:%s: timeout! did not receive a frame within %.3fs\n",
				driverName, functionName, timeout);
		asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
				"%s:%s Attempting to stop video transmission\n", driverName, functionName);
		err=dc1394_video_set_transmission(camera, DC1394_OFF);
		PERR(this->pasynUserSelf, err);
		asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
				"%s:%s Attempting to restart video transmission\n", driverName, functionName);
		err=dc1394_video_set_transmission(camera, DC1394_ON);
		PERR(this->pasynUserSelf, err);
	}

	*frame = fptr;
	return err;
}


/** Task to grab images off the camera and send them up to areaDetector
 *
 */
void FirewireDCAM::imageGrabTask()
{
	int status = asynSuccess;
	int acquire;
	dc1394error_t err;
	epicsUInt32 iBandwidth;
	epicsFloat64 dBandwidthPercent;
	dc1394video_frame_t * dc1394_frame;
	epicsFloat64 fr;
	int stopFromExternalThread = 0; /* The first time, we need this to be 0 so it doesn't signal stop event */
	const char *functionName = "imageGrabTask";

	printf("FirewireDCAM::imageGrabTask: Got the image grabbing thread started!\n");
	this->lock();
	while (1) /* ... round and round and round we go ... */
	{
		/* Is acquisition active? */
		getIntegerParam(ADAcquire, &acquire);

		/* If we are not acquiring then wait for a semaphore that is given when acquisition is started */
		if (!acquire)
		{
			setIntegerParam(ADStatus, ADStatusIdle);
			callParamCallbacks();

			/* Release the lock while we wait for an event that says acquire has started, then lock again */
			this->unlock();

			/* Wait for a signal that tells this thread that the transmission
			 * has started and we can start asking for image buffers...	 */
			if (stopFromExternalThread)	{
				asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
						"%s::%s [%s]: signalling stop event\n", driverName, functionName, this->portName);
				epicsEventSignal(this->stopEventId);
			}
			stopFromExternalThread = 1;
			asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
					"%s::%s [%s]: waiting for acquire to start\n", driverName, functionName, this->portName);
			/* This is essentially a breakpoint.
			 * If someone sets ADAquire to 0, and then waits with 2 second timeout for
			 * stopEventId, then this thread will be at this point */
			status = epicsEventWait(this->startEventId);
			asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
					"%s::%s [%s]: started!\n", driverName, functionName, this->portName);
			this->lock();
		}

		/* We are now waiting for an image  */
		setIntegerParam(ADStatus, ADStatusWaiting);
		callParamCallbacks();
		getDoubleParam((int)DC1394_FEATURE_FRAME_RATE - (int)DC1394_FEATURE_MIN, FDC_feat_val_abs, &fr);
		if (fr<=0) fr=1;
		err = this->captureDequeueTimeout(this->camera, &dc1394_frame, 5/fr);
		if (err) {
			/* Frame error */
			status = PERR( this->pasynUserSelf, err );
			continue;
		} else if (dc1394_frame==NULL) {
			/* No frame yet */
			asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
					"%s::%s [%s]: camera didn't produce frame. Timed out after %.3f s!\n", driverName, functionName, this->portName, 5/fr);
			continue;
		}

		/* We now have a frame, so process it */
		if (this->decodeFrame(dc1394_frame)) {
			/* We abort if we had some problem with grabbing an image...
			 * This is perhaps not always the desired behaviour but it'll do for now. */
			this->stopCapture(this->pasynUserSelf);
			stopFromExternalThread = 0;
			continue;
		}

		/* Get some information about the current feature settings */
		this->getAllFeatures();

		/* Calculate the bandwidth usage. The 1394 bus has 4915 bandwidth units
         * available per cycle. Each unit corresponds to the time it takes to send one
         * quadlet at ISO speed S1600. The bandwidth usage at S400 is thus four times the
         * number of quadlets per packet.		 
         */
		dc1394_video_get_bandwidth_usage(this->camera, &iBandwidth);
		dBandwidthPercent = 100.0 * iBandwidth / 4915.0;
		setDoubleParam(FDC_bandwidth, dBandwidthPercent);

	}/* back to the top... */
	return;
}

struct pix_lookup {
	dc1394color_coding_t colorCoding;
    int colorMode, dataType;
};

static const struct pix_lookup pix_lookup[] = {
    { DC1394_COLOR_CODING_MONO8,   NDColorModeMono,   NDUInt8  },
    { DC1394_COLOR_CODING_RGB8,    NDColorModeRGB1,   NDUInt8  },
    { DC1394_COLOR_CODING_RAW8,    NDColorModeBayer,  NDUInt8  },
    { DC1394_COLOR_CODING_YUV411,  NDColorModeYUV411, NDUInt8  },
    { DC1394_COLOR_CODING_YUV422,  NDColorModeYUV422, NDUInt8  },
    { DC1394_COLOR_CODING_YUV444,  NDColorModeYUV444, NDUInt8  },
    { DC1394_COLOR_CODING_MONO16,  NDColorModeMono,   NDUInt16 },
    { DC1394_COLOR_CODING_MONO16S, NDColorModeMono,   NDInt16  },
    { DC1394_COLOR_CODING_RGB16,   NDColorModeRGB1,   NDUInt16 },
    { DC1394_COLOR_CODING_RGB16S,  NDColorModeRGB1,   NDInt16  },
    { DC1394_COLOR_CODING_RAW16,   NDColorModeBayer,  NDUInt16 },
};

/** Lookup a colorMode and dataType from a dc1394color_coding_t */
asynStatus FirewireDCAM::lookupColorMode(dc1394color_coding_t colorCoding, int *colorMode, int *dataType) {
    const int N = sizeof(pix_lookup) / sizeof(struct pix_lookup);
    for (int i = 0; i < N; i ++)
        if (pix_lookup[i].colorCoding == colorCoding) {
            *colorMode   = pix_lookup[i].colorMode;
            *dataType    = pix_lookup[i].dataType;
            return asynSuccess;
        }
    return asynError;
}

/** Lookup a dc1394color_coding_t from a colorMode and dataType */
dc1394color_coding_t FirewireDCAM::lookupColorCoding(int colorMode, int dataType) {
    const int N = sizeof(pix_lookup) / sizeof(struct pix_lookup);
    for (int i = 0; i < N; i ++)
        if (colorMode == pix_lookup[i].colorMode &&
            dataType == pix_lookup[i].dataType) {
            return pix_lookup[i].colorCoding;
        }
    return (dc1394color_coding_t) 0;
}

/** Look up a valid video mode based on a dc1394color_coding_t */
dc1394video_mode_t FirewireDCAM::lookupVideoMode(dc1394color_coding_t colorCoding)
{
	dc1394error_t err;
	dc1394video_modes_t video_modes;
	dc1394color_coding_t rbColorCoding;
	dc1394color_codings_t codings;
	dc1394video_mode_t match;
	unsigned int j;
	const char *functionName = "lookupVideoMode";

	match = (dc1394video_mode_t) 0;

	err=dc1394_video_get_supported_modes(this->camera, &video_modes);
	ERR( err );
	/* Loop through all the supported non-scalable modes first to see if there are
	 * any video modes that match our requirement.
	 * We only keep the last match we find in the list as this will be the highest resolution. */
	for (unsigned int i = 0; i < video_modes.num; i++) {
		/* If the video mode is scalable we break out of the loop
		 * because all subsequent modes will be scalable */
		if (video_modes.modes[i] >= DC1394_VIDEO_MODE_FORMAT7_MIN) break;

		// Check that our demand color coding matches what's available
		ERR( dc1394_get_color_coding_from_video_mode(this->camera,video_modes.modes[i], &rbColorCoding) );
		asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "\tchecking mode: %d color code: %d\n",
				video_modes.modes[i], rbColorCoding);
		if (rbColorCoding == colorCoding) {
			match = video_modes.modes[i];
			asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s Non-scalable match found. mode: %d color code: %d\n",
						driverName, functionName, (int)match, (int)colorCoding);
		}
	}

	/* Loop through all the supported scalable modes to see if anyone matches our
	 * requirements. For each mode we need to investigate all possible colour modes
	 * as each format7 supports multiple color modes...
	 *
	 */
	for(int i = ((int)video_modes.num)-1; i>=0; i--) {
		// If we want to ignore scalables, just break out of the loop
		if (this->disableScalable) break;

		// if video mode is less than the first format7 mode, we're done.
		if (video_modes.modes[i] < DC1394_VIDEO_MODE_FORMAT7_MIN) break;

		// Check that our demand color coding matches what's available
		ERR( dc1394_format7_get_color_codings (this->camera,video_modes.modes[i], &codings) );
		for(j=0;j<codings.num;j++) {
			asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "\tchecking format7 mode: %d color code: %d\n",
					video_modes.modes[i],codings.codings[j]);
			if (codings.codings[j] == colorCoding) {
				match = video_modes.modes[i];
				asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s Scalable match found. mode: %d color code: %d\n",
							driverName, functionName, (int)match, (int)colorCoding);
			}
		}
	}

	if ((int)match == 0)
		asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s Warning: no match found for color code: %d\n",
					driverName, functionName, (int)colorCoding);
	else
		asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s Best match mode: %d for color code: %d\n",
					driverName, functionName, (int)match, (int)colorCoding);

	return match;
}

asynStatus FirewireDCAM::setVideoMode(asynUser* pasynUser) {
	int acquiring;
	asynStatus status = asynSuccess;
	dc1394video_mode_t videoMode, rbVideoMode;
	dc1394color_coding_t colorCoding, rbColorCoding;
	int colorMode, dataType;
	const char* functionName = "setVideoMode";

	// Grab input data
	getIntegerParam(NDColorMode, &colorMode);
	getIntegerParam(NDDataType, &dataType);
	if (pasynUser == NULL) pasynUser = this->pasynUserSelf;

	// Check if we can get a valid colorCoding
	colorCoding = this->lookupColorCoding(colorMode, dataType);
	if (colorCoding == 0) return asynError;

	// Check if we can get a valid videoMode
	videoMode = this->lookupVideoMode(colorCoding);
	if (((int)videoMode) == 0) return asynError;

	// We check whether the new video mode is really the same as our current mode
	// -in which case we just return success
	PERR( pasynUser, dc1394_video_get_mode(this->camera, &rbVideoMode) );
	if (rbVideoMode == videoMode) {
		// If we are in scalable mode, need to check the color coding
		if (dc1394_is_video_mode_scalable (rbVideoMode)) {
			// Check the particular color coding in this mode.
			status = PERR( pasynUser, dc1394_format7_get_color_coding(this->camera, videoMode, &rbColorCoding) );
			if (rbColorCoding == colorCoding) {
				setIntegerParam(FDC_videomode, (int)videoMode);
				callParamCallbacks();
				return asynSuccess;
			}
		} else {
			setIntegerParam(FDC_videomode, (int)videoMode);
			callParamCallbacks();
			return asynSuccess;
		}
	}

	// We've got this far, so we need to change video modes. First stop camera running
	getIntegerParam(ADAcquire, &acquiring);
	if (acquiring) this->stopCaptureAndWait(pasynUser);

	// Set the video mode and check for failure
	asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s::%s Setting video mode: %d\n",
			driverName, functionName, videoMode);
	status = PERR(pasynUser, dc1394_video_set_mode(this->camera, videoMode));
	if (status == asynError) return status;

	// If this is format 7, set the color coding
	if (dc1394_is_video_mode_scalable (videoMode))
	{
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s::%s Setting color coding: %d\n",
				driverName, functionName, colorCoding);
		status = PERR( pasynUser, dc1394_format7_set_color_coding(this->camera, videoMode, colorCoding) );
		if (status == asynError) return status;
	}

	// Start the camera again if needed
	if (acquiring) status = this->startCapture(pasynUser);

	// Update params
	setIntegerParam(FDC_videomode, (int)videoMode);
	callParamCallbacks();

	return status;
}

asynStatus FirewireDCAM::setRoi(asynUser *pasynUser)
{
	asynStatus status = asynSuccess;
	dc1394video_mode_t videoMode;
	unsigned int sizeX, sizeY, maxSizeX, maxSizeY;
	unsigned int unitSizeX, unitSizeY, unitOffsetX, unitOffsetY;
	int offsetX, offsetY, wasAcquiring;
	unsigned int packetSize;
	unsigned int modulus;
	dc1394color_coding_t rbColorCoding;
	const char* functionName="setRoi";

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s setRoi()\n", driverName, functionName);

	PERR( this->pasynUserSelf, dc1394_video_get_mode(this->camera, &videoMode) );
	if (!dc1394_is_video_mode_scalable(videoMode))
	{
		asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s This video mode (%d) is not scalable\n",
				driverName, functionName, (int)videoMode);
		status = PERR( pasynUser, dc1394_get_image_size_from_video_mode (this->camera, videoMode, &sizeX, &sizeY) );
		maxSizeX = sizeX;
		maxSizeY = sizeY;
		offsetX=0; offsetY=0;
		goto finally;
	}
	getIntegerParam(ADAcquire, &wasAcquiring);
	if (wasAcquiring)
	{
		status = this->stopCaptureAndWait(pasynUser);
		if (status == asynError) return status;
	}

	getIntegerParam(ADSizeX, (int*)&sizeX);
	getIntegerParam(ADSizeY, (int*)&sizeY);
	getIntegerParam(ADMaxSizeX, (int*)&maxSizeX);
	getIntegerParam(ADMaxSizeY, (int*)&maxSizeY);
	getIntegerParam(ADMinX, &offsetX);
	getIntegerParam(ADMinY, &offsetY);

	PERR(this->pasynUserSelf, dc1394_format7_get_max_image_size (this->camera, videoMode, &maxSizeX, &maxSizeY) );
	PERR(this->pasynUserSelf, dc1394_format7_get_unit_size      (this->camera, videoMode, &unitSizeX, &unitSizeY) );
	PERR(this->pasynUserSelf, dc1394_format7_get_unit_position  (this->camera, videoMode, &unitOffsetX, &unitOffsetY) );

	// First adjust offset to fit the ROI within the sensor size
	if (sizeX+offsetX > maxSizeX) offsetX = maxSizeX-sizeX;
	if (offsetX < 0) offsetX = 0;
	if (sizeY+offsetY > maxSizeY) offsetY = maxSizeY-sizeY;
	if (offsetY < 0) offsetY = 0;
	// Second adjust size to fit ROI within the sensor
	if (sizeX+offsetX > maxSizeX) sizeX = maxSizeX;
	if (sizeY+offsetY > maxSizeY) sizeY = maxSizeY;

	// Adjust size to be a multiple of the unitSize (the minimum step change the camera will allow)
	modulus = sizeX % unitSizeX;
	if (modulus != 0) sizeX = sizeX - modulus;
	modulus = sizeY % unitSizeY;
	if (modulus != 0) sizeY = sizeY - modulus;
	// Adjust the offsets to be a multiple of the unitOffset
	modulus = offsetX % unitOffsetX;
	if (modulus != 0) offsetX = offsetX - modulus;
	modulus = offsetY % unitOffsetY;
	if (modulus != 0) offsetY = offsetY - modulus;

	// Get the color coding
	PERR(pasynUser, dc1394_format7_get_color_coding(this->camera, videoMode, &rbColorCoding) );

	// Actually set the ROI settings in one big go
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s dc1394_format7_set_roi(): video=%d color=%d\n"
			"\t\tmax = (%d,%d) size = (%d,%d) unit = (%d,%d)\n"
			"\t\toffset = (%d,%d) unit = (%d,%d)\n",
			driverName, functionName, (int)videoMode, (int)rbColorCoding,
			maxSizeX, maxSizeY, sizeX, sizeY, unitSizeX, unitSizeY,
			offsetX, offsetY, unitOffsetX, unitOffsetY);
	status = PERR( this->pasynUserSelf, dc1394_format7_set_roi(this->camera, videoMode, rbColorCoding,
													  DC1394_USE_RECOMMENDED,
													  offsetX, offsetY, sizeX, sizeY) );

	// Read back the ROI settings from the camera again to verify
	PERR( pasynUser, dc1394_format7_get_roi(this->camera, videoMode, &rbColorCoding, &packetSize,
										   (unsigned int*)&offsetX, (unsigned int*)&offsetY, &sizeX, &sizeY) );
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s dc1394_format7_get_roi(): video=%d color=%d\n"
			"\t\tsize = (%d,%d) offset = (%d,%d)\n",
			driverName, functionName, (int)videoMode, (int)rbColorCoding,
			sizeX, sizeY,offsetX, offsetY);

	if (wasAcquiring) {
		/* It seems to take the firewirebus about 0.35s to receive the first frames after setting framerate.
		 * Timeout is 5*framerate, so only need to wait for 0.2s here, even at 30fps */
//		epicsThreadSleep(0.2);
		this->startCapture(pasynUser);
	}

	finally:
	setIntegerParam( ADSizeX,    (int)sizeX);
	setIntegerParam( ADSizeY,    (int)sizeY);
	setIntegerParam( ADMaxSizeX, (int)maxSizeX);
	setIntegerParam( ADMaxSizeY, (int)maxSizeY);
	setIntegerParam( ADMinX,     offsetX);
	setIntegerParam( ADMinY,     offsetY);

	return status;
}

/** decodes a dc1394video_frame_t, turns it into an NDArray,
 * finally clears the buffer off the dc1394 queue.
 * This function expects the mutex to be locked already by the caller!
 */
int FirewireDCAM::decodeFrame(dc1394video_frame_t * dc1394_frame)
{
	int status = asynSuccess;
//	int minX, minY, sizeX, sizeY;
	int ndims;
	int dims[3];
    int arrayCallbacks, imageCounter, numImages, numImagesCounter, imageMode;
    int colorMode, dataType;
    int xDim=0, yDim=1, binX, binY;
    double acquirePeriod;


	dc1394error_t err;
	const char* functionName = "grabImage";

	/* Change the status to be readout... */
	setIntegerParam(ADStatus, ADStatusReadout);
	callParamCallbacks();

    /* Get the current parameters */
    getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
    getIntegerParam(NDArrayCounter, &imageCounter);
    getIntegerParam(ADNumImages, &numImages);
    getIntegerParam(ADNumImagesCounter, &numImagesCounter);
    getIntegerParam(ADImageMode, &imageMode);
    getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
    getDoubleParam(ADAcquirePeriod, &acquirePeriod);

    /* The buffer structure does not contain the binning, get that from param lib,
     * but it could be wrong for this frame if recently changed */
    getIntegerParam(ADBinX, &binX);
    getIntegerParam(ADBinY, &binY);
/*
    for (int i=0; i<8; i++) {
    	printf("%d ", ((unsigned char *) dc1394_frame->image)[i]);
    }
    printf("dim: %d x %d cc: %d\n", dc1394_frame->size[0], dc1394_frame->size[1], dc1394_frame->color_coding);
*/
    /* Report a new frame with the counters */
    imageCounter++;
    numImagesCounter++;
    setIntegerParam(NDArrayCounter, imageCounter);
    setIntegerParam(ADNumImagesCounter, numImagesCounter);
    if (imageMode == ADImageMultiple) {
        setDoubleParam(ADTimeRemaining, (numImages - numImagesCounter) * acquirePeriod);
    }

    /* Find the data needed to alloc a new frame */
    this->lookupColorMode(dc1394_frame->color_coding, &colorMode, &dataType);
    switch (colorMode) {
        case NDColorModeMono:
    	case NDColorModeBayer:
    		/* WARNING: Bayer mode doesn't seem to work in format 7 on the colour flea2.
    		 * It isn't available in non-scalable modes either so can't test this code for bayer
    		 */
            xDim = 0;
            yDim = 1;
            ndims = 2;
            break;
        case NDColorModeRGB1:
            xDim = 1;
            yDim = 2;
            ndims = 3;
            dims[0] = 3;
            break;
        default:
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s: unknown colorMode\n",
                        driverName, functionName);
            return asynError;
    }
    dims[xDim] = dc1394_frame->size[0];
    dims[yDim] = dc1394_frame->size[1];
    this->pRaw = this->pNDArrayPool->alloc(ndims, dims, (NDDataType_t)dataType, 0, NULL);

    if (dc1394_frame->frames_behind > 0) {
    	asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s [%s] %d frames behind!\n",
					driverName, functionName, this->portName, dc1394_frame->frames_behind);
    }

    if (!this->pRaw)
    {
    	/* If we didn't get a valid buffer from the NDArrayPool we must abort
    	 * the acquisition as we have nowhere to dump the data...  	 */
		setIntegerParam(ADStatus, ADStatusAborting);
		callParamCallbacks();
    	asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s [%s] ERROR: Serious problem: not enough buffers left! Aborting acquisition!\n",
					driverName, functionName, this->portName);
		return asynError;
    }

	memcpy(this->pRaw->pData, dc1394_frame->image, dc1394_frame->image_bytes);

    /* fill in the data */
    //this->pRaw->pAttributeList->add("BayerPattern", "Bayer Pattern", NDAttrInt32, &bayerFormat);
    this->pRaw->pAttributeList->add("ColorMode", "Color Mode", NDAttrInt32, &colorMode);
    this->pRaw->dataType = (NDDataType_t) dataType;
    this->pRaw->dims[0].size    = 3;
    this->pRaw->dims[0].offset  = 0;
    this->pRaw->dims[0].binning = 1;
    this->pRaw->dims[xDim].size    = dc1394_frame->size[0];
    this->pRaw->dims[xDim].offset  = dc1394_frame->position[0];
    this->pRaw->dims[xDim].binning = binX;
    this->pRaw->dims[yDim].size    = dc1394_frame->size[1];
    this->pRaw->dims[yDim].offset  = dc1394_frame->position[1];
    this->pRaw->dims[yDim].binning = binY;

    /* Put the frame number and time stamp into the buffer */
    this->pRaw->uniqueId = imageCounter;
    this->pRaw->timeStamp = dc1394_frame->timestamp / 1.e6;

    /* Get any attributes that have been defined for this driver */
    this->getAttributes(this->pRaw->pAttributeList);

    /* Call the callbacks to update any changes */
    callParamCallbacks();

    /* this is a good image, so callback on it */
    if (arrayCallbacks) {
        /* Call the NDArray callback */
        /* Must release the lock here, or we can get into a deadlock, because we can
         * block on the plugin lock, and the plugin can be calling us */
        this->unlock();
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
             "%s:%s: calling imageData callback\n", driverName, functionName);
        doCallbacksGenericPointer(this->pRaw, NDArrayData, 0);
        this->lock();
        this->pRaw->release();
        this->pRaw = NULL;
    }

    /* See if acquisition is done */
    if ((imageMode == ADImageSingle) ||
        ((imageMode == ADImageMultiple) &&
         (numImagesCounter >= numImages))) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: acquisition completed\n", driverName, functionName);
        return asynError;
    }

	/* Free the frame off the dc1394 queue. We can do this now as
	 * we have just taken a local copy of the data.  */
	err=dc1394_capture_enqueue(this->camera, dc1394_frame);
	status |= PERR( this->pasynUserSelf, err );

	return (status);
}


/** Write integer value to the drivers parameter table.
 * \param pasynUser
 * \param value
 * \return asynStatus Either asynError or asynSuccess
 */
asynStatus FirewireDCAM::writeInt32( asynUser *pasynUser, epicsInt32 value)
{
	asynStatus status = asynSuccess;
	int function = pasynUser->reason;
	epicsInt32 old_value;
	int addr;
	pasynManager->getAddr(pasynUser, &addr);
	dc1394error_t err;

    /* Set the parameter and readback in the parameter library.
     * This may be overwritten when we read back the
     * status at the end, but that's OK */
	getIntegerParam(addr, function, &old_value);
    status = setIntegerParam(addr, function, value);

    if (function == ADAcquire) {
    	if (value != old_value) {
    		if (value) {
    			/* start acquisition */
    			status = this->startCapture(pasynUser);
    		} else {
    			status = this->stopCaptureAndWait(pasynUser);
    		}
    	}
    } else if ( function == NDDataType ||
    		    function == NDColorMode ) {
    	status = this->setVideoMode(pasynUser);
    } else if ( function == ADBinX ||
    		    function == ADBinY ) {
    	if (value != 1) status = asynError;
    } else if ( function == ADReverseX ||
    		    function == ADReverseY ) {
    	if (value != 0) status = asynError;
    } else if ( function == ADTriggerMode ) {
    	/* Flea specific, see:
    	 * http://www.ptgrey.com/support/downloads/documents/TAN2004004_Synchronizing_to_external_signal_DCAM1.31.pdf
    	 */
    	if (value) {
    		err = dc1394_set_control_register(camera, REG_CAMERA_TRIGGER_MODE, 0x82100000);
    	} else {
    		err = dc1394_set_control_register(camera, REG_CAMERA_TRIGGER_MODE, 0x80100000);
    	}
    	status = PERR( pasynUser, err );
    } else if ( (function == ADSizeX) ||
                (function == ADSizeY) ||
                (function == ADMinX)  ||
                (function == ADMinY) ){
    	status = this->setRoi(pasynUser);
    } else if (function == FDC_feat_val) {
		status = this->setFeatureValue(pasynUser, addr, value, NULL);
    } else if (function == FDC_feat_mode) {
		status = this->setFeatureMode(pasynUser, addr, value, NULL);
    } else if (function == FDC_framerate) {
		status = this->setFrameRate(pasynUser, value);
    } else if (function < FIRST_FDC_PARAM) {
		/* If this parameter belongs to a base class call its method */
		status = ADDriver::writeInt32(pasynUser, value);
    } else { 
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s Function not implemented: %d; val=%d\n",
					driverName, "writeInt32", function, value);
		status = asynError;
	}

	if (status) {
		/* write the old value back */
		setIntegerParam(addr, function, old_value);
	} else {
		/* update all feature values to check if any settings have changed */
		status = (asynStatus) this->getAllFeatures();
		/* Call the callback */
		callParamCallbacks();
	}

	return status;
}

/** Write floating point value to the drivers parameter table and possibly to the hardware.
 * \param pasynUser
 * \param value
 * \return asynStatus Either asynError or asynSuccess
 */
asynStatus FirewireDCAM::writeFloat64( asynUser *pasynUser, epicsFloat64 value)
{
	asynStatus status = asynSuccess;
	int function = pasynUser->reason;
	epicsFloat64 old_value;
	int addr;
	pasynManager->getAddr(pasynUser, &addr);

    /* Set the parameter and readback in the parameter library.
     * This may be overwritten when we read back the
     * status at the end, but that's OK */
	getDoubleParam(addr, function, &old_value);
    status = setDoubleParam(addr, function, value);

	if (function == FDC_feat_val_abs) {
		status = this->setFeatureAbsValue(pasynUser, addr, value, NULL);
	} else if (function == ADAcquireTime) {
		addr = DC1394_FEATURE_SHUTTER - DC1394_FEATURE_MIN;
		status = this->setFeatureAbsValue(pasynUser, addr, value, NULL);
	} else if (function == ADAcquirePeriod) {
		addr = DC1394_FEATURE_FRAME_RATE - DC1394_FEATURE_MIN;
		if (value>0) {
			status = this->setFeatureAbsValue(pasynUser, addr, 1/value, NULL);
		} else {
			status = asynError;
		}
	} else if (function == ADGain) {
		addr = DC1394_FEATURE_GAIN - DC1394_FEATURE_MIN;
		status = this->setFeatureAbsValue(pasynUser, addr, value, NULL);
    } else if (function < FIRST_FDC_PARAM) {
		/* If this parameter belongs to a base class call its method */
		status = ADDriver::writeFloat64(pasynUser, value);
	} else {
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s Function not implemented for function %d; val=%.4f\n",
					"FirewireDCAM", "writeFloat64", function, value);
		status = asynError;
	}

	if (status) {
		/* write the old value back */
		setDoubleParam(addr, function, old_value);
	} else {
		/* update all feature values to check if any settings have changed */
		status = (asynStatus) this->getAllFeatures();
		/* Call the callback */
		callParamCallbacks();
	}
	return status;
}

/** Check if a requested feature is valid
 *
 * Checks for:
 * <ol>
 * <li>Valid range of the asyn request address against feature index.
 * <li>Availability of the requested feature in the current camera.
 * </ol>
 * \param pasynUser Asyn User to write messages to
 * \param addr Address of the parameter to get
 * \param featInfo Pointer to a feature info structure pointer. The function
 *        will write a valid feature info struct into this pointer or NULL on error.
 * \param featureName The function will return a string in this parameter with a
 *                    readable name for the given feature.
 * \param functionName The caller can pass a string which contain the callers function
 *                     name. For debugging/printing purposes only.
 * \return asyn status
 */
asynStatus FirewireDCAM::checkFeature(	asynUser *pasynUser, int addr, dc1394feature_info_t **featInfo,
										char** featureName, const char* functionName)
{
	asynStatus status = asynSuccess;
	dc1394feature_info_t *tmpFeatInfo;
	const char* localFunctionName = "checkFeature";

	*featInfo = NULL;/* set the default return pointer to nothing... */
	if (functionName == NULL) functionName = localFunctionName;

	if (addr < 0 || addr >= DC1394_FEATURE_NUM)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s ERROR addr: %d is out of range [0..%d]\n",
					driverName, functionName, addr, DC1394_FEATURE_NUM);
		return asynError;
	}
	tmpFeatInfo = &(this->features.feature[addr]);

	/* Get a readable name for the feature we are working on */
	*featureName = (char*)dc1394_feature_get_string (tmpFeatInfo->id);

	/* check if the feature we are working on is even available in this camera */
	if (!tmpFeatInfo->available)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s ERROR Port \'%s\' Feature \'%s\' is not available in camera\n",
					driverName, functionName, this->portName, *featureName);
		return asynError;
	}

	*featInfo = tmpFeatInfo;
	return status;
}

asynStatus FirewireDCAM::setFeatureMode(asynUser *pasynUser, int addr, epicsInt32 value, epicsInt32 *rbValue)
{
	asynStatus status = asynSuccess;
	dc1394feature_info_t *featInfo;
	dc1394feature_mode_t mode;
	dc1394feature_modes_t modes;
	dc1394error_t err;
	const char *functionName = "setFeatureMode";
	char *featureName = NULL;
	unsigned int i;

	/* First check if the feature is valid for this camera */
	status = this->checkFeature(pasynUser, addr, &featInfo, &featureName, functionName);
	if (status == asynError) return status;

	/* translate the PV value into a dc1394 mode enum... */
	if (value == 0) mode = DC1394_FEATURE_MODE_MANUAL;
	else mode = DC1394_FEATURE_MODE_AUTO;

	/* Check if the desired mode is even supported by the camera on this feature */
	err = dc1394_feature_get_modes (this->camera, featInfo->id, &modes);
	if (status == asynError) return status;
	for (i = 0; i < modes.num; i++) { if (modes.modes[i] == mode) break; }
	if (i >= modes.num)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s ERROR [%s] feature \'%s\' does not support mode %d\n",
					driverName, functionName, this->portName, featureName, mode);
		return asynError;
	}

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s got val=%d setting mode=%d\n",
				driverName, functionName, value, (int)mode);

	/* Send the feature mode to the cam */
	err = dc1394_feature_set_mode(this->camera, featInfo->id, mode);
	status = PERR(pasynUser, err);
	if (status == asynError) return status;

	/* if the caller is not interested in getting the readback, we won't collect it! */
	if (rbValue == NULL) return status;

	/* Finally read back the current value from the cam and set the readback */
	err = dc1394_feature_get_mode(this->camera, featInfo->id, &mode);
	status = PERR(pasynUser, err);
	if (status == asynError) return status;
	if (mode == DC1394_FEATURE_MODE_MANUAL) *rbValue = 0;
	else *rbValue = 1;
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s rbmode=%d rbValue=%d\n",
				driverName, functionName, (int)mode, *rbValue);
	return status;
}


asynStatus FirewireDCAM::setFeatureValue(asynUser *pasynUser, int addr, epicsInt32 value, epicsInt32 *rbValue)
{
	asynStatus status = asynSuccess;
	dc1394feature_info_t *featInfo;
	dc1394error_t err;
	epicsUInt32 min, max;
	const char *functionName = "setFeatureValue";
	char *featureName;
	int tmpVal;

	/* First check if the feature is valid for this camera */
	status = this->checkFeature(pasynUser, addr, &featInfo, (char**)&featureName, functionName);
	if (status == asynError) return status;

	/* Check the value is within the expected boundaries */
	err = dc1394_feature_get_boundaries (this->camera, featInfo->id, &min, &max);
	status = PERR(pasynUser, err);
	if(status == asynError) return status;
	if ((epicsUInt32)value < min || (epicsUInt32)value > max)
	{
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s ERROR [%s] setting feature %s, value %d is out of range [%d..%d]\n",
					driverName, functionName, this->portName, featureName, value, min, max);
		return asynError;
	}

	/* Check if the camera is set for manual control... */
	getIntegerParam(addr, FDC_feat_mode, &tmpVal);
	/* if it is not set to 'manual' (0) then we do set it to manual */
	if (tmpVal != 0) status = this->setFeatureMode(pasynUser, addr, 0, NULL);
	if(status == asynError) return status;

	/* Set the feature for non-absolute control in the camera */
	err = dc1394_feature_set_absolute_control(this->camera, featInfo->id, DC1394_OFF);
	status = PERR(NULL, err);
	if(status == asynError) return status;

	/* Set the feature value in the camera */
	err = dc1394_feature_set_value (this->camera, featInfo->id, (epicsUInt32)value);	
	status = PERR(NULL, err);
	if(status == asynError) return status;



	/* if the caller is not interested in getting the readback, we won't collect it! */
	if (rbValue != NULL)
	{
		/* Finally read back the value from the camera and set that as the new value */
		err = dc1394_feature_get_value(this->camera, featInfo->id, (epicsUInt32*)rbValue);
		status = PERR(NULL, err);
		if (status == asynError) return status;
		asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s set value to cam: %d readback value from cam: %d\n",
				driverName, functionName, value, *rbValue);
	} else
	{
		asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s set value to cam: %d\n",
				driverName, functionName, value);
	}
	return status;
}


asynStatus FirewireDCAM::setFeatureAbsValue(asynUser *pasynUser, int addr, epicsFloat64 value, epicsFloat64 *rbValue)
{
	asynStatus status = asynSuccess;
	dc1394feature_info_t *featInfo;
	dc1394error_t err;
	dc1394bool_t featAbsControl;
	float min, max;
	const char *functionName = "setFeatureAbsValue";
	char *featureName;
	int tmpVal;

	/* First check if the feature is valid for this camera */
	status = this->checkFeature(pasynUser, addr, &featInfo, (char**)&featureName, functionName);
	if (status == asynError) return status;

	/* Check if the specific feature supports absolute values */
	err = dc1394_feature_has_absolute_control (this->camera, featInfo->id, &featAbsControl);
	status = PERR(NULL, err);
	if(status == asynError) return status;
	if (!featAbsControl)
	{
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s ERROR [%s] setting feature \'%s\': No absolute control for this feature\n",
					driverName, functionName, this->portName, featureName);
		return asynError;
	}

	/* Check the value is within the expected boundaries */
	err = dc1394_feature_get_absolute_boundaries (this->camera, featInfo->id, &min, &max);
	status = PERR(NULL, err);
	if(status == asynError) return status;
	if (value < min || value > max)
	{
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s ERROR [%s] setting feature %s, value %.5f is out of range [%.3f..%.3f]\n",
					driverName, functionName, this->portName, featureName, value, min, max);
		return asynError;
	}

	/* Check if the camera is set for manual control... */
	getIntegerParam(addr, FDC_feat_mode, &tmpVal);
	/* if it is not set to 'manual' (0) then we do set it to manual */
	if (tmpVal != 0) status = this->setFeatureMode(pasynUser, addr, 0, NULL);
	if(status == asynError) return status;

	/* Set the feature for non-absolute control in the camera */
	err = dc1394_feature_set_absolute_control(this->camera, featInfo->id, DC1394_ON);
	status = PERR(NULL, err);
	if(status == asynError) return status;

	/* Finally set the feature value in the camera */
	err = dc1394_feature_set_absolute_value (this->camera, featInfo->id, (float)value);
	status = PERR(NULL, err);
	if(status == asynError) return status;

	/* if the caller is not interested in getting the readback, we won't collect it! */
	if (rbValue != NULL)
	{
		err = dc1394_feature_get_absolute_value (this->camera, featInfo->id, (float*)rbValue);
		status = PERR(NULL, err);
		asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s set value to cam: %.3f readback value from cam: %.3f\n",
				driverName, functionName, value, *rbValue);

	} else
	{
		asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s set value to cam: %.3f\n",
				driverName, functionName, value);
	}

	return status;
}

/** Set the framerate in the camera.
 *  Can set it with an enum when using the iframerate or as a double when using the dframerate parameter
 */
asynStatus FirewireDCAM::setFrameRate( asynUser *pasynUser, epicsInt32 iframerate)
{
	asynStatus status = asynSuccess;
	unsigned int newframerate = 0;
	dc1394error_t err;
	dc1394framerates_t framerates;
	unsigned int i;
	int wasAcquiring;
	float newdecimalframerate;
	dc1394video_mode_t videoMode;
	const char* functionName = "setFrameRate";

	asynPrint(pasynUser, ASYN_TRACE_FLOW, "FDC_framerate: setting value: %d\n", iframerate);

	/* if we set framerate to N/A we just return happily... */
	if (iframerate == 0) return asynSuccess;

	PERR( pasynUser, dc1394_video_get_mode(this->camera, &videoMode) );
	if (dc1394_is_video_mode_scalable (videoMode)) {
		/* This is a format 7 video mode, can't set frame rate */
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "FDC_framerate: This is a format 7 mode, doing nothing\n");
		return asynSuccess;
	}

	getIntegerParam(ADAcquire, &wasAcquiring);
	if (wasAcquiring)
	{
		status = this->stopCaptureAndWait(pasynUser);
		if (status == asynError) return status;
	}

	if (iframerate >= 1)
	{
		if (DC1394_FRAMERATE_MIN + iframerate -1 > DC1394_FRAMERATE_MAX ||
			DC1394_FRAMERATE_MIN + iframerate -1 < DC1394_FRAMERATE_MIN)
		{
			asynPrint( pasynUser, ASYN_TRACE_ERROR, "%s::%s ERROR [%s]: invalid framerate enum %d\n",
						driverName, functionName, this->portName, iframerate);
			return asynError;
		}
		newframerate = DC1394_FRAMERATE_MIN + (iframerate -1);
	}

	/* Translate enum framerate into human readable format.  */
	err = dc1394_framerate_as_float ((dc1394framerate_t)newframerate, &newdecimalframerate);
	PERR(pasynUser, err);

	/* check if selected framerate is even supported by the camera in this mode */
	err = dc1394_video_get_supported_framerates (this->camera, videoMode, &framerates);
	status = PERR(pasynUser, err);
	if (status == asynError) return status;
	for(i = 0; i < framerates.num; i++)
	{
		if ((unsigned int)framerates.framerates[i] == newframerate) break;
	}
	if (i == framerates.num)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s ERROR [%s]: camera does not support framerate %f [%d]\n",
					driverName, functionName, this->portName, newdecimalframerate, newframerate);
		return asynError;
	}

	/* attempt to write the framerate to camera */
	asynPrint( 	pasynUser, ASYN_TRACE_FLOW, "%s::%s [%s]: setting framerate: %.3f (%d)\n",
				driverName, functionName, this->portName, newdecimalframerate, newframerate - DC1394_FRAMERATE_MIN);
	err = dc1394_video_set_framerate(this->camera, (dc1394framerate_t)newframerate);
	if (PERR( pasynUser, err ) == asynError) return asynError;

	/* TODO: I would like to restart acquisition after changing the framerate,
	 *       however it seems to be a problem to signal the ADAcquire back to 1
	 *       automatically when called through this asyn command.
	 *       It may be that it is just because the Acquire record is just a bo and does not seem
	 *       to have a read back associated with it?
	 *       Or maybe a mutex need to be unlocked for a quick moment to allow setting ADAcquire
	 *       but I don't know when... */
	if (wasAcquiring) {
		/* It seems to take the firewirebus about 0.35s to receive the first frames after setting framerate.
		 * Timeout is 5*framerate, so only need to wait for 0.2s here, even at 30fps */
		epicsThreadSleep(0.2);
		this->startCapture(pasynUser);
	}

//	setDoubleParam(FDC_framerate, newdecimalframerate);
	return status;
}

asynStatus FirewireDCAM::setFrameRate( asynUser *pasynUser, epicsFloat64 dframerate)
{
	asynStatus status = asynSuccess;
	int iFramerate;
	const char* functionName = "setFrameRate";

	if (dframerate <= 2.0) iFramerate = 1;
	else if (dframerate <= 5.0) iFramerate = 2;
	else if (dframerate <= 12.0) iFramerate = 3;
	else if (dframerate <= 22.0) iFramerate = 4;
	else if (dframerate <= 45.0) iFramerate = 5;
	else if (dframerate <= 60.0) iFramerate = 6;
	else
	{
		asynPrint( pasynUser, ASYN_TRACE_ERROR, "%s::%s ERROR [%s]: invalid framerate %f\n",
					driverName, functionName, this->portName, dframerate);
		return asynError;
	}

	status = this->setFrameRate(pasynUser, iFramerate);
	if (status == asynError) return status;

	return status;
}

/** Read all the feature settings and values from the camera.
 * This function will collect all the current values and settings from the camera,
 * and set the appropriate integer/double parameters in the param lib. If a certain feature
 * is not available in the given camera, this function will set all the parameters relating to that
 * feature to -1 or -1.0 to indicate it is not available.
 * Note the caller is responsible for calling any update callbacks if I/O interrupts
 * are to be processed after calling this function.
 * \returns asynStatus asynError or asynSuccess as an int.
 */
int FirewireDCAM::getAllFeatures()
{
	int status = (int)asynSuccess;
	dc1394featureset_t features;
	dc1394feature_info_t* f;
	dc1394error_t err;
	int i, tmp, addr;
	double dtmp;

	err = dc1394_feature_get_all (this->camera, &features);
	status = PERR(this->pasynUserSelf, err);
	if(status == asynError) return status;

	/* Iterate through all of the available features and update their values and settings  */
	for (i = 0; i < this->maxAddr; i++)
	{
		f = &(features.feature[i]);
		addr = (int)f->id - (int)DC1394_FEATURE_MIN;

		// Say whether this feature is in manual or auto mode
		if (f->current_mode == DC1394_FEATURE_MODE_MANUAL) tmp = 0;
		else tmp = 1;
		status |= setIntegerParam(addr, FDC_feat_mode, tmp);

		/* If the feature is not available in the camera, we just set
		 * all the parameters to -1 to indicate this is not available to the user. */
		if (f->available == DC1394_FALSE)
		{
			tmp = -1;
			dtmp = -1.0;
			setIntegerParam(addr, FDC_feat_available, 0);
			setIntegerParam(addr, FDC_feat_val, tmp);
			setIntegerParam(addr, FDC_feat_val_min, tmp);
			setIntegerParam(addr, FDC_feat_val_max, tmp);
			setDoubleParam(addr, FDC_feat_val_abs, dtmp);
			setDoubleParam(addr, FDC_feat_val_abs_max, dtmp);
			setDoubleParam(addr, FDC_feat_val_abs_min, dtmp);
			continue;
		}

		status |= setIntegerParam(addr, FDC_feat_available, 1);
		status |= setIntegerParam(addr, FDC_feat_val, f->value);
		status |= setIntegerParam(addr, FDC_feat_val_min, f->min);
		status |= setIntegerParam(addr, FDC_feat_val_max, f->max);

		/* If the feature does not support 'absolute' control then we just
		 * set all the absolute values to -1.0 to indicate it is not available to the user */
		if (f->absolute_capable == DC1394_FALSE)
		{
			dtmp = -1.0;
			setIntegerParam(addr, FDC_feat_absolute, 0);
			setDoubleParam(addr, FDC_feat_val_abs, dtmp);
			setDoubleParam(addr, FDC_feat_val_abs_max, dtmp);
			setDoubleParam(addr, FDC_feat_val_abs_min, dtmp);
		} else {
			status |= setIntegerParam(addr, FDC_feat_absolute, 1);
			status |= setDoubleParam(addr, FDC_feat_val_abs, f->abs_value);
			status |= setDoubleParam(addr, FDC_feat_val_abs_max, f->abs_max);
			status |= setDoubleParam(addr, FDC_feat_val_abs_min, f->abs_min);
		}
		callParamCallbacks(addr);
	}

	/* Finally map a few of the AreaDetector parameters on to the camera 'features' */
	addr = DC1394_FEATURE_SHUTTER - DC1394_FEATURE_MIN;
	getDoubleParam(addr, FDC_feat_val_abs, &dtmp);
	status |= setDoubleParam(ADAcquireTime, dtmp);

	addr = DC1394_FEATURE_FRAME_RATE - DC1394_FEATURE_MIN;
	getDoubleParam(addr, FDC_feat_val_abs, &dtmp);
	if (dtmp<=0) dtmp=1;
	status |= setDoubleParam(ADAcquirePeriod, 1/dtmp);

	addr = DC1394_FEATURE_GAIN - DC1394_FEATURE_MIN;
	getDoubleParam(addr, FDC_feat_val_abs, &dtmp);
	status |= setDoubleParam(ADGain, dtmp);

	callParamCallbacks();

	return status;
}


asynStatus FirewireDCAM::startCapture(asynUser *pasynUser)
{
	asynStatus status = asynSuccess;
	dc1394error_t err;
	const char* functionName = "startCapture";

	epicsMutexLock(setupLock);
	err = dc1394_capture_setup(this->camera,FDC_DC1394_NUM_BUFFERS, DC1394_CAPTURE_FLAGS_DEFAULT);
	status = PERR( this->pasynUserSelf, err );
	if (status == asynError)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s [%s] Setting up capture failed... Staying in idle state.\n",
					driverName, functionName, this->portName);
		setIntegerParam(ADAcquire, 0);
		callParamCallbacks();
		epicsMutexUnlock(setupLock);		
		return status;
	}

	asynPrint( pasynUser, ASYN_TRACE_FLOW, "%s::%s [%s] Starting firewire transmission\n",
				driverName, functionName, this->portName);
	/* Start the camera transmission... */
	err=dc1394_video_set_transmission(this->camera, DC1394_ON);
	status = PERR( this->pasynUserSelf, err );
	if (status == asynError)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s [%s] starting transmission failed... Staying in idle state.\n",
					driverName, functionName, this->portName);
		setIntegerParam(ADAcquire, 0);
		callParamCallbacks();
		epicsMutexUnlock(setupLock);
		return status;
	}

	/* Signal the image grabbing thread that the acquisition/transmission has
	 * started and it can start dequeueing images from the driver buffer */
    epicsMutexUnlock(setupLock);
	setIntegerParam(ADNumImagesCounter, 0);
	setIntegerParam(ADAcquire, 1);
	epicsEventSignal(this->startEventId);
	return status;
}


asynStatus FirewireDCAM::stopCaptureAndWait(asynUser *pasynUser) {
	const char * functionName = "stopCaptureAndWait";
	epicsEventWaitStatus eventStatus;
	setIntegerParam(ADAcquire, 0);
	/* Now wait for the capture thread to actually stop */
	asynPrint( pasynUser, ASYN_TRACE_FLOW, "%s::%s [%s] waiting for stopped event...\n",
					driverName, functionName, this->portName);
	/* unlock the mutex while we're waiting for the capture thread to stop acquiring */
	this->unlock();
	eventStatus = epicsEventWaitWithTimeout(this->stopEventId, 10.0);
	this->lock();
	asynPrint( pasynUser, ASYN_TRACE_FLOW, "%s::%s [%s] Done waiting for stopped event.\n",
					driverName, functionName, this->portName);
	if (eventStatus != epicsEventWaitOK)
	{
		asynPrint( pasynUser, ASYN_TRACE_FLOW, "%s::%s [%s] ERROR: Timeout when trying to stop image grabbing thread.\n",
					driverName, functionName, this->portName);
	}
	return this->stopCapture(pasynUser);
}

/* called with lock taken */
asynStatus FirewireDCAM::stopCapture(asynUser *pasynUser) {
	asynStatus status = asynSuccess;
	dc1394error_t err;
	const char * functionName = "stopCapture";
	setIntegerParam(ADAcquire, 0);

	asynPrint( pasynUser, ASYN_TRACE_FLOW, "%s::%s [%s] Stopping firewire transmission\n",
				driverName, functionName, this->portName);

	/* Stop the actual transmission! */
	err=dc1394_video_set_transmission(this->camera, DC1394_OFF);
	status = PERR( pasynUser, err );
	asynPrint( pasynUser, ASYN_TRACE_FLOW, "%s::%s [%s]    - done stopping transmission\n",
				driverName, functionName, this->portName);
	err=dc1394_capture_stop(this->camera);
	status = PERR( pasynUser, err );
	asynPrint( pasynUser, ASYN_TRACE_FLOW, "%s::%s [%s]    - done stopping capture\n",
				driverName, functionName, this->portName);
	if (status == asynError)
	{
		/* if stopping transmission results in an error (weird situation!) we print a message
		 * but does not abort the function because we still want to set status to stopped...  */
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s [%s] Stopping transmission failed...\n",
					driverName, functionName, this->portName);
	}
	setIntegerParam(ADStatus, ADStatusAborting);
	return status;
}


/** Parse a dc1394 error code into a user readable string
 * Defaults to printing out using the pasynUser.
 * \param asynUser The asyn user to print out with on ASYN_TRACE_ERR. If pasynUser == NULL just print to stderr.
 * \param dc1394_err The error code, returned from the dc1394 function call. If the error code is OK we just ignore it.
 * \param errOriginLine Line number where the error came from.
 */
asynStatus FirewireDCAM::err( asynUser* asynUser, dc1394error_t dc1394_err, int errOriginLine)
{
	const char * errMsg = NULL;
	if (dc1394_err == 0) return asynSuccess; /* if everything is OK we just ignore it */

	/* retrieve the translation of the error code from the dc1394 library */
	errMsg = dc1394_error_get_string(dc1394_err);

	if (this->pasynUserSelf == NULL) fprintf(stderr, "### ERROR_%d [%d][%s]: dc1394 says: \"%s\" ###\n", dc1394_err, errOriginLine, this->portName, errMsg);
	else asynPrint( this->pasynUserSelf, ASYN_TRACE_ERROR, "### ERROR_%d [%d][%s]: dc1394 says: \"%s\" ###\n", dc1394_err, errOriginLine, this->portName, errMsg);
	return asynError;
}

/** Print out a report. Not yet implemented!
 * \param fp Stream or file pointer to write the report to.
 * \param details Configurable level of details in the report.
 * \return Nothing
 */
void FirewireDCAM::report(FILE *fp, int details)
{
	ADDriver::report(fp, details);
}

