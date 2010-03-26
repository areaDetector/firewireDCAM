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
 * asyn, areaDetector, dc1394 and raw1394 */
#include <ADDriver.h>

/* libdc1394 includes */
#include <dc1394/dc1394.h>

/** Print an errorcode to stderr. Convenience macro to be used when an asynUser is not yet available. */
#define ERR(errCode) if (errCode != 0) fprintf(stderr, "ERROR [%s:%d]: dc1394 code: %d\n", __FILE__, __LINE__, errCode)
/** Convenience macro to be used inside the firewireDCAM class. */
#define PERR(pasynUser, errCode) this->err(pasynUser, errCode, __LINE__)
/** Number of image buffers the dc1394 library will use internally */
#define FDC_DC1394_NUM_BUFFERS 5

/** Only used for debugging/error messages to identify where the message comes from*/
static const char *driverName = "FirewireDCAM";
/** dc1394 handle to the firewire bus.  */
static dc1394_t * dc1394fwbus;
/** List of dc1394 camera handles. */
static dc1394camera_list_t * dc1394camList;

/** Main driver class inherited from areaDetectors ADDriver class.
 * One instance of this class will control one firewire camera on the bus.
 */
class FirewireDCAM : public ADDriver
{
public:
	FirewireDCAM(	const char *portName, const char* camid, int speed,
					int maxBuffers, size_t maxMemory, int colour );

	/* virtual methods to override from ADDriver */
	virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value);
	virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value);
	virtual asynStatus drvUserCreate( asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize);
	void report(FILE *fp, int details);

	/* Local methods to this class */
	asynStatus err( asynUser* asynUser, dc1394error_t dc1394_err, int errOriginLine);
	void imageGrabTask();
	int grabImage();
	asynStatus startCapture(asynUser *pasynUser);
	asynStatus stopCapture(asynUser *pasynUser);


	/* camera feature control functions */
	asynStatus setFeatureValue(asynUser *pasynUser, epicsInt32 value, epicsInt32 *rbValue);
	asynStatus setFeatureAbsValue(asynUser *pasynUser, epicsFloat64 value, epicsFloat64 *rbValue);
	asynStatus setFeatureMode(asynUser *pasynUser, epicsInt32 value, epicsInt32 *rbValue);
	asynStatus checkFeature(asynUser *pasynUser, dc1394feature_info_t **featInfo, char** featureName, const char* functionName);
	asynStatus setFrameRate( asynUser *pasynUser, epicsInt32 iframerate);
	asynStatus setFrameRate( asynUser *pasynUser, epicsFloat64 dframerate);
	int getAllFeatures();


	/* Data */
	NDArray *pRaw;
	dc1394camera_t *camera;
	dc1394video_mode_t video_mode;
	dc1394color_coding_t colour_coding;
    epicsEventId startEventId;
    epicsEventId stopEventId;
    dc1394featureset_t features;
    int colour;

};
/* end of FirewireDCAM class description */

typedef struct camNode_t {
	ELLNODE node;
	uint32_t generation;
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
		printf("cam ID: %16.16llX", list->ids[i].guid);
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
    unsigned int i;

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

	printf("Found %d cameras the bus:\n", dc1394camList->num);
	for (i = 0; i < dc1394camList->num; i++)
	{
		printf("GUID:\t0x%16.16llX\n", dc1394camList->ids[i].guid);
	}

	printf("\n");
	return 0;
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
 * \param colour Colour mode to run the camera in. 0 = MONO8, 1 = RGB8. Remember to multiply maxMemory by 3 if you set this
 */
extern "C" int FDC_Config(const char *portName, const char* camid, int speed, int maxBuffers, size_t maxMemory, int colour)
{
	new FirewireDCAM( portName, camid, speed, maxBuffers, maxMemory, colour);
	return asynSuccess;
}


/** Specific asyn commands for this support module. These will be used and
 * managed by the parameter library (part of areaDetector). */
typedef enum FDCParam_t {
	FDC_feat_val = ADLastStdParam, /** Feature value (int32 read/write) addr: 0-17 */
	FDC_feat_val_max,                  /** Feature maximum boundry value (int32 read) addr: 0-17 */
	FDC_feat_val_min,                  /** Feature minimum boundry value (int32 read)  addr: 0-17*/
	FDC_feat_val_abs,                  /** Feature absolute value (float64 read/write) addr: 0-17 */
	FDC_feat_val_abs_max,              /** Feature absolute maximum boundry value (float64 read) addr: 0-17 */
	FDC_feat_val_abs_min,              /** Feature absolute minimum boundry value (float64 read) addr: 0-17 */
	FDC_feat_mode,                     /** Feature control mode: 0:manual or 1:automatic (camera controlled) (int32 read/write)*/
	FDC_feat_available,                /** Is a given featurea available in the camera 1=available 0=not available (int32, read) */
	FDC_feat_absolute,                 /** Feature has absolute (floating point) controls available 1=available 0=not available (int32 read) */
	FDC_framerate,                     /** Set and read back the frame rate (float64 and int32 (enums) read/write)*/
	ADLastDriverParam
	} FDCParam_t;

static asynParamString_t FDCParamString[] = {
	{FDC_feat_val,           "FDC_FEAT_VAL"},
	{FDC_feat_val_max,       "FDC_FEAT_VAL_MAX"},
	{FDC_feat_val_min,       "FDC_FEAT_VAL_MIN"},
	{FDC_feat_val_abs,       "FDC_FEAT_VAL_ABS"},
	{FDC_feat_val_abs_max,   "FDC_FEAT_VAL_ABS_MAX"},
	{FDC_feat_val_abs_min,   "FDC_FEAT_VAL_ABS_MIN"},
	{FDC_feat_mode,          "FDC_FEAT_MODE"},
	{FDC_feat_available,     "FDC_FEAT_AVAILABLE"},
	{FDC_feat_absolute,      "FDC_FEAT_ABSOLUTE"},
	{FDC_framerate,          "FDC_FRAMERATE"},
	};

/** Feature mapping from DC1394 library enums to a local driver enum
 * The local driver identifies a feature based on the address of the asyn request.
 * The address range is [0..DC1394_FEATURE_NUM-1] and the dc1394 feature enum starts
 * at an offset of DC1394_FEATURE_MIN... */
#define FDC_DC1394_FEATOFFSET DC1394_FEATURE_MIN

/** Number of asyn parameters (asyn commands) this driver supports. */
#define FDC_N_PARAMS (sizeof( FDCParamString)/ sizeof(FDCParamString[0]))

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
 * \param colour 0 for MONO8 mode, 1 for RGB8 mode
 */
FirewireDCAM::FirewireDCAM(	const char *portName, const char* camid, int speed,
							int maxBuffers, size_t maxMemory, int colour )
	: ADDriver(portName, DC1394_FEATURE_NUM, ADLastDriverParam, maxBuffers, maxMemory,
			0, 0, // interfacemask and interruptmask
			ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, // asynflags and autoconnect,
			0, 0),// thread priority and stack size
		pRaw(NULL)
{
	const char *functionName = "FirewireDCAM";
	int dimensions[2];
	unsigned int sizeX, sizeY;
	unsigned int i, status, ret;
	unsigned long long int camUID = 0;
	dc1394operation_mode_t opMode;
	dc1394speed_t opSpeed;
	char chMode = 'A';
	unsigned int busSpeed;

	dc1394error_t err;
	dc1394camera_t *camera = NULL;
	dc1394video_modes_t video_modes;
	dc1394color_coding_t colour_coding;

	/* parse the string of hex-numbers that is the cameras unique ID */
	ret = sscanf(camid, "0x%16llX", &camUID);

	/* configure the camera to the mode and so on... */
	for (i = 0; i < dc1394camList->num; i++)
	{
		/* See if we can find the camera on the bus with the specific ID */
		if (camUID == dc1394camList->ids[i].guid)
		{
			/* initialise the camera on the bus */
			camera = dc1394_camera_new (dc1394fwbus, dc1394camList->ids[i].guid);
			/*printf("cameraInit: Using camera with GUID %llX\n", camera->guid);*/
			break;
		}
	}

	/* If we didn't find the camera with the specific ID we return... */
	if (camera == NULL)
	{
		fprintf(stderr,"### ERROR ### Did not find camera with GUID: 0x%16.16llX\n", camUID);
		return;
	} else this->camera = camera;

	/* Let dc1394 print out the camera info to stdout. This could possibly be
	 * located in the report function instead but I like to see the info at startup */
	err=dc1394_camera_print_info(this->camera, stdout);
	ERR( err );

	/* Find the video mode and colour coding with highest resolution and MONO8 color coding.
	 * TODO: In this revision we ignore the better 'scalable modes' (format7)
	 *       and just go for the simple mode with the largest MONO8 image.
	 *       Use of format7 is one of the first upgrades to implement as it
	 *       significantly decreases the used bandwidth when only using a small
	 *       ROI on the CCD. */
	err=dc1394_video_get_supported_modes(this->camera, &video_modes);
	ERR( err );
	/* Loop through all modes backwards as they are defined in descending order of image size */
	for (i = video_modes.num-1; i >= 0; i--)
	{
		dc1394_get_color_coding_from_video_mode(camera,video_modes.modes[i], &colour_coding);

		/* If the video mode is not scalable AND the colour coding is 8bit MONO,
		 * we've found our required settings! */
		if (	!dc1394_is_video_mode_scalable(video_modes.modes[i])
				&& (( colour == 0 && colour_coding == DC1394_COLOR_CODING_MONO8) ||
	    		    ( colour == 1 && colour_coding == DC1394_COLOR_CODING_RGB8)) )
		{
			this->video_mode = video_modes.modes[i];
			this->colour_coding = colour_coding;
			this->colour = colour;
			/* The first one we find is the largest image so we just break
			 * out of the loop when we find a match */
			break;
		}
	}
	/* If we went through all video modes and didn't find a suitable mode... */
	if (i < 0)
	{
		fprintf(stderr, "### ERROR ### Could not get a valid MONO8 mode\n");
		return;
	}

	/* Getting all available features and their information from the camera */
	err = dc1394_feature_get_all (this->camera, &(this->features));
	ERR(err);


	/* writing the selected video mode to the camera */
	printf("Setting video mode: %d...               ", this->video_mode);
	fflush(stdout);
	err=dc1394_video_set_mode(this->camera, this->video_mode );
	ERR( err );
	printf("OK\n");

	/* Get the image size from our current video mode */
	printf("Getting image dimensions from mode...   ");
	fflush(stdout);
	err = dc1394_get_image_size_from_video_mode(this->camera, this->video_mode, &sizeX, &sizeY);
	dimensions[0] = (int)sizeX;
	dimensions[1] = (int)sizeY;
	ERR( err );
	printf("%dx%d\n", dimensions[0], dimensions[1]);


	if (speed == 800)
	{
		opMode = DC1394_OPERATION_MODE_1394B;
		opSpeed = DC1394_ISO_SPEED_800;
		chMode = 'B';
		busSpeed = 800;
	} else
	{
		opMode = DC1394_OPERATION_MODE_LEGACY;
		opSpeed = DC1394_ISO_SPEED_400;
		chMode = 'A';
		busSpeed = 400;
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

	printf("Setting ISO speed to %dMb/s...         ", busSpeed);
	fflush(stdout);
	err=dc1394_video_set_iso_speed(this->camera, opSpeed);
	ERR( err );
	printf("OK\n");

	printf("Setting framerate (7.5): %d...          ", DC1394_FRAMERATE_7_5);
	fflush(stdout);
	err=dc1394_video_set_framerate(this->camera, DC1394_FRAMERATE_7_5);
	ERR( err );
	printf("OK\n");

	printf("Preparing capture...                    ");
	fflush(stdout);
	err=dc1394_capture_setup(this->camera,FDC_DC1394_NUM_BUFFERS, DC1394_CAPTURE_FLAGS_DEFAULT);
	ERR( err );
	printf("OK\n");


	/* Create the start and stop event that will be used to signal our
	 * image grabbing thread when to start/stop	 */
	printf("Creating epicsevents...                 ");
	this->startEventId = epicsEventCreate(epicsEventEmpty);
	this->stopEventId = epicsEventCreate(epicsEventFull);
	printf("OK\n");

	/* Set the parameters from the camera in our areaDetector param lib */
	printf("Setting the areaDetector parameters...  ");
	fflush(stdout);
    status =  setStringParam (ADManufacturer, this->camera->vendor);
    status |= setStringParam (ADModel, this->camera->model);
    status |= setIntegerParam(ADMaxSizeX, dimensions[0]);
    status |= setIntegerParam(ADMaxSizeY, dimensions[1]);
    status |= setIntegerParam(ADSizeX, dimensions[0]);
    status |= setIntegerParam(ADSizeY, dimensions[1]);

    status |= setIntegerParam(ADImageMode, ADImageContinuous);
    status |= setIntegerParam(ADNumImages, 100);
    status |= this->getAllFeatures();
    if (status)
    {
         fprintf(stderr, "ERROR %s: unable to set camera parameters\n", functionName);
         return;
    } else printf("OK\n");

	/* Start up acquisition thread */
    printf("Starting up image grabbing task...     ");
	fflush(stdout);
    status = (epicsThreadCreate("imageGrabTask",
    		epicsThreadPriorityMedium,
    		epicsThreadGetStackSize(epicsThreadStackMedium),
    		(EPICSTHREADFUNC)imageGrabTaskC,
    		this) == NULL);
    if (status) {
    	printf("%s:%s epicsThreadCreate failure for image task\n",
    			driverName, functionName);
    	return;
    } else printf("OK\n");
    printf("Configuration complete!\n");
	return;
}


/** Task to grab images off the camera and send them up to areaDetector
 *
 */
void FirewireDCAM::imageGrabTask()
{
	int status = asynSuccess;
	int numImages, numImagesCounter;
	int imageMode;
	int arrayCallbacks;
	int maxDims[3];
	NDDataType_t dataType;
	epicsTimeStamp startTime;
	int acquire, addr;
	dc1394error_t err;
	int externalStopCmd = 0;
	const char *functionName = "imageGrabTask";

	printf("FirewireDCAM::imageGrabTask: Got the image grabbing thread started!\n");

	epicsEventTryWait(this->stopEventId); /* clear the stop event if it wasn't already */

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

			if (externalStopCmd)
			{
				/* Signal someone that the thread is really stopping to wait for start event */
				asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
									"%s::%s [%s]: Signalling stop event\n", driverName, functionName, this->portName);
				epicsEventSignal(this->stopEventId);
			} else externalStopCmd = 1;

			/* Release the lock while we wait for an event that says acquire has started, then lock again */
			this->unlock();


			/* Wait for a signal that tells this thread that the transmission
			 * has started and we can start asking for image buffers...	 */
			asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
					"%s::%s [%s]: waiting for acquire to start\n", driverName, functionName, this->portName);
			status = epicsEventWait(this->startEventId);
			asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
					"%s::%s [%s]: started!\n", driverName, functionName, this->portName);
			this->lock();
			setIntegerParam(ADNumImagesCounter, 0);
			setIntegerParam(ADAcquire, 1);
		}

		/* Get the current time */
		epicsTimeGetCurrent(&startTime);
		/* We are now waiting for an image  */
		setIntegerParam(ADStatus, ADStatusWaiting);
		/* Call the callbacks to update any changes */
		callParamCallbacks();

		/* Allocate an NDArray for dumping the dc1394 bus data into */
		getIntegerParam(NDDataType, (int *)&dataType);
		if (this->colour == 1)
		{
			maxDims[0] = 3;
			getIntegerParam(ADMaxSizeX, &maxDims[1]);
			getIntegerParam(ADMaxSizeY, &maxDims[2]);
		}
		else
		{
			getIntegerParam(ADMaxSizeX, &maxDims[0]);
			getIntegerParam(ADMaxSizeY, &maxDims[1]);
		}

	    this->pRaw = this->pNDArrayPool->alloc(2+this->colour, maxDims, dataType, 0, NULL);
		if (this->colour == 1)
		{
			setIntegerParam(NDColorMode, NDColorModeRGB1);
		}
		else
		{
			setIntegerParam(NDColorMode, NDColorModeMono);
		}

	    if (!this->pRaw)
	    {
	    	/* If we didn't get a valid buffer from the NDArrayPool we must abort
	    	 * the acquisition as we have nowhere to dump the data...  	 */
			setIntegerParam(ADStatus, ADStatusAborting);
			callParamCallbacks();
	    	asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s [%s] ERROR: Serious problem: not enough buffers left! Aborting acquisition!\n",
						driverName, functionName, this->portName);
			setIntegerParam(ADAcquire, 0);
			err=dc1394_video_set_transmission(this->camera, DC1394_OFF);
			PERR( this->pasynUserSelf, err );
			continue;
	    }

		status = this->grabImage();		/* #### GET THE IMAGE FROM CAMERA HERE! ##### */
		if (status == asynError) 		/* check for error */
		{
			/* remember to release the NDArray back to the pool now
			 * that we are not using it (we didn't get an image...) */
			if(this->pRaw) this->pRaw->release();
			/* We abort if we had some problem with grabbing an image...
			 * This is perhaps not always the desired behaviour but it'll do for now. */
			setIntegerParam(ADStatus, ADStatusAborting);
			this->stopCapture(this->pasynUserSelf);
			continue;
		}

		/* Set a bit of image/frame statistics... */
		getIntegerParam(ADNumImages, &numImages);
		getIntegerParam(ADNumImagesCounter, &numImagesCounter);
		getIntegerParam(ADImageMode, &imageMode);
		getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
		numImagesCounter++;
		setIntegerParam(ADNumImagesCounter, numImagesCounter);
		/* Put the frame number into the buffer */
		this->pRaw->uniqueId = numImagesCounter;
		/* Set a timestamp in the buffer */
		/* This is disabled now as we set the timestamp from the dc1394_frame
		 * in the buffer instead...		 */
		//this->pRaw->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;

		/* Call the callbacks to update any changes */
		callParamCallbacks();

		/* Get some information about the current feature settings */
		this->getAllFeatures();
		/* issue the callbacks for all the updated feature values */
		for (addr=0; addr < this->maxAddr; addr++) callParamCallbacks(addr, addr);

		if (arrayCallbacks)
		{
			/* Call the NDArray callback */
			/* Must release the lock here, or we can get into a deadlock, because we can
			 * block on the plugin lock, and the plugin can be calling us */
			this->unlock();
			doCallbacksGenericPointer(this->pRaw, NDArrayData, 0);
			this->lock();
		}
		/* Release the NDArray buffer now that we are done with it.
		 * After the callback just above we don't need it anymore */
		this->pRaw->release();
		this->pRaw = NULL;

		/* See if acquisition is done if we are in single or multiple mode */
		if ((imageMode == ADImageSingle) || ((imageMode == ADImageMultiple) && (numImagesCounter >= numImages)))
		{
			/* command the camera to stop acquiring.. */
			setIntegerParam(ADAcquire, 0);
			externalStopCmd = 0;
			err=dc1394_video_set_transmission(this->camera, DC1394_OFF);
			status = PERR( this->pasynUserSelf, err );
			if (status == asynError)
			{
				asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s [%s] Stopping transmission failed...\n",
							driverName, functionName, this->portName);
			}
		}
	}/* back to the top... */
	return;
}

/** Grabs one image off the dc1394 queue, notifies areaDetector about it and
 * finally clears the buffer off the dc1394 queue.
 * This function expects the mutex to be locked already by the caller!
 */
int FirewireDCAM::grabImage()
{
	int status = asynSuccess;
	NDDataType_t dataType;
	int binX, binY, minX, minY, sizeX, sizeY, reverseX, reverseY;
	int maxSizeX, maxSizeY;
	NDArrayInfo_t arrayInfo;
	unsigned char * pTmpData;

	dc1394video_frame_t * dc1394_frame;
	dc1394error_t err;
	const char* functionName = "grabImage";

	status |= getIntegerParam(ADBinX,         &binX);
	status |= getIntegerParam(ADBinY,         &binY);
	status |= getIntegerParam(ADMinX,         &minX);
	status |= getIntegerParam(ADMinY,         &minY);
	status |= getIntegerParam(ADSizeX,        &sizeX);
	status |= getIntegerParam(ADSizeY,        &sizeY);
	status |= getIntegerParam(ADReverseX,     &reverseX);
	status |= getIntegerParam(ADReverseY,     &reverseY);
	status |= getIntegerParam(ADMaxSizeX,     &maxSizeX);
	status |= getIntegerParam(ADMaxSizeY,     &maxSizeY);
	status |= getIntegerParam(NDDataType,     (int *)&dataType);
	if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
			"%s:%s: error getting parameters\n",
			driverName, functionName);

	/* Make sure parameters are consistent, fix them if they are not */
	if (binX < 1) { binX = 1; status |= setIntegerParam(ADBinX, binX); }
	if (binY < 1) { binY = 1; status |= setIntegerParam(ADBinY, binY); }
	if (minX < 0) { minX = 0; status |= setIntegerParam(ADMinX, minX); }
	if (minY < 0) { minY = 0;  status |= setIntegerParam(ADMinY, minY); }
	if (minX > maxSizeX-1) { minX = maxSizeX-1; status |= setIntegerParam(ADMinX, minX); }
	if (minY > maxSizeY-1) { minY = maxSizeY-1; status |= setIntegerParam(ADMinY, minY); }
	if (minX+sizeX > maxSizeX) { sizeX = maxSizeX-minX; status |= setIntegerParam(ADSizeX, sizeX); }
	if (minY+sizeY > maxSizeY) { sizeY = maxSizeY-minY; status |= setIntegerParam(ADSizeY, sizeY); }

	/* unlock the mutex while we wait for a new image to be ready */
	this->unlock();
	err = dc1394_capture_dequeue(this->camera, DC1394_CAPTURE_POLICY_WAIT, &dc1394_frame);
	status = PERR( this->pasynUserSelf, err );
	this->lock();
	if (status) return status;   /* if we didn't get an image properly... */

	/* tell our driver where to find the dc1394 image buffer with this latest image */
	pTmpData = dc1394_frame->image;

	/* Change the status to be readout... */
	setIntegerParam(ADStatus, ADStatusReadout);
	callParamCallbacks();

	/* If size parameters are different from what we get from the frame buffer we correct them */
	if (sizeX != (int)dc1394_frame->size[0]) { sizeX = dc1394_frame->size[0]; status |= setIntegerParam(ADSizeX, sizeX); }
	if (sizeY != (int)dc1394_frame->size[1]) { sizeY = dc1394_frame->size[1]; status |= setIntegerParam(ADSizeY, sizeY); }
	status |= setIntegerParam(NDArraySizeX, (int)dc1394_frame->size[0]);
	status |= setIntegerParam(NDArraySizeY, (int)dc1394_frame->size[1]);
	status |= setIntegerParam(NDArraySize, (int)dc1394_frame->image_bytes);

	/* copy the data from the dc1394 frame into our raw driver buffer */
	memcpy(this->pRaw->pData, dc1394_frame->image, dc1394_frame->image_bytes);

	this->pRaw->timeStamp = ((double)dc1394_frame->timestamp)/1000000.0;

	/* Free the frame off the dc1394 queue. We can do this now as
	 * we have just taken a local copy of the data.  */
	err=dc1394_capture_enqueue(this->camera, dc1394_frame);
	status |= PERR( this->pasynUserSelf, err );

	/* let's just check we have some valid data in our output buffer... */
	pTmpData = (unsigned char*)this->pRaw->pData;
	this->pRaw->getInfo(&arrayInfo);
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
	int adstatus;
	//dc1394error_t err;
	int addr, rbValue, tmpVal;
	pasynManager->getAddr(pasynUser, &addr);

	rbValue = value;

	switch(function)
	{
	case ADAcquire:
		getIntegerParam(ADStatus, &adstatus);
		if (value && (adstatus == ADStatusIdle))
		{
			/* start acquisition */
			status = this->startCapture(pasynUser);
		} else if (!value)
		{
			status = this->stopCapture(pasynUser);
		}
		break;

	case ADImageMode:
	case ADNumImages:
	case ADAcquirePeriod:
	case ADNumImagesCounter:
	case ADTriggerMode:
	case NDArrayCallbacks:

	// Reversing X and Y has no effect anyway.
	// If the user want to reverse the image he can use the ROI plug-in
	//case ADReverseX:
	//case ADReverseY:

		break;

	case FDC_feat_val:
		/* First check if the camera is set for manual control... */
		getIntegerParam(addr, FDC_feat_mode, &tmpVal);
		/* if it is not set to 'manual' (0) then we do set it to manual */
		if (tmpVal != 0) status = this->setFeatureMode(pasynUser, 0, NULL);
		if (status == asynError) break;

		/* now send the feature value to the camera */
		status = this->setFeatureValue(pasynUser, value, &rbValue);
		if (status == asynError) break;

		/* update all feature values to check if any settings have changed */
		status = (asynStatus) this->getAllFeatures();
		break;

	case FDC_feat_mode:
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "FDC_feat_mode: setting value: %d\n", value);
		status = this->setFeatureMode(pasynUser, value, &rbValue);
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "FDC_feat_mode: readback value: %d\n", rbValue);
		break;

	case FDC_framerate:
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "FDC_framerate: setting value: %d\n", value);
		status = this->setFrameRate(pasynUser, value);
		break;

	default:
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s Function not implemented: %d; val=%d\n",
					driverName, "writeInt32", function, value);
		status = asynError;
		break;
	}

	//if (status != asynError) status = setIntegerParam(function, value);
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "writeInt32: addr=%d function=%d rbvalue=%d\n", addr, function, rbValue);
	if (status != asynError) status = setIntegerParam(addr, function, rbValue);
	//if (status != asynError) callParamCallbacks();
	/* Call the callback for the specific address .. and address ... weird? */
	if (status != asynError) callParamCallbacks(addr, addr);
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
	epicsFloat64 rbValue;
	int addr, tmpVal;
	pasynManager->getAddr(pasynUser, &addr);

	switch(function)
	{
	case FDC_feat_val_abs:
		/* First check if the camera is set for manual control... */
		getIntegerParam(addr, FDC_feat_mode, &tmpVal);
		/* if it is not set to 'manual' (0) then we do set it to manual */
		if (tmpVal != 0) status = this->setFeatureMode(pasynUser, 0, NULL);
		if (status == asynError) break;

		status = this->setFeatureAbsValue(pasynUser, value, &rbValue);
		if (status == asynError) break;
		/* update all feature values to check if any settings have changed */
		status = (asynStatus) this->getAllFeatures();
		break;

	default:
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s Function not implemented for function %d; val=%.4f\n",
					"FirewireDCAM", "writeFloat64", function, value);
		status = asynError;
		break;
	}

	if (status != asynError) status = setDoubleParam(addr, function, rbValue);
	if (status != asynError) callParamCallbacks(addr, addr);
	return status;
}

/** Check if a requested feature is valid
 *
 * Checks for:
 * <ol>
 * <li>Valid range of the asyn request address against feature index.
 * <li>Availability of the requested feature in the current camera.
 * </ol>
 * \param pasynUser
 * \param featInfo Pointer to a feature info structure pointer. The function
 *        will write a valid feature info struct into this pointer or NULL on error.
 * \param featureName The function will return a string in this parameter with a
 *                    readable name for the given feature.
 * \param functionName The caller can pass a string which contain the callers function
 *                     name. For debugging/printing purposes only.
 * \return asyn status
 */
asynStatus FirewireDCAM::checkFeature(	asynUser *pasynUser, dc1394feature_info_t **featInfo,
										char** featureName, const char* functionName)
{
	asynStatus status = asynSuccess;
	dc1394feature_info_t *tmpFeatInfo;
	int addr;
	const char* localFunctionName = "checkFeature";

	*featInfo = NULL;/* set the default return pointer to nothing... */
	if (functionName == NULL) functionName = localFunctionName;

	pasynManager->getAddr(pasynUser, &addr);

	if (addr < 0 || addr >= DC1394_FEATURE_NUM)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s ERROR addr: %d is out of range [0..%d]\n",
					driverName, functionName, addr, DC1394_FEATURE_NUM);
		return asynError;
	}
	tmpFeatInfo = &(this->features.feature[addr]);

	/* Get a readable name for the feature we are working on */
	if (*featureName != NULL)
	{
		*featureName = (char*)dc1394_feature_get_string (tmpFeatInfo->id);
		if (status != asynSuccess) return status;
	}

	/* check if the feature we are working on is even available in this camera */
	if (!tmpFeatInfo->available)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s ERROR Port \'%s\' Feature \'%s\' is not available in camera\n",
					driverName, functionName, this->portName, dc1394_feature_get_string (tmpFeatInfo->id));
		return asynError;
	}

	*featInfo = tmpFeatInfo;
	return status;
}

asynStatus FirewireDCAM::setFeatureMode(asynUser *pasynUser, epicsInt32 value, epicsInt32 *rbValue)
{
	asynStatus status = asynSuccess;
	dc1394feature_info_t *featInfo;
	dc1394feature_mode_t mode;
	dc1394feature_modes_t modes;
	dc1394error_t err;
	const char *functionName = "setFeatureMode";
	char *featureName = NULL;
	unsigned int i;

	featureName = (char*)calloc(255, sizeof(char));

	/* First check if the feature is valid for this camera */
	status = this->checkFeature(pasynUser, &featInfo, &featureName, functionName);
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


asynStatus FirewireDCAM::setFeatureValue(asynUser *pasynUser, epicsInt32 value, epicsInt32 *rbValue)
{
	asynStatus status = asynSuccess;
	dc1394feature_info_t *featInfo;
	dc1394error_t err;
	epicsUInt32 min, max;
	const char *functionName = "setFeatureValue";
	char featureName[255];

	/* First check if the feature is valid for this camera */
	status = this->checkFeature(pasynUser, &featInfo, (char**)&featureName, functionName);
	if (status == asynError) return status;

	/* Check the value is within the expected boundaries */
	err = dc1394_feature_get_boundaries (this->camera, featInfo->id, &min, &max);
	status = PERR(pasynUser, err);
	if(status == asynError) return status;
	if ((epicsUInt32)value < min || (epicsUInt32)value > max)
	{
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s ERROR [%s] setting feature %s, value %d is out of range [%d..%d]\n",
					driverName, functionName, this->portName, featureName, value, min, max);
		return asynError;
	}

	/* Set the feature value in the camera */
	err = dc1394_feature_set_value (this->camera, featInfo->id, (epicsUInt32)value);
	status = PERR(pasynUser, err);
	if(status == asynError) return status;

	/* if the caller is not interested in getting the readback, we won't collect it! */
	if (rbValue != NULL)
	{
		/* Finally read back the value from the camera and set that as the new value */
		err = dc1394_feature_get_value(this->camera, featInfo->id, (epicsUInt32*)rbValue);
		status = PERR(pasynUser, err);
		if (status == asynError) return status;
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s set value to cam: %d readback value from cam: %d\n",
				driverName, functionName, value, *rbValue);
	} else
	{
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s set value to cam: %d\n",
				driverName, functionName, value);
	}
	return status;
}


asynStatus FirewireDCAM::setFeatureAbsValue(asynUser *pasynUser, epicsFloat64 value, epicsFloat64 *rbValue)
{
	asynStatus status = asynSuccess;
	dc1394feature_info_t *featInfo;
	dc1394error_t err;
	dc1394bool_t featAbsControl;
	float min, max;
	const char *functionName = "setFeatureAbsValue";
	char featureName[255];

	/* First check if the feature is valid for this camera */
	status = this->checkFeature(pasynUser, &featInfo, (char**)&featureName, functionName);
	if (status == asynError) return status;

	/* Check if the specific feature supports absolute values */
	err = dc1394_feature_has_absolute_control (this->camera, featInfo->id, &featAbsControl);
	status = PERR(pasynUser, err);
	if(status == asynError) return status;
	if (!featAbsControl)
	{
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s ERROR [%s] setting feature \'%s\': No absolute control for this feature\n",
					driverName, functionName, this->portName, featureName);
		return asynError;
	}

	/* Check the value is within the expected boundaries */
	err = dc1394_feature_get_absolute_boundaries (this->camera, featInfo->id, &min, &max);
	status = PERR(pasynUser, err);
	if(status == asynError) return status;
	if (value < min || value > max)
	{
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s ERROR [%s] setting feature %s, value %.5f is out of range [%.3f..%.3f]\n",
					driverName, functionName, this->portName, featureName, value, min, max);
		return asynError;
	}

	/* Finally set the feature value in the camera */
	err = dc1394_feature_set_absolute_value (this->camera, featInfo->id, (float)value);
	status = PERR(pasynUser, err);
	if(status == asynError) return status;

	/* if the caller is not interested in getting the readback, we won't collect it! */
	if (rbValue != NULL)
	{
		err = dc1394_feature_get_absolute_value (this->camera, featInfo->id, (float*)rbValue);
		status = PERR(pasynUser, err);
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s set value to cam: %.3f readback value from cam: %.3f\n",
				driverName, functionName, value, *rbValue);

	} else
	{
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s::%s set value to cam: %.3f\n",
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
	const char* functionName = "setFrameRate";

	/* if we set framerate to N/A we just return happily... */
	if (iframerate == 0) return asynSuccess;

	getIntegerParam(ADAcquire, &wasAcquiring);
	if (wasAcquiring)
	{
		status = this->stopCapture(pasynUser);
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
	err = dc1394_video_get_supported_framerates (this->camera, this->video_mode, &framerates);
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
//	if (wasAcquiring) { this->startCapture(pasynUser); }

	setDoubleParam(FDC_framerate, newdecimalframerate);
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

		/* If the feature is not available in the camera, we just set
		 * all the parameters to -1 to indicate this is not available to the user. */
		if (f->available == DC1394_FALSE)
		{
			status |= setIntegerParam(addr, FDC_feat_available, 0);
			tmp = -1;
			dtmp = -1.0;
			setIntegerParam(addr, FDC_feat_val, tmp);
			setIntegerParam(addr, FDC_feat_val_min, tmp);
			setIntegerParam(addr, FDC_feat_val_max, tmp);
			//setIntegerParam(addr, FDC_feat_mode, tmp);
			setDoubleParam(addr, FDC_feat_val_abs, dtmp);
			setDoubleParam(addr, FDC_feat_val_abs_max, dtmp);
			setDoubleParam(addr, FDC_feat_val_abs_min, dtmp);
			continue;
		}

		status |= setIntegerParam(addr, FDC_feat_available, 1);
		status |= setIntegerParam(addr, FDC_feat_val, f->value);
		status |= setIntegerParam(addr, FDC_feat_val_min, f->min);
		status |= setIntegerParam(addr, FDC_feat_val_max, f->max);

		if (f->current_mode == DC1394_FEATURE_MODE_MANUAL) tmp = 0;
		else tmp = 1;
		status |= setIntegerParam(addr, FDC_feat_mode, tmp);

		/* If the feature does not support 'absolute' control then we just
		 * set all the absolute values to -1.0 to indicate it is not available to the user */
		if (f->absolute_capable == DC1394_FALSE)
		{
			dtmp = -1.0;
			status |= setIntegerParam(addr, FDC_feat_absolute, 0);
			setDoubleParam(addr, FDC_feat_val_abs, dtmp);
			setDoubleParam(addr, FDC_feat_val_abs_max, dtmp);
			setDoubleParam(addr, FDC_feat_val_abs_min, dtmp);
			continue;
		}
		status |= setIntegerParam(addr, FDC_feat_absolute, 1);
		status |= setDoubleParam(addr, FDC_feat_val_abs, f->abs_value);
		status |= setDoubleParam(addr, FDC_feat_val_abs_max, f->abs_max);
		status |= setDoubleParam(addr, FDC_feat_val_abs_min, f->abs_min);
	}

	/* Finally map a few of the AreaDetector parameters on to the camera 'features' */
	addr = DC1394_FEATURE_SHUTTER - DC1394_FEATURE_MIN;
	getDoubleParam(addr, FDC_feat_val_abs, &dtmp);
	status |= setDoubleParam(ADAcquireTime, dtmp);
	getDoubleParam(addr, FDC_feat_val_abs_max, &dtmp);
	status |= setDoubleParam(ADAcquirePeriod, dtmp);

	addr = DC1394_FEATURE_GAIN - DC1394_FEATURE_MIN;
	getDoubleParam(addr, FDC_feat_val_abs, &dtmp);
	status |= setDoubleParam(ADGain, dtmp);

	return status;
}


asynStatus FirewireDCAM::startCapture(asynUser *pasynUser)
{
	asynStatus status = asynSuccess;
	dc1394error_t err;
	int acquiring;
	const char* functionName = "startCapture";

	/* return error if already acquiring */
	getIntegerParam(ADAcquire, &acquiring);
	if (acquiring)
	{
		asynPrint( pasynUser, ASYN_TRACE_ERROR, "%s::%s [%s] Camera already acquiring.\n",
					driverName, functionName, this->portName);
		return asynError;
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
		return status;
	}

	/* Signal the image grabbing thread that the acquisition/transmission has
	 * started and it can start dequeueing images from the driver buffer */
	epicsEventSignal(this->startEventId);
	return status;
}


asynStatus FirewireDCAM::stopCapture(asynUser *pasynUser)
{
	asynStatus status = asynSuccess;
	dc1394error_t err;
	epicsEventWaitStatus eventStatus;
	int acquiring;
	const char * functionName = "stopCapture";

	/* return error if already stopped */
	getIntegerParam(ADAcquire, &acquiring);
	if (!acquiring)
	{
		asynPrint( pasynUser, ASYN_TRACE_ERROR, "%s::%s [%s] Camera already stopped.\n",
					driverName, functionName, this->portName);
		return asynError;
	}

	setIntegerParam(ADAcquire, 0);		/* set acquiring state to stopped */
	callParamCallbacks(); 				/* update whoever is interested in the acquire-state */

	/* Now wait for the capture thread to actually stop */
	asynPrint( pasynUser, ASYN_TRACE_ERROR, "%s::%s [%s] waiting for stopped event...\n",
					driverName, functionName, this->portName);
	/* unlock the mutex while we're waiting for the capture thread to stop acquiring */
	this->unlock();
	eventStatus = epicsEventWaitWithTimeout(this->stopEventId, 3.0);
	this->lock();
	if (eventStatus != epicsEventWaitOK)
	{
		asynPrint( pasynUser, ASYN_TRACE_FLOW, "%s::%s [%s] ERROR: Timeout when trying to stop image grabbing thread.\n",
					driverName, functionName, this->portName);
	}

	asynPrint( pasynUser, ASYN_TRACE_FLOW, "%s::%s [%s] Stopping firewire transmission\n",
				driverName, functionName, this->portName);

	/* Stop the actual transmission! */
	err=dc1394_video_set_transmission(this->camera, DC1394_OFF);
	status = PERR( pasynUser, err );
	if (status == asynError)
	{
		/* if stopping transmission results in an error (weird situation!) we print a message
		 * but does not abort the function because we still want to set status to stopped...  */
		asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s::%s [%s] Stopping transmission failed...\n",
					driverName, functionName, this->portName);
	}

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

/** Create an asyn user for the driver.
 * Maps the integer/enum asyn commands on to a string representation that
 * can be used to indicate a certain command in in the INP/OUT field of a record.
 * \param pasynUser
 * \param drvInfo
 * \param pptypeName
 * \param psize
 * \return asynStatus Either asynError or asynSuccess
 */
asynStatus FirewireDCAM::drvUserCreate( asynUser *pasynUser,
										const char *drvInfo,
										const char **pptypeName,
										size_t *psize)
{
	asynStatus status;

	status = this->drvUserCreateParam(	pasynUser, drvInfo, pptypeName,
										psize, FDCParamString, FDC_N_PARAMS);

	/* If not a local driver parameter, then see if it is a base class parameter */
	if (status) status = ADDriver::drvUserCreate(pasynUser, drvInfo, pptypeName, psize);
	return status;
}


/** Print out a report. Not yet implemented!
 * \param fp Stream or file pointer to write the report to.
 * \param details Configurable level of details in the report.
 * \return Nothing
 */
void FirewireDCAM::report(FILE *fp, int details)
{
	return;
}

