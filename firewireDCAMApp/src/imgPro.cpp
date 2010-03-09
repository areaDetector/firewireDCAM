
/* Standard Includes */

#include <stdio.h>
#include <stdlib.h>

/* Image Processing Includes */

#include "imgPro.h"

/* EPICs Includes */

#include <epicsTypes.h>
#include <epicsString.h>
#include <epicsMutex.h>

/* Asyn Includes */

#include <asynStandardInterfaces.h>

/* AreaDetector Includes */

#include "NDPluginDriver.h"
#include "ADStdDriverParams.h"
#include "NDArray.h"

/** Max width of an image, used to define jpeg buffer*/
#define MAX_WIDTH 2560
/** Max height of an image, used to define jpeg buffer*/
#define MAX_HEIGHT 2048

/* Specific asyn commands for this support module. These will be used and
   managed by the parameter library (part of areaDetector) */

typedef enum imgProParam_t 
{
	imgPro_width,			/* Width of the image in pixels (int32 read) */
	imgPro_height,			/* Height of the image in pixels (int32 read) */
	imgPro_avg_pixel,		/* Average number of pixels above the threshold (float32 write) */
	imgPro_num_of_pixels,		/* Number of pixels above a threshold (int32 write) */
	imgPro_max_pixel_value,		/* Highest pixel value within an image (int32 write) */
	imgPro_threshold,		/* User chosen pixel value threshold (int32 read) */
	imgProLastParam
} imgProParam_t;

static asynParamString_t imgProParamString[] = 
{
	{imgPro_width,      	"IMGPRO_WIDTH"},
	{imgPro_height,		"IMGPRO_HEIGHT"},
	{imgPro_avg_pixel,	"IMGPRO_AVG_PIXEL"},
	{imgPro_num_of_pixels,  "IMGPRO_NUM_PIXELS"},
	{imgPro_max_pixel_value,"IMGPRO_MAX_PIXEL"},
	{imgPro_threshold, 	"IMGPRO_THRESHOLD"},
};

/* Number of asyn parameters (asyn commands) this driver supports */
#define IMGPRO_N_PARAMS (sizeof(imgProParamString) / sizeof(imgProParamString[0]))

/* Class imgPro is derived from the NDPluginDriver public class */
class imgPro: public NDPluginDriver 
{
public:
    imgPro(const char *portName, int queueSize, int blockingCallbacks, const char *NDArrayPort, int NDArrayAddr, int maxBuf, int maxMem);

    unsigned char *destFrame;    
                 
    /* These methods override those in the base class */
    void processCallbacks(NDArray *pArray);
    virtual asynStatus drvUserCreate(asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize);

    /* Add in virtual functions to handle read and write operations for each supported data type */

};

void imgPro::processCallbacks(NDArray *pArray)
{
    /* we're going to get these with getIntegerParam */
    
    /*int quality, centrex, centrey, scale, weight, grid;*/
    
    /* we're going to get these from the dims of the image */
    int width, height, colour;
    unsigned char *srcFrame;

    /* Call the base class method */
    NDPluginDriver::processCallbacks(pArray);    
    /* Locking mutex so nothing else uses the data */
    pArray->reserve();

    /* pArray->pData is the data set */
				            
	    /* get the dimensions */
	    if (pArray->colorMode == NDColorModeMono )
	    {
			width = (int) pArray->dims[0].size;
			height = (int) pArray->dims[1].size;
			colour = 0;
		}
		else
		{
			width = (int) pArray->dims[1].size;
			height = (int) pArray->dims[2].size;
			colour = 1;
		}		
		srcFrame = (unsigned char*) pArray->pData;
		
	/* get the configuration values */
		setIntegerParam(imgPro_width, width);
		setIntegerParam(imgPro_height, height);

	/* Release the frame (unlocking mutex) */
    pArray->release();

    /* Go through the variables, check for updates. Call IOInterupt to get latest parameters.  */
    /* It is at this point when data appears in the database */
    callParamCallbacks();    
}

/* The constructor for this class
		portName = asyn port name of this driver
		queueSize = size of the input queue (in number of NDArrays), normally 1
		blockingCallbacks = if 1 then block while processing, normally 0
		NDArrayport = asyn port name of the image source (driver or ROI) */

/* Invoking the base class constructor - called when instantiating plugin */
imgPro::imgPro(const char *portName, int queueSize, int blockingCallbacks, const char *NDArrayPort, int NDArrayAddr, int maxBuf, int maxMem)
	: NDPluginDriver(portName, 
					 queueSize, 
					 blockingCallbacks, 
					 NDArrayPort, 
					 NDArrayAddr, 
					 1, 
					 imgProLastParam, 
					 maxBuf, 
					 maxMem, 
					 asynFloat64ArrayMask | asynInt32ArrayMask | asynGenericPointerMask | asynOctetMask | asynInt32Mask | asynFloat64Mask,
					 asynFloat64ArrayMask | asynInt32ArrayMask | asynGenericPointerMask | asynOctetMask | asynInt32Mask | asynFloat64Mask)			 
{
	/* Do not call updateParam from here - will crash iocInit as the database won't be ready */

    asynStatus status;
    destFrame = (unsigned char*)malloc(MAX_WIDTH*MAX_HEIGHT*3*sizeof(unsigned char));  
    
    /*********** Calloc not malloc */
          
    /* Try to connect to the NDArray port */
    status = connectToArrayPort();
        
    /* Set the initial values of some parameters */
    
   setIntegerParam(0, imgPro_threshold, 20);
}

/** asynDrvUser interface methods */
asynStatus imgPro::drvUserCreate(asynUser *pasynUser, const char *drvInfo, const char **pptypeName, size_t *psize)
{
	asynStatus status;
	int param;
	const char *functionName = "drvUserCreate";

	/* See if this is one of the drivers local parameters */
	status = findParam(imgProParamString, IMGPRO_N_PARAMS, drvInfo, &param);

	if (status == asynSuccess)
	{
		pasynUser->reason = param;
		if (pptypeName) 
		{
			*pptypeName = epicsStrDup(drvInfo); 
		}
		if (psize)
		{
			*psize = sizeof(param); 
		}
		asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s:%s: drvInfo=%s, param=%d\n", "imgPro", functionName, drvInfo, param);
		return asynSuccess;
	}

	/* If not a local driver parameter, then see if it is a base class parameter */
	status = NDPluginDriver::drvUserCreate(pasynUser, drvInfo, pptypeName, psize);
	return status;
}

/* Configuration routine.  Called directly, or from the iocsh function in imgProRegister - calls the imgPro constructor */

extern "C" int imgProConfigure(const char *portName, int queueSize, int blockingCallbacks, const char *NDArrayPort, int NDArrayAddr, int maxBuf, int maxMem)
{
    new imgPro(portName, queueSize, blockingCallbacks, NDArrayPort, NDArrayAddr, maxBuf, maxMem);
    return(asynSuccess);
}
