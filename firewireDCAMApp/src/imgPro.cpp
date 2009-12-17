
/* Standard Includes */

#include <stdio.h>
#include <stdlib.h>

/* Image Processing Includes */

#include "ImgProcessing.h"

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
#include "falseColour.h"

/* Specific asyn commands for this support module. These will be used and
   managed by the parameter library (part of areaDetector) */

typedef enum imgProParam_t
{
	imgPro_width				/* Width of the image in pixels */
	imgPro_height			/* Height of the image in pixels */
	imgPro_avg_pixel		/* Average number of pixels above the threshold */
	imgPro_num_of_pixels	/* Number of pixels above a threshold */
	imgPro_max_pixel_value	/* Highest pixel value within an image */
	imgPro_threshold		/* User chosen pixel value threshold */
} imgProParam_t;

static asynParamString_t imgProParamString[] = 
{
	{imgPro_width,      		"IMGPRO_WIDTH"},
	{imgPro_height,			"IMGPRO_HEIGHT"},
	{imgPro_avg_pixel,		"IMGPRO_AVG_PIXEL"},
	{imgPro_num_of_pixels,  "IMGPRO_NUM_PIXELS"},
	{imgPro_max_pixel_value,"IMGPRO_MAX_PIXEL"},
	{imgPro_threshold, 		"IMGPRO_THRESHOLD"},
};

/* Number of asyn parameters (asyn commands) this driver supports */
#define IMGPRO_N_PARAMS (sizeof(imgProParamString) / sizeof(imgProParamString[0]))

/* Class imgPro is derived from the NDPluginDriver public class */
class imgPro: public NDPluginDriver 
{
public:
    imgPro(const char *portName, int queueSize, int blockingCallbacks, const char *NDArrayPort, int NDArrayAddr);
    /* globals */
    globals *pglobal;
    unsigned char *destFrame;    
                 
    /* These methods override those in the base class */
    void processCallbacks(NDArray *pArray);
    virtual asynStatus drvUserCreate(asynUser *pasynUser, const char *drvInfo, 
                             const char **pptypeName, size_t *psize);

    /* Add in virtual functions to handle read and write operations for each supported data type */

};


void imgPro::processCallbacks(NDArray *pArray)
{
    /* we're going to get these with getIntegerParam */
    int quality, centrex, centrey, scale, weight, grid, false_col;
    /* we're going to get these from the dims of the image */
    int width,height,colour;
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
imgPro::imgPro(const char *portName, int queueSize, int blockingCallbacks, const char *NDArrayPort, int NDArrayAddr)
: NDPluginDriver(portName, queueSize, blockingCallbacks, NDArrayPort, NDArrayAddr)
{
	/* Do not call updateParam from here - will crash iocInit as the database won't be ready */

	char outputParam[MAX_CONFIG_STR];
    asynStatus status;
    destFrame = (unsigned char*)malloc(MAX_WIDTH*MAX_HEIGHT*3*sizeof(unsigned char));  
    pglobal = (globals*)malloc(sizeof(globals));
     
   
    /* Try to connect to the NDArray port */
    status = connectToArrayPort();
    
 

 

  
    
    /* Set the initial values of some parameters */
    setIntegerParam(0, mjpg_quality, 85);
    setIntegerParam(0, mjpg_grid, 0);
    setIntegerParam(0, mjpg_grid_weight, 30);
    setIntegerParam(0, mjpg_grid_scale, 15);
    setIntegerParam(0, mjpg_grid_centrex, 512);
    setIntegerParam(0, mjpg_grid_centrey, 384);
    setIntegerParam(0, mjpg_false_col, 0);
    setIntegerParam(0, mjpg_clients, 0);    
    setIntegerParam(0, mjpg_httpPort, httpPort);
}

/** asynDrvUser interface methods */
asynStatus mjpgServer::drvUserCreate(asynUser *pasynUser, const char *drvInfo, 
                                       const char **pptypeName, size_t *psize)
{
	asynStatus status;
    status = this->drvUserCreateParam(pasynUser, drvInfo, pptypeName, psize,
            imgProParamString, IMGPRO_N_PARAMS);
    /* If not, then call the base class method, see if it is known there */
    if (status) status = NDPluginDriver::drvUserCreate(pasynUser, drvInfo, pptypeName, psize);
	return(status);
}

/* Configuration routine.  Called directly, or from the iocsh function in imgProRegister - calls the imgPro constructor */

extern "C" int imgProConfigure(const char *portName, int queueSize, int blockingCallbacks, const char *NDArrayPort, int NDArrayAddr)
{
    new imgPro(portName, queueSize, blockingCallbacks, NDArrayPort, NDArrayAddr);
    return(asynSuccess);
}