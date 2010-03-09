#ifndef IMGPROCESSING_H
#define IMGPROCESSING_H

#ifdef __cplusplus
extern "C" {
#endif

int imgProConfigure(const char *portName, int queueSize, int blockingCallbacks, const char *NDArrayPort, int NDArrayAddr, int maxBuf, int maxMem);

#ifdef __cplusplus
}
#endif

/*  Returned data structure */
typedef struct retData
{
	unsigned long *xDist;     /* Pointer to x distribution array */
	unsigned long *yDist;     /* Pointer to y distribution array */
	unsigned int xCoOrd;      /* x coordinate of maximum value pixel */
	unsigned int yCoOrd;      /* y coordinate of maximum value pixel */
	double avgValue;          /* Average pixel value in image */
	unsigned int maxValue;    /* Maximum pixel value in image */
	unsigned int CentreX;     /* X-Axis centre point */
	unsigned int CentreY;     /* Y-Axis centre point */ 
	unsigned int Width;       /* Width of the beamspot */
	unsigned int Height;      /* Height of the beamspot */
} retData;

retData * ImgPro_createStruct( unsigned int maxWidth, unsigned int maxHeight);

/* Calculates the average pixel value of the image (above a threshold) */
int ImgPro_avgPixValue(	const unsigned char *image,
			unsigned int width,
			unsigned int height,
			retData* MyData);

/* Calculates the maximum pixel value in the image and its coordinates*/
int ImgPro_maxPixValue(	const unsigned char *image,
			unsigned int width,
			unsigned int height,
			retData* result);

/* Calculates the distribution along the x-axis*/
int ImgPro_xyDist(	const unsigned char *image,
			unsigned int width,
			unsigned int height,
			retData* MyData);

/* Calculates the beam's centre of mass */
int ImgPro_beamCentre(	const unsigned char *image,
			unsigned int width, 
			unsigned int height, 
			unsigned int BlockWidth,			
			retData *MyData);

/* Calculates the maximum width of the beam spot */
int ImgPro_beamDim(	const unsigned char *image,
			unsigned int width,
			unsigned int height,
			unsigned char Threshold,
			retData *MyData);

/* Run all of the above functions in one single function*/
int ImgPro_ImgProcessing(const unsigned char *image,
			unsigned int width,
			unsigned int height,
			unsigned char Threshold,
			unsigned int BlockWidth,
			retData *MyData);

#endif

