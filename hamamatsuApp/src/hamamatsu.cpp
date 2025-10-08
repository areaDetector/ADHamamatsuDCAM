/* hamamatsu.cpp
 *
 * This is a driver for a Hamamatsu camera.
 *
 * Author: Hamamatsu Photonics K.K.
 *
 * Created:  February 9, 2024
 *
 * For end use and not for commercial purposes
 *
 */

#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsExit.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <cantProceed.h>
#include <iocsh.h>

#include "ADDriver.h"
#include <epicsExport.h>


#include "hamamatsu.h"
#include "dcamapi4.h"
#include "dcamprop.h"

static const char *driverName = "hamamatsu";

#define MIN_DELAY       1e-5
#define USE_COPYFRAME   1    // 0: call dcambuf_lockframe to access image, 1: call dcambuf_copyframe to access image
#define OUTPUT_IMAGE    0    // 0: not output, 1: output the accessed image by raw data

/** Controls the shutter */
void hamamatsu::setShutter(int open)
{
    int shutterMode;

    getIntegerParam(ADShutterMode, &shutterMode);
    if (shutterMode == ADShutterModeDetector) {
        /* Simulate a shutter by just changing the status readback */
        setIntegerParam(ADShutterStatus, open);
    } else {
        /* For no shutter or EPICS shutter call the base class method */
        ADDriver::setShutter(open);
    }
}


static void hamaTaskC(void *drvPvt)
{
    hamamatsu *pPvt = (hamamatsu *)drvPvt;

    pPvt->hamaAcquire();
}

void hamamatsu::isSysAlive()
{
    double readVal;
    DCAMERR err;

    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_SYSTEM_ALIVE, &readVal );
    if ( checkErrAndPrint(__FUNCTION__, err, "IDPROP:SYSTEM_ALIVE") )
        return;

    setIntegerParam(HamaSystemAlive, readVal);

}

void hamamatsu::getErrString(HDCAM handle, DCAMERR err, char* buf, unsigned int bufsize) {

    DCAMDEV_STRING param;
    memset( &param, 0, sizeof(param) );
    param.size = sizeof(param);
    param.text = buf;
    param.textbytes = bufsize;
    param.iString = err;

    err = dcamdev_getstring( handle, &param );

}

/** If DCAMERR has error, print error string and return true. Otherwise return false.
  * \param[in] functionName Name of function calling this. Will be printed along with driver name in error message.
  * \param[in] err DCAMERR structure from the operation to be checked.
  * \param[in] paramName Parameter name to be printed in error message. */
inline const bool hamamatsu::checkErrAndPrint(const char* functionName, DCAMERR err, const char* paramName, ...) {

    char str[256];
    char operation[256];

    if (!failed(err))
        return false;

    va_list args;
    va_start(args, paramName);
    epicsVsnprintf(operation, sizeof(operation), paramName, args);
    va_end(args);

    hamamatsu::getErrString(this->hdcam, err, str, 256);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
        "%s:%s: Error with operation %s. Error code: 0x%x, error string: %s\n",
        driverName, functionName, operation, err, str);

    return true;

}

/** Calls setgetvalue for given property. If error happens, report it through hamamatsu::checkErrAndPrint.
  * \param[in] iProp Hamamatsu dcam property to be set.
  * \param[inout] pValue Pointer to value which will be set. Pointer will also point to new value gotten from device.
  * \param[in] functionName Name of caller function to be printed in error message.
  * \param[in] paramName Name of parameter to be printed in error message. */
inline const bool hamamatsu::checkSetGet(int32 iProp, double* pValue, const char* functionName, const char* paramName) {

    char operation[256];
    epicsSnprintf(operation, sizeof(operation), "dcamprop_setgetvalue(hdcam, %s, VALUE: %f)", paramName, *pValue);

    DCAMERR err = dcamprop_setgetvalue( this->hdcam, iProp, pValue );
    return checkErrAndPrint(functionName, err, operation);

}

/** Calls setvalue for given property. If error happens, report it through hamamatsu::checkErrAndPrint.
  * \param[in] iProp Hamamatsu dcam property to be set.
  * \param[inout] pValue Value to set property.
  * \param[in] functionName Name of caller function to be printed in error message.
  * \param[in] paramName Name of parameter to be printed in error message. */
inline const bool hamamatsu::checkSet(int32 iProp, double pValue, const char* functionName, const char* paramName) {

    char operation[256];
    epicsSnprintf(operation, sizeof(operation), "dcamprop_setvalue(hdcam, %s, VALUE: %f)", paramName, pValue);

    DCAMERR err = dcamprop_setvalue( this->hdcam, iProp, pValue );
    return checkErrAndPrint(functionName, err, operation);

}

/** Calls getvalue for given property. If error happens, report it through hamamatsu::checkErrAndPrint.
  * \param[in] iProp Hamamatsu dcam property to get value.
  * \param[inout] pValue Pointer to put property value.
  * \param[in] functionName Name of caller function to be printed in error message.
  * \param[in] paramName Name of parameter to be printed in error message. */
inline const bool hamamatsu::checkGet(int32 iProp, double *pValue, const char* functionName, const char* paramName) {

    char operation[256];
    epicsSnprintf(operation, sizeof(operation), "dcamprop_getvalue(hdcam, %s)", paramName);

    DCAMERR err = dcamprop_getvalue( this->hdcam, iProp, pValue );
    return checkErrAndPrint(functionName, err, operation);

}

//Hamamatsu functions
BOOL hamamatsu::copy_targetarea( HDCAM hdcam, int32 iFrame, void* buf, int32 rowbytes, int32 ox, int32 oy, int32 cx, int32 cy )
{
    DCAMERR err;

    // prepare frame param
    DCAMBUF_FRAME bufframe;
    memset( &bufframe, 0, sizeof(bufframe) );
    bufframe.size    = sizeof(bufframe);
    bufframe.iFrame  = iFrame;

#if USE_COPYFRAME
    // set user buffer information and copied ROI
    bufframe.buf        = buf;
    bufframe.rowbytes   = rowbytes;
    bufframe.left       = ox;
    bufframe.top        = oy;
    bufframe.width      = cx;
    bufframe.height     = cy;
    
    // access image
    err = dcambuf_copyframe( hdcam, &bufframe );
    if ( checkErrAndPrint(__FUNCTION__, err, "dcambuf_copyframe()") )
        return FALSE;
#else
    // access image
    err = dcambuf_lockframe( hdcam, &bufframe );
    if ( checkErrAndPrint(__FUNCTION__, err, "dcambuf_lockframe()") )
        return FALSE;

    if( bufframe.type != DCAM_PIXELTYPE_MONO16 )
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: Pixel type not implemented. Only available pixel type is DCAM_PIXELTYPE_MONO16.\n",
            driverName, __FUNCTION__);
        return FALSE;
    }

    // copy target ROI
    int32 copyrowbytes = cx * 2;
    char* pSrc = (char*)bufframe.buf + oy * bufframe.rowbytes + ox * 2;
    char* pDst = (char*)buf;

    int y;
    for( y = 0; y < cy; y++ )
    {
        memcpy_s( pDst, rowbytes, pSrc, copyrowbytes );

        pSrc += bufframe.rowbytes;
        pDst += rowbytes;
    }
#endif

    return TRUE;
}

/**
 @brief    get image information from properties.
 @param    hdcam        DCAM handle
 @param pixeltype    DCAM_PIXELTYPE value
 @param width        image width
 @param rowbytes    image rowbytes
 @param height        image height
 */
void hamamatsu::get_image_information( HDCAM hdcam, int32& pixeltype, int32& width, int32& rowbytes, int32& height )
{
    DCAMERR err;

    double v;

    // image pixel type(DCAM_PIXELTYPE_MONO16, MONO8, ... )
    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_IMAGE_PIXELTYPE, &v );
    if ( checkErrAndPrint(__FUNCTION__, err, "dcamprop_getvalue(..., IDPROP:IMAGE_PIXELTYPE)") ) {
        return;
    }
    else
        pixeltype = (int32) v;

    // image width
    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_IMAGE_WIDTH, &v );
    if ( checkErrAndPrint(__FUNCTION__, err, "dcamprop_getvalue(..., IDPROP:IMAGE_WIDTH)") ) {
        return;
    }
    else
        width = (int32)v;

    // image row bytes
    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_IMAGE_ROWBYTES, &v );
    if ( checkErrAndPrint(__FUNCTION__, err, "dcamprop_getvalue(..., IDPROP:IMAGE_ROWBYTES)") ) {
        return;
    }
    else
        rowbytes = (int32)v;

    // image height
    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_IMAGE_HEIGHT, &v );
    if( checkErrAndPrint(__FUNCTION__, err, "dcamprop_getvalue(..., IDPROP:IMAGE_HEIGHT)") ) {
        return;
    }
    else
        height = (int32)v;
}

/**
 @brief    sample used to process image after capturing.
 @details    This function copies the target area that is 10% of full area on the center.
 @param    hdcam        DCAM handle
 @sa    get_image_information, copy_targetarea
 */
void hamamatsu::sample_access_image( HDCAM hdcam )
{

    int status = asynSuccess;
    int imageCounter;
    int numImages, numImagesCounter;
    int arrayCallbacks;
    NDArray *pImage;
    epicsTimeStamp startTime;
    const char *functionName = "sample_access_image";
    
    epicsTimeGetCurrent(&startTime);
    DCAMERR err;

    // transferinfo param
    DCAMCAP_TRANSFERINFO captransferinfo;
    memset( &captransferinfo, 0, sizeof(captransferinfo) );
    captransferinfo.size    = sizeof(captransferinfo);

    // get number of captured image
    err = dcamcap_transferinfo( hdcam, &captransferinfo );
    if( checkErrAndPrint(__FUNCTION__, err, "dcamcap_transferinfo()") ) {
        return;
    }

    if( captransferinfo.nFrameCount < 1 )
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_WARNING,
                "%s:%s: No images captured. nFrameCount: %u\n",
                driverName, functionName, captransferinfo.nFrameCount);
        return;
    }

    // get image information
    int32 pixeltype = 0, width = 0, rowbytes = 0, height = 0;
    get_image_information( hdcam, pixeltype, width, rowbytes, height );

    setIntegerParam(ADStatus, ADStatusReadout);

    int binX;
    status |= getIntegerParam(ADBinX, &binX);
    if (binX < 1) 
    {
        binX = 1;
        status |= setIntegerParam(ADBinX, binX);
    }
   
    //set binX and binY to the same value
    status |= setIntegerParam(ADBinY,  binX);
    int binY;
    status |= getIntegerParam(ADBinY,  &binY);
    status |= setIntegerParam(ADSizeX, width*binX);
    status |= setIntegerParam(ADSizeY, height*binY);
     
    //printf( "\nwidth after image%d \n", width );
    //printf( "\nheight after image%d\n", height );

    /* Update the image */
    /* Allocate the raw buffer we use to compute images. */
    size_t dims[2];
    dims[0] = width;
    dims[1] = height;
    pImage = this->pNDArrayPool->alloc(2, dims, NDUInt16, 0, NULL);
    if (pArrays[0]) pArrays[0]->release();
    this->pArrays[0] = pImage;

    //printf ("\nframe count %d\n", captransferinfo.nFrameCount);

    //int iFrame=captransferinfo.nFrameCount;

    // copy image
    copy_targetarea( hdcam, -1, pImage->pData, rowbytes, 0, 0, width, height );
    /* Get the current parameters */
    getIntegerParam(NDArrayCounter, &imageCounter);
    getIntegerParam(ADNumImages, &numImages);
    getIntegerParam(ADNumImagesCounter, &numImagesCounter);
    getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
    
    imageCounter++;
    numImagesCounter++;
    setIntegerParam(NDArrayCounter, captransferinfo.nFrameCount);
    setIntegerParam(ADNumImagesCounter, captransferinfo.nFrameCount);

    /* Put the frame number and time stamp into the buffer */
    pImage->uniqueId = captransferinfo.nFrameCount;
    pImage->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;
    updateTimeStamp(&pImage->epicsTS);

    /* Get any attributes that have been defined for this driver */
    this->getAttributes(pImage->pAttributeList);

    if (arrayCallbacks) {
        /* Call the NDArray callback */
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s: calling imageData callback\n", driverName, functionName);
        doCallbacksGenericPointer(pImage, NDArrayData, 0);
    }
    
    /* First do callback on ADStatus. */
    setStringParam(ADStatusMessage, "Waiting for acquisition");
    setIntegerParam(ADStatus, ADStatusIdle);
 
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
          "%s:%s: acquisition completed\n", driverName, functionName);

    /* Call the callbacks to update any changes */
    callParamCallbacks();
    //printf("\n end sample image\n");
}

void hamamatsu::hamaAcquire()
{
    DCAMERR err;
    const char *functionName = "hamaAcquire";
    double exposureTime;
    //double acquirePeriod;
    int status = asynSuccess;
    int binX, binY, minX, minY, sizeX, sizeY;
    int acquire=0;

    this->lock();
    /* Loop forever */
    while (1) {
        /* If we are not acquiring then wait for a semaphore that is given when acquisition is started */
        if (!acquire) {
          /* Release the lock while we wait for an event that says acquire has started, then lock again */
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s: waiting for acquire to start\n", driverName, functionName);
            this->unlock();
            status = epicsEventWait(startEventId_);
            this->lock();
            acquire = 1;
            setStringParam(ADStatusMessage, "Acquiring data");
            setIntegerParam(ADNumImagesCounter, 0);
        }

        status |= getIntegerParam(ADBinX,         &binX);
        status |= getIntegerParam(ADBinY,         &binY);
        status |= getIntegerParam(ADMinX,         &minX);
        status |= getIntegerParam(ADMinY,         &minY);
        status |= getIntegerParam(ADSizeX,        &sizeX);
        status |= getIntegerParam(ADSizeY,        &sizeY);
        
        getDoubleParam(ADAcquireTime,     &exposureTime);
        checkSetGet(DCAM_IDPROP_EXPOSURETIME, &exposureTime, __FUNCTION__, "DCAM_IDPROP_EXPOSURETIME");
        setDoubleParam(ADAcquireTime, exposureTime);

        /*
        getDoubleParam(ADAcquirePeriod,     &acquirePeriod);
        err = dcamprop_setgetvalue( hdcam, DCAM_IDPROP_INTERNALFRAMERATE, &acquirePeriod );
        if( failed(err) )
        {
            dcamcon_show_dcamerr( hdcam, err, "dcamprop_setgetvalue() for DCAM_IDPROP_INTERNALFRAMERATE" );
        }
        setDoubleParam(ADAcquirePeriod, acquirePeriod);
        */

        double tempHPOS=(double)minX;
        if ( checkSetGet(DCAM_IDPROP_SUBARRAYHPOS, &tempHPOS, functionName, "DCAM_IDPROP_SUBARRAYHPOS") ) {
            acquire=0;
            hamaAbortAcquisition();
            continue;
        }
        
        double tempVPOS=(double)minY;
        if ( checkSetGet(DCAM_IDPROP_SUBARRAYVPOS, &tempVPOS, functionName, "DCAM_IDPROP_SUBARRAYVPOS") ) {
            acquire=0;
            hamaAbortAcquisition();
            continue;   
        }
        
        double tempHSIZE=(double)sizeX;
        if ( checkSetGet(DCAM_IDPROP_SUBARRAYHSIZE, &tempHSIZE, functionName, "DCAM_IDPROP_SUBARRAYHSIZE") ) {
            acquire=0;
            hamaAbortAcquisition();
            continue;           
        }

        double tempVSIZE=(double)sizeY;
        if ( checkSetGet(DCAM_IDPROP_SUBARRAYVSIZE, &tempVSIZE, functionName, "DCAM_IDPROP_SUBARRAYVSIZE") ) {
            acquire=0;
            hamaAbortAcquisition();
            continue;
        }
        
        if ( checkSet(DCAM_IDPROP_SUBARRAYMODE, DCAMPROP_MODE__ON, functionName, "IDPROP:SUBARRAYMODE") ) {
            acquire=0;
            hamaAbortAcquisition();
            continue;
        } else {
            setIntegerParam (ADMinX,   (int)tempHPOS);
            setIntegerParam (ADSizeX,  (int)tempHSIZE);
            setIntegerParam (ADMinY,   (int)tempVPOS);
            setIntegerParam (ADSizeY,  (int)tempVSIZE);
        }
        callParamCallbacks();
        
        // set binning value to the camera
        double binning=(double)binX;
        if ( checkSetGet(DCAM_IDPROP_BINNING, &binning, functionName, "IDPROP:BINNING") ) {
            binning=1;
            checkSetGet(DCAM_IDPROP_BINNING, &binning, functionName, "IDPROP:BINNING");
            setIntegerParam (ADBinX,     (int)binning);
            setIntegerParam (ADBinY,     (int)binning);
        } else {
            setIntegerParam (ADBinX,     (int)binning);
        }

        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: MinX: %d, SizeX: %d, MinY: %d, SizeY: %d, binning: %d\n",
            driverName, functionName, (int)tempHPOS, (int)tempHSIZE,
            (int)tempVPOS, (int)tempVSIZE, (int)binning);

        setIntegerParam(ADStatus, ADStatusAcquire);

        /* Open the shutter */
        setShutter(ADShutterOpen);

        // open wait handle
        DCAMWAIT_OPEN    waitopen;
        memset( &waitopen, 0, sizeof(waitopen) );
        waitopen.size    = sizeof(waitopen);
        waitopen.hdcam    = hdcam;
    
        err = dcamwait_open( &waitopen );
        if ( !checkErrAndPrint(__FUNCTION__, err, "dcamwait_open()") ) {
            this->hwait = waitopen.hwait;
    
            // allocate buffer
            int32 number_of_buffer = 10;
            err = dcambuf_alloc( hdcam, number_of_buffer );
            if ( !checkErrAndPrint(__FUNCTION__, err, "dcambuf_alloc()") ) {
                // start capture
                err = dcamcap_start( hdcam, DCAMCAP_START_SEQUENCE );
                if ( !checkErrAndPrint(__FUNCTION__, err, "dcamcap_start()") ) {
                    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                        "%s:%s: Starting capture.\n", driverName, functionName);
                    // set wait param
                    DCAMWAIT_START waitstart;
                    memset( &waitstart, 0, sizeof(waitstart) );
                    waitstart.size        = sizeof(waitstart);
                    waitstart.eventmask    = DCAMWAIT_CAPEVENT_FRAMEREADY;
                    waitstart.timeout    = 5000;
                    
                    int imageMode;
                    getIntegerParam(ADImageMode, &imageMode);
                    if (imageMode == ADImageContinuous) {
                        while (1) {
                            // wait image
                            this->unlock();
                            err = dcamwait_start( hwait, &waitstart );
                            this->lock();
                            if ( checkErrAndPrint(__FUNCTION__, err, "dcamwait_start()") ) {
                                setStringParam(ADStatusMessage, "Stopped acquisition");
                                setIntegerParam(ADStatus, ADStatusIdle);
                                callParamCallbacks();
    
                                acquire = 0;
                                setIntegerParam(ADAcquire, acquire);
                                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                                      "%s:%s: acquisition completed\n", driverName, functionName);
                                
                                dcamcap_stop( hdcam );
                                
                                // release buffer
                                dcambuf_release( hdcam );
    
                                // close wait handle
                                dcamwait_close( hwait );
                                epicsEventSignal(stopEventId_);
                                callParamCallbacks();
                                
                                this->unlock();
                                status = epicsEventWaitWithTimeout(stopEventId_, MIN_DELAY);
                                callParamCallbacks();
                                this->lock();
                                epicsThreadSleep(0.01);
                                break;
                            } else {
                                // access image
                               // printf( "Access Image\n" );
                                sample_access_image( hdcam );
                            }
                            acquire = 0;
        
                            this->unlock();
                            status = epicsEventWaitWithTimeout(stopEventId_, MIN_DELAY);
                            this->lock();
                            if (status == epicsEventWaitOK) {
                                acquire = 0;
                                if (imageMode == ADImageContinuous) {
                                  setIntegerParam(ADStatus, ADStatusIdle);
                                } else {
                                  setIntegerParam(ADStatus, ADStatusAborted);
                                }
                                callParamCallbacks();
                                /* Close the shutter */
                                setShutter(ADShutterClosed);
                                setIntegerParam(ADStatus, ADStatusReadout);                              
                                dcamcap_stop( hdcam );
                                // release buffer
                                dcambuf_release( hdcam );
                                // close wait handle
                                dcamwait_close( hwait );
                                break;
                            }
                            /* Call the callbacks to update any changes */
                            callParamCallbacks();
                        }
                    } else if (imageMode == ADImageMultiple) {
                        int numImages;
                        getIntegerParam(ADNumImages, &numImages);
                        for (int i=0; i<numImages; i++) {
                            // wait image
                            this->unlock();
                            err = dcamwait_start( hwait, &waitstart );
                            this->lock();
                            if ( checkErrAndPrint(__FUNCTION__, err, "dcamwait_start()") ) {
                                setStringParam(ADStatusMessage, "Stopped acquisition");
                                setIntegerParam(ADStatus, ADStatusIdle);
                                callParamCallbacks();
                                acquire = 0;
                                setIntegerParam(ADAcquire, acquire);
                                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                                      "%s:%s: acquisition completed\n", driverName, functionName);
                                
                                dcamcap_stop( hdcam );
                                // release buffer
                                dcambuf_release( hdcam );
                                // close wait handle
                                dcamwait_close( hwait );
                                epicsEventSignal(stopEventId_);
                                callParamCallbacks();
                                this->unlock();
                                status = epicsEventWaitWithTimeout(stopEventId_, MIN_DELAY);
                                callParamCallbacks();
                                this->lock();
                                epicsThreadSleep(0.01);
                                break;
                            }
                            // access image
                            //printf( "Access Image\n" );
                            sample_access_image( hdcam );
                            
                            acquire = 0;
    
                            this->unlock();
                            status = epicsEventWaitWithTimeout(stopEventId_, MIN_DELAY);
                            this->lock();
                            if (status == epicsEventWaitOK) {
                                acquire = 0;
                                if (imageMode == ADImageContinuous) {
                                  setIntegerParam(ADStatus, ADStatusIdle);
                                } else {
                                  setIntegerParam(ADStatus, ADStatusAborted);
                                }
                                callParamCallbacks();

                                /* Close the shutter */
                                setShutter(ADShutterClosed);
                                setIntegerParam(ADStatus, ADStatusReadout);
                                dcamcap_stop( hdcam );
                                
                                // release buffer
                                dcambuf_release( hdcam );
                                // close wait handle
                                dcamwait_close( hwait );
                                callParamCallbacks();
                                break;
                            }
                            callParamCallbacks();
                        }
                    } else { // Not ADImageContinuous or ADImageMultiple
                        // wait image
                        this->unlock();
                        err = dcamwait_start( hwait, &waitstart );
                        this->lock();
                        if ( checkErrAndPrint(__FUNCTION__, err, "dcamwait_start()") ) {
                            
                            setStringParam(ADStatusMessage, "Stopped acquisition");
                            setIntegerParam(ADStatus, ADStatusIdle);
                            callParamCallbacks();
    
                            acquire = 0;
                            setIntegerParam(ADAcquire, acquire);
                            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                                  "%s:%s: acquisition completed\n", driverName, functionName);
                            
                            dcamcap_stop( hdcam );
                            
                            // release buffer
                            dcambuf_release( hdcam );
    
                            // close wait handle
                            dcamwait_close( hwait );
                            epicsEventSignal(stopEventId_);
                            callParamCallbacks();
                            
                            this->unlock();
                            status = epicsEventWaitWithTimeout(stopEventId_, MIN_DELAY);
                            callParamCallbacks();
                            this->lock();
                            epicsThreadSleep(0.01);
                        } else {
                            // access image
                            //printf( "Access Image\n" );
                            sample_access_image( hdcam );
                        }
                    }
                    acquire = 0;
                    this->unlock();
                    status = epicsEventWaitWithTimeout(stopEventId_, MIN_DELAY);
                    this->lock();
                    if (status == epicsEventWaitOK) {
                        acquire = 0;
                        if (imageMode == ADImageContinuous) {
                          setIntegerParam(ADStatus, ADStatusIdle);
                        } else {
                          setIntegerParam(ADStatus, ADStatusAborted);
                        }
                        callParamCallbacks();
                    }
    
                    //Close the shutter 
                    setShutter(ADShutterClosed);
                    
                    setIntegerParam(ADStatus, ADStatusReadout);
                    /* Call the callbacks to update any changes */
                    callParamCallbacks();
                    
                    // stop capture
                    dcamcap_stop( hdcam );
                    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                        "%s:%s: Stopped acquisition.\n",
                        driverName, functionName);
                    setIntegerParam(ADAcquire, 0);
                    callParamCallbacks();
                }
    
                // release buffer
                dcambuf_release( hdcam );
            }
    
            // close wait handle
            dcamwait_close( hwait );
        }  // !failed dcam_waitopen()
    }  // while(1)
}

/** Called when asyn clients call pasynInt32->write().
  * This function performs actions for some parameters, including ADAcquire, ADColorMode, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus hamamatsu::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    const char *functionName = "writeInt32";
    int function = pasynUser->reason;
    int adstatus;
    int acquiring;
    int imageMode;
    asynStatus status = asynSuccess;

    /* Ensure that ADStatus is set correctly before we set ADAcquire.*/
    getIntegerParam(ADStatus, &adstatus);
    getIntegerParam(ADAcquire, &acquiring);
    getIntegerParam(ADImageMode, &imageMode);
 
    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setIntegerParam(function, value);

    /* For a real detector this is where the parameter is sent to the hardware */
    if (function == ADAcquire) {
        if (value && !acquiring) {
            /* Send an event to wake up the simulation task.
             * It won't actually start generating new images until we release the lock below */
            epicsEventSignal(startEventId_); 
        }
        if (!value && acquiring) {
            /* This was a command to stop acquisition */
            /* Send the stop event */
            
            //abort any waiting
            dcamwait_abort( hwait);
            dcamcap_stop( hdcam );    
            
             // release buffer
            dcambuf_release( hdcam );

            // close wait handle
            dcamwait_close( hwait );
            /* Send the stop event */
            epicsEventSignal(stopEventId_);
            callParamCallbacks();
        }

    } else if (function == NDDataType) {
        int dataType;
        getIntegerParam(NDDataType, &dataType);
        if (dataType ==0 || dataType ==1)//0= int8, 1=uint8
        {
            if ( checkSet(DCAM_IDPROP_IMAGE_PIXELTYPE, DCAM_PIXELTYPE_MONO8, functionName, "IDPROP:IMAGE_PIXELTYPE") ) {
                checkSet( DCAM_IDPROP_IMAGE_PIXELTYPE, DCAM_PIXELTYPE_MONO16, functionName, "IDPROP:IMAGE_PIXELTYPE" );
                setIntegerParam(NDDataType, 3);
            }
        } else// 2=int16, 3=uint16
        {
            checkSet(DCAM_IDPROP_IMAGE_PIXELTYPE, DCAM_PIXELTYPE_MONO16, functionName, "DCAM_IDPROP_IMAGE_PIXELTYPE");
        }
        callParamCallbacks();
    }
    
    else if (function == HamaRegionReset) {
        double tempResetVal=0;
        checkSet( DCAM_IDPROP_SUBARRAYMODE, DCAMPROP_MODE__OFF, functionName, "IDPROP:SUBARRAYMODE" );
        if ( !checkSetGet( DCAM_IDPROP_SUBARRAYHPOS, &tempResetVal, functionName, "DCAM_IDPROP_SUBARRAYHPOS" ) ) {
            setIntegerParam (ADMinX,    (int)tempResetVal);
        }
        if ( !checkSetGet( DCAM_IDPROP_SUBARRAYVPOS, &tempResetVal, functionName, "DCAM_IDPROP_SUBARRAYVPOS" ) ) {
            setIntegerParam (ADMinY,    (int)tempResetVal);
        }
        int tempSensorSize; 
        getIntegerParam(ADMaxSizeX,     &tempSensorSize);
        double tempMax=(double)tempSensorSize;
        if ( !checkSetGet(DCAM_IDPROP_SUBARRAYHSIZE, &tempMax, functionName, "DCAM_IDPROP_SUBARRAYHSIZE") ) {
            setIntegerParam (ADSizeX, (int)tempMax);
        }
        callParamCallbacks();
        getIntegerParam(ADMaxSizeY,     &tempSensorSize);
        tempMax=(double)tempSensorSize;
        if ( !checkSetGet(DCAM_IDPROP_SUBARRAYVSIZE, &tempMax, functionName, "DCAM_IDPROP_SUBARRAYVSIZE") ) {
            setIntegerParam (ADSizeY,  (int)tempMax);
        }

        callParamCallbacks();
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: HamaRegionReset finished. " 
            "minX: %f, minY: %f, sizeX: %f, sizeY: %f\n", 
            driverName, functionName, tempResetVal, tempResetVal, tempMax, tempMax);
    }
    
    else if (function == HamaTriggerSource) {
        callParamCallbacks();
        checkSet(DCAM_IDPROP_TRIGGERSOURCE, value, functionName, "IDPROP:TRIGGERSOURCE");
    }

    else if (function == ADTriggerMode) {
        checkSet(DCAM_IDPROP_TRIGGER_MODE, value, functionName, "IDPROP:TRIGGER_MODE");
        callParamCallbacks();
    }

    else if (function == HamaTriggerActive) { 
        //0: edge
        //1: level
        //2: syncreadout
        if (value ==0) {
            checkSet(DCAM_IDPROP_TRIGGERACTIVE, DCAMPROP_TRIGGERACTIVE__EDGE, functionName, "IDPROP:TRIGGERACTIVE");
        }
        else if (value == 1) {
            checkSet(DCAM_IDPROP_TRIGGERACTIVE, DCAMPROP_TRIGGERACTIVE__LEVEL, functionName, "IDPROP:TRIGGERACTIVE");
        }
        else if (value == 2) {
            checkSet(DCAM_IDPROP_TRIGGERACTIVE, DCAMPROP_TRIGGERACTIVE__SYNCREADOUT, functionName, "IDPROP:TRIGGERACTIVE");
        } else {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Invalid trigger active option. "
                " Available options are 0 (edge), 1 (level) and 2 (syncreadout).\n",
                driverName, functionName);
        }
        callParamCallbacks();
    }

    else if (function == HamaTriggerPolarity) { 
        //0: negative
        //1: positive
        if (value == 0) {
            checkSet(DCAM_IDPROP_TRIGGERPOLARITY, DCAMPROP_TRIGGERPOLARITY__NEGATIVE, functionName, "IDPROP:TRIGGERPOLARITY");
        }
        else if (value == 1) {
            checkSet(DCAM_IDPROP_TRIGGERPOLARITY, DCAMPROP_TRIGGERPOLARITY__POSITIVE, functionName, "IDPROP:TRIGGERPOLARITY");
        } else {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Invalid trigger polarity option. "
                " Available options are 0 (negative) and 1 (positive).\n",
                driverName, functionName);
        }
    }

    else if (function == HamaFireTrigger) {
        dcamcap_firetrigger( hdcam );
        int triggermode;
        getIntegerParam(ADTriggerMode, &triggermode);
        if (triggermode == 1) { //trigger mode = start
             setIntegerParam(HamaTriggerSource,0);// internal
        }
        callParamCallbacks();
    }
    
    else if (function == HamaTriggerTimes) { 
        int tempTrigger; 
        getIntegerParam(HamaTriggerTimes,     &tempTrigger);
        double tempTriggerTimes=(double)tempTrigger;
        if (tempTriggerTimes<1) {
            tempTriggerTimes=1;
        }
        if (tempTriggerTimes>10000) {
            tempTriggerTimes=10000;
        }
        if ( !checkSetGet(DCAM_IDPROP_TRIGGERTIMES, &tempTriggerTimes, functionName, "DCAM_IDPROP_TriggerTimes") ) {
            setIntegerParam (HamaTriggerTimes, (int)tempTriggerTimes);
        }
        callParamCallbacks();
    }
    
    else if (function ==ADReadStatus) {
        updateCoolerInfo();
        this->isSysAlive();
    }
    
    else if (function == HamaReadoutSpeed) { 
        if (value == 1) {
            checkSet(DCAM_IDPROP_READOUTSPEED, DCAMPROP_READOUTSPEED__SLOWEST, functionName, "IDPROP:READOUTSPEED");
        } else if (value == 2) {
            checkSet(DCAM_IDPROP_READOUTSPEED, DCAMPROP_READOUTSPEED__FASTEST, functionName, "IDPROP:READOUTSPEED");
        }
        double readSpeed;
        if ( !checkGet(DCAM_IDPROP_READOUTSPEED, &readSpeed, functionName, "IDPROP:READOUTSPEED") ) {
            setIntegerParam(HamaReadoutSpeed, (int)readSpeed);
        }
    }
    
    else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_HAMA_DETECTOR_PARAM) status = ADDriver::writeInt32(pasynUser, value);
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "\n%s:writeInt32 error, status=%d function=%d, value=%d\n",
              driverName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "\n%s:writeInt32: function=%d, value=%d\n",
              driverName, function, value);
    return status;
}


/** Called when asyn clients call pasynFloat64->write().
  * This function performs actions for some parameters, including ADAcquireTime, ADGain, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus hamamatsu::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *functionName = "writeFloat64";

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setDoubleParam(function, value);

    /* Changing any of the simulation parameters requires recomputing the base image */
    
    if (function == HamaTriggerDelay) { 
        double tempDelay; 
        getDoubleParam(HamaTriggerDelay,     &tempDelay);
        if (tempDelay<0) {
            tempDelay=0;
        }
        if (tempDelay>10) {
            tempDelay=10;
        }
        if ( !checkSetGet(DCAM_IDPROP_TRIGGERDELAY, &tempDelay, functionName, "DCAM_IDPROP_TRIGGERDELAY") ) {
            setDoubleParam (HamaTriggerDelay,     tempDelay);
        }
        callParamCallbacks();
    }
    /*Set exposure time when camera is live*/
    else if (function == ADAcquireTime) {
        double exposure = value;
        if ( !checkSetGet(DCAM_IDPROP_EXPOSURETIME, &exposure, functionName, "DCAM_IDPROP_EXPOSURETIME") ) {
            setDoubleParam(ADAcquireTime, exposure); // Store actual value applied
        }
        callParamCallbacks();
    }
    else if ((function < FIRST_HAMA_DETECTOR_PARAM)) {
        /* This parameter belongs to a base class call its method */
        status = ADDriver::writeFloat64(pasynUser, value);
    }
    
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "\n%s:writeFloat64 error, status=%d function=%d, value=%f\n",
              driverName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "\n%s:writeFloat64: function=%d, value=%f\n",
              driverName, function, value);
    return status;
}

void hamamatsu::updateCoolerInfo(void)
{
    const char *functionName = "updateCoolerInfo";
    double tempCoolerStatus; 
    DCAMERR err;

    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_SENSORCOOLERSTATUS, &tempCoolerStatus);

    char text[64];
    DCAMPROP_VALUETEXT valueText;
    memset( &valueText, 0, sizeof(valueText) );
    valueText.cbSize    = sizeof(valueText);
    valueText.iProp    = DCAM_IDPROP_SENSORCOOLERSTATUS;
    valueText.value    = tempCoolerStatus;
    valueText.text    = text;
    valueText.textbytes = sizeof(text);
    err = dcamprop_getvaluetext( hdcam, &valueText );
    if ( checkErrAndPrint(__FUNCTION__, err, "dcamprop_getvaluetext(..., IDPROP:SENSORCOOLERSTATUS)") ) {
        setStringParam (HamaSensorCoolerStatus, "Unknown" );
    } else {
        setStringParam (HamaSensorCoolerStatus, valueText.text );
    }
    
    double temperature;
    if ( checkGet(DCAM_IDPROP_SENSORTEMPERATURE, &temperature, functionName, "IDPROP:SENSORTEMPERATURE") ) {
        setDoubleParam (ADTemperatureActual, 0. );
    } else  {
        setDoubleParam (ADTemperatureActual, temperature );
    }
}

/** Report status of the driver.
  * Prints details about the driver if details>0.
  * It then calls the ADDriver::report() method.
  * \param[in] fp File pointed passed by caller where the output is written to.
  * \param[in] details If >0 then driver details are printed.
  */
void hamamatsu::report(FILE *fp, int details)
{

    fprintf(fp, "Hamamatsu detector %s\n", this->portName);
    if (details > 0) {
        int nx, ny, dataType;
        getIntegerParam(ADSizeX, &nx);
        getIntegerParam(ADSizeY, &ny);
        getIntegerParam(NDDataType, &dataType);
        fprintf(fp, "  NX, NY:            %d  %d\n", nx, ny);
        fprintf(fp, "  Data type:         %d\n", dataType);
    }
    /* Invoke the base class method */
    ADDriver::report(fp, details);
}

//on exit do the following.
void hamamatsu::hamaExit()
{
    dcamapi_uninit();
}

//deconstructor
hamamatsu::~hamamatsu()
{
    dcamapi_uninit();
}

void hamamatsu::hamaAbortAcquisition()
{
    const char* functionName = "hamaAbortAcquisition";
    
    setStringParam(ADStatusMessage, "Stopped acquisition");
    setIntegerParam(ADStatus, ADStatusIdle);
    callParamCallbacks();

    setIntegerParam(ADAcquire, 0);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
          "%s:%s: acquisition completed\n", driverName, functionName);

    epicsEventSignal(stopEventId_);
    callParamCallbacks();

    this->unlock();
    epicsEventWaitWithTimeout(stopEventId_, MIN_DELAY);
    callParamCallbacks();
    this->lock();
    epicsThreadSleep(0.01);
}

/** Constructor for hamamatsu; most parameters are simply passed to ADDriver::ADDriver.
  * After calling the base class constructor this method creates a thread to compute the simulated detector data,
  * and sets reasonable default values for parameters defined in this class, asynNDArrayDriver and ADDriver.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  */
hamamatsu::hamamatsu(const char *portName, int camIndex, int maxBuffers, size_t maxMemory, int priority, int stackSize)

    : ADDriver(portName, 1, 0, maxBuffers, maxMemory,
               0, 0, /* No interfaces beyond those set in ADDriver.cpp */
               0, 1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=0, autoConnect=1 */
               priority, stackSize),
      hdcam(0)

{
    int status = asynSuccess;
    const char *functionName = "hamamatsu";

    /* Create the epicsEvents for signaling to the acquisition task when acquisition starts and stops */
    startEventId_ = epicsEventCreate(epicsEventEmpty);
    if (!startEventId_) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s epicsEventCreate failure for start event\n",
            driverName, functionName);
        return;
    }
    stopEventId_ = epicsEventCreate(epicsEventEmpty);
    if (!stopEventId_) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s epicsEventCreate failure for stop event\n",
            driverName, functionName);
        return;
    }

     //Hamamatsu Specific Parameters
    createParam(HamaRegionResetString,        asynParamInt32, &HamaRegionReset);
    
    //Hamamatsu Trigger parameters
    createParam(HamaTriggerSourceString,      asynParamInt32,   &HamaTriggerSource);
    createParam(HamaTriggerActiveString,      asynParamInt32,   &HamaTriggerActive);
    createParam(HamaTriggerPolarityString,    asynParamInt32,   &HamaTriggerPolarity);
    createParam(HamaFireTriggerString,        asynParamInt32,   &HamaFireTrigger);
    createParam(HamaTriggerTimesString,       asynParamInt32,   &HamaTriggerTimes);
    createParam(HamaTriggerDelayString,       asynParamFloat64, &HamaTriggerDelay);
    
    //Hamamatsu Cooler parameters
    createParam(HamaSensorCoolerStatusString, asynParamOctet, &HamaSensorCoolerStatus);
    
    createParam(HamaReadoutSpeedString,       asynParamInt32, &HamaReadoutSpeed);
    createParam(HamaSystemAliveString,        asynParamInt32, &HamaSystemAlive);

    /* Set some default values for parameters */
    status |= setIntegerParam(ADTriggerMode,0);

    status |= setIntegerParam(HamaRegionReset,0);
    status |= setIntegerParam(HamaTriggerSource,0);
    status |= setIntegerParam(HamaTriggerActive,0);
    status |= setIntegerParam(HamaTriggerPolarity,0);
    status |= setIntegerParam(HamaFireTrigger,0);
    status |= setIntegerParam(HamaTriggerTimes,1);
    status |= setDoubleParam(HamaTriggerDelay,0);
    status |= setStringParam(HamaSensorCoolerStatus,"Unknown");
    status |= setIntegerParam(HamaReadoutSpeed,2);
    
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s Unable to set camera parameters.\n", driverName, functionName);
        return;
    }

    DCAMERR err;
    memset( &apiinit, 0, sizeof(apiinit) );
    apiinit.size    = sizeof(apiinit);
    
    err = dcamapi_init( &apiinit );
    if ( checkErrAndPrint(__FUNCTION__, err, "dcamapi_init()") ) {
        dcamapi_uninit();
        return;
    }

    int32    nDevice = apiinit.iDeviceCount;
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s DCAM initialized. dcamapi_init() found %d devices\n",
            driverName, functionName, nDevice);
    // open specified camera
    int32 iDevice;
    for( iDevice = 0; iDevice < nDevice; iDevice++ ) {
        DCAMDEV_OPEN    devopen;
        memset( &devopen, 0, sizeof(devopen) );
        devopen.size    = sizeof(devopen);
        devopen.index    = iDevice;
        err = dcamdev_open( &devopen );
        if ( !checkErrAndPrint(__FUNCTION__, err, "dcamdev_open()", "index is %d\n", iDevice) ) {
            hdcam = devopen.hdcam;
            dcamcon_show_dcamdev_info( hdcam );
            DCAMDEV_STRING    strStruct;
            memset( &strStruct, 0, sizeof(strStruct) );
            strStruct.size=sizeof(strStruct);
            char strVal[256];
            strStruct.text=strVal;
            strStruct.textbytes = sizeof(strVal);
            strStruct.iString=DCAM_IDSTR_MODEL;
            dcamdev_getstring(hdcam, &strStruct);
            dcamdev_close( hdcam );
        }
    }

    // open specified camera
    DCAMDEV_OPEN    devopen;
    memset( &devopen, 0, sizeof(devopen) );
    devopen.size    = sizeof(devopen);
    devopen.index = camIndex;
    err = dcamdev_open( &devopen );
    if(failed(err)) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "Failed to open camera %d, err=0x%x\n", camIndex, err);
        return;
    }
    hdcam = devopen.hdcam;
    
    dcamcon_show_dcamdev_info( hdcam );
    
    DCAMDEV_STRING    strStruct;
    memset( &strStruct, 0, sizeof(strStruct) );
    strStruct.size=sizeof(strStruct);
    char strVal[256];
    strStruct.text=strVal;
    strStruct.textbytes = sizeof(strVal);
    
    //get strings for device/ dcam information
    strStruct.iString=DCAM_IDSTR_VENDOR;
    dcamdev_getstring(hdcam, &strStruct);
    status =  setStringParam (ADManufacturer, strVal);
    
    strStruct.iString=DCAM_IDSTR_MODEL;
    dcamdev_getstring(hdcam, &strStruct);
    status = setStringParam (ADModel, strStruct.text);
    
    //epicsSnprintf(versionString, sizeof(versionString), "%d.%d.%d", DRIVER_VERSION, DRIVER_REVISION, DRIVER_MODIFICATION);
    
    strStruct.iString=DCAM_IDSTR_DRIVERVERSION;
    dcamdev_getstring(hdcam, &strStruct);
    setStringParam(NDDriverVersion, strStruct.text);
    
    strStruct.iString=DCAM_IDSTR_MODULEVERSION;
    dcamdev_getstring(hdcam, &strStruct);
    setStringParam(ADSDKVersion, strStruct.text);
    
    strStruct.iString=DCAM_IDSTR_CAMERAID;
    dcamdev_getstring(hdcam, &strStruct);
    setStringParam(ADSerialNumber, strStruct.text);
    
    strStruct.iString=DCAM_IDSTR_CAMERAVERSION;
    dcamdev_getstring(hdcam, &strStruct);
    setStringParam(ADFirmwareVersion, strStruct.text);
    
    //get sensor size
    double tempSensorSize; 
    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_SUBARRAYHSIZE, &tempSensorSize );
    setIntegerParam (ADMaxSizeX, (int)tempSensorSize);

    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_SUBARRAYVSIZE, &tempSensorSize );
    setIntegerParam (ADMaxSizeY, (int)tempSensorSize);
    
    //get pixel type
    int dataType;
    getIntegerParam(NDDataType, &dataType);
    if (dataType ==0 || dataType ==1) {  // @TODO: implement correct data types
        err = dcamprop_setvalue( hdcam, DCAM_IDPROP_IMAGE_PIXELTYPE, DCAM_PIXELTYPE_MONO8 );
    }  else {
        err = dcamprop_setvalue( hdcam, DCAM_IDPROP_IMAGE_PIXELTYPE, DCAM_PIXELTYPE_MONO16 );   
    }
    
    updateCoolerInfo();
         
    //get readout speed
    double readSpeed;
    if ( !checkGet(DCAM_IDPROP_READOUTSPEED, &readSpeed, functionName, "IDPROP:READOUTSPEED") ) {
        setIntegerParam(HamaReadoutSpeed, (int)readSpeed);
    }
        
    /* Create the thread that updates the images */
    status = (epicsThreadCreate("hamaDetTask",
                                epicsThreadPriorityMedium,
                                epicsThreadGetStackSize(epicsThreadStackMedium),
                                (EPICSTHREADFUNC)hamaTaskC,
                                this) == NULL);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: epicsThreadCreate failure for image task. Error code: %d\n",
            driverName, functionName, status);
        return;
    }
    
    atexit(hamaExit);
    
}

/** Configuration command, called directly or from iocsh */
extern "C" int hamamatsuConfig(const char *portName, int camIndex, int maxBuffers, int maxMemory, int priority, int stackSize)
{
    new hamamatsu(portName, camIndex,
                    (maxBuffers < 0) ? 0 : maxBuffers,
                    (maxMemory < 0) ? 0 : maxMemory, 
                    priority, stackSize);
    return(asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg hamamatsuConfigArg0 = {"Port name", iocshArgString};
static const iocshArg hamamatsuConfigArg1 = {"camIndex", iocshArgInt};
static const iocshArg hamamatsuConfigArg2 = {"maxBuffers", iocshArgInt};
static const iocshArg hamamatsuConfigArg3 = {"maxMemory", iocshArgInt};
static const iocshArg hamamatsuConfigArg4 = {"priority", iocshArgInt};
static const iocshArg hamamatsuConfigArg5 = {"stackSize", iocshArgInt};
static const iocshArg * const hamamatsuConfigArgs[] =  {&hamamatsuConfigArg0,
                                                        &hamamatsuConfigArg1,
                                                        &hamamatsuConfigArg2,
                                                        &hamamatsuConfigArg3,
                                                        &hamamatsuConfigArg4,
                                                        &hamamatsuConfigArg5};
static const iocshFuncDef confighamamatsu = {"hamamatsuConfig", 6, hamamatsuConfigArgs};
static void confighamamatsuCallFunc(const iocshArgBuf *args)
{
    hamamatsuConfig(args[0].sval, args[1].ival, args[2].ival, args[3].ival, args[4].ival, args[5].ival);
}

static void hamamatsuRegister(void)
{
    iocshRegister(&confighamamatsu, confighamamatsuCallFunc);
}

extern "C" {
epicsExportRegistrar(hamamatsuRegister);
}