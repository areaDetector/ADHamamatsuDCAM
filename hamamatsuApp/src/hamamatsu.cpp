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
    if( failed(err) )
    {
        dcamcon_show_dcamerr( hdcam, err, "dcambuf_copyframe()" );
        return FALSE;
    }
#else
    // access image
    err = dcambuf_lockframe( hdcam, &bufframe );
    if( failed(err) )
    {
        dcamcon_show_dcamerr( hdcam, err, "dcambuf_lockframe()" );
        return FALSE;
    }

    if( bufframe.type != DCAM_PIXELTYPE_MONO16 )
    {
        printf( "not implement pixel type\n" );
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
    if( failed(err) )
    {
        dcamcon_show_dcamerr( hdcam, err, "dcamprop_getvalue()", "IDPROP:IMAGE_PIXELTYPE" );
        return;
    }
    else
        pixeltype = (int32) v;

    // image width
    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_IMAGE_WIDTH, &v );
    if( failed(err) )
    {
        dcamcon_show_dcamerr( hdcam, err, "dcamprop_getvalue()", "IDPROP:IMAGE_WIDTH" );
        return;
    }
    else
        width = (int32)v;

    // image row bytes
    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_IMAGE_ROWBYTES, &v );
    if( failed(err) )
    {
        dcamcon_show_dcamerr( hdcam, err, "dcamprop_getvalue()", "IDPROP:IMAGE_ROWBYTES" );
        return;
    }
    else
        rowbytes = (int32)v;

    // image height
    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_IMAGE_HEIGHT, &v );
    if( failed(err) )
    {
        dcamcon_show_dcamerr( hdcam, err, "dcamprop_getvalue()", "IDPROP:IMAGE_HEIGHT" );
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
    if( failed(err) )
    {
        dcamcon_show_dcamerr( hdcam, err, "dcamcap_transferinfo()" );
        return;
    }

    if( captransferinfo.nFrameCount < 1 )
    {
        printf( "not capture image\n" );
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
        err = dcamprop_setgetvalue( hdcam, DCAM_IDPROP_EXPOSURETIME, &exposureTime );
        printf("\nexposure time %f", exposureTime);
        setDoubleParam(ADAcquireTime, exposureTime);

        /*
        getDoubleParam(ADAcquirePeriod,     &acquirePeriod);
        err = dcamprop_setgetvalue( hdcam, DCAM_IDPROP_INTERNALFRAMERATE, &acquirePeriod );
        if( failed(err) )
        {
            dcamcon_show_dcamerr( hdcam, err, "dcamprop_setgetvalue() for DCAM_IDPROP_INTERNALFRAMERATE" );
        }
        printf("\nacquire period %f", acquirePeriod);
        setDoubleParam(ADAcquirePeriod, acquirePeriod);
        */

        double tempHPOS=(double)minX;
        err = dcamprop_setgetvalue( hdcam, DCAM_IDPROP_SUBARRAYHPOS, &tempHPOS );
        if( failed(err) ) {
            printf("\n");
            dcamcon_show_dcamerr( hdcam, err, "DCAM_IDPROP_SUBARRAYHPOS" );
            acquire=0;
            hamaAbortAcquisition();
            continue;
        }
        
        double tempVPOS=(double)minY; 
        err = dcamprop_setgetvalue( hdcam, DCAM_IDPROP_SUBARRAYVPOS, &tempVPOS );
        if( failed(err) ) {
            printf("\n");
            dcamcon_show_dcamerr( hdcam, err, "DCAM_IDPROP_SUBARRAYVPOS" );
            acquire=0;
            hamaAbortAcquisition();
            continue;   
        }
        
        double tempHSIZE=(double)sizeX; 
        err = dcamprop_setgetvalue( hdcam, DCAM_IDPROP_SUBARRAYHSIZE, &tempHSIZE );
        if( failed(err) ) {
            printf("\n");
            dcamcon_show_dcamerr( hdcam, err, "DCAM_IDPROP_SUBARRAYHSIZE" );
            acquire=0;
            hamaAbortAcquisition();
            continue;           
        }

        double tempVSIZE=(double)sizeY;
        err = dcamprop_setgetvalue( hdcam, DCAM_IDPROP_SUBARRAYVSIZE, &tempVSIZE );
        if( failed(err) ) {
            printf("\n");
            dcamcon_show_dcamerr( hdcam, err, "DCAM_IDPROP_SUBARRAYVSIZE" );
            acquire=0;
            hamaAbortAcquisition();
            continue;
        }
        
        err = dcamprop_setvalue( hdcam, DCAM_IDPROP_SUBARRAYMODE, DCAMPROP_MODE__ON );
        if( failed(err) ) {
            printf("\n");
            dcamcon_show_dcamerr( hdcam, err, "dcamprop_setvalue()", "IDPROP:SUBARRAYMODE, VALUE:ON" );
            acquire=0;
            hamaAbortAcquisition();
            continue;
        } else {
            setIntegerParam (ADMinX,   (int)tempHPOS);
            setIntegerParam (ADSizeX,  (int)tempHSIZE);
            setIntegerParam (ADMinY,   (int)tempVPOS);
            setIntegerParam (ADSizeY,  (int)tempVSIZE);
            
            printf( "\nminX: %f", tempHPOS);
            printf( "\nsizeX: %f", tempHSIZE );
            printf( "\nminY: %f", tempVPOS );
            printf( "\nsizeY: %f", tempVSIZE );
        }
        callParamCallbacks();
        
        // check whether the camera supports binning or not.
        double binning=(double)binX;
        err = dcamprop_queryvalue( hdcam, DCAM_IDPROP_BINNING, &binning );
        if( failed(err) ) {
            dcamcon_show_dcamerr( hdcam, err, "dcamprop_queryvalue()", "IDPROP:BINNING, VALUE:%d", binning ); 
        }
    
        // set binning value to the camera
        err = dcamprop_setgetvalue( hdcam, DCAM_IDPROP_BINNING, &binning );
        if( failed(err) ) {
            dcamcon_show_dcamerr( hdcam, err, "dcamprop_setvalue()", "IDPROP:BINNING, VALUE:%d", binning ); 
            binning=1;
            err = dcamprop_setgetvalue( hdcam, DCAM_IDPROP_BINNING, &binning );
            setIntegerParam (ADBinX,     (int)binning);
            setIntegerParam (ADBinY,     (int)binning);
        } else {
            setIntegerParam (ADBinX,     (int)binning);
            printf( "\nbinX: %f", binning );
        }
        setIntegerParam(ADStatus, ADStatusAcquire);

        /* Open the shutter */
        setShutter(ADShutterOpen);

        // open wait handle
        DCAMWAIT_OPEN    waitopen;
        memset( &waitopen, 0, sizeof(waitopen) );
        waitopen.size    = sizeof(waitopen);
        waitopen.hdcam    = hdcam;
    
        err = dcamwait_open( &waitopen );
        if( failed(err) ) {
            dcamcon_show_dcamerr( hdcam, err, "dcamwait_open()" );
        } else {
            //HDCAMWAIT hwait = waitopen.hwait;
            hwait = waitopen.hwait;
    
            // allocate buffer
            int32 number_of_buffer = 10;
            err = dcambuf_alloc( hdcam, number_of_buffer );
            if( failed(err) ) {
                dcamcon_show_dcamerr( hdcam, err, "dcambuf_alloc()" );
            } else {
                // start capture
                err = dcamcap_start( hdcam, DCAMCAP_START_SEQUENCE );
                if( failed(err) ) {
                    dcamcon_show_dcamerr( hdcam, err, "dcamcap_start()" );
                } else  {
                    printf( "\nStart Capture\n" );
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
                            if( failed(err) ) {
                                dcamcon_show_dcamerr( hdcam, err, "dcamwait_start()" );
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
                            if( failed(err) ) {
                                dcamcon_show_dcamerr( hdcam, err, "dcamwait_start()" );
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
                        if( failed(err) ) {
                            dcamcon_show_dcamerr( hdcam, err, "dcamwait_start()" );
                            
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
                    printf( "Stop Capture\n" );
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
    int function = pasynUser->reason;
    int adstatus;
    int acquiring;
    int imageMode;
    asynStatus status = asynSuccess;
    DCAMERR err;

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
            printf( "Start Capture pressed\n" );
        }
        if (!value && acquiring) {
            /* This was a command to stop acquisition */
            /* Send the stop event */
            //epicsEventSignal(stopEventId_); 
            // stop capture
            printf( "Stop Capture pressed\n" );
            
            
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

            //this->unlock();
            //status = epicsEventWaitWithTimeout(stopEventId_, MIN_DELAY);
            //callParamCallbacks();
            //this->lock();
            //epicsThreadSleep(0.01);
        }

    } else if (function == NDDataType) {
        int dataType;
        getIntegerParam(NDDataType, &dataType);
        if (dataType ==0 || dataType ==1)//0= int8, 1=uint8
        {
            printf( "\nInt 8\n");
            //getDoubleParam(ADAcquireTime,     &exposureTime);
            err = dcamprop_setvalue( hdcam, DCAM_IDPROP_IMAGE_PIXELTYPE, DCAM_PIXELTYPE_MONO8 );
            if( failed(err) ) {
                dcamcon_show_dcamerr( hdcam, err, "dcamprop_setvalue()", "IDPROP:IMAGE_PIXELTYPE, VALUE:MONO8" );
                err = dcamprop_setvalue( hdcam, DCAM_IDPROP_IMAGE_PIXELTYPE, DCAM_PIXELTYPE_MONO16 );
                dataType=3;
                setIntegerParam(NDDataType, dataType);
            }
        } else// if (dataType ==2 || dataType ==3) 2=int16, 3=uint16
        {
            printf( "\nInt 16\n");
            err = dcamprop_setvalue( hdcam, DCAM_IDPROP_IMAGE_PIXELTYPE, DCAM_PIXELTYPE_MONO16 );
        }
        callParamCallbacks();
    }
    
    else if (function == HamaRegionReset) {
        //printf ("\nHama Reset: %d", value);
        double tempResetVal=0;
        err = dcamprop_setvalue( hdcam, DCAM_IDPROP_SUBARRAYMODE, DCAMPROP_MODE__OFF );
        if( failed(err) ) {
            dcamcon_show_dcamerr( hdcam, err, "dcamprop_setvalue()", "IDPROP:SUBARRAYMODE, VALUE:OFF" );
        }
        err = dcamprop_setgetvalue( hdcam, DCAM_IDPROP_SUBARRAYHPOS, &tempResetVal );
        if( failed(err) ) {
            dcamcon_show_dcamerr( hdcam, err, "DCAM_IDPROP_SUBARRAYHPOS" );
        } else {
            setIntegerParam (ADMinX,    (int)tempResetVal);
            printf( "\nminX reset: %f", tempResetVal);
        }
        err = dcamprop_setgetvalue( hdcam, DCAM_IDPROP_SUBARRAYVPOS, &tempResetVal );
        if( failed(err) ) {
            dcamcon_show_dcamerr( hdcam, err, "DCAM_IDPROP_SUBARRAYVPOS" );
        } else {
            setIntegerParam (ADMinY,    (int)tempResetVal);
            printf( "\nminY reset: %f", tempResetVal);
        }
        int tempSensorSize; 
        getIntegerParam(ADMaxSizeX,     &tempSensorSize);
        double tempMax=(double)tempSensorSize;
        err = dcamprop_setgetvalue( hdcam, DCAM_IDPROP_SUBARRAYHSIZE, &tempMax );
        if( failed(err) ) {
            dcamcon_show_dcamerr( hdcam, err, "DCAM_IDPROP_SUBARRAYHSIZE" );
        } else {
            setIntegerParam (ADSizeX, (int)tempMax);
            printf( "\nSizeX reset: %f", tempMax );
        }
        callParamCallbacks();
        //tempVal=(double)sizeY; 
        getIntegerParam(ADMaxSizeY,     &tempSensorSize);
        tempMax=(double)tempSensorSize;
        err = dcamprop_setgetvalue( hdcam, DCAM_IDPROP_SUBARRAYVSIZE, &tempMax );
        if( failed(err) ) {
            dcamcon_show_dcamerr( hdcam, err, "DCAM_IDPROP_SUBARRAYVSIZE" );
        } else {
            setIntegerParam (ADSizeY,  (int)tempMax);
            printf( "\nSizeY reset: %f", tempMax);
        }
        //setIntegerParam(HamaRegionReset,0);
        callParamCallbacks();
        //printf ("\nHama Reset after: %d", HamaRegionReset);
    }
    
    else if (function == HamaTriggerSource) { 
        //0: internal 
        //1: external 
        //2: software 
        printf("\nTrigger Source: %d",value);
        callParamCallbacks();
        if (value ==0) {
            // set internal trigger mode
            //tempTrigger=1;
            //printf("\nTrigger temp: %d",tempTrigger);
            err = dcamprop_setvalue( hdcam, DCAM_IDPROP_TRIGGERSOURCE, DCAMPROP_TRIGGERSOURCE__INTERNAL );
            if( failed(err) ) {
                dcamcon_show_dcamerr( hdcam, err, "dcamprop_setvalue()", "IDPROP:TRIGGERSOURCE, VALUE:Internal" );
            }
        }
        else if (value ==1) {
            // set external trigger mode
            //tempTrigger=2;
            //printf("\nTrigger temp: %d",tempTrigger);
            err = dcamprop_setvalue( hdcam, DCAM_IDPROP_TRIGGERSOURCE, DCAMPROP_TRIGGERSOURCE__EXTERNAL );
            if( failed(err) ) {
                dcamcon_show_dcamerr( hdcam, err, "dcamprop_setvalue()", "IDPROP:TRIGGERSOURCE, VALUE:External" );
            }
        }
        else if (value ==2) {
            // set software trigger mode
            //tempTrigger=3;
            //printf("\nTrigger temp: %d",tempTrigger);
            err = dcamprop_setvalue( hdcam, DCAM_IDPROP_TRIGGERSOURCE, DCAMPROP_TRIGGERSOURCE__SOFTWARE );
            if( failed(err) ) {
                dcamcon_show_dcamerr( hdcam, err, "dcamprop_setvalue()", "IDPROP:TRIGGERSOURCE, VALUE:Software" );
            }
        }
    }

    else if (function == HamaTriggerMode) { 
        //0:normal
        //1:start
        printf("\nTrigger Mode: %d",value);
        if (value ==0) {
            err = dcamprop_setvalue( hdcam, DCAM_IDPROP_TRIGGER_MODE, DCAMPROP_TRIGGER_MODE__NORMAL );
            if( failed(err) ) {
                dcamcon_show_dcamerr( hdcam, err, "dcamprop_setvalue()", "IDPROP:TRIGGER_MODE, VALUE:Normal" );
            }
        }
        else if (value ==1) {
            err = dcamprop_setvalue( hdcam, DCAM_IDPROP_TRIGGER_MODE, DCAMPROP_TRIGGER_MODE__START );
            if( failed(err) ) {
                dcamcon_show_dcamerr( hdcam, err, "dcamprop_setvalue()", "IDPROP:TRIGGER_MODE, VALUE:Start" );
            }
        }
        callParamCallbacks();
    }

    else if (function == HamaTriggerActive) { 
        //0: edge
        //1: level
        //2: syncreadout
        printf("\nTrigger Active: %d",value);
        if (value ==0) {
            err = dcamprop_setvalue( hdcam, DCAM_IDPROP_TRIGGERACTIVE, DCAMPROP_TRIGGERACTIVE__EDGE );
            if( failed(err) ) {
                dcamcon_show_dcamerr( hdcam, err, "dcamprop_setvalue()", "IDPROP:TRIGGERACTIVE, VALUE:EDGE" );
            }
        }
        else if (value ==1) {
            err = dcamprop_setvalue( hdcam, DCAM_IDPROP_TRIGGERACTIVE, DCAMPROP_TRIGGERACTIVE__LEVEL );
            if( failed(err) ) {
                dcamcon_show_dcamerr( hdcam, err, "dcamprop_setvalue()", "IDPROP:TRIGGERACTIVE, VALUE:LEVEL" );
            }
        }
        else if (value ==2) {
            err = dcamprop_setvalue( hdcam, DCAM_IDPROP_TRIGGERACTIVE, DCAMPROP_TRIGGERACTIVE__SYNCREADOUT );
            if( failed(err) ) {
                dcamcon_show_dcamerr( hdcam, err, "dcamprop_setvalue()", "IDPROP:TRIGGERACTIVE, VALUE:SYNCREADOUT" );
            }
        }
        callParamCallbacks();
    }

    else if (function == HamaTriggerPolarity) { 
        //0: negative
        //1: positive
        printf("\nTrigger Polarity: %d",value);
        if (value ==0) {
            err = dcamprop_setvalue( hdcam, DCAM_IDPROP_TRIGGERPOLARITY, DCAMPROP_TRIGGERPOLARITY__NEGATIVE );
            if( failed(err) ) {
                dcamcon_show_dcamerr( hdcam, err, "dcamprop_setvalue()", "IDPROP:TRIGGERPOLARITY, VALUE:NEGATIVE" );
            }
        }
        else if (value ==1) {
            err = dcamprop_setvalue( hdcam, DCAM_IDPROP_TRIGGERPOLARITY, DCAMPROP_TRIGGERPOLARITY__POSITIVE );
            if( failed(err) ) {
                dcamcon_show_dcamerr( hdcam, err, "dcamprop_setvalue()", "IDPROP:TRIGGERPOLARITY, VALUE:POSITIVE" );
            }
        }
    }

    else if (function ==HamaFireTrigger) {
        printf("\nFire Trigger");
        dcamcap_firetrigger( hdcam );
        int triggermode;
        getIntegerParam(HamaTriggerMode, &triggermode);
        if (triggermode == 1) { //trigger mode = start
             setIntegerParam(HamaTriggerSource,0);// internal
        }
        callParamCallbacks();
    }
    
    else if (function == HamaTriggerTimes) { 
        printf("\nTrigger Times: %d",value);
        int tempTrigger; 
        getIntegerParam(HamaTriggerTimes,     &tempTrigger);
        double tempTriggerTimes=(double)tempTrigger;
        if (tempTriggerTimes<1) {
            tempTriggerTimes=1;
        }
        if (tempTriggerTimes>10000) {
            tempTriggerTimes=10000;
        }
        err = dcamprop_setgetvalue( hdcam, DCAM_IDPROP_TRIGGERTIMES, &tempTriggerTimes );
        if( failed(err) ) {
            dcamcon_show_dcamerr( hdcam, err, "DCAM_IDPROP_TriggerTimes" );
        } else {
            setIntegerParam (HamaTriggerTimes, (int)tempTriggerTimes);
            printf( "\nTrigger Times after: %f", tempTriggerTimes );
        }
        callParamCallbacks();
    }
    
    else if (function ==ADReadStatus) {
        updateCoolerInfo();
    }
    
    else if (function == HamaReadoutSpeed) { 
        //printf("\nReadout speed value: %d",value);
        if (value ==1) {
            err = dcamprop_setvalue( hdcam, DCAM_IDPROP_READOUTSPEED, DCAMPROP_READOUTSPEED__SLOWEST );
            if( failed(err) ) {
                dcamcon_show_dcamerr( hdcam, err, "dcamprop_setvalue()", "IDPROP:READOUTSPEED, VALUE:1" );
            }
        } else if (value ==2) {
            err = dcamprop_setvalue( hdcam, DCAM_IDPROP_READOUTSPEED, DCAMPROP_READOUTSPEED__FASTEST );
            if( failed(err) ) {
                dcamcon_show_dcamerr( hdcam, err, "dcamprop_setvalue()", "IDPROP:READOUTSPEED, VALUE:2" );
            }
        }
        double readSpeed;
        err = dcamprop_getvalue( hdcam, DCAM_IDPROP_READOUTSPEED, &readSpeed );
        if( failed(err) ) {
            dcamcon_show_dcamerr( hdcam, err, "dcamprop_getvalue()", "IDPROP:READOUTSPEED" );
        } else {
            printf("\nReadout speed: %f",readSpeed);
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
    DCAMERR err;

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setDoubleParam(function, value);

    /* Changing any of the simulation parameters requires recomputing the base image */
    
    if (function == HamaTriggerDelay) { 
        printf("\nTrigger Delay: %f",value);
        double tempDelay; 
        getDoubleParam(HamaTriggerDelay,     &tempDelay);
        if (tempDelay<0) {
            tempDelay=0;
        }
        if (tempDelay>10) {
            tempDelay=10;
        }
        err = dcamprop_setgetvalue( hdcam, DCAM_IDPROP_TRIGGERDELAY, &tempDelay );
        if( failed(err) ) {
            dcamcon_show_dcamerr( hdcam, err, "DCAM_IDPROP_TRIGGERDELAY" );
        } else {
            setDoubleParam (HamaTriggerDelay,     tempDelay);
            printf( "\nDelay after: %f", tempDelay );
        }
        callParamCallbacks();
    }
    /*Set exposure time when camera is live*/
    else if (function == ADAcquireTime) {
        double exposure = value;
        err = dcamprop_setgetvalue(hdcam, DCAM_IDPROP_EXPOSURETIME, &exposure);
        if (failed(err)) {
            dcamcon_show_dcamerr(hdcam, err, "DCAM_IDPROP_EXPOSURETIME");
        } else {
            setDoubleParam(ADAcquireTime, exposure); // Store actual value applied
            printf("\nExposure after set: %f", exposure);
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
    double tempCoolerStatus; 
    DCAMERR err;

    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_SENSORCOOLERSTATUS, &tempCoolerStatus);
    //setStringParam (HamaSensorCoolerStatus, std::to_string(tempCoolerStatus).c_str() );

    char text[64];
    DCAMPROP_VALUETEXT valueText;
    memset( &valueText, 0, sizeof(valueText) );
    valueText.cbSize    = sizeof(valueText);
    valueText.iProp    = DCAM_IDPROP_SENSORCOOLERSTATUS;
    valueText.value    = tempCoolerStatus;
    valueText.text    = text;
    valueText.textbytes = sizeof(text);
    err = dcamprop_getvaluetext( hdcam, &valueText );
    if( failed(err) ) {
        dcamcon_show_dcamerr( hdcam, err, "dcamprop_getvaluetext()", "IDPROP:SENSORCOOLERSTATUS" );
        setStringParam (HamaSensorCoolerStatus, "Unknown" );
    } else {
        setStringParam (HamaSensorCoolerStatus, valueText.text );
    }
    
    double temperature; 
    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_SENSORTEMPERATURE, &temperature);
    if( failed(err) ) {
        dcamcon_show_dcamerr( hdcam, err, "dcamprop_getvaluetext()", "IDPROP:SENSORTEMPERATURE" );
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
    printf ("/n HamaExit, exiting\n");
    dcamapi_uninit();
}

//deconstructor
hamamatsu::~hamamatsu()
{
    //const char *functionName = "~hamamatsu";
    printf ("/n deconstructor, exiting\n");
    dcamapi_uninit();
}

void hamamatsu::hamaAbortAcquisition()
{
    const char* functionName = "hamaAbortAcquisition";
    //asynStatus status = asynSuccess;
    printf ("\nAborting Acquisition\n");    
    
    setStringParam(ADStatusMessage, "Stopped acquisition");
    setIntegerParam(ADStatus, ADStatusIdle);
    callParamCallbacks();

    int acquire = 0;
    setIntegerParam(ADAcquire, acquire);
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
    //char versionString[20];
    const char *functionName = "hamamatsu";

    /* Create the epicsEvents for signaling to the simulate task when acquisition starts and stops */
    startEventId_ = epicsEventCreate(epicsEventEmpty);
    if (!startEventId_) {
        printf("%s:%s epicsEventCreate failure for start event\n",
            driverName, functionName);
        return;
    }
    stopEventId_ = epicsEventCreate(epicsEventEmpty);
    if (!stopEventId_) {
        printf("%s:%s epicsEventCreate failure for stop event\n",
            driverName, functionName);
        return;
    }

     //Hamamatsu Specific Parameters
    createParam(HamaRegionResetString,        asynParamInt32, &HamaRegionReset);
    
    //Hamamatsu Trigger parameters
    createParam(HamaTriggerSourceString,      asynParamInt32,   &HamaTriggerSource);
    createParam(HamaTriggerModeString,        asynParamInt32,   &HamaTriggerMode);
    createParam(HamaTriggerActiveString,      asynParamInt32,   &HamaTriggerActive);
    createParam(HamaTriggerPolarityString,    asynParamInt32,   &HamaTriggerPolarity);
    createParam(HamaFireTriggerString,        asynParamInt32,   &HamaFireTrigger);
    createParam(HamaTriggerTimesString,       asynParamInt32,   &HamaTriggerTimes);
    createParam(HamaTriggerDelayString,       asynParamFloat64, &HamaTriggerDelay);
    
    //Hamamatsu Cooler parameters
    createParam(HamaSensorCoolerStatusString, asynParamOctet, &HamaSensorCoolerStatus);
    
    createParam(HamaReadoutSpeedString,       asynParamInt32, &HamaReadoutSpeed);

    /* Set some default values for parameters 
    */

    //Setting Hamamatsu Defaults
    status |= setIntegerParam(HamaRegionReset,0);
    status |= setIntegerParam(HamaTriggerSource,0);
    status |= setIntegerParam(HamaTriggerMode,0);
    status |= setIntegerParam(HamaTriggerActive,0);
    status |= setIntegerParam(HamaTriggerPolarity,0);
    status |= setIntegerParam(HamaFireTrigger,0);
    status |= setIntegerParam(HamaTriggerTimes,1);
    status |= setDoubleParam(HamaTriggerDelay,0);
    char coolerStatusString [256]= "Unknown";
    status |= setStringParam(HamaSensorCoolerStatus,coolerStatusString);
    status |= setIntegerParam(HamaReadoutSpeed,2);
    
    if (status) {
        printf("%s: unable to set camera parameters\n", functionName);
        return;
    }

    DCAMERR err;
    printf( "\n\nHama Init\n\n");
    memset( &apiinit, 0, sizeof(apiinit) );
    apiinit.size    = sizeof(apiinit);
    
    err = dcamapi_init( &apiinit );
    if( failed(err) ) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "Error calling dcamapi_init=0x%x\n", err);
        dcamcon_show_dcamerr( NULL, err, "dcamapi_init()" );
        dcamapi_uninit();
        return;
    }
    printf( "\n\nDCAM initialized\n\n");
    int32    nDevice = apiinit.iDeviceCount;
    printf( "dcamapi_init() found %d device(s).\n", nDevice );
    // open specified camera
    int32 iDevice;
    for( iDevice = 0; iDevice < nDevice; iDevice++ ) {
        printf( "#%d: ", iDevice );
        DCAMDEV_OPEN    devopen;
        memset( &devopen, 0, sizeof(devopen) );
        devopen.size    = sizeof(devopen);
        devopen.index    = iDevice;
        err = dcamdev_open( &devopen );
        if( ! failed(err) ) {
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
        } else {
            dcamcon_show_dcamerr( (HDCAM)(intptr_t)iDevice, err, "dcamdev_open()", "index is %d\n", iDevice );
        }
    }
    printf( "\n\nOpen\n\n");
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
    printf( "\nMaxsizeX: %f", tempSensorSize );
    
    //tempVal=(double)sizeY; 
    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_SUBARRAYVSIZE, &tempSensorSize );
    setIntegerParam (ADMaxSizeY, (int)tempSensorSize);
    printf( "\nMaxsizeY: %f", tempSensorSize);
    
    //get pixel type
    int dataType;
    getIntegerParam(NDDataType, &dataType);
    if (dataType ==0 || dataType ==1) {
        printf( "\nInt 8\n");
        //getDoubleParam(ADAcquireTime,     &exposureTime);
        err = dcamprop_setvalue( hdcam, DCAM_IDPROP_IMAGE_PIXELTYPE, DCAM_PIXELTYPE_MONO8 );
    }  else { // if (dataType ==2 || dataType ==3)
        printf( "\nInt 16\n");
        err = dcamprop_setvalue( hdcam, DCAM_IDPROP_IMAGE_PIXELTYPE, DCAM_PIXELTYPE_MONO16 );   
    }
    
    updateCoolerInfo();
         
    //get readout speed
    double readSpeed;
    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_READOUTSPEED, &readSpeed );
    if( failed(err) ) {
        dcamcon_show_dcamerr( hdcam, err, "dcamprop_getvalue()", "IDPROP:READOUTSPEED" );
    } else {
        printf("\nReadout speed: %f",readSpeed);
        setIntegerParam(HamaReadoutSpeed, (int)readSpeed);
    }
        
    /* Create the thread that updates the images */
    status = (epicsThreadCreate("hamaDetTask",
                                epicsThreadPriorityMedium,
                                epicsThreadGetStackSize(epicsThreadStackMedium),
                                (EPICSTHREADFUNC)hamaTaskC,
                                this) == NULL);
    if (status) {
        printf("%s:%s epicsThreadCreate failure for image task\n",
            driverName, functionName);
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