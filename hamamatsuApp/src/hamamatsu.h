/* For end use and not for commercial purposes */

#include <epicsEvent.h>
#include "ADDriver.h"

#include "console4.h"
#include "common.h"
#include "dcamapi4.h"
#include "dcamprop.h"



#define DRIVER_VERSION      2
#define DRIVER_REVISION     9
#define DRIVER_MODIFICATION 0

/** Hamamatsu detector driver */
class epicsShareClass hamamatsu : public ADDriver {
public:
    hamamatsu(const char *portName, int camIndex, int maxBuffers, size_t maxMemory,
              int priority, int stackSize);

    /* These are the methods that we override from ADDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual void setShutter(int open);
    virtual void report(FILE *fp, int details);
    void simTask(); /**< Should be private, but gets called from C, so must be public */
    ~hamamatsu();
    static void hamaExit();
    void hamaAcquire();
    void hamaAbortAcquisition();

protected:
    int HamaRegionReset;
    #define FIRST_HAMA_DETECTOR_PARAM HamaRegionReset
    int HamaTriggerSource;
    int HamaTriggerMode;
    int HamaTriggerActive;
    int HamaTriggerPolarity;
    int HamaFireTrigger;
    int HamaTriggerTimes;
    int HamaTriggerDelay;
    int HamaSensorCoolerStatus;
    int HamaReadoutSpeed;

private:
    /* These are the methods that are new to this class */
    BOOL copy_targetarea( HDCAM hdcam, int32 iFrame, void* buf, int32 rowbytes, int32 ox, int32 oy, int32 cx, int32 cy );
    void get_image_information( HDCAM hdcam, int32& pixeltype, int32& width, int32& rowbytes, int32& height );
    void sample_access_image( HDCAM hdcam );
    void updateCoolerInfo(void);

    /* Our data */
    epicsEventId startEventId_;
    epicsEventId stopEventId_;
    HDCAM    hdcam;
    DCAMAPI_INIT    apiinit;
};

#define HamaRegionResetString         "HamaRegionReset"
#define HamaTriggerSourceString       "HamaTriggerSource"
#define HamaTriggerModeString         "HamaTriggerMode"
#define HamaTriggerActiveString       "HamaTriggerActive"
#define HamaTriggerPolarityString     "HamaTriggerPolarity"
#define HamaFireTriggerString         "HamaFireTrigger"
#define HamaTriggerTimesString        "HamaTriggerTimes"
#define HamaTriggerDelayString        "HamaTriggerDelay"
#define HamaSensorCoolerStatusString  "HamaSensorCoolerStatus"
#define HamaReadoutSpeedString        "HamaReadoutSpeed"
