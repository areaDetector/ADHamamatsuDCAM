# Must have loaded envPaths via st.cmd*

errlogInit(20000)

dbLoadDatabase("$(TOP)/dbd/hamamatsuApp.dbd")
hamamatsuApp_registerRecordDeviceDriver(pdbbase) 

# Prefix for all records
epicsEnvSet("PREFIX", "13HAMA1:")
# The port name for the detector
epicsEnvSet("PORT",   "HAMA1")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")
# The maximum image width; used to set the maximum size for this driver and for row profiles in the NDPluginStats plugin
epicsEnvSet("XSIZE",  "4432")
# The maximum image height; used to set the maximum size for this driver and for column profiles in the NDPluginStats plugin
epicsEnvSet("YSIZE",  "2368")
# The maximum number of time series points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "2048")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")
# The maximum number of threads for plugins which can run in multiple threads
epicsEnvSet("MAX_THREADS", "8")
# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

asynSetMinTimerPeriod(0.001)

# The EPICS environment variable EPICS_CA_MAX_ARRAY_BYTES needs to be set to a value at least as large
# as the largest image that the standard arrays plugin will send.
# That value is $(XSIZE) * $(YSIZE) * sizeof(FTVL data type) for the FTVL used when loading the NDStdArrays.template file.
# The variable can be set in the environment before running the IOC or it can be set here.
# It is often convenient to set it in the environment outside the IOC to the largest array any client 
# or server will need.  For example 10000000 (ten million) bytes may be enough.
# If it is set here then remember to also set it outside the IOC for any CA clients that need to access the waveform record.  
# Do not set EPICS_CA_MAX_ARRAY_BYTES to a value much larger than that required, because EPICS Channel Access actually
# allocates arrays of this size every time it needs a buffer larger than 16K.
# Uncomment the following line to set it in the IOC.
#epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "10000000")

# Create a hamamatsu driver
# hamamatsuConfig(const char *portName, int camIndex, int maxBuffers, int maxMemory, int priority, int stackSize)
hamamatsuConfig("$(PORT)", 0, 0)
# To have the rate calculation use a non-zero smoothing factor use the following line
#dbLoadRecords("hamamatsu.template",     "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1,RATE_SMOOTH=0.2")
dbLoadRecords("$(ADHAMAMATSU)/db/hamamatsu.template","P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Create a second hamamatsu driver
#hamamatsuConfig("HAMA2", 300, 200, 1, 50, 50000000)
#dbLoadRecords("$(ADHAMAMATSU)/db/hamamatsu.template","P=$(PREFIX),R=cam2:,PORT=HAMA2,ADDR=0,TIMEOUT=1")

# Load an NDFile database.  This is not supported for the hamamatsu which does not write files.
#dbLoadRecords("NDFile.template","P=$(PREFIX),R=cam1:,PORT=HAMA1,ADDR=0,TIMEOUT=1")

# Create a standard arrays plugin, set it to get data from first hamamatsu driver.
NDStdArraysConfigure("Image1", 20, 0, "$(PORT)", 0, 0, 0, 0, 0, 5)

# This creates a waveform large enough for 2000x2000x3 (e.g. RGB color) arrays.
# This waveform only allows transporting 8-bit images
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int8,FTVL=UCHAR,NELEMENTS=12000000")
# This waveform only allows transporting 16-bit images
#dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=12000000")
# This waveform allows transporting 32-bit images
#dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=12000000")
# This waveform allows transporting 64-bit float images
#dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image1:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=12000000")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot/commonPlugins.cmd

# Create a standard arrays plugin, set it to get data from FFT plugin.
NDStdArraysConfigure("Image2", 3, 0, "FFT1", 0)
# This waveform allows transporting 64-bit images, so it can handle any detector data type at the expense of more memory and bandwidth
dbLoadRecords("NDStdArrays.template", "P=$(PREFIX),R=image2:,PORT=Image2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=FFT1,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=12000000")

set_requestfile_path("$(ADHAMAMATSU)/hamamatsuApp/Db")

asynSetTraceIOMask("$(PORT)",0,ESCAPE)
#asynSetTraceMask("$(PORT)",0,ERROR|FLOW|DRIVER)
#asynSetTraceIOMask("FileNetCDF",0,2)
#asynSetTraceMask("FileNetCDF",0,255)
#asynSetTraceMask("FileNexus",0,255)
#asynSetTraceMask("HAMA2",0,255)

callbackSetQueueSize(5000)

iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30, "P=$(PREFIX)")
