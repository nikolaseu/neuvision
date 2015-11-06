/*****************************************************************************
*
* Copyright (c) 2002-2008 Lumenera Corporation. All rights reserved.
*
*****************************************************************************/

#ifndef __LUCAMAPI_H
#define __LUCAMAPI_H



#ifdef LUCAMAPI_EXPORTS
#ifdef __cplusplus
#define LUCAM_API extern "C" __declspec(dllexport)
#else
#define LUCAM_API __declspec(dllexport)
#endif
#else
#ifdef __cplusplus
#define LUCAM_API extern "C" __declspec(dllimport)
#else
#define LUCAM_API __declspec(dllimport)
#endif
#endif

#if (_MSC_VER >= 1300)
#define LUCAM_DEPRECATED   __declspec(deprecated)
#else
#define LUCAM_DEPRECATED
#endif


#define LUCAM_EXPORT __stdcall




//----------- Pixel Format IDs ------------------

#define LUCAM_PF_8                           0
#define LUCAM_PF_16                          1
#define LUCAM_PF_24                          2
#define LUCAM_PF_YUV422                      3
#define LUCAM_PF_COUNT                       4
#define LUCAM_PF_FILTER                      5
#define LUCAM_PF_32                          6
#define LUCAM_PF_48                          7


//----------- Properties ------------------------

#define LUCAM_PROP_BRIGHTNESS                0
#define LUCAM_PROP_CONTRAST            	   1
#define LUCAM_PROP_HUE                       2
#define LUCAM_PROP_SATURATION                3
#define LUCAM_PROP_SHARPNESS                 4
#define LUCAM_PROP_GAMMA                     5

#define LUCAM_PROP_PAN                       16
#define LUCAM_PROP_TILT                      17
#define LUCAM_PROP_ROLL                      18
#define LUCAM_PROP_ZOOM                      19
#define LUCAM_PROP_EXPOSURE                  20
#define LUCAM_PROP_IRIS                      21
#define LUCAM_PROP_FOCUS                     22



#define LUCAM_PROP_GAIN                      40
#define LUCAM_PROP_GAIN_RED                  41
#define LUCAM_PROP_GAIN_BLUE                 42
#define LUCAM_PROP_GAIN_GREEN1               43
#define LUCAM_PROP_GAIN_GREEN2               44
#define LUCAM_PROP_GAIN_MAGENTA              41
#define LUCAM_PROP_GAIN_CYAN                 42
#define LUCAM_PROP_GAIN_YELLOW1              43
#define LUCAM_PROP_GAIN_YELLOW2              44


#define LUCAM_PROP_DEMOSAICING_METHOD        64
#define LUCAM_PROP_CORRECTION_MATRIX         65
#define LUCAM_PROP_FLIPPING                  66

#define LUCAM_PROP_DIGITAL_WHITEBALANCE_U    69 // from -100 to 100
#define LUCAM_PROP_DIGITAL_WHITEBALANCE_V    70 // from -100 to 100
#define LUCAM_PROP_DIGITAL_GAIN              71 // from 0 to 2, 1 means a gain of 1.0
#define LUCAM_PROP_DIGITAL_GAIN_RED          72 // from 0 to 2.5, 1 means a gain of 1.0. Relates to GAIN_Y and WHITEBALANCE
#define LUCAM_PROP_DIGITAL_GAIN_GREEN        73 // from 0 to 2.5, 1 means a gain of 1.0. Relates to GAIN_Y and WHITEBALANCE
#define LUCAM_PROP_DIGITAL_GAIN_BLUE         74 // from 0 to 2.5, 1 means a gain of 1.0. Relates to GAIN_Y and WHITEBALANCE

#define LUCAM_PROP_COLOR_FORMAT              80 // (read only)
#define LUCAM_PROP_MAX_WIDTH                 81 // (read only)
#define LUCAM_PROP_MAX_HEIGHT                82 // (read only)

#define LUCAM_PROP_ABS_FOCUS                 85 // requires the auto lens to be initialized
#define LUCAM_PROP_BLACK_LEVEL               86

#define LUCAM_PROP_KNEE1_EXPOSURE            96
#define LUCAM_PROP_STILL_KNEE1_EXPOSURE      96
#define LUCAM_PROP_KNEE2_EXPOSURE            97
#define LUCAM_PROP_STILL_KNEE2_EXPOSURE      97
#define LUCAM_PROP_STILL_KNEE3_EXPOSURE      98
#define LUCAM_PROP_VIDEO_KNEE                99
#define LUCAM_PROP_KNEE1_LEVEL               99
#define LUCAM_PROP_THRESHOLD                 101
#define LUCAM_PROP_AUTO_EXP_TARGET           103
#define LUCAM_PROP_TIMESTAMPS                105
#define LUCAM_PROP_SNAPSHOT_CLOCK_SPEED      106 // 0 is the fastest
#define LUCAM_PROP_AUTO_EXP_MAXIMUM          107
#define LUCAM_PROP_TEMPERATURE               108
#define LUCAM_PROP_TRIGGER                   110
#define LUCAM_PROP_FRAME_GATE                112
#define LUCAM_PROP_EXPOSURE_INTERVAL         113
#define LUCAM_PROP_PWM                       114
#define LUCAM_PROP_MEMORY                    115 // value is RO and represent # of frames in memory
#define LUCAM_PROP_STILL_STROBE_DURATION     116
#define LUCAM_PROP_FAN                       118
#define LUCAM_PROP_SYNC_MODE                 119
#define LUCAM_PROP_SNAPSHOT_COUNT            120
#define LUCAM_PROP_LSC_X                     121
#define LUCAM_PROP_LSC_Y                     122
#define LUCAM_PROP_AUTO_IRIS_MAX             123
#define LUCAM_PROP_LENS_STABILIZATION        124
#define LUCAM_PROP_VIDEO_TRIGGER             125
#define LUCAM_PROP_KNEE2_LEVEL               163
#define LUCAM_PROP_THRESHOLD_LOW             165
#define LUCAM_PROP_THRESHOLD_HIGH            166
#define LUCAM_PROP_TEMPERATURE2              167
#define LUCAM_PROP_LIGHT_FREQUENCY           168
#define LUCAM_PROP_LUMINANCE                 169
#define LUCAM_PROP_AUTO_GAIN_MAXIMUM         170
#define LUCAM_PROP_AUTO_SHARPNESS_GAIN_THRESHOLD_LOW  171
#define LUCAM_PROP_AUTO_SHARPNESS_GAIN_THRESHOLD_HIGH 172
#define LUCAM_PROP_AUTO_SHARPNESS_LOW        173
#define LUCAM_PROP_AUTO_SHARPNESS_HIGH       174

#define LUCAM_PROP_JPEG_QUALITY              256


#define LUCAM_PROP_FLAG_USE                  0x80000000
#define LUCAM_PROP_FLAG_AUTO                 0x40000000
#define LUCAM_PROP_FLAG_MASTER               0x40000000 // for LUCAM_PROP_SYNC_MODE
#define LUCAM_PROP_FLAG_STROBE_FROM_START_OF_EXPOSURE   0x20000000
#define LUCAM_PROP_FLAG_BACKLASH_COMPENSATION           0x20000000 // LUCAM_PROP_IRIS and LUCAM_PROP_FOCUS
#define LUCAM_PROP_FLAG_USE_FOR_SNAPSHOTS    0x04000000 // For LUCAM_PROP_IRIS
#define LUCAM_PROP_FLAG_POLARITY             0x10000000
#define LUCAM_PROP_FLAG_MEMORY_READBACK      0x08000000 // for LUCAM_PROP_MEMORY
#define LUCAM_PROP_FLAG_BUSY                 0x00040000
#define LUCAM_PROP_FLAG_UNKNOWN_MAXIMUM      0x00020000
#define LUCAM_PROP_FLAG_UNKNOWN_MINIMUM      0x00010000
#define LUCAM_PROP_FLAG_LITTLE_ENDIAN        0x80000000 // for LUCAM_PROP_COLOR_FORMAT
#define LUCAM_PROP_FLAG_ALTERNATE            0x00080000 
#define LUCAM_PROP_FLAG_READONLY             0x00010000 // in caps param of GetPropertyRange

// Prop flags for VIDEO_TRIGGER (also uses LUCAM_PROP_FLAG_USE)
#define LUCAM_PROP_FLAG_HW_ENABLE            0x40000000
#define LUCAM_PROP_FLAG_SW_TRIGGER           0x00200000 // self cleared


// Those flags can be used with the LUCAM_PROP_GAMMA / LUCAM_PROP_BRIGHTNESS / LUCAM_PROP_CONTRAST
// properties. They are available on specifica cameras only.
#define LUCAM_PROP_FLAG_RED                  0x00000001
#define LUCAM_PROP_FLAG_GREEN1               0x00000002
#define LUCAM_PROP_FLAG_GREEN2               0x00000004
#define LUCAM_PROP_FLAG_BLUE                 0x00000008


// Do not access these properties unless you know what you are doing.

#define LUCAM_PROP_STILL_EXPOSURE            50
#define LUCAM_PROP_STILL_GAIN                51
#define LUCAM_PROP_STILL_GAIN_RED            52
#define LUCAM_PROP_STILL_GAIN_GREEN1         53
#define LUCAM_PROP_STILL_GAIN_GREEN2         54
#define LUCAM_PROP_STILL_GAIN_BLUE           55
#define LUCAM_PROP_STILL_GAIN_MAGENTA        52
#define LUCAM_PROP_STILL_GAIN_YELLOW1        53
#define LUCAM_PROP_STILL_GAIN_YELLOW2        54
#define LUCAM_PROP_STILL_GAIN_CYAN           55


// color pattern for the LUCAM_PROP_COLOR_FORMAT property
#define LUCAM_CF_MONO                        0
#define LUCAM_CF_BAYER_RGGB                  8
#define LUCAM_CF_BAYER_GRBG                  9
#define LUCAM_CF_BAYER_GBRG                  10
#define LUCAM_CF_BAYER_BGGR                  11
#define LUCAM_CF_BAYER_CYYM                  16
#define LUCAM_CF_BAYER_YCMY                  17
#define LUCAM_CF_BAYER_YMCY                  18
#define LUCAM_CF_BAYER_MYYC                  19

// parameter for the LUCAM_PROP_FLIPPING property
#define LUCAM_PROP_FLIPPING_NONE             0
#define LUCAM_PROP_FLIPPING_X                1
#define LUCAM_PROP_FLIPPING_Y                2
#define LUCAM_PROP_FLIPPING_XY               3



//----------- Version Structure ------------------

typedef struct {
		ULONG firmware; 	// Firmware version
		ULONG fpga; 		// FPGA version
		ULONG api;			// API version
		ULONG driver; 		// Device driver version
      ULONG serialnumber; // of the camera
      ULONG reserved;   // do not use
} LUCAM_VERSION;


//------------- Frame format --------------------

typedef struct {
   ULONG xOffset; 	// x coordinate on imager of top left corner of subwindow in pixels
   ULONG yOffset; 	// y coordinate on imager of top left corner of subwindow in pixels
   ULONG width; 	// width in pixels of subwindow
   ULONG height; 	// height in pixels of subwindow
   ULONG pixelFormat; // pixel format for data
   union
   {
      USHORT subSampleX;	// sub-sample ratio in x direction in pixels (x:1)
      USHORT binningX;	// binning ratio in x direction in pixels (x:1)
   };
   USHORT flagsX; // LUCAM_FRAME_FORMAT_FLAGS_*
   union
   {
      USHORT subSampleY;  // sub-sample ratio in y direction in pixels (y:1)
      USHORT binningY;	// binning ratio in y direction in pixels (y:1)
   };
   USHORT flagsY; // LUCAM_FRAME_FORMAT_FLAGS_*
} LUCAM_FRAME_FORMAT;

#define LUCAM_FRAME_FORMAT_FLAGS_BINNING     0x0001


//------------ Snapshot Settings Structure -------


typedef struct {
		FLOAT exposure;		// Exposure in milliseconds
		FLOAT gain;			// Overall gain as a multiplicative factor
        union {
         struct {
          FLOAT gainRed; 	// Gain for Red pixels as multiplicative factor
          FLOAT gainBlue; 	// Gain for Blue pixels as multiplicative factor
          FLOAT gainGrn1; 	// Gain for Green pixels on Red rows as multiplicative factor
          FLOAT gainGrn2; 	// Gain for Green pixels on Blue rows as multiplicative factor
         };
         struct {  
          FLOAT gainMag; 	// Gain for Magenta pixels as multiplicative factor
          FLOAT gainCyan; 	// Gain for Cyan pixels as multiplicative factor
          FLOAT gainYel1; 	// Gain for Yellow pixels on Magenta rows as multiplicative factor
          FLOAT gainYel2; 	// Gain for Yellow pixels on Cyan rows as multiplicative factor
         };
        };
      union
      {
		   BOOL useStrobe;		// for backward compatibility
         ULONG strobeFlags;   // use LUCAM_PROP_FLAG_USE and/or LUCAM_PROP_FLAG_STROBE_FROM_START_OF_EXPOSURE
      };
		FLOAT strobeDelay;	// time interval from when exposure starts to time the flash is fired in milliseconds
		BOOL useHwTrigger;	// wait for hardware trigger
		FLOAT timeout;		// maximum time to wait for hardware trigger prior to returning from function in milliseconds
      LUCAM_FRAME_FORMAT format;   // frame format for data
      ULONG shutterType;
      FLOAT exposureDelay;
      union
      {
         BOOL bufferlastframe;   // set to TRUE if you want TakeFastFrame to return an already received frame.
         ULONG ulReserved1;
      };
      ULONG ulReserved2;   // must be set to 0
      FLOAT flReserved1;   // must be set to 0
      FLOAT flReserved2;   // must be set to 0
} LUCAM_SNAPSHOT;


//------------ Streaming Video Modes --------------

#define STOP_STREAMING	0
#define START_STREAMING 1
#define START_DISPLAY	2
#define PAUSE_STREAM    3
#define START_RGBSTREAM 6

//------------ Streaming AVI Modes --------------
#define STOP_AVI 0
#define START_AVI 1
#define PAUSE_AVI 2

//------------ Parameters for AVI types --------------
#define AVI_RAW_LUMENERA	0
#define AVI_STANDARD_24		1
#define AVI_STANDARD_32		2
#define AVI_XVID_24			3
#define AVI_STANDARD_8     4 // for monochrome only


//---------- Parameters for LucamConvertFrameToRgb24 --------------

typedef struct
{
    ULONG DemosaicMethod;
    ULONG CorrectionMatrix;
    
}LUCAM_CONVERSION;

// to use with LUCAM_CONVERSION.DemosaicMethod
#define LUCAM_DM_NONE                   0
#define LUCAM_DM_FAST                   1
#define LUCAM_DM_HIGH_QUALITY           2
#define LUCAM_DM_HIGHER_QUALITY         3
#define LUCAM_DM_SIMPLE                 8


// to use with LUCAM_CONVERSION.CorrectionMatrix
#define LUCAM_CM_NONE                   0
#define LUCAM_CM_FLUORESCENT            1
#define LUCAM_CM_DAYLIGHT               2
#define LUCAM_CM_INCANDESCENT           3
#define LUCAM_CM_XENON_FLASH            4
#define LUCAM_CM_HALOGEN                5

#define LUCAM_CM_IDENTITY               14
#define LUCAM_CM_CUSTOM                 15


//----------- Shutter types ------------

#define LUCAM_SHUTTER_TYPE_GLOBAL         0
#define LUCAM_SHUTTER_TYPE_ROLLING        1


//----------- Extern interfaces -------

#define LUCAM_EXTERN_INTERFACE_USB1       1
#define LUCAM_EXTERN_INTERFACE_USB2       2



//---------- Functions ----------------

LUCAM_API LONG LUCAM_EXPORT LucamNumCameras(void);
LUCAM_API LONG LUCAM_EXPORT LucamEnumCameras(LUCAM_VERSION *pVersionsArray, ULONG arrayCount);

LUCAM_API HANDLE LUCAM_EXPORT LucamCameraOpen(ULONG index);
LUCAM_API BOOL LUCAM_EXPORT LucamCameraClose(HANDLE hCamera);
LUCAM_API BOOL LUCAM_EXPORT LucamCameraReset(HANDLE hCamera);

LUCAM_API BOOL LUCAM_EXPORT LucamQueryVersion(HANDLE hCamera, LUCAM_VERSION *pVersion);
LUCAM_API BOOL LUCAM_EXPORT LucamQueryExternInterface(HANDLE hCamera, ULONG *pExternInterface);
LUCAM_API BOOL LUCAM_EXPORT LucamGetCameraId(HANDLE hCamera, ULONG *pId);

LUCAM_API BOOL LUCAM_EXPORT LucamGetProperty(HANDLE hCamera, ULONG property, FLOAT *pValue, LONG *pFlags);
LUCAM_API BOOL LUCAM_EXPORT LucamSetProperty(HANDLE hCamera, ULONG property, FLOAT value, LONG flags);
LUCAM_API BOOL LUCAM_EXPORT LucamPropertyRange(HANDLE hCamera, ULONG property, FLOAT *pMin, FLOAT *pMax, FLOAT *pDefault, LONG *pFlags);

LUCAM_API BOOL LUCAM_EXPORT LucamDisplayPropertyPage(HANDLE hCamera, HWND parentWnd);
LUCAM_API BOOL LUCAM_EXPORT LucamDisplayVideoFormatPage(HANDLE hCamera, HWND parentWnd);

LUCAM_API BOOL LUCAM_EXPORT LucamQueryDisplayFrameRate(HANDLE hCamera, FLOAT *pValue);

LUCAM_API BOOL LUCAM_EXPORT LucamCreateDisplayWindow(HANDLE hCamera, LPCTSTR lpTitle = NULL, DWORD dwStyle = WS_OVERLAPPED|WS_MINIMIZEBOX|WS_CAPTION|WS_SYSMENU|WS_VISIBLE, int x = 0, int y = 0, int width = 0, int height = 0, HWND parent = NULL, HMENU childId = NULL);
LUCAM_API BOOL LUCAM_EXPORT LucamDestroyDisplayWindow(HANDLE hCamera);
LUCAM_API BOOL LUCAM_EXPORT LucamAdjustDisplayWindow(HANDLE hCamera, LPCTSTR lpTitle = NULL, int x = 0, int y = 0, int width = 0, int height = 0);

LUCAM_API BOOL LUCAM_EXPORT LucamReadRegister(HANDLE hCamera, LONG address, LONG numReg, LONG *pValue);
LUCAM_API BOOL LUCAM_EXPORT LucamWriteRegister(HANDLE hCamera, LONG address, LONG numReg, LONG *pValue);

LUCAM_API BOOL LUCAM_EXPORT LucamSetFormat(HANDLE hCamera, LUCAM_FRAME_FORMAT *pFormat, FLOAT frameRate);
LUCAM_API BOOL LUCAM_EXPORT LucamGetFormat(HANDLE hCamera, LUCAM_FRAME_FORMAT *pFormat, FLOAT *pFrameRate);

LUCAM_API ULONG LUCAM_EXPORT LucamEnumAvailableFrameRates(HANDLE hCamera, ULONG entryCount, FLOAT *pAvailableFrameRates);

LUCAM_API BOOL LUCAM_EXPORT LucamStreamVideoControl (HANDLE hCamera, ULONG controlType, HWND hWnd);
LUCAM_API BOOL LUCAM_EXPORT LucamStreamVideoControlAVI (HANDLE hCamera, ULONG controlType, LPCWSTR pFileName, HWND hWnd);

LUCAM_API BOOL LUCAM_EXPORT LucamTakeVideo(HANDLE hCamera, LONG numFrames, BYTE *pData);
LUCAM_API BOOL LUCAM_EXPORT LucamTakeVideoEx(HANDLE hCamera, BYTE *pData, ULONG *pLength, ULONG timeout);
LUCAM_API BOOL LUCAM_EXPORT LucamCancelTakeVideo(HANDLE hCamera);

LUCAM_API BOOL LUCAM_EXPORT LucamTakeSnapshot(HANDLE hCamera, LUCAM_SNAPSHOT *pSettings, BYTE *pData);

LUCAM_API LUCAM_DEPRECATED BOOL LUCAM_EXPORT LucamSaveImage(ULONG width, ULONG height, ULONG pixelFormat, BYTE *pData, const CHAR *pFilename);
LUCAM_API BOOL LUCAM_EXPORT LucamSaveImageEx(HANDLE hCamera, ULONG width, ULONG height, ULONG pixelFormat, BYTE *pData, const CHAR *pFilename);
LUCAM_API LUCAM_DEPRECATED BOOL LUCAM_EXPORT LucamSaveImageW(ULONG width, ULONG height, ULONG pixelFormat, BYTE *pData, const WCHAR *pFilename);
LUCAM_API BOOL LUCAM_EXPORT LucamSaveImageWEx(HANDLE hCamera, ULONG width, ULONG height, ULONG userPixelFormat, BYTE *pData, const WCHAR *pWFilename);

LUCAM_API LONG LUCAM_EXPORT LucamAddStreamingCallback(HANDLE hCamera, VOID (__stdcall *VideoFilter)(VOID *pContext, BYTE *pData, ULONG dataLength), VOID *pCBContext);
LUCAM_API BOOL LUCAM_EXPORT LucamRemoveStreamingCallback(HANDLE hCamera, LONG callbackId);

LUCAM_API LONG LUCAM_EXPORT LucamAddRgbPreviewCallback(HANDLE hCamera, VOID (__stdcall *RgbVideoFilter)(VOID *pContext, BYTE *pData, ULONG dataLength, ULONG unused), VOID *pContext, ULONG rgbPixelFormat);
LUCAM_API BOOL LUCAM_EXPORT LucamRemoveRgbPreviewCallback(HANDLE hCamera, LONG callbackId);
LUCAM_API BOOL LUCAM_EXPORT LucamQueryRgbPreviewPixelFormat(HANDLE hCamera, ULONG *pRgbPixelFormat);

LUCAM_API LONG LUCAM_EXPORT LucamAddSnapshotCallback(HANDLE hCamera, VOID (__stdcall *SnapshotCallback)(VOID *pContext, BYTE *pData, ULONG dataLength), VOID *pCBContext);
LUCAM_API BOOL LUCAM_EXPORT LucamRemoveSnapshotCallback(HANDLE hCamera, LONG callbackId);

LUCAM_API BOOL LUCAM_EXPORT LucamConvertFrameToRgb24(HANDLE hCamera, BYTE *pDest, BYTE *pSrc, ULONG width, ULONG height, ULONG pixelFormat, LUCAM_CONVERSION *pParams);
LUCAM_API BOOL LUCAM_EXPORT LucamConvertFrameToRgb32(HANDLE hCamera, BYTE *pDest, BYTE *pSrc, ULONG width, ULONG height, ULONG pixelFormat, LUCAM_CONVERSION *pParams);
LUCAM_API BOOL LUCAM_EXPORT LucamConvertFrameToRgb48(HANDLE hCamera, USHORT *pDest, USHORT *pSrc, ULONG width, ULONG height, ULONG userPixelFormat, LUCAM_CONVERSION *pParams);
LUCAM_API BOOL LUCAM_EXPORT LucamConvertFrameToGreyscale8(HANDLE hCamera, BYTE *pDest, BYTE *pSrc, ULONG width, ULONG height, ULONG userPixelFormat, LUCAM_CONVERSION *pParams);
LUCAM_API BOOL LUCAM_EXPORT LucamConvertFrameToGreyscale16(HANDLE hCamera, USHORT *pDest, USHORT *pSrc, ULONG width, ULONG height, ULONG userPixelFormat, LUCAM_CONVERSION *pParams);
LUCAM_API VOID LUCAM_EXPORT LucamConvertBmp24ToRgb24(UCHAR *pFrame, ULONG width, ULONG height);

//This function is used for converting a raw AVI to a standard AVI
LUCAM_API BOOL LUCAM_EXPORT LucamConvertRawAVIToStdVideo(HANDLE hCamera, const WCHAR *pOutputFileName, const WCHAR *pInputFileName, ULONG outputType);

//The 3 next functions are used for previewing raw AVI files
LUCAM_API HANDLE LUCAM_EXPORT LucamPreviewAVIOpen(const WCHAR *pFileName);
LUCAM_API BOOL LUCAM_EXPORT LucamPreviewAVIClose(HANDLE hAVI);
LUCAM_API BOOL LUCAM_EXPORT LucamPreviewAVIControl(HANDLE hAVI, ULONG previewControlType, HWND previewWindow);
LUCAM_API BOOL LUCAM_EXPORT LucamPreviewAVIGetDuration(HANDLE hAVI, LONGLONG *pDurationMinutes, LONGLONG *pDurationSeconds, LONGLONG *pDurationMilliseconds, LONGLONG *pDurationMicroSeconds);
LUCAM_API BOOL LUCAM_EXPORT LucamPreviewAVIGetFrameCount(HANDLE hAVI, LONGLONG *pFrameCount);
LUCAM_API BOOL LUCAM_EXPORT LucamPreviewAVIGetFrameRate(HANDLE hAVI, FLOAT *pFrameRate);
LUCAM_API BOOL LUCAM_EXPORT LucamPreviewAVIGetPositionTime(HANDLE hAVI, LONGLONG *pPositionMinutes, LONGLONG *pPositionSeconds, LONGLONG *pPositionMilliSeconds, LONGLONG *pPositionMicroSeconds);
LUCAM_API BOOL LUCAM_EXPORT LucamPreviewAVIGetPositionFrame(HANDLE hAVI, LONGLONG *pPositionCurrentFrame);
LUCAM_API BOOL LUCAM_EXPORT LucamPreviewAVISetPositionTime(HANDLE hAVI, LONGLONG pPositionMinutes, LONGLONG pPositionSeconds, LONGLONG pPositionMilliSeconds, LONGLONG pPositionMicroSeconds);
LUCAM_API BOOL LUCAM_EXPORT LucamPreviewAVISetPositionFrame(HANDLE hAVI, LONGLONG pPositionFrame);
LUCAM_API BOOL LUCAM_EXPORT LucamPreviewAVIGetFormat(HANDLE hAVI, LONG *width, LONG *height, LONG *fileType, LONG *bitDepth);


LUCAM_API BOOL LUCAM_EXPORT LucamSetupCustomMatrix(HANDLE hCamera, FLOAT *pMatrix);
LUCAM_API BOOL LUCAM_EXPORT LucamGetCurrentMatrix(HANDLE hCamera, FLOAT *pMatrix);

LUCAM_API BOOL LUCAM_EXPORT LucamEnableFastFrames(HANDLE hCamera, LUCAM_SNAPSHOT *pSettings);
LUCAM_API BOOL LUCAM_EXPORT LucamTakeFastFrame(HANDLE hCamera, BYTE *pData);
LUCAM_API BOOL LUCAM_EXPORT LucamForceTakeFastFrame(HANDLE hCamera, BYTE *pData);
LUCAM_API BOOL LUCAM_EXPORT LucamTakeFastFrameNoTrigger(HANDLE hCamera, BYTE *pData);
LUCAM_API BOOL LUCAM_EXPORT LucamDisableFastFrames(HANDLE hCamera);
LUCAM_API BOOL LUCAM_EXPORT LucamSetTriggerMode(HANDLE hCamera, BOOL useHwTrigger);
LUCAM_API BOOL LUCAM_EXPORT LucamTriggerFastFrame(HANDLE hCamera);
LUCAM_API BOOL LUCAM_EXPORT LucamCancelTakeFastFrame(HANDLE hCamera);

LUCAM_API HANDLE LUCAM_EXPORT LucamEnableSynchronousSnapshots(ULONG numberOfCameras, HANDLE *phCameras, LUCAM_SNAPSHOT **ppSettings);
LUCAM_API BOOL LUCAM_EXPORT LucamTakeSynchronousSnapshots(HANDLE syncSnapsHandle, BYTE **ppBuffers);
LUCAM_API BOOL LUCAM_EXPORT LucamDisableSynchronousSnapshots(HANDLE syncSnapsHandle);

LUCAM_API BOOL LUCAM_EXPORT LucamGpioRead(HANDLE hCamera, BYTE *pGpoValues, BYTE *pGpiValues);
LUCAM_API BOOL LUCAM_EXPORT LucamGpioWrite(HANDLE hCamera, BYTE gpoValues);
LUCAM_API BOOL LUCAM_EXPORT LucamGpoSelect(HANDLE hCamera, BYTE gpoEnable); // Selects between GPO output or alternate function
LUCAM_API BOOL LUCAM_EXPORT LucamGpioConfigure(HANDLE hCamera, BYTE enableOutput); // Enables output drive on a pin.

LUCAM_API BOOL LUCAM_EXPORT LucamOneShotAutoExposure(HANDLE hCamera, UCHAR target, ULONG startX, ULONG startY, ULONG width, ULONG height);
LUCAM_API BOOL LUCAM_EXPORT LucamOneShotAutoWhiteBalance(HANDLE hCamera, ULONG startX, ULONG startY, ULONG width, ULONG height);
LUCAM_API BOOL LUCAM_EXPORT LucamOneShotAutoWhiteBalanceEx(HANDLE hCamera, float redOverGreen, float blueOverGreen, ULONG startX, ULONG startY, ULONG width, ULONG height);
LUCAM_API BOOL LUCAM_EXPORT LucamDigitalWhiteBalance(HANDLE hCamera, ULONG startX, ULONG startY, ULONG width, ULONG height);
LUCAM_API BOOL LUCAM_EXPORT LucamDigitalWhiteBalanceEx(HANDLE hCamera, float redOverGreen, float blueOverGreen, ULONG startX, ULONG startY, ULONG width, ULONG height);
LUCAM_API BOOL LUCAM_EXPORT LucamAdjustWhiteBalanceFromSnapshot(HANDLE hCamera, LUCAM_SNAPSHOT *pSettings, BYTE *pData, float redOverGreen, float blueOverGreen, ULONG startX, ULONG startY, ULONG width, ULONG height);
LUCAM_API BOOL LUCAM_EXPORT LucamOneShotAutoIris(HANDLE hCamera, UCHAR target, ULONG startX, ULONG startY, ULONG width, ULONG height);
LUCAM_API BOOL LUCAM_EXPORT LucamContinuousAutoExposureEnable(HANDLE hCamera, UCHAR target, ULONG startX, ULONG startY, ULONG width, ULONG height, FLOAT lightingPeriod);
LUCAM_API BOOL LUCAM_EXPORT LucamContinuousAutoExposureDisable(HANDLE hCamera);

LUCAM_API BOOL LUCAM_EXPORT LucamAutoFocusStart(HANDLE hCamera, ULONG startX, ULONG startY, ULONG width, ULONG height, FLOAT putZeroThere1, FLOAT putZeroThere2, FLOAT putZeroThere3, BOOL (__stdcall * ProgressCallback)(void *context, FLOAT percentageCompleted), void *contextForCallback);
LUCAM_API BOOL LUCAM_EXPORT LucamAutoFocusWait(HANDLE hCamera, DWORD timeout);
LUCAM_API BOOL LUCAM_EXPORT LucamAutoFocusStop(HANDLE hCamera);
LUCAM_API BOOL LUCAM_EXPORT LucamAutoFocusQueryProgress(HANDLE hCamera, FLOAT *pPercentageCompleted);
LUCAM_API BOOL LUCAM_EXPORT LucamInitAutoLens(HANDLE hCamera, BOOL force);

LUCAM_API BOOL LUCAM_EXPORT LucamSetup8bitsLUT(HANDLE hCamera, UCHAR *pLut, ULONG length);   // length must be 0 or 256
LUCAM_API BOOL LUCAM_EXPORT LucamSetup8bitsColorLUT(HANDLE hCamera, UCHAR *pLut, ULONG length, BOOL applyOnRed, BOOL applyOnGreen1, BOOL applyOnGreen2 , BOOL applyOnBlue);   // length must be 0 or 256

LUCAM_API int LUCAM_EXPORT LucamRs232Transmit(HANDLE hCamera, char *pData, int length);
LUCAM_API int LUCAM_EXPORT LucamRs232Receive(HANDLE hCamera, char *pData, int maxLength);
LUCAM_API BOOL LUCAM_EXPORT LucamAddRs232Callback(HANDLE hCamera, void (__stdcall * callback)(void *), void *context);
LUCAM_API VOID LUCAM_EXPORT LucamRemoveRs232Callback(HANDLE hCamera);

LUCAM_API BOOL LUCAM_EXPORT LucamPermanentBufferRead(HANDLE hCamera, UCHAR *pBuf, ULONG offset, ULONG length);
LUCAM_API BOOL LUCAM_EXPORT LucamPermanentBufferWrite(HANDLE hCamera, UCHAR *pBuf, ULONG offset, ULONG length);

LUCAM_API BOOL LUCAM_EXPORT LucamGetTruePixelDepth(HANDLE hCamera, ULONG *pCount);

LUCAM_API BOOL LUCAM_EXPORT LucamSetTimeout(HANDLE hCamera, BOOL still, FLOAT timeout);

LUCAM_API ULONG LUCAM_EXPORT LucamGetLastError(void);
LUCAM_API ULONG LUCAM_EXPORT LucamGetLastErrorForCamera(HANDLE hCamera);

// New Structure used for the new conversion functions

typedef struct _LUCAM_CONVERSION_PARAMS
{
   ULONG Size;  // of this structure
   ULONG DemosaicMethod;
   ULONG CorrectionMatrix;
   BOOL FlipX;
   BOOL FlipY;
   FLOAT Hue;
   FLOAT Saturation;
   BOOL UseColorGainsOverWb;
   union
   {
      struct
      {
         FLOAT DigitalGain;
         FLOAT DigitalWhiteBalanceU;
         FLOAT DigitalWhiteBalanceV;
      };
      struct
      {
         FLOAT DigitalGainRed;
         FLOAT DigitalGainGreen;
         FLOAT DigitalGainBlue;
      };
   };
}LUCAM_CONVERSION_PARAMS, *PLUCAM_CONVERSION_PARAMS;


typedef struct _LUCAM_IMAGE_FORMAT
{
   ULONG Size; // of this structure
   ULONG Width;
   ULONG Height;
   ULONG PixelFormat;
   ULONG ImageSize;

   ULONG LucamReserved[8];

}LUCAM_IMAGE_FORMAT, *PLUCAM_IMAGE_FORMAT;

LUCAM_API BOOL LUCAM_EXPORT LucamGetVideoImageFormat(HANDLE hCamera, LUCAM_IMAGE_FORMAT *pImageFormat);
LUCAM_API BOOL LUCAM_EXPORT LucamGetStillImageFormat(HANDLE hCamera, LUCAM_IMAGE_FORMAT *pImageFormat);

LUCAM_API BOOL LUCAM_EXPORT LucamConvertFrameToRgb24Ex(HANDLE hCamera, BYTE *pDest, const BYTE *pSrc, const LUCAM_IMAGE_FORMAT *pImageFormat, const LUCAM_CONVERSION_PARAMS *pParams);
LUCAM_API BOOL LUCAM_EXPORT LucamConvertFrameToRgb32Ex(HANDLE hCamera, BYTE *pDest, const BYTE *pSrc, const LUCAM_IMAGE_FORMAT *pImageFormat, const LUCAM_CONVERSION_PARAMS *pParams);
LUCAM_API BOOL LUCAM_EXPORT LucamConvertFrameToRgb48Ex(HANDLE hCamera, USHORT *pDest, const USHORT *pSrc, const LUCAM_IMAGE_FORMAT *pImageFormat, const LUCAM_CONVERSION_PARAMS *pParams);
LUCAM_API BOOL LUCAM_EXPORT LucamConvertFrameToGreyscale8Ex(HANDLE hCamera, BYTE *pDest, const BYTE *pSrc, LUCAM_IMAGE_FORMAT *pImageFormat, LUCAM_CONVERSION_PARAMS *pParams);
LUCAM_API BOOL LUCAM_EXPORT LucamConvertFrameToGreyscale16Ex(HANDLE hCamera, USHORT *pDest, const USHORT *pSrc, LUCAM_IMAGE_FORMAT *pImageFormat, LUCAM_CONVERSION_PARAMS *pParams);


LUCAM_API PVOID LUCAM_EXPORT LucamRegisterEventNotification(HANDLE hCamera, DWORD eventId, HANDLE hEvent);
LUCAM_API BOOL LUCAM_EXPORT LucamUnregisterEventNotification(HANDLE hCamera, PVOID pEventInformation);

// For use with LucamRegisterEventNotification
#define LUCAM_EVENT_START_OF_READOUT         2
#define LUCAM_EVENT_GPI1_CHANGED             4
#define LUCAM_EVENT_GPI2_CHANGED             5
#define LUCAM_EVENT_GPI3_CHANGED             6
#define LUCAM_EVENT_GPI4_CHANGED             7
#define LUCAM_EVENT_DEVICE_SURPRISE_REMOVAL  32


LUCAM_API BOOL LUCAM_EXPORT LucamPerformDualTapCorrection(HANDLE hCamera, BYTE *pFrame, const LUCAM_IMAGE_FORMAT *pImageFormat);
LUCAM_API BOOL LUCAM_EXPORT LucamPerformMonoGridCorrection(HANDLE hCamera, BYTE *pFrame, const LUCAM_IMAGE_FORMAT *pImageFormat);

LUCAM_API BOOL LUCAM_EXPORT LucamGetImageIntensity(HANDLE hCamera, BYTE *pFrame, LUCAM_IMAGE_FORMAT *pImageFormat , ULONG startX, ULONG startY, ULONG width, ULONG height, FLOAT *pIntensity, FLOAT *pRedIntensity, FLOAT *pGreen1Intensity, FLOAT *pGreen2Intensity, FLOAT *pBlueIntensity);

LUCAM_API BOOL LUCAM_EXPORT LucamAutoRoiGet(HANDLE hCamera, LONG *pStartX, LONG *pStartY, LONG *pWidth, LONG *pHeight);
LUCAM_API BOOL LUCAM_EXPORT LucamAutoRoiSet(HANDLE hCamera, LONG startX, LONG startY, LONG width, LONG height);


#endif // __LUCAMAPI_H

