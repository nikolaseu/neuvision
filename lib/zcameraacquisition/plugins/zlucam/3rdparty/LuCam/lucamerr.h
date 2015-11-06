/*****************************************************************************
*
* Copyright (c) Lumenera Corporation 2002-2008. All rights reserved.
*
*****************************************************************************/
#ifndef __LUCAMERR_H
#define __LUCAMERR_H

//-----------------------------------------------------------------------------
// Id: LucamNoError
//
// Meaning:
// Initialization value in the API.
//
#define LucamNoError                      0

//-----------------------------------------------------------------------------
// Id: LucamNoSuchIndex
//
// Meaning:
// The index passed to LucamCameraOpen was 0. It must be >= 1.
//
#define LucamNoSuchIndex                  1

//-----------------------------------------------------------------------------
// Id: LucamSnapshotNotSupported
//
// Meaning:
// The camera does not support snapshots or fast frames.
//
#define LucamSnapshotNotSupported         2

//-----------------------------------------------------------------------------
// Id: LucamInvalidPixelFormat
//
// Meaning:
// The pixel format parameter passed to the function is invalid
//
#define LucamInvalidPixelFormat           3

//-----------------------------------------------------------------------------
// Id: LucamSubsamplingZero
//
// Meaning:
// A subsampling of zero was passed to a function.
//
#define LucamSubsamplingZero              4

//-----------------------------------------------------------------------------
// Id: LucamBusy
//
// Meaning:
// The function is unavailable because the camera is busy with streaming or
// capturing fast frames.
//
#define LucamBusy                         5

//-----------------------------------------------------------------------------
// Id: LucamFailedToSetSubsampling
//
// Meaning:
// The API failed to set the requested subsampling. This can be due to
// the camera being disconnected. It can also be due to a filter
// not being there.
//
#define LucamFailedToSetSubsampling       6

//-----------------------------------------------------------------------------
// Id: LucamFailedToSetStartPosition
//
// Meaning:
// The API failed to set the requested subsampling. This can be due to
// the camera being disconnected.
//
#define LucamFailedToSetStartPosition     7

//-----------------------------------------------------------------------------
// Id: LucamPixelFormatNotSupported
//
// Meaning:
// The camera does not support the pixel format passed to the function.
//
#define LucamPixelFormatNotSupported      8

//-----------------------------------------------------------------------------
// Id: LucamInvalidFrameFormat
//
// Meaning:
// The format passed to the function does not pass the camera requirements.
// Verify that (xOffset + width) is not greater than the camera's maximum
// width. Verify that (width / subSamplingX) is a multiple of some 'nice'
// value. Do the same for the y.
//
#define LucamInvalidFrameFormat           9

//-----------------------------------------------------------------------------
// Id: LucamPreparationFailed
//
// Meaning:
// The API failed to prepare the camera for streaming or snapshot. This can
// due to the camera being disconnected. If START_STREAMING succeeds and 
// START_DISPLAY fails with this error, this can be due to a filter not
// being there or registered.
//
#define LucamPreparationFailed            10

//-----------------------------------------------------------------------------
// Id: LucamCannotRun
//
// Meaning:
// The API failed to get the camera running. This can be due to a bandwidth
// problem.
//
#define LucamCannotRun                    11

//-----------------------------------------------------------------------------
// Id: LucamNoTriggerControl
//
// Meaning:
// Contact Lumenera.
//
#define LucamNoTriggerControl             12

//-----------------------------------------------------------------------------
// Id: LucamNoPin
//
// Meaning:
// Contact Lumenera.
//
#define LucamNoPin                        13

//-----------------------------------------------------------------------------
// Id: LucamNotRunning
//
// Meaning:
// The function failed because it requires the camera to be running, and it 
// is not.
//
#define LucamNotRunning                   14

//-----------------------------------------------------------------------------
// Id: LucamTriggerFailed
//
// Meaning:
// Contact Lumenera.
//
#define LucamTriggerFailed                15

//-----------------------------------------------------------------------------
// Id: LucamCannotSetupFrameFormat
//
// Meaning:
// The camera does not support that its frame format get setup. This will
// happen if your camera is plugged to a USB 1 port and it does not
// support it. LucamCameraOpen will still succeeds, but if a call to
// LucamGetLastError will return this value.
//
#define LucamCannotSetupFrameFormat       16

//-----------------------------------------------------------------------------
// Id: LucamDirectShowInitError
//
// Meaning:
// Direct Show initialization error. This may happen if you run the API
// before having installed a DS compatible camera ever before. The
// Lumenera camera is DS compatible.
//
#define LucamDirectShowInitError          17

//-----------------------------------------------------------------------------
// Id: LucamCameraNotFound
//
// Meaning:
// The function LucamCameraOpen did not find the camera # index.
//
#define LucamCameraNotFound               18

//-----------------------------------------------------------------------------
// Id: LucamTimeout
//
// Meaning:
// The function timed out.
//
#define LucamTimeout                      19

//-----------------------------------------------------------------------------
// Id: LucamPropertyUnknown
//
// Meaning:
// The API does not know the property passed to LucamGetProperty, 
// LucamSetProperty or LucamGetPropertyRange. You may be using an old dll.
//
#define LucamPropertyUnknown              20

//-----------------------------------------------------------------------------
// Id: LucamPropertyUnsupported
//
// Meaning:
// The camera does not support that property. 
//
#define LucamPropertyUnsupported          21

//-----------------------------------------------------------------------------
// Id: LucamPropertyAccessFailed
//
// Meaning:
// The API failed to access the property. Most likely, the reason is that the
// camera does not support that property.
//
#define LucamPropertyAccessFailed         22

//-----------------------------------------------------------------------------
// Id: LucamLucustomNotFound
//
// Meaning:
// The lucustom.ax filter was not found.
//
#define LucamLucustomNotFound             23

//-----------------------------------------------------------------------------
// Id: LucamPreviewNotRunning
//
// Meaning:
// The function failed because preview is not running.
//
#define LucamPreviewNotRunning            24

//-----------------------------------------------------------------------------
// Id: LucamLutfNotLoaded
//
// Meaning:
// The function failed because lutf.ax is not loaded.
//
#define LucamLutfNotLoaded                25

//-----------------------------------------------------------------------------
// Id: LucamDirectShowError
//
// Meaning:
// An error related to the operation of DirectShow occured.
//
#define LucamDirectShowError              26

//-----------------------------------------------------------------------------
// Id: LucamNoMoreCallbacks
//
// Meaning:
// The function LucamAddStreamingCallback failed because the API cannot 
// support any more callbacks
//
#define LucamNoMoreCallbacks              27

//-----------------------------------------------------------------------------
// Id: LucamUndeterminedFrameFormat
//
// Meaning:
// The API does not know what is the frame format of the camera.
//
#define LucamUndeterminedFrameFormat      28

//-----------------------------------------------------------------------------
// Id: LucamInvalidParameter
//
// Meaning:
// An parameter has an obviously wrong value.
//
#define LucamInvalidParameter             29

//-----------------------------------------------------------------------------
// Id: LucamNotEnoughResources
//
// Meaning:
// Resource allocation failed.
//
#define LucamNotEnoughResources           30

//-----------------------------------------------------------------------------
// Id: LucamNoSuchConversion
//
// Meaning:
// One of the members of the LUCAM_CONVERSION structure passed is either
// unknown or inappropriate
//
#define LucamNoSuchConversion             31

//-----------------------------------------------------------------------------
// Id: LucamParameterNotWithinBoundaries
//
// Meaning:
// A parameter representing a quantity is outside the allowed boundaries.
//
#define LucamParameterNotWithinBoundaries 32

//-----------------------------------------------------------------------------
// Id: LucamBadFileIo
//
// Meaning:
// An error occured creating a file or writing to it. Verify that the
// path exists.
//
#define LucamBadFileIo                    33

//-----------------------------------------------------------------------------
// Id: LucamGdiplusNotFound
//
// Meaning:
// gdiplus.dll is needed and was not found.
//
#define LucamGdiplusNotFound              34

//-----------------------------------------------------------------------------
// Id: LucamGdiplusError
//
// Meaning:
// gdiplus.dll reported an error. This may happen if there is a file io error.
//
#define LucamGdiplusError                 35

//-----------------------------------------------------------------------------
// Id: LucamUnknownFormatType
//
// Meaning:
// Contact Lumenera.
//
#define LucamUnknownFormatType            36

//-----------------------------------------------------------------------------
// Id: LucamFailedCreateDisplay
//
// Meaning:
// The API failed to create the display window. The reason could be
// unsufficient resources.
//
#define LucamFailedCreateDisplay          37

//-----------------------------------------------------------------------------
// Id: LucamDpLibNotFound
//
// Meaning:
// deltapolation.dll is needed and was not found.
//
#define LucamDpLibNotFound                38

//-----------------------------------------------------------------------------
// Id: LucamDpCmdNotSupported
//
// Meaning:
// The deltapolation command is not supported by the delta polation library.
//
#define LucamDpCmdNotSupported            39

//-----------------------------------------------------------------------------
// Id: LucamDpCmdUnknown
//
// Meaning:
// The deltapolation command is unknown or invalid.
//
#define LucamDpCmdUnknown                 40

//-----------------------------------------------------------------------------
// Id: LucamNotWhilePaused
//
// Meaning:
// The function cannot be performed when the camera is in paused state.
//
#define LucamNotWhilePaused               41

//-----------------------------------------------------------------------------
// Id: LucamCaptureFailed
//
// Meaning:
// Contact Lumenera.
//
#define LucamCaptureFailed                42

//-----------------------------------------------------------------------------
// Id: LucamDpError
//
// Meaning:
// Contact Lumenera.
//
#define LucamDpError                      43

//-----------------------------------------------------------------------------
// Id: LucamNoSuchFrameRate
//
// Meaning:
// Contact Lumenera.
//
#define LucamNoSuchFrameRate              44

//-----------------------------------------------------------------------------
// Id: LucamInvalidTarget
//
// Meaning:
// One of the target parameters is wrong. This error code is used when
// startX + width > (frameFormat.width / frameFormat.subSampleX) or
// startY + height > (frameFormat.height / frameFormat.subSampleY) or
// if any of those parameter is odd (not a multiple of 2) or
// if width or height is 0.
//
#define LucamInvalidTarget                45

//-----------------------------------------------------------------------------
// Id: LucamFrameTooDark
//
// Meaning:
// The frame is too dark to perform white balance.
//
#define LucamFrameTooDark                 46

//-----------------------------------------------------------------------------
// Id: LucamKsPropertySetNotFound
//
// Meaning:
// A DirectShow interface necessary to carry out the operation was not found.
//
#define LucamKsPropertySetNotFound        47

//-----------------------------------------------------------------------------
// Id: LucamCancelled
//
// Meaning:
// The user cancelled the operation.
//
#define LucamCancelled                    48

//-----------------------------------------------------------------------------
// Id: LucamKsControlNotSupported
//
// Meaning:
// The DirectShow IKsControl interface is not supported (did you unplugged the camera?).
//
#define LucamKsControlNotSupported        49

//-----------------------------------------------------------------------------
// Id: LucamEventNotSupported
//
// Meaning:
// Some module attempted to register an unsupported event.
//
#define LucamEventNotSupported            50

//-----------------------------------------------------------------------------
// Id: LucamNoPreview
//
// Meaning:
// The function failed because preview was not setup.
//
#define LucamNoPreview                    51

//-----------------------------------------------------------------------------
// Id: LucamSetPositionFailed
//
// Meaning:
// A function setting window position failed (invalid parameters??).
//
#define LucamSetPositionFailed            52

//-----------------------------------------------------------------------------
// Id: LucamNoFrameRateList
//
// Meaning:
// The frame rate list is not available.
//
#define LucamNoFrameRateList              53

//-----------------------------------------------------------------------------
// Id: LucamFrameRateInconsistent
//
// Meaning:
// There was an error building the frame rate list.
//
#define LucamFrameRateInconsistent        54

//-----------------------------------------------------------------------------
// Id: LucamCameraNotConfiguredForCmd
//
// Meaning:
// The camera does not support that particular command.
//
#define LucamCameraNotConfiguredForCmd    55

//----------------------------------------------------------------------------
// Id: LucamGraphNotReady
//
// Meaning:
// The graph is not ready.
//
#define LucamGraphNotReady                56

//----------------------------------------------------------------------------
// Id: LucamCallbackSetupError
//
// Meaning:
// Contact Lumenera.
//
#define LucamCallbackSetupError           57

//----------------------------------------------------------------------------
// Id: LucamInvalidTriggerMode
//
// Meaning:
// You cannot cause a soft trigger when hw trigger is enabled.
//
#define LucamInvalidTriggerMode           58

//----------------------------------------------------------------------------
// Id: LucamNotFound
//
// Meaning:
// The API was asked to return soomething that is not there.
//
#define LucamNotFound                     59

//----------------------------------------------------------------------------
// Id: LucamEepromTooSmall
//
// Meaning:
// The onboard EEPROM is too small.
//
#define LucamPermanentBufferNotSupported  60

//----------------------------------------------------------------------------
// Id: LucamEepromWriteFailed
//
// Meaning:
// The API failed to write to the onboard eeprom.
//
#define LucamEepromWriteFailed            61

//----------------------------------------------------------------------------
// Id: LucamUnknownFileType
//
// Meaning:
// The API failed because it failed to recognize the file type of a 
// file name passed to it..
//
#define LucamUnknownFileType              62

//----------------------------------------------------------------------------
// Id: LucamEventIdNotSupported
//
// Meaning:
// LucamRegisterEventNotification failed because the event is not supported.
//
#define LucamEventIdNotSupported          63

//----------------------------------------------------------------------------
// Id: LucamEepromCorrupted
//
// Meaning:
// The API found that the EEPROM was corrupted.
//
#define LucamEepromCorrupted              64

//----------------------------------------------------------------------------
// Id: LucamSectionTooBig
//
// Meaning:
// The VPD section to write to the eeprom is too big.
//
#define LucamSectionTooBig                65

//-----------------------------------------------------------------------------
// Id: LucamFrameTooBright
//
// Meaning:
// The frame is too bright to perform white balance.
//
#define LucamFrameTooBright               66

//-----------------------------------------------------------------------------
// Id: LucamNoCorrectionMatrix
//
// Meaning:
// The camera is configured to have no correction matrix (LUCAM_PROP_CORRECTION_MATRIX 
// is LUCAM_CM_NONE).
//
#define LucamNoCorrectionMatrix           67

//-----------------------------------------------------------------------------
// Id: LucamUnknownCameraModel
//
// Meaning:
// The API failed because it needs to know the camera model and it is not available. 
//
#define LucamUnknownCameraModel           68

//-----------------------------------------------------------------------------
// Id: LucamApiTooOld
//
// Meaning:
// The API failed because it needs to be upgraded to access a feature of the camera.
//
#define LucamApiTooOld                    69

//-----------------------------------------------------------------------------
// Id: LucamSaturationZero
//
// Meaning:
// The API failed because the saturation is currently 0.
//
#define LucamSaturationZero					70

//-----------------------------------------------------------------------------
// Id: LucamAlreadyInitialised
//
// Meaning:
// The API failed because the object was already initialised.
//
#define LucamAlreadyInitialised				71

//-----------------------------------------------------------------------------
// Id: LucamSameInputAndOutputFile
//
// Meaning:
// The API failed because the object was already initialised.
//
#define LucamSameInputAndOutputFile			72

//-----------------------------------------------------------------------------
// Id: LucamFileConversionFailed
//
// Meaning:
// The API failed because the file conversion was not completed.
//
#define LucamFileConversionFailed			73

//-----------------------------------------------------------------------------
// Id: LucamFileAlreadyConverted
//
// Meaning:
// The API failed because the file is already converted in the desired format.
//
#define LucamFileAlreadyConverted			74

//-----------------------------------------------------------------------------
// Id: LucamPropertyPageNotSupported
//
// Meaning:
// The API failed to display the property page.
//
#define LucamPropertyPageNotSupported     75

//-----------------------------------------------------------------------------
// Id: LucamPropertyPageCreationFailed
//
// Meaning:
// The API failed to create the property page.
//
#define LucamPropertyPageCreationFailed   76

//-----------------------------------------------------------------------------
// Id: LucamDirectShowFilterNotInstalled
//
// Meaning:
// The API did not find the required direct show filter.
//
#define LucamDirectShowFilterNotInstalled 77

//-----------------------------------------------------------------------------
// Id: LucamIndividualLutNotAvailable
//
// Meaning:
// The camera does not support that different LUTs are applied to each color.
//
#define LucamIndividualLutNotAvailable    78

//-----------------------------------------------------------------------------
// Id: LucamUnexpectedError
//
// Meaning:
// Contact Lumenera.
//
#define LucamUnexpectedError              79

//-----------------------------------------------------------------------------
// Id: LucamStreamingStopped
//
// Meaning:
// LucamTakeFastFrame or LucamTakeVideo failed because another thread interrupted
// the streaming by a call to LucamDisableFastFrames or LucamStreamVideoControl.
//
#define LucamStreamingStopped             80

//-----------------------------------------------------------------------------
// Id: LucamMustBeInSwTriggerMode
//
// Meaning:
// LucamForceTakeFastFrame was called while the camera is in hardware trigger
// still mode and the camera does not support taking a sw trigger snapshot while
// in that state.
//
#define LucamMustBeInSwTriggerMode        81

//-----------------------------------------------------------------------------
// Id: LucamTargetFlaky
//
// Meaning:
// The target is too flaky to perform auto focus.
//
#define LucamTargetFlaky                  82

//-----------------------------------------------------------------------------
// Id: LucamAutoLensUninitialized
//
// Meaning:
// The auto lens needs to be initialized before the function is used.
//
#define LucamAutoLensUninitialized        83

//-----------------------------------------------------------------------------
// Id: LucamLensNotInstalled
//
// Meaning:
// The function failed because the lens were not installed correctly. Verify
// that changing the focus has any effect.
//
#define LucamLensNotInstalled             84

//-----------------------------------------------------------------------------
// Id: LucamUnknownError
//
// Meaning:
// The function failed because of an unknoen error. Contact Lumenera.
//
#define LucamUnknownError                 85

//-----------------------------------------------------------------------------
// Id: LucamFocusNoFeedbackError
//
// Meaning:
// There is no feedback available for focus.
//
#define LucamFocusNoFeedbackError         86

//-----------------------------------------------------------------------------
// Id: LucamLutfTooOld
//
// Meaning:
// LuTF.ax is too old for this feature.
//
#define LucamLutfTooOld                   87

//-----------------------------------------------------------------------------
// Id: LucamUnknownAviFormat
//
// Meaning:
// Unknown or invalid AVI format for input file.
//
#define LucamUnknownAviFormat             88

//-----------------------------------------------------------------------------
// Id: LucamUnknownAviType
//
// Meaning:
// Unknown AVI type. Verify the AVI type parameter.
//
#define LucamUnknownAviType               89

//-----------------------------------------------------------------------------
// Id: LucamInvalidAviConversion
//
// Meaning:
// The AVI conversion is invalid.
//
#define LucamInvalidAviConversion         90

//-----------------------------------------------------------------------------
// Id: LucamSeekFailed
//
// Meaning:
// The seeking operation failed.
//
#define LucamSeekFailed                   91

//-----------------------------------------------------------------------------
// Id: LucamAviRunning
//
// Meaning:
// The function cannot be performed while an AVI is being captured.
//
#define LucamAviRunning                   92

//-----------------------------------------------------------------------------
// Id: LucamCameraAlreadyOpened
//
// Meaning:
// An attempt was made to open a camera for streaming-related reasons while 
// it is already opened for such.
//
#define LucamCameraAlreadyOpened          93

//-----------------------------------------------------------------------------
// Id: LucamNoSubsampledHighRes
//
// Meaning:
// The API cannot take a high resolution image in subsampled mode or binning mode.
//
#define LucamNoSubsampledHighRes          94

//-----------------------------------------------------------------------------
// Id: LucamOnlyOnMonochrome
//
// Meaning:
// The API function is only available on monochrome cameras.
//
#define LucamOnlyOnMonochrome             95

//-----------------------------------------------------------------------------
// Id: LucamNo8bppTo48bpp
//
// Meaning:
// Building a 48 bpp image from an 8 bpp image is invalid.
//
#define LucamNo8bppTo48bpp                96

//-----------------------------------------------------------------------------
// Id: LucamLut8Obsolete
//
// Meaning:
// Use 12 bits lut instead.
//
#define LucamLut8Obsolete                 97

//-----------------------------------------------------------------------------
// Id: LucamFunctionNotSupported
//
// Meaning:
// That functionnality is not supported.
//
#define LucamFunctionNotSupported         98

//-----------------------------------------------------------------------------
// Id: LucamRetryLimitReached
//
// Meaning:
// Property access failed due to a retry limit.
//
#define LucamRetryLimitReached            99


#endif // __LUCAMERR_H

