#include "apriltag_detector.h"

#include <camera/NdkCameraManager.h>
#include <media/NdkImageReader.h>
#include <stdlib.h>
#include <media/NdkImage.h>
#include <camera/NdkCameraCaptureSession.h>
#include <memory>
#include <vector>
#include <utility>
#include "logger.h"
#include "common.h"
#include <frc/apriltag/AprilTagDetector.h>

using UniqueCameraIdListPtr = std::unique_ptr<ACameraIdList, decltype(&ACameraManager_deleteCameraIdList)>;
using UniqueCameraDevicePtr = std::unique_ptr<ACameraDevice, decltype(&ACameraDevice_close)>;
using UniqueImageReaderPtr = std::unique_ptr<AImageReader, decltype(&AImageReader_delete)>;
using UniqueCameraOutputTargetPtr = std::unique_ptr<ACameraOutputTarget, decltype(&ACameraOutputTarget_free)>;
using UniqueCaptureSessionOutputPtr = std::unique_ptr<ACaptureSessionOutput, decltype(&ACaptureSessionOutput_free)>;
using UniqueCaptureSessionOutputContainer = std::unique_ptr<ACaptureSessionOutputContainer, decltype(&ACaptureSessionOutputContainer_free)>;
using UniqueCameraCaptureSession = std::unique_ptr<ACameraCaptureSession, decltype(&ACameraCaptureSession_close)>;
using UniqueCaptureRequest = std::unique_ptr<ACaptureRequest, decltype(&ACaptureRequest_free)>;

// Define a callback for camera device state changes
void onDisconnected(void* context, ACameraDevice* device) {
    context;
    device;
    // Camera opened–store or manage the camera device handle as needed.
}

void onCameraDeviceError(void* context, ACameraDevice* device, int error) {
    context;
    device;
    error;
    // Handle errors accordingly.
}

void onImageAvailable(void* context, AImageReader* reader) {
    context;
    AImage* image = nullptr;
    if (AImageReader_acquireNextImage(reader, &image) == AMEDIA_OK) {
        // Process the image. You might convert or directly access the raw buffers.
        // Once done, release the image:
        AImage_delete(image);
    }
}

// Define session callbacks
void onCaptureSessionConfigured(void* context, ACameraCaptureSession *session) {
    context;
    session;
    // Ready to issue capture requests.
}
void onCaptureSessionActive(void* context, ACameraCaptureSession *session) {
    context;
    session;
    // Handle session configuration failure.
}

// This callback is called when an image capture has fully completed and all the
// result metadata is available.
// For repeating requests, this is called for each frame captured.
void onCaptureCompleted(void* context, ACameraCaptureSession* session,
                        ACaptureRequest* request, const ACameraMetadata* result) {
    context;
    session;
    request;
    result;
    // MyCameraContext* cameraContext = static_cast<MyCameraContext*>(context);
    // (void)cameraContext; // Mark as used if not directly used in simple log

    // Process the result metadata if needed.
    // For example, you might check AE_STATE, AF_STATE, AWB_STATE, timestamp, etc.
    // ACameraMetadata_const_entry entry;
    // if (ACameraMetadata_getConstEntry(result, ACAMERA_SENSOR_TIMESTAMP, &entry) == ACAMERA_OK) {
    //     int64_t timestamp = entry.data.i64[0];
    //     __android_log_print(ANDROID_LOG_INFO, APP_NAME, "Capture completed, Timestamp: %lld", timestamp);
    // } else {
    //     __android_log_print(ANDROID_LOG_INFO, APP_NAME, "Capture completed.");
    // }

    // If you are managing ACaptureRequest objects specifically for each capture,
    // you might want to release or reuse the 'request' pointer here,
    // though for setRepeatingRequest, the NDK often manages the lifecycle internally
    // for the request provided.
}

// This callback is called instead of onCaptureCompleted when the camera device
// failed to produce a capture result for the request.
void onCaptureFailed(void* context, ACameraCaptureSession* session,
                     ACaptureRequest* request, ACameraCaptureFailure* failure) {
    context;
    session;
    request;
    // MyCameraContext* cameraContext = static_cast<MyCameraContext*>(context);
    // (void)cameraContext;

    // Handle the capture failure.
    // `failure` contains information about the reason, frame number, etc.
    Log::Write(Log::Level::Error,
                        Fmt("Capture failed. Frame: %lld, Reason: %d, SequenceId: %d",
                        failure->frameNumber, failure->reason, failure->sequenceId));

    // If `failure->wasImageCaptured` is true, an image buffer might still have been
    // produced and sent to its destination ANativeWindow, even if metadata was lost.
}

// This callback is called when a capture sequence has finished and all
// onCaptureCompleted/onCaptureFailed callbacks have been called.
// For a repeating request, this is typically called when the repeating request
// is stopped (e.g., by calling ACameraCaptureSession_stopRepeating).
void onCaptureSequenceCompleted(void* context, ACameraCaptureSession* session,
                                int sequenceId, int64_t frameNumber) {
    context;
    session;
    // MyCameraContext* cameraContext = static_cast<MyCameraContext*>(context);
    // (void)cameraContext;

    Log::Write(Log::Level::Info,
                        Fmt("Capture sequence completed. SequenceId: %d, Last Frame: %lld",
                        sequenceId, frameNumber));
}

// This callback is called when a capture sequence has aborted before completion.
// For a repeating request, this might be called if the session is abruptly closed.
void onCaptureSequenceAborted(void* context, ACameraCaptureSession* session,
                              int sequenceId) {
    context;
    session;
    // MyCameraContext* cameraContext = static_cast<MyCameraContext*>(context);
    // (void)cameraContext;

    Log::Write(Log::Level::Warning,
                        Fmt("Capture sequence aborted. SequenceId: %d", sequenceId));
}

// This callback is called if a single buffer for a capture could not be sent
// to its destination ANativeWindow.
// This is less common for typical preview scenarios but can happen.
void onCaptureBufferLost(void* context, ACameraCaptureSession* session,
                         ACaptureRequest* request, ANativeWindow* window,
                         int64_t frameNumber) {
    context;
    session;
    request;
    // MyCameraContext* cameraContext = static_cast<MyCameraContext*>(context);
    // (void)cameraContext;

    Log::Write(Log::Level::Warning,
                        Fmt("Capture buffer lost. Frame: %lld, for window: %p",
                        frameNumber, window));
}

void DetectAprilTag() {
    frc::AprilTagDetector detector;
    uint8_t image[100*100] = {0};
    auto results = detector.Detect(100, 100, image);
    if (!results.empty()) {
        // Very surprised
    }
}

AprilTagDetector::AprilTagDetector() {
    // Create the camera manager
    ACameraManager *cameraManager = ACameraManager_create();

    // Get list of available camera IDs
    UniqueCameraIdListPtr cameraIdList(nullptr , ACameraManager_deleteCameraIdList);
    {
        ACameraIdList *idList = nullptr;
        camera_status_t result = ACameraManager_getCameraIdList(cameraManager, &idList);
        cameraIdList.reset(idList);

        if (cameraIdList->numCameras < 2) {
            Log::Write(Log::Level::Error,
                       Fmt("ACameraManager_getCameraIdList returned fewer than two cameras, with result %d",
                           result));
            return;
        }
    }
    
    Log::Write(Log::Level::Info, Fmt("ACameraManager_getCameraIdList returned %d cameras", cameraIdList->numCameras));
    for (int i = 0; i < cameraIdList->numCameras; i++) {
        Log::Write(Log::Level::Info, Fmt("Camera %d: %s", i, cameraIdList->cameraIds[i]));
    }

    // For demonstration, use the second camera
    const char *cameraId = cameraIdList->cameraIds[1];

    ACameraDevice_stateCallbacks deviceCallbacks = {
            .onDisconnected = onDisconnected,
            .onError = onCameraDeviceError
    };

    // Need to have both horizonos.permission.HEADSET_CAMERA and android.permission.CAMERA
    UniqueCameraDevicePtr cameraDevicePtr(nullptr, ACameraDevice_close);
    {
        ACameraDevice* cameraDevice = nullptr;
        camera_status_t result = ACameraManager_openCamera(cameraManager, cameraId, &deviceCallbacks, &cameraDevice);
        if (result != ACAMERA_OK) {
            Log::Write(Log::Level::Error,
                       Fmt("ACameraManager_openCamera failed to open camera %s, with result %d",
                           cameraId,
                           result));
            return;
        }
        cameraDevicePtr.reset(cameraDevice);
    }
    
    UniqueImageReaderPtr imageReaderPtr(nullptr, AImageReader_delete);
    {
        // Set the desired image dimensions and format
        int width = 1920;
        int height = 1080;
        int format = AIMAGE_FORMAT_YUV_420_888;  // Change as needed for RAW data if available.
        int maxImages = 4;  // Number of images to be queued

        AImageReader* imageReader = nullptr;
        media_status_t result = AImageReader_new(width, height, format, maxImages, &imageReader);
        if (result != AMEDIA_OK) {
            // Handle error creating the image reader.
            Log::Write(Log::Level::Error,
                       Fmt("AImageReader_new failed to create image reader, with result %d", result));
            return;
        }
        imageReaderPtr.reset(imageReader);
    }

    // Obtain the native window from the image reader to use as an output target
    // This window is managed by the image reader and will be deleted when the image reader is destroyed
    ANativeWindow* window = nullptr;
    {
        media_status_t result = AImageReader_getWindow(imageReaderPtr.get(), &window);
        if (result != AMEDIA_OK) {
            // Handle error getting the window.
            Log::Write(Log::Level::Error,
                       Fmt("AImageReader_getWindow failed to get window, with result %d", result));
            return;
        }
    }

    AImageReader_ImageListener imageListener = { .context = nullptr, .onImageAvailable = onImageAvailable };
    {
        media_status_t result = AImageReader_setImageListener(imageReaderPtr.get(), &imageListener);
        if (result != AMEDIA_OK) {
            // Handle error setting the image listener.
            Log::Write(Log::Level::Error,
                       Fmt("AImageReader_setImageListener failed to set image listener, with result %d", result));
            return;
        }
    }

    // Build the list of output surfaces
    UniqueCameraOutputTargetPtr outputTargetPtr(nullptr, ACameraOutputTarget_free);
    {
        ACameraOutputTarget *outputTarget = nullptr;
        camera_status_t result = ACameraOutputTarget_create(window, &outputTarget);
        if (result != ACAMERA_OK) {
            Log::Write(Log::Level::Error,
                       Fmt("ACameraOutputTarget_create failed to create output target, with result %d", result));
            return;
        }
        outputTargetPtr.reset(outputTarget);
    }

    UniqueCaptureSessionOutputPtr captureSessionOutputPtr(nullptr, ACaptureSessionOutput_free);
    {
        ACaptureSessionOutput *sessionOutput = nullptr;
        // 'window' is your ANativeWindow* obtained from AImageReader_getWindow or another source.
        camera_status_t status = ACaptureSessionOutput_create(window, &sessionOutput);
        if (status != ACAMERA_OK) {
            Log::Write(Log::Level::Error,
                       Fmt("ACaptureSessionOutput_create failed to create session output, with result %d", status));
            return;
        }
        captureSessionOutputPtr.reset(sessionOutput);
    }
    
    // Set up the capture session output configuration (typically one or more targets)
    UniqueCaptureSessionOutputContainer outputContainerPtr(nullptr, ACaptureSessionOutputContainer_free);
    {
        ACaptureSessionOutputContainer *outputs = nullptr;
        camera_status_t result = ACaptureSessionOutputContainer_create(&outputs);
        if (result != ACAMERA_OK) {
            Log::Write(Log::Level::Error,
                       Fmt("ACaptureSessionOutputContainer_create failed to create output container, with result %d", result));
            return;
        }
        outputContainerPtr.reset(outputs);
    }

    {
        camera_status_t result = ACaptureSessionOutputContainer_add(outputContainerPtr.get(), captureSessionOutputPtr.get());
        if (result != ACAMERA_OK) {
            Log::Write(Log::Level::Error,
                       Fmt("ACaptureSessionOutputContainer_add failed to add output to container, with result %d", result));
            return;
        }
    }

    ACameraCaptureSession_stateCallbacks sessionCallbacks = {
            .onClosed = nullptr,
            .onReady = onCaptureSessionConfigured,
            .onActive = onCaptureSessionActive
    };

    UniqueCameraCaptureSession cameraCaptureSessionPtr(nullptr, ACameraCaptureSession_close);
    {
        ACameraCaptureSession* captureSession = nullptr;
        camera_status_t result = ACameraDevice_createCaptureSession(cameraDevicePtr.get(), outputContainerPtr.get(),
                                               &sessionCallbacks, &captureSession);
        if (result != ACAMERA_OK) {
            Log::Write(Log::Level::Error,
                       Fmt("ACameraDevice_createCaptureSession failed to create capture session, with result %d", result));
            return;
        }
        cameraCaptureSessionPtr.reset(captureSession);
    }

// Now create a capture request for repeating capture (preview)
    ACameraCaptureSession_captureCallbacks captureCallbacks = {
            .onCaptureCompleted = onCaptureCompleted,
            .onCaptureFailed = onCaptureFailed,
            .onCaptureSequenceCompleted = onCaptureSequenceCompleted,
            .onCaptureSequenceAborted = onCaptureSequenceAborted,
            .onCaptureBufferLost = onCaptureBufferLost
    };

    UniqueCaptureRequest captureRequestPtr(nullptr, ACaptureRequest_free);
    {
        // TODO: Update TEMPLATE_PREVIEW to the appropriate request template and set camera exposure parameters, etc.
        ACaptureRequest* captureRequest = nullptr;
        camera_status_t result = ACameraDevice_createCaptureRequest(cameraDevicePtr.get(), TEMPLATE_PREVIEW,
                                                                    &captureRequest);
        if (result != ACAMERA_OK) {
            Log::Write(Log::Level::Error,
                       Fmt("ACameraDevice_createCaptureRequest failed to create capture request, with result %d", result));
            return;
        }
        captureRequestPtr.reset(captureRequest);
    }

    {
        camera_status_t result = ACaptureRequest_addTarget(captureRequestPtr.get(), outputTargetPtr.get());
        if (result != ACAMERA_OK) {
            Log::Write(Log::Level::Error,
                       Fmt("ACaptureRequest_addTarget failed to add output target to capture request, with result %d", result));
            return;
        }
    }

    // Start the repeating request
    {
        int sequenceId = 0; // To store the ID of this repeating request sequence
        ACaptureRequest* captureRequest = captureRequestPtr.get();
        camera_status_t result = ACameraCaptureSession_setRepeatingRequest(cameraCaptureSessionPtr.get(),
                                                                                 &captureCallbacks,
                                                                                 1, &captureRequest,
                                                                                 &sequenceId);
        if (result == ACAMERA_OK) {
            Log::Write(Log::Level::Info,
                       Fmt("Set repeating request successfully. Sequence ID: %d", sequenceId));
        } else {
            Log::Write(Log::Level::Error,
                       Fmt("Failed to set repeating request. Status: %d", result));
        }
    }
}

AprilTagDetector::~AprilTagDetector() {}
