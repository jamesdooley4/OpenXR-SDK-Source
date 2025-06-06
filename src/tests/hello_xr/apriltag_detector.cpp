#include "apriltag_detector.h"

#include <camera/NdkCameraManager.h>
#include <media/NdkImageReader.h>
#include <stdlib.h>
#include <media/NdkImage.h>
#include <camera/NdkCameraCaptureSession.h>
#include <vector>
#include "logger.h"
#include "common.h"
#include <frc/apriltag/AprilTagDetector.h>

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
    ACameraIdList *idList = nullptr;
    ACameraManager_getCameraIdList(cameraManager, &idList);

    if (idList->numCameras < 1) {
        // Handle the error: No cameras available.
    }

    // For demonstration, use the first camera
    const char *cameraId = idList->cameraIds[0];

    ACameraDevice_stateCallbacks deviceCallbacks = {
            .onDisconnected = onDisconnected,
            .onError = onCameraDeviceError
    };

    ACameraDevice* cameraDevice = nullptr;
    if (ACameraManager_openCamera(cameraManager, cameraId, &deviceCallbacks, &cameraDevice) != ACAMERA_OK) {
        // Handle error in opening the camera.
    }
    
    // When done, free the camera list later:
    ACameraManager_deleteCameraIdList(idList);

    // Set the desired image dimensions and format
    int width = 1920;
    int height = 1080;
    int format = AIMAGE_FORMAT_YUV_420_888;  // Change as needed for RAW data if available.
    int maxImages = 4;  // Number of images to be queued

    AImageReader* imageReader = nullptr;
    if (AImageReader_new(width, height, format, maxImages, &imageReader) != AMEDIA_OK) {
        // Handle error creating the image reader.
    }

// Obtain the native window from the image reader to use as an output target
    ANativeWindow* window = nullptr;
    AImageReader_getWindow(imageReader, &window);

    AImageReader_ImageListener imageListener = { .context = nullptr, .onImageAvailable = onImageAvailable };
    AImageReader_setImageListener(imageReader, &imageListener);

    // Build the list of output surfaces
    ACameraOutputTarget* outputTarget = nullptr;
    ACameraOutputTarget_create(window, &outputTarget);

    // 'window' is your ANativeWindow* obtained from AImageReader_getWindow or another source.
    ACaptureSessionOutput* sessionOutput = nullptr;
    camera_status_t status = ACaptureSessionOutput_create(window, &sessionOutput);
    if (status != ACAMERA_OK) {
        // Handle the error appropriately (e.g., logging and cleanup)
    }
    // Set up the capture session output configuration (typically one or more targets)
    ACaptureSessionOutputContainer* outputs = nullptr;
    ACaptureSessionOutputContainer_create(&outputs);
    ACaptureSessionOutputContainer_add(outputs, sessionOutput);

    ACameraCaptureSession_stateCallbacks sessionCallbacks = {
            .onClosed = nullptr,
            .onReady = onCaptureSessionConfigured,
            .onActive = onCaptureSessionActive
    };

    ACameraCaptureSession* captureSession = nullptr;
    if (ACameraDevice_createCaptureSession(cameraDevice, outputs, &sessionCallbacks, &captureSession) != ACAMERA_OK) {
        // Handle error in creating the session.
    }

// Now create a capture request for repeating capture (preview)
    ACameraCaptureSession_captureCallbacks captureCallbacks = {
            .onCaptureCompleted = onCaptureCompleted,
            .onCaptureFailed = onCaptureFailed,
            .onCaptureSequenceCompleted = onCaptureSequenceCompleted,
            .onCaptureSequenceAborted = onCaptureSequenceAborted,
            .onCaptureBufferLost = onCaptureBufferLost
    };
    
    ACaptureRequest* captureRequest = nullptr;
    ACameraDevice_createCaptureRequest(cameraDevice, TEMPLATE_PREVIEW, &captureRequest);
    ACaptureRequest_addTarget(captureRequest, outputTarget);

    // Start the repeating request
    int sequenceId = 0; // To store the ID of this repeating request sequence
    camera_status_t cameraStatus = ACameraCaptureSession_setRepeatingRequest(captureSession, &captureCallbacks, 1, &captureRequest, &sequenceId);
    if (status == ACAMERA_OK) {
        Log::Write(Log::Level::Info,
                   Fmt("Set repeating request successfully. Sequence ID: %d", sequenceId));
    } else {
        Log::Write(Log::Level::Error,
           Fmt("Failed to set repeating request. Status: %d", status));
    }
}

AprilTagDetector::~AprilTagDetector() {}
