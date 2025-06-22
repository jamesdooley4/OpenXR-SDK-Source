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
#include <cassert> // For assert

using UniqueCameraManagerPtr = std::unique_ptr<ACameraManager, decltype(&ACameraManager_delete)>;
using UniqueCameraIdListPtr = std::unique_ptr<ACameraIdList, decltype(&ACameraManager_deleteCameraIdList)>;
using UniqueCameraMetadataPtr = std::unique_ptr<ACameraMetadata, decltype(&ACameraMetadata_free)>;
using UniqueCameraDevicePtr = std::unique_ptr<ACameraDevice, decltype(&ACameraDevice_close)>;
using UniqueImageReaderPtr = std::unique_ptr<AImageReader, decltype(&AImageReader_delete)>;
using UniqueCameraOutputTargetPtr = std::unique_ptr<ACameraOutputTarget, decltype(&ACameraOutputTarget_free)>;
using UniqueCaptureSessionOutputPtr = std::unique_ptr<ACaptureSessionOutput, decltype(&ACaptureSessionOutput_free)>;
using UniqueCaptureSessionOutputContainer = std::unique_ptr<ACaptureSessionOutputContainer, decltype(&ACaptureSessionOutputContainer_free)>;
using UniqueCameraCaptureSession = std::unique_ptr<ACameraCaptureSession, decltype(&ACameraCaptureSession_close)>;
using UniqueCaptureRequest = std::unique_ptr<ACaptureRequest, decltype(&ACaptureRequest_free)>;
using namespace rt;

namespace rt {
    class AprilTagDetectorImp;
}

#define LOG_TAG "AprilTag: "

// Define a callback for camera device state changes
void onDisconnected(void* context, ACameraDevice* device) {
    Log::Write(Log::Level::Info,
               Fmt(LOG_TAG "Camera device disconnected. Context: %p, Device: %p", context, device));
    AprilTagDetectorImp* detector = static_cast<AprilTagDetectorImp*>(context);
    if (detector) {
        // Handle disconnection, e.g., flag that the camera is no longer valid
    }
}

void onCameraDeviceError(void* context, ACameraDevice* device, int error) {
    Log::Write(Log::Level::Error,
               Fmt(LOG_TAG "Camera device error. Context: %p, Device: %p, Error: %d", context, device, error));
    AprilTagDetectorImp* detector = static_cast<AprilTagDetectorImp*>(context);
    if (detector) {
        // Handle error
    }
}

// onImageAvailable now needs to be a static member or a free function.
// If it needs to access AprilTagDetectorImp members, it needs the context.
void onImageAvailable(void* context, AImageReader* reader) {
    Log::Write(Log::Level::Verbose,
               Fmt(LOG_TAG "Image available. Context: %p, Reader: %p", context, reader));

    AprilTagDetectorImp* detector = static_cast<AprilTagDetectorImp*>(context);
    if (!detector) {
        Log::Write(Log::Level::Error, LOG_TAG "onImageAvailable: context is null!");
        AImage* image = nullptr;
        // Still try to acquire and delete to clear the queue if reader is valid
        if (AImageReader_acquireNextImage(reader, &image) == AMEDIA_OK) {
            if (image) AImage_delete(image);
        }
        return;
    }

    AImage* image = nullptr;
    media_status_t status = AImageReader_acquireNextImage(reader, &image);
    if (status == AMEDIA_OK && image != nullptr) {
        Log::Write(Log::Level::Info, LOG_TAG "Image acquired successfully!");
        // Process the image. You might convert or directly access the raw buffers.
        // detector->ProcessImage(image); // Example: call a member function

        // TODO: Implement image processing using frc::AprilTagDetector
        // int32_t width, height, format;
        // AImage_getWidth(image, &width);
        // AImage_getHeight(image, &height);
        // AImage_getFormat(image, &format);
        // Log::Write(Log::Level::Verbose, Fmt(LOG_TAG "Image details: %dx%d, format %d", width, height, format));

        // For YUV_420_888, you'd get plane data:
        // AImageCropRect cropRect;
        // AImage_getCropRect(image, &cropRect);
        // int32_t yStride, uStride, vStride;
        // uint8_t *yPixel, *uPixel, *vPixel;
        // int32_t yLen, uLen, vLen;
        // AImage_getPlaneRowStride(image, 0, &yStride);
        // AImage_getPlaneRowStride(image, 1, &uStride);
        // AImage_getPlaneRowStride(image, 2, &vStride);
        // AImage_getPlaneData(image, 0, &yPixel, &yLen);
        // AImage_getPlaneData(image, 1, &uPixel, &uLen);
        // AImage_getPlaneData(image, 2, &vPixel, &vLen);
        // detector->m_aprilTagCppDetector.Detect(width, height, yPixel /* or converted grayscale */);

        AImage_delete(image);
    } else {
        Log::Write(Log::Level::Error, Fmt(LOG_TAG "Failed to acquire image, status: %d", status));
    }
}

// --- Capture Callbacks (static or free functions) ---

void captureStarted(void* context, ACameraCaptureSession* session,
        const ACaptureRequest* request, int64_t timestamp) {
    Log::Write(Log::Level::Info,
               Fmt(LOG_TAG "Capture started. Context: %p, Session: %p, Request: %p, Timestamp: %lld", context, session, request, timestamp));
}

void captureCompleted(void* context, ACameraCaptureSession* session,
                      ACaptureRequest* request, const ACameraMetadata* result) {
    AprilTagDetectorImp* detector = static_cast<AprilTagDetectorImp*>(context);
    // Log::Write(Log::Level::Verbose, Fmt(LOG_TAG "Capture completed. Detector: %p", (void*)detector));
    // Check if context is valid and cast
    if (detector) {
        ACameraMetadata_const_entry entry;
        int64_t timestamp = 987654321;
        if (ACameraMetadata_getConstEntry(result, ACAMERA_SENSOR_TIMESTAMP, &entry) == ACAMERA_OK) {
            timestamp = entry.data.i64[0];
        }
        Log::Write(Log::Level::Verbose,
                   Fmt(LOG_TAG "Capture completed. Context: %p, Session: %p, Request: %p, Result: %p, Timestamp: %lld", context, session, request, result, timestamp));
    } else {
        Log::Write(Log::Level::Warning, LOG_TAG "Capture completed but context is null.");
    }
}

void captureFailed(void* context, ACameraCaptureSession* session,
                   ACaptureRequest* request, ACameraCaptureFailure* failure) {
    Log::Write(Log::Level::Error,
               Fmt(LOG_TAG "Capture failed. Context: %p, Session: %p, Request: %p, Frame: %lld, Reason: %d, SequenceId: %d",
                   context, session, request, failure->frameNumber, failure->reason, failure->sequenceId));

    // If `failure->wasImageCaptured` is true, an image buffer might still have been
    // produced and sent to its destination ANativeWindow, even if metadata was lost.
}

// This callback is called when a capture sequence has finished and all
// onCaptureCompleted/onCaptureFailed callbacks have been called.
// For a repeating request, this is typically called when the repeating request
// is stopped (e.g., by calling ACameraCaptureSession_stopRepeating).
void captureSequenceCompleted(void* context, ACameraCaptureSession* session,
                              int sequenceId, int64_t frameNumber) {
    Log::Write(Log::Level::Info,
               Fmt(LOG_TAG "Capture sequence completed. Context: %p, Session: %p, SequenceId: %d, Last Frame: %lld",
                   context, session, sequenceId, frameNumber));
}

// This callback is called when a capture sequence has aborted before completion.
// For a repeating request, this might be called if the session is abruptly closed.
void captureSequenceAborted(void* context, ACameraCaptureSession* session,
                            int sequenceId) {
    Log::Write(Log::Level::Warning,
               Fmt(LOG_TAG "Capture sequence aborted. Context: %p, Session: %p, SequenceId: %d", context, session, sequenceId));
}

void captureBufferLost(void* context, ACameraCaptureSession* session,
                       ACaptureRequest* request, ANativeWindow* window,
                       int64_t frameNumber) {
    Log::Write(Log::Level::Warning,
               Fmt(LOG_TAG "Capture buffer lost. Context: %p, Session: %p, Request: %p, Frame: %lld, for window: %p",
                   context, session, request, frameNumber, window));
}

void DetectAprilTag() {
    frc::AprilTagDetector detector;
    uint8_t image[100*100] = {0};
    auto results = detector.Detect(100, 100, image);
    if (!results.empty()) {
        // Very surprised
    }
}

namespace rt {
    class AprilTagDetectorImp : public AprilTagDetector {
    public:
        AprilTagDetectorImp() :
                m_cameraManagerPtr(nullptr, ACameraManager_delete),
                m_cameraDevicePtr(nullptr, ACameraDevice_close),
                m_imageReaderPtr(nullptr, AImageReader_delete),
                m_imageReaderWindow(nullptr),
                m_outputTargetPtr(nullptr, ACameraOutputTarget_free),
                m_captureSessionOutputPtr(nullptr, ACaptureSessionOutput_free),
                m_outputContainerPtr(nullptr, ACaptureSessionOutputContainer_free),
                m_cameraCaptureSessionPtr(nullptr, ACameraCaptureSession_close),
                m_captureRequestRepeatingPtr(nullptr, ACaptureRequest_free)
        {
            Log::Write(Log::Level::Info, LOG_TAG "AprilTagDetectorImp constructor");
        }

        virtual ~AprilTagDetectorImp() {
            Log::Write(Log::Level::Info, LOG_TAG "AprilTagDetectorImp destructor. Stopping repeating request and closing session.");
            // Stop repeating requests and close the session before members are destroyed
            if (m_cameraCaptureSessionPtr && m_cameraDevicePtr) { // Check device too, as session depends on it
                ACameraCaptureSession_stopRepeating(m_cameraCaptureSessionPtr.get());
                // ACameraCaptureSession_close is called by unique_ptr deleter
            }
            // m_imageReaderWindow is owned by m_imageReaderPtr, no separate free needed
            // Other unique_ptrs will handle their resources.
        }

        virtual void Initialize();
        // void ProcessImage(AImage* image); // Example if you add image processing method

        // frc::AprilTagDetector m_aprilTagCppDetector; // Your actual detector instance

    private:
        // Static session state callbacks that call member implementations
        static void sessionOnReadyCb(void* context, ACameraCaptureSession *session) {
            Log::Write(Log::Level::Info, Fmt(LOG_TAG "Static: Capture session ready. Context: %p, Session: %p", context, session));
            // Often onReady is the same as onConfigured for simple cases
            if (context) static_cast<AprilTagDetectorImp*>(context)->onCaptureSessionConfiguredImpl(session);
        }
        static void sessionOnActiveCb(void* context, ACameraCaptureSession *session) {
            Log::Write(Log::Level::Info, Fmt(LOG_TAG "Static: Capture session active. Context: %p, Session: %p", context, session));
            // You could start requests here too, but onConfigured/onReady is more common
        }
        static void sessionOnClosedCb(void* context, ACameraCaptureSession *session) {
            Log::Write(Log::Level::Info, Fmt(LOG_TAG "Static: Capture session closed. Context: %p, Session: %p", context, session));
            AprilTagDetectorImp* detector = static_cast<AprilTagDetectorImp*>(context);
            if (detector) {
                // If the session closes unexpectedly, we might want to null out our unique_ptr
                // or re-initialize. However, the unique_ptr's deleter will still try to close.
                // This callback is more for knowing the session *has* closed.
                // If we explicitly closed it, this is expected.
                // If it closed due to an error, we might log it.
                if (detector->m_cameraCaptureSessionPtr.get() == session) {
                    // Our session was closed. The unique_ptr will be reset or already is.
                    Log::Write(Log::Level::Info, LOG_TAG "Our managed capture session has been closed.");
                }
            }
        }

        void onCaptureSessionConfiguredImpl(ACameraCaptureSession *session);
        
        bool openCamera(const char *cameraId);
        bool createImageReader(int32_t width, int32_t height, int32_t format, int32_t maxImages);
        bool createCaptureSession();
        bool createCaptureRequest();

        bool m_initialized = false;
        UniqueCameraManagerPtr m_cameraManagerPtr;
        UniqueCameraDevicePtr m_cameraDevicePtr;
        UniqueImageReaderPtr m_imageReaderPtr;
        ANativeWindow* m_imageReaderWindow; // Store the ANativeWindow to add to request

        UniqueCameraOutputTargetPtr m_outputTargetPtr; // Only one target for AImageReader
        UniqueCaptureSessionOutputPtr m_captureSessionOutputPtr;
        UniqueCaptureSessionOutputContainer m_outputContainerPtr;

        UniqueCameraCaptureSession m_cameraCaptureSessionPtr;
        UniqueCaptureRequest m_captureRequestRepeatingPtr; // For the repeating request
    };
} // namespace rt

bool AprilTagDetectorImp::openCamera(const char *cameraId) {
    ACameraDevice_stateCallbacks deviceCallbacks = {
            .context = this, // Pass this instance as context
            .onDisconnected = onDisconnected,
            .onError = onCameraDeviceError
    };

    {
        ACameraDevice* cameraDevice = nullptr;
        camera_status_t result = ACameraManager_openCamera(m_cameraManagerPtr.get(), cameraId, &deviceCallbacks, &cameraDevice);
        if (result != ACAMERA_OK) {
            Log::Write(Log::Level::Error,
                       Fmt(LOG_TAG "ACameraManager_openCamera failed for camera %s, result %d", cameraId, result));
            return false;
        }
        m_cameraDevicePtr.reset(cameraDevice);
        Log::Write(Log::Level::Info, LOG_TAG "Camera device opened successfully.");
        return true;
    }
}

bool AprilTagDetectorImp::createImageReader(int32_t width, int32_t height, int32_t format, int32_t maxImages) {
    AImageReader* imageReader = nullptr;
    media_status_t result = AImageReader_new(width, height, format, maxImages, &imageReader);
    if (result != AMEDIA_OK) {
        Log::Write(Log::Level::Error,
                   Fmt(LOG_TAG "AImageReader_new failed, result %d", result));
        return false;
    }
    m_imageReaderPtr.reset(imageReader);
    Log::Write(Log::Level::Info, LOG_TAG "AImageReader created.");

    AImageReader_ImageListener imageListener = {
            .context = this, // Pass this AprilTagDetectorImp instance as context
            .onImageAvailable = ::onImageAvailable // Use the global/static callback
    };

    result = AImageReader_setImageListener(m_imageReaderPtr.get(), &imageListener);
    if (result != AMEDIA_OK) {
        Log::Write(Log::Level::Error,
                   Fmt(LOG_TAG "AImageReader_setImageListener failed, result %d", result));
        m_imageReaderPtr.release();
        return false;
    }
    Log::Write(Log::Level::Info, LOG_TAG "AImageReader_setImageListener set.");

    result = AImageReader_getWindow(m_imageReaderPtr.get(), &m_imageReaderWindow);
    if (result != AMEDIA_OK || !m_imageReaderWindow) {
        Log::Write(Log::Level::Error,
                   Fmt(LOG_TAG  "AImageReader_getWindow failed, result %d", result));
        m_imageReaderPtr.release();
        return false;
    }
    Log::Write(Log::Level::Info, LOG_TAG "AImageReader window obtained.");
    return true;
}

bool AprilTagDetectorImp::createCaptureSession() {
    {
        ACaptureSessionOutput* sessionOutput = nullptr;
        camera_status_t status = ACaptureSessionOutput_create(m_imageReaderWindow, &sessionOutput);
        if (status != ACAMERA_OK) {
            Log::Write(Log::Level::Error, Fmt(LOG_TAG "ACaptureSessionOutput_create failed, result %d", status));
            return false;
        }
        m_captureSessionOutputPtr.reset(sessionOutput);
        Log::Write(Log::Level::Info, LOG_TAG "ACaptureSessionOutput created.");
    }

    {
        ACaptureSessionOutputContainer* outputs = nullptr;
        camera_status_t result = ACaptureSessionOutputContainer_create(&outputs);
        if (result != ACAMERA_OK) {
            Log::Write(Log::Level::Error, Fmt(LOG_TAG "ACaptureSessionOutputContainer_create failed, result %d", result));
            return false;
        }
        m_outputContainerPtr.reset(outputs);
        Log::Write(Log::Level::Info, LOG_TAG "ACaptureSessionOutputContainer created.");
    }

    {
        camera_status_t result = ACaptureSessionOutputContainer_add(m_outputContainerPtr.get(), m_captureSessionOutputPtr.get());
        if (result != ACAMERA_OK) {
            Log::Write(Log::Level::Error, Fmt(LOG_TAG "ACaptureSessionOutputContainer_add failed, result %d", result));
            return false;
        }
        Log::Write(Log::Level::Info, LOG_TAG "ACaptureSessionOutput added to container.");
    }

    ACameraCaptureSession_stateCallbacks sessionCallbacks = {
            .context = this, // Pass this AprilTagDetectorImp instance as context
            .onClosed = AprilTagDetectorImp::sessionOnClosedCb, // Static callback
            .onReady = AprilTagDetectorImp::sessionOnReadyCb,   // Static callback
            .onActive = AprilTagDetectorImp::sessionOnActiveCb  // Static callback
    };

    {
        ACameraCaptureSession* captureSession = nullptr;
        camera_status_t result = ACameraDevice_createCaptureSession(m_cameraDevicePtr.get(), m_outputContainerPtr.get(),
                                                                    &sessionCallbacks, &captureSession);
        if (result != ACAMERA_OK) {
            Log::Write(Log::Level::Error,
                       Fmt(LOG_TAG "ACameraDevice_createCaptureSession failed, result %d", result));
            return false;
        }
        m_cameraCaptureSessionPtr.reset(captureSession);
        Log::Write(Log::Level::Info, LOG_TAG "ACameraDevice_createCaptureSession call succeeded. Waiting for onReady callback...");
    }
    return true;
}

bool AprilTagDetectorImp::createCaptureRequest() {
    {
        ACaptureRequest *repeatingRequest = nullptr;
        camera_status_t status = ACameraDevice_createCaptureRequest(m_cameraDevicePtr.get(),
                                                                    TEMPLATE_PREVIEW, // Or TEMPLATE_RECORD for video, TEMPLATE_STILL_CAPTURE for single shots
                                                                    &repeatingRequest);
        if (status != ACAMERA_OK || repeatingRequest == nullptr) {
            Log::Write(Log::Level::Error,
                       Fmt(LOG_TAG "Failed to create capture request, status: %d", status));
            return false;
        }
        m_captureRequestRepeatingPtr.reset(repeatingRequest); // Manage with unique_ptr
        Log::Write(Log::Level::Info, LOG_TAG "Capture request created.");
    }

    // Build the list of output surfaces
    {
        ACameraOutputTarget *outputTarget = nullptr;
        camera_status_t status = ACameraOutputTarget_create(m_imageReaderWindow, &outputTarget);
        if (status != ACAMERA_OK) {
            Log::Write(Log::Level::Error,
                       Fmt(LOG_TAG "ACameraOutputTarget_create failed to create output target, with status %d",
                           status));
            return false;
        }
        m_outputTargetPtr.reset(outputTarget);
        Log::Write(Log::Level::Info, LOG_TAG "Output target created.");
    }

    // Add the AImageReader's window as a target for this request
    {
        camera_status_t status = ACaptureRequest_addTarget(m_captureRequestRepeatingPtr.get(),
                                                           m_outputTargetPtr.get());
        if (status != ACAMERA_OK) {
            Log::Write(Log::Level::Error,
                       Fmt(LOG_TAG "Failed to add target to capture request, status: %d", status));
            return false;
        }
        Log::Write(Log::Level::Info, LOG_TAG "Target added to capture request.");
    }
    return true;
}

void AprilTagDetectorImp::Initialize() {
    Log::Write(Log::Level::Info, LOG_TAG "Initialize start");
    
    if (m_initialized) {
        Log::Write(Log::Level::Warning, LOG_TAG "Initialize called when already initialized.");
        return;
    }

    m_initialized = true;
    // Create the camera manager
    m_cameraManagerPtr.reset(ACameraManager_create());

    // Get list of available camera IDs
    UniqueCameraIdListPtr cameraIdList(nullptr , ACameraManager_deleteCameraIdList);
    {
        ACameraIdList *rawIdList = nullptr;
        camera_status_t result = ACameraManager_getCameraIdList(m_cameraManagerPtr.get(), &rawIdList);
        cameraIdList.reset(rawIdList); // Transfer ownership

        if (result != ACAMERA_OK || !cameraIdList) {
            Log::Write(Log::Level::Error, Fmt(LOG_TAG "ACameraManager_getCameraIdList failed, result %d", result));
            return;
        }
        if (cameraIdList->numCameras < 1) {
            Log::Write(Log::Level::Error, LOG_TAG "No cameras returned");
            return;
        }
    }

    Log::Write(Log::Level::Info, Fmt(LOG_TAG "Found %d cameras", cameraIdList->numCameras));
    // Prefer back camera if available, otherwise first. For headsets, it might be specific.
    const char *selectedCameraId = cameraIdList->cameraIds[0]; // Default to first
    UniqueCameraMetadataPtr selectedCameraMetadataPtr(nullptr, ACameraMetadata_free);
    for (int i = 0; i < cameraIdList->numCameras; ++i) {
        ACameraMetadata* metadata = nullptr;
        ACameraManager_getCameraCharacteristics(m_cameraManagerPtr.get(), cameraIdList->cameraIds[i], &metadata);
        if (metadata) {
            UniqueCameraMetadataPtr metadataPtr(metadata, ACameraMetadata_free);
            ACameraMetadata_const_entry lensFacingEntry;
            if (ACameraMetadata_getConstEntry(metadataPtr.get(), ACAMERA_LENS_FACING, &lensFacingEntry) == ACAMERA_OK) {
                uint8_t lensFacing = lensFacingEntry.data.u8[0];
                Log::Write(Log::Level::Info, Fmt(LOG_TAG "Camera %s, Lens Facing: %d", cameraIdList->cameraIds[i], lensFacing));
                if (lensFacing == ACAMERA_LENS_FACING_BACK) { // Prefer back camera
                    selectedCameraId = cameraIdList->cameraIds[i];
                    selectedCameraMetadataPtr = std::move(metadataPtr);
                    break; // Choose the first back camera found
                }
            }
        }
    }
    Log::Write(Log::Level::Info, Fmt(LOG_TAG "Selected camera ID: %s", selectedCameraId));

    if (!openCamera(selectedCameraId)) {
        return;
    }

    {
        ACameraMetadata_const_entry streamConfigurationEntry;
        int32_t streamFormat = 0;
        int32_t streamWidth = 0;
        int32_t streamHeight = 0;
        if (ACameraMetadata_getConstEntry(selectedCameraMetadataPtr.get(), ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS, &streamConfigurationEntry) == ACAMERA_OK) {
            for (int i=0; i<streamConfigurationEntry.count/4; ++i) {
                int32_t format = streamConfigurationEntry.data.i32[i * 4 + 0];
                int32_t width = streamConfigurationEntry.data.i32[i * 4 + 1];
                int32_t height = streamConfigurationEntry.data.i32[i * 4 + 2];
                int32_t input = streamConfigurationEntry.data.i32[i * 4 + 3];
                Log::Write(Log::Level::Info, Fmt(LOG_TAG "Stream configuration %d: Format: %d, Width: %d, Height: %d, Input: %d", i, format, width, height, input));
                if ((format == AIMAGE_FORMAT_YUV_420_888) &&
                    (width > streamWidth) &&
                    (height > streamHeight)) {
                    streamFormat = format;
                    streamWidth = width;
                    streamHeight = height;
                }
            }
        }

        ACameraMetadata_const_entry sensorPixelArraySizeEntry;
        if (ACameraMetadata_getConstEntry(selectedCameraMetadataPtr.get(), ACAMERA_SENSOR_INFO_PIXEL_ARRAY_SIZE, &sensorPixelArraySizeEntry) == ACAMERA_OK) {
            for (int i=0; i<sensorPixelArraySizeEntry.count/2; ++i) {
                int32_t width = sensorPixelArraySizeEntry.data.i32[i * 2 + 0];
                int32_t height = sensorPixelArraySizeEntry.data.i32[i * 2 + 1];
                Log::Write(Log::Level::Info, Fmt(LOG_TAG "Sensor pixel array size %d: Width: %d, Height: %d", i, width, height));
            }
        }
        
        // AIMAGE_FORMAT_YUV_420_888 is good for processing, can convert to grayscale easily
        int maxImages = 4;
        if (!createImageReader(streamWidth, streamHeight, streamFormat, maxImages)) {
            return;
        }
    }
    
    // Create capture session
    if (!createCaptureSession()) {
        return;
    }

    if (!createCaptureRequest()) {
        return;
    }

    {
        Log::Write(Log::Level::Info, LOG_TAG "Setting repeating request...");
        ACameraCaptureSession_captureCallbacks captureCallbacks = {
                .context = this, // Pass this AprilTagDetectorImp instance
                .onCaptureStarted = captureStarted,
                .onCaptureProgressed = nullptr, // Optional
                .onCaptureCompleted = ::captureCompleted,
                .onCaptureFailed = ::captureFailed,
                .onCaptureSequenceCompleted = ::captureSequenceCompleted,
                .onCaptureSequenceAborted = ::captureSequenceAborted,
                .onCaptureBufferLost = ::captureBufferLost
        };
        int sequenceId;
        ACaptureRequest *captureRequest = m_captureRequestRepeatingPtr.get();
        camera_status_t status = ACameraCaptureSession_setRepeatingRequest(m_cameraCaptureSessionPtr.get(),
                                                           &captureCallbacks,
                                                           1, // numRequests
                                                           &captureRequest, // array of requests
                                                           &sequenceId); // optional output for sequence ID
        if (status == ACAMERA_OK) {
            Log::Write(Log::Level::Info,
                       Fmt(LOG_TAG "Successfully set repeating request. Sequence ID: %d",
                           sequenceId));
        } else {
            Log::Write(Log::Level::Error,
                       Fmt(LOG_TAG "Failed to set repeating request, status: %d", status));
        }
    }
    
    Log::Write(Log::Level::Info, LOG_TAG "Initialize end");
}


void AprilTagDetectorImp::onCaptureSessionConfiguredImpl(ACameraCaptureSession *session) {
    Log::Write(Log::Level::Info, Fmt(LOG_TAG "onCaptureSessionConfiguredImpl. Session: %p", (void*)session));

    if (!m_cameraDevicePtr) {
        Log::Write(Log::Level::Error, LOG_TAG "Camera device is null in onCaptureSessionConfiguredImpl.");
        return;
    }
    if (m_cameraCaptureSessionPtr.get() != session) {
        Log::Write(Log::Level::Error, Fmt(LOG_TAG "Session mismatch in onCaptureSessionConfiguredImpl. Expected: %p, Got: %p",
                                          (void*)m_cameraCaptureSessionPtr.get(), (void*)session));
        // It's possible the session was reconfigured or this is an old callback.
        // For safety, let's assume our m_cameraCaptureSessionPtr is the correct one to use for new requests.
        // However, if 'session' is different, it means the NDK is telling us THIS session is ready.
        // We should ensure our m_cameraCaptureSessionPtr is up-to-date or handle this case.
        // For now, let's proceed assuming they should match if initialization flow is correct.
        // If they don't match, something is wrong. We might have received a callback for an old session.
        if (m_cameraCaptureSessionPtr.get() == nullptr) {
            Log::Write(Log::Level::Warning, LOG_TAG "Our session ptr is null, but callback received for a session. This is unexpected.");
            // Potentially take ownership of 'session' if our unique_ptr is empty,
            // but this indicates a flawed state management.
            // For now, we'll rely on the one set during createCaptureSession.
            return; // Avoid proceeding if state is inconsistent.
        }
    }

    // Set capture parameters if needed (e.g., FPS range, exposure)
    // Example: Setting FPS range (consult NDK docs for available keys and values)
    // Range<AE fps range> ranges[];
    // ACameraMetadata* cameraCharacteristics;
    // ACameraManager_getCameraCharacteristics(m_cameraManagerRawPtr, cameraId, &cameraCharacteristics);
    // ACameraMetadata_const_entry entry;
    // ACameraMetadata_getConstEntry(cameraCharacteristics, ACAMERA_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES, &entry);
    // if (entry.count > 0) {
    //    ACaptureRequest_setEntry_i32(m_captureRequestRepeatingPtr.get(), ACAMERA_CONTROL_AE_TARGET_FPS_RANGE, 2, entry.data.i32);
    //    Log::Write(Log::Level::Info, Fmt(LOG_TAG "Set FPS range to: [%d, %d]", entry.data.i32[0], entry.data.i32[1]));
    // }
    // ACameraMetadata_free(cameraCharacteristics);
    
}

std::unique_ptr<AprilTagDetector> rt::GetAprilTagDetector() {
    Log::Write(Log::Level::Info, LOG_TAG "GetAprilTagDetector enter");

    std::unique_ptr<AprilTagDetectorImp> detector = std::make_unique<AprilTagDetectorImp>();

    return detector;
}
