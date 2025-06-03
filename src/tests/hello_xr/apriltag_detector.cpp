#include "apriltag_detector.h"

#include <camera/NdkCameraManager.h>
#include <stdlib.h>

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

    // When done, free the camera list later:
    // ACameraManager_deleteCameraIdList(idList);
}
