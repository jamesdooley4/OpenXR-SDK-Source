#ifndef HELLO_XR_APRILTAG_DETECTOR_H
#define HELLO_XR_APRILTAG_DETECTOR_H

#include <memory>
namespace rt {
    class AprilTagDetector {
    public:
        AprilTagDetector() {};

        virtual ~AprilTagDetector() {};

        virtual void Initialize() = 0;
    };

    std::unique_ptr<AprilTagDetector> GetAprilTagDetector();
}

#endif //HELLO_XR_APRILTAG_DETECTOR_H
