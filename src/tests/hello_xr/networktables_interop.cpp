#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/FloatArrayTopic.h>

#include <string>

#include "logger.h"
#include "common.h"
#include "QuestNavConstants.h"
#include "networktables_interop.h"

// It's good practice to define this before including GLM headers
// if you want GLM to use a right-handed system by default and
// potentially match depth conventions (though for quaternions to Euler,
// depth convention is less directly relevant than handedness).
// For Android/OpenXR, a right-handed system is typical.
#define GLM_FORCE_RIGHT_HANDED
// #define GLM_FORCE_DEPTH_ZERO_TO_ONE // If your graphics API uses 0-1 depth

// Must define in order to use glm::eulerAngles
#define GLM_ENABLE_EXPERIMENTAL

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp> // For glm::eulerAngles
#include <glm/gtx/string_cast.hpp> // For easily printing GLM types (optional)

using namespace NTInterop;

// Helper function to convert XrQuaternionf to glm::quat
inline glm::quat xrToGlmQuat(const XrQuaternionf &xrQuat) {
    return glm::quat(xrQuat.w, xrQuat.x, xrQuat.y,
                     xrQuat.z); // GLM constructor order: w, x, y, z
}

// Helper function to convert glm::vec3 (Euler angles) to a more readable format or your own struct
struct EulerAngles {
    float roll;  // Rotation around Z-axis (or forward/backward axis)
    float pitch; // Rotation around X-axis (or right/left axis)
    float yaw;   // Rotation around Y-axis (or up/down axis)

    // Optional: Constructor for convenience
    EulerAngles(float y, float p, float r) : roll(r), pitch(p), yaw(y) {}
};

EulerAngles convertXrQuaternionToEulerAngles(const XrQuaternionf &xrOrientation) {
    // 1. Convert XrQuaternionf to glm::quat
    glm::quat glmOrientation = xrToGlmQuat(xrOrientation);

    // 2. Extract Euler angles from the glm::quat
    // glm::eulerAngles returns a vec3 with (yaw, pitch, roll) in radians by default
    // The order of rotations to achieve the orientation is Y (yaw), then X (pitch), then Z (roll)
    glm::vec3 eulerRadians = glm::eulerAngles(glmOrientation);

    // 3. Convert radians to degrees if needed (optional)
    // float yawDegrees = glm::degrees(eulerRadians.y); // Yaw is often around Y
    // float pitchDegrees = glm::degrees(eulerRadians.x); // Pitch is often around X
    // float rollDegrees = glm::degrees(eulerRadians.z);  // Roll is often around Z

    // 4. Store or return them. Note the order from glm::eulerAngles:
    // eulerRadians.x is pitch
    // eulerRadians.y is yaw
    // eulerRadians.z is roll
    // (This might seem counter-intuitive, but it corresponds to the YXZ rotation order)

    return EulerAngles(eulerRadians.y, eulerRadians.x, eulerRadians.z); // Yaw, Pitch, Roll
}

namespace NTInterop {
    class NTPublisher : public Publisher {
    public:
        NTPublisher(std::string const &serverAddress) {
            // Get the NetworkTables instance
            networkTableInstance = nt::NetworkTableInstance::GetDefault();
            networkTableInstance.AddLogger(7, UINT_MAX, [](auto& event) {
                if (auto msg = event.GetLogMessage()) {
                    Log::Write(Log::Level::Info, Fmt("NTInstance: %d: %s", msg->level, msg->message.c_str()));
                }
            });
            networkTableInstance.SetServer(serverAddress);
            networkTableInstance.StartClient4("Quest3S");

            // Create the per-frame publishers
            frameCountPublisher = networkTableInstance.GetIntegerTopic(QuestNavConstants::Topics::FRAME_COUNT).Publish();
            timestampPublisher = networkTableInstance.GetDoubleTopic(QuestNavConstants::Topics::TIMESTAMP).Publish();
            positionPublisher = networkTableInstance.GetFloatArrayTopic(QuestNavConstants::Topics::POSITION).Publish();
            quaternionPublisher = networkTableInstance.GetFloatArrayTopic(QuestNavConstants::Topics::QUATERNION).Publish();
            eulerAnglesPublisher = networkTableInstance.GetFloatArrayTopic(QuestNavConstants::Topics::EULER_ANGLES).Publish();

        }

        ~NTPublisher() override {};

        void PublishPose(PoseData const &poseData) override {
            if (networkTableInstance.IsConnected()) {
                frameCountPublisher.Set(poseData.framecount);
                timestampPublisher.Set(poseData.timeStamp);
                positionPublisher.Set({&poseData.pose.position.x, 3}); //std::span<const float>
                quaternionPublisher.Set({&poseData.pose.orientation.x, 4});
                auto eulerAngles = convertXrQuaternionToEulerAngles(poseData.pose.orientation);
                eulerAnglesPublisher.Set({&eulerAngles.roll, 3});
            }
        }

    private:
        // Network table items
        nt::NetworkTableInstance networkTableInstance;
        nt::IntegerPublisher frameCountPublisher;
        nt::DoublePublisher timestampPublisher;
        nt::FloatArrayPublisher positionPublisher;
        nt::FloatArrayPublisher quaternionPublisher;
        nt::FloatArrayPublisher eulerAnglesPublisher;
    };
}

std::unique_ptr<Publisher> NTInterop::StartNetworkTablesClient(std::string const &serverAddress) {
    Log::Write(Log::Level::Info, Fmt("NTInstance: Initializing network table instance for %s", serverAddress.c_str()));

    std::unique_ptr<Publisher> publisher = std::make_unique<NTPublisher>(serverAddress);
    
    return publisher;
}
