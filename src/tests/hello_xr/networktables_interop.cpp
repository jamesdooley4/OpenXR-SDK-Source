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
#include <glm/gtc/matrix_transform.hpp> // For glm::rotate
        
using namespace NTInterop;

// Helper function to convert XrQuaternionf to glm::quat
inline glm::quat xrToGlmQuat(const XrQuaternionf &xrQuat) {
    return glm::quat(xrQuat.w, xrQuat.x, xrQuat.y,
                     xrQuat.z); // GLM constructor order: w, x, y, z
}

// Helper function to convert XrVector3f to glm::vec3
inline glm::vec3 xrToGlmVec3(const XrVector3f &xrVec) {
    return glm::vec3(xrVec.x, xrVec.y, xrVec.z);
}

// WPILib Euler Angles (Roll: around Fwd X, Pitch: around Left Y, Yaw: around Up Z)
struct WpiEulerAngles {
    float roll;  // Rotation around WPILib +X (Forward)
    float pitch; // Rotation around WPILib +Y (Left)
    float yaw;   // Rotation around WPILib +Z (Up)

    WpiEulerAngles(float r, float p, float y) : roll(r), pitch(p), yaw(y) {}
};

// Converts a GLM quaternion (assumed to be in WPILib coordinate system)
// to WPILib-defined Euler angles (Roll:X, Pitch:Y, Yaw:Z)
// IMPORTANT: Euler angle extraction is tricky and order-dependent.
// This assumes an intrinsic ZYX rotation sequence to get Yaw (Z), Pitch (Y), Roll (X)
WpiEulerAngles glmQuatToWpiEulerAngles(const glm::quat& q_wpilib) {
    // GLM's eulerAngles(q) by default gives yaw (Y), pitch (X), roll (Z) for a YXZ application order.
    // We need to be careful. The order of rotation for WPILib's Roll, Pitch, Yaw
    // is often applied as: Yaw (around Z_up), then Pitch (around new Y_left), then Roll (around new X_fwd).
    // This is an intrinsic ZYX rotation order.

    // Convert quaternion to rotation matrix (WPILib convention)
    glm::mat4 rotMat = glm::mat4_cast(q_wpilib);

    float r, p, y;

    // Calculation for ZYX intrinsic rotations (Yaw, Pitch, Roll)
    // Yaw (around Z), Pitch (around Y'), Roll (around X'')
    // singularity at pitch = +/- 90 degrees
    p = asin(-rotMat[2][0]); // Pitch from M(3,1) (0-indexed M(2,0))

    if (abs(cos(p)) > 1e-4) { // Not in singularity
        r = atan2(rotMat[2][1], rotMat[2][2]); // Roll from M(3,2)/M(3,3)
        y = atan2(rotMat[1][0], rotMat[0][0]); // Yaw from M(2,1)/M(1,1)
    } else { // Gimbal lock
        r = 0.0f; // Set roll to 0 (or some other convention)
        y = atan2(-rotMat[0][1], rotMat[1][1]); // Yaw from -M(1,2)/M(2,2) ( M(row,col) )
    }
    return WpiEulerAngles(r, p, y); // Roll, Pitch, Yaw in radians
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
            if (!networkTableInstance.IsConnected()) {
                return;
            }

            // --- Original OpenXR Data ---
            glm::vec3 pos_ox = xrToGlmVec3(poseData.pose.position);
            glm::quat q_ox = xrToGlmQuat(poseData.pose.orientation);

            // --- 1. Position Conversion (OpenXR to WPILib) ---
            // OpenXR: +X Right, +Y Up,   -Z Forward
            // WPILib: +X Fwd,   +Y Left, +Z Up
            glm::vec3 pos_wp;
            pos_wp.x = -pos_ox.z; // WPILib Forward X = OpenXR Negative Z
            pos_wp.y = -pos_ox.x; // WPILib Left Y    = OpenXR Negative X
            pos_wp.z =  pos_ox.y; // WPILib Up Z      = OpenXR Positive Y
            
            // --- 2. Orientation Conversion (OpenXR to WPILib) ---
            // This requires rotating the OpenXR frame by +90 degrees around OpenXR's +Y axis.
            // Quaternion for +90 deg rotation around Y (0,1,0):
            // w = cos(pi/4), x = 0, y = sin(pi/4), z = 0
            // sqrt(0.5f) is approx 0.70710678118f
            const float cos_45_deg = sqrt(0.5f);
            const float sin_45_deg = sqrt(0.5f);
            glm::quat q_offset_rotation = glm::quat(cos_45_deg, 0.0f, sin_45_deg, 0.0f); // GLM: w, x, y, z

            glm::quat q_wp = q_offset_rotation * q_ox;
            q_wp = glm::normalize(q_wp); // Ensure it's still a unit quaternion

            // --- 3. Convert WPILib Quaternion to WPILib Euler Angles (Roll, Pitch, Yaw) ---
            // Roll: About WPILib X (Fwd), Pitch: About WPILib Y (Left), Yaw: About WPILib Z (Up)
            WpiEulerAngles wpi_euler = glmQuatToWpiEulerAngles(q_wp); // This will be in radians

            // --- Publish to NetworkTables ---
            frameCountPublisher.Set(poseData.framecount);
            timestampPublisher.Set(poseData.timeStamp); // Consider converting XrTime if needed

            // Position (WPILib: X Fwd, Y Left, Z Up)
            std::array<float, 3> positionData_wp = {pos_wp.x, pos_wp.y, pos_wp.z};
            positionPublisher.Set(positionData_wp);

            // Quaternion (WPILib convention)
            // NetworkTables expects x,y,z,w typically, but GLM stores w,x,y,z.
            // Check your NT receiver. If it expects x,y,z,w:
            std::array<float, 4> quaternionData_wp_xyzw = {q_wp.x, q_wp.y, q_wp.z, q_wp.w};
            quaternionPublisher.Set(quaternionData_wp_xyzw);
            // If it expects w,x,y,z (less common for NT but good to be aware):
            // std::array<float, 4> quaternionData_wp_wxyz = {q_wp.w, q_wp.x, q_wp.y, q_wp.z};
            // quaternionPublisher.Set(quaternionData_wp_wxyz);


            // Euler Angles (WPILib: Roll (X), Pitch (Y), Yaw (Z) in RADIANS)
            // If NetworkTables expects degrees, convert them:
            // float roll_deg = glm::degrees(wpi_euler.roll);
            // float pitch_deg = glm::degrees(wpi_euler.pitch);
            // float yaw_deg = glm::degrees(wpi_euler.yaw);
            // std::array<float, 3> eulerAnglesData_wp = {roll_deg, pitch_deg, yaw_deg};
            std::array<float, 3> eulerAnglesData_wp_rad = {wpi_euler.roll, wpi_euler.pitch, wpi_euler.yaw};
            eulerAnglesPublisher.Set(eulerAnglesData_wp_rad);

            //Log::Write(Log::Level::Info, Fmt("WPILib pose: %s, %f, %f, %f", glm::to_string(pos_wp).c_str(), wpi_euler.roll, wpi_euler.pitch, wpi_euler.yaw));
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
