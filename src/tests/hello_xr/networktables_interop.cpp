#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/FloatArrayTopic.h>
#include <networktables/StructTopic.h>
#include <frc/geometry/CoordinateSystem.h>
#include <frc/geometry/Pose3d.h>
#include <units/length.h>

#include <string>

#include "logger.h"
#include "common.h"
#include "QuestNavConstants.h"
#include "networktables_interop.h"

using namespace NTInterop;

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
            pose2dPublisher = networkTableInstance.GetStructTopic<frc::Pose2d>(QuestNavConstants::Topics::POSE2D).Publish();
        }

        ~NTPublisher() override {
            networkTableInstance.StopClient();
        };

        void PublishPose(PoseData const &poseData) override {
            if (!networkTableInstance.IsConnected()) {
                return;
            }

            // Import OpenXR pose
            frc::Pose3d eusPose{
                    poseData.pose.position.x * 1_m,
                    poseData.pose.position.y * 1_m,
                    poseData.pose.position.z * 1_m,
                    frc::Rotation3d{
                            frc::Quaternion{
                                    poseData.pose.orientation.w,
                                    poseData.pose.orientation.x,
                                    poseData.pose.orientation.y,
                                    poseData.pose.orientation.z}}};

            // Convert to WPILib pose
            frc::Pose3d nwuPose
                    = frc::CoordinateSystem::Convert(
                            eusPose,
                            frc::CoordinateSystem{
                                    frc::CoordinateAxis::E(),
                                    frc::CoordinateAxis::U(),
                                    frc::CoordinateAxis::S()},
                            frc::CoordinateSystem::NWU());

            // --- Publish to NetworkTables ---
            frameCountPublisher.Set(poseData.framecount);
            timestampPublisher.Set(poseData.timeStamp); // Consider converting XrTime if needed

            // Position (WPILIb)
            std::array<float, 3> positionData_wp{
                    static_cast<float>(nwuPose.X().value()),
                    static_cast<float>(nwuPose.Y().value()),
                    static_cast<float>(nwuPose.Z().value())};
            positionPublisher.Set(positionData_wp);
            
            // Pose2d (WPILib)
            pose2dPublisher.Set(nwuPose.ToPose2d());

            // Quaternion (WPILib)
            // QuestNav uses xyzw: https://github.com/QuestNav/QuestNav/blob/main/unity/Assets/QuestNav/Utils/QuaternionExtensions.cs
            std::array<float, 4> quaternionData_wp_xyzw{
                    static_cast<float>(nwuPose.Rotation().GetQuaternion().W()),
                    static_cast<float>(nwuPose.Rotation().GetQuaternion().X()),
                    static_cast<float>(nwuPose.Rotation().GetQuaternion().Y()),
                    static_cast<float>(nwuPose.Rotation().GetQuaternion().Z())};
            quaternionPublisher.Set(quaternionData_wp_xyzw);

            // Euler angles (WPILib)
            std::array<float, 3> eulerAnglesData_wp_rad{
                    static_cast<float>(nwuPose.Rotation().X().value()),
                    static_cast<float>(nwuPose.Rotation().Y().value()),
                    static_cast<float>(nwuPose.Rotation().Z().value())};
            eulerAnglesPublisher.Set(eulerAnglesData_wp_rad);

            //Log::Write(Log::Level::Info, Fmt("WPILib pose: %f, %f, %f Euler: (%f, %f, %f)", positionData_wp[0], positionData_wp[1], positionData_wp[2], eulerAnglesData_wp_rad[0], eulerAnglesData_wp_rad[1], eulerAnglesData_wp_rad[2]));
        }

    private:
        // Network table items
        nt::NetworkTableInstance networkTableInstance;
        nt::IntegerPublisher frameCountPublisher;
        nt::DoublePublisher timestampPublisher;
        nt::FloatArrayPublisher positionPublisher;
        nt::FloatArrayPublisher quaternionPublisher;
        nt::FloatArrayPublisher eulerAnglesPublisher;
        nt::StructPublisher<frc::Pose2d> pose2dPublisher;
    };
}

std::unique_ptr<Publisher> NTInterop::StartNetworkTablesClient(std::string const &serverAddress) {
    Log::Write(Log::Level::Info, Fmt("NTInstance: Initializing network table instance for %s", serverAddress.c_str()));

    std::unique_ptr<Publisher> publisher = std::make_unique<NTPublisher>(serverAddress);
    
    return publisher;
}
