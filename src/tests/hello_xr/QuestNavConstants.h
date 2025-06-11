#ifndef HELLO_XR_QUESTNAVCONSTANTS_H
#define HELLO_XR_QUESTNAVCONSTANTS_H

#include <string_view> // Use string_view for compile-time string literals

namespace QuestNavConstants {

    /// <summary>
    /// Constants related to NetworkTables topics and paths.
    /// </summary>
    namespace Topics {
        /// <summary>
        /// Base path for all QuestNav topics
        /// </summary>s
        constexpr std::string_view BASE_PATH = "/questnav";

        /// <summary>
        /// Command response topic (Quest to robot)
        /// </summary>
        constexpr std::string_view MISO = "/questnav/miso";

        /// <summary>
        /// Command request topic (robot to Quest)
        /// </summary>
        constexpr std::string_view MOSI = "/questnav/mosi";

        /// <summary>
        /// Frame count topic
        /// </summary>
        constexpr std::string_view FRAME_COUNT = "/questnav/frameCount";

        /// <summary>
        /// Timestamp topic
        /// </summary>
        constexpr std::string_view TIMESTAMP = "/questnav/timestamp";

        /// <summary>
        /// Position topic
        /// </summary>
        constexpr std::string_view POSITION = "/questnav/position";

        /// <summary>
        /// Position topic
        /// </summary>
        constexpr std::string_view POSE2D = "/questnav/pose2d";

        /// <summary>
        /// Position topic
        /// </summary>
        constexpr std::string_view POSE3D = "/questnav/pose3d";

        /// <summary>
        /// Quaternion rotation topic
        /// </summary>
        constexpr std::string_view QUATERNION = "/questnav/quaternion";

        /// <summary>
        /// Euler angles topic
        /// </summary>
        constexpr std::string_view EULER_ANGLES = "/questnav/eulerAngles";

        /// <summary>
        /// Initial position topic
        /// </summary>
        constexpr std::string_view INIT_POSITION = "/questnav/init/position";

        /// <summary>
        /// Initial euler angles topic
        /// </summary>
        constexpr std::string_view INIT_EULER_ANGLES = "/questnav/init/eulerAngles";

        /// <summary>
        /// Reset pose topic
        /// </summary>
        constexpr std::string_view RESET_POSE = "/questnav/resetpose";

        /// <summary>
        /// Heartbeat topic (Quest to robot)
        /// </summary>
        constexpr std::string_view HEARTBEAT_TO_ROBOT = "/questnav/heartbeat/quest_to_robot";

        /// <summary>
        /// Heartbeat topic (robot to Quest)
        /// </summary>
        constexpr std::string_view HEARTBEAT_FROM_ROBOT = "/questnav/heartbeat/robot_to_quest";

        /// <summary>
        /// How many times we have lost tracking this reboot
        /// </summary>
        constexpr std::string_view TRACKING_LOST_COUNTER = "/questnav/device/trackingLostCounter";

        /// <summary>
        /// The current tracking state
        /// </summary>
        constexpr std::string_view CURRENTLY_TRACKING = "/questnav/device/isTracking";

        /// <summary>
        /// Battery percentage topic
        /// </summary>
        constexpr std::string_view BATTERY_PERCENT = "/questnav/device/batteryPercent";
    }
}

#endif //HELLO_XR_QUESTNAVCONSTANTS_H
