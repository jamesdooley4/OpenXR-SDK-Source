#ifndef HELLO_XR_NETWORKTABLES_INTEROP_H
#define HELLO_XR_NETWORKTABLES_INTEROP_H

#include <string>
#include <memory>

namespace NTInterop {
    struct PoseData {
        uint64_t framecount;
        double timeStamp;
        XrPosef pose;
    };

    class Publisher {
    public:
        Publisher() = default;
        virtual ~Publisher() = default;
        
        virtual void PublishPose(PoseData const &poseData) = 0;
    };

    std::unique_ptr<NTInterop::Publisher> StartNetworkTablesClient(std::string const &serverAddress);
    std::unique_ptr<NTInterop::Publisher> StartNetworkTablesClient(uint16_t teamNumber);
}

#endif //HELLO_XR_NETWORKTABLES_INTEROP_H
