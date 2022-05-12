//
// Created by thomas on 5/22/17.
//

#ifndef PROJECT_FRAMEBASE_H
#define PROJECT_FRAMEBASE_H

#include "Grabelement.h"

#include <sensor_msgs/CameraInfo.h>

namespace RoposeGrabber {

    class FrameBase : public Grabelement {
    public:
        FrameBase(std::string baseTopic, std::string infoTopic, std::string path);

        void InitTopics(ros::NodeHandle* node) override;
        void SaveCurrentState(int frameNr, ros::Time time) override;
        std::string cameraName;

    protected:

        std::string _infoTopic;

        //callbacks
        void CameraInfoCallback(const sensor_msgs::CameraInfo &msg);

        bool savedCameraInfo = false;
        sensor_msgs::CameraInfo _latestCameraInfo;

    };

}
#endif //PROJECT_FRAMEBASE_H
