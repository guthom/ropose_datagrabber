//
// Created by thomas on 5/22/17.
//

#ifndef PROJECT_FRAMERGB_H
#define PROJECT_FRAMERGB_H

#include "FrameBase.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

namespace RoposeGrabber {

    class FrameRGB : public FrameBase {
    public:
        FrameRGB(std::string baseTopic, std::string infoTopic, std::string path);

        void InitTopics(ros::NodeHandle* node) override;
        void SaveCurrentState(int frameNr, ros::Time time) override;

    protected:
        //callbacks
        void ImageRawCallback(const sensor_msgs::Image &msg);

        bool savedCameraInfo = false;

        sensor_msgs::Image _latestImage;

    };

}
#endif //PROJECT_FRAMERGB_H
