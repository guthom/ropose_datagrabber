//
// Created by thomas on 5/22/17.
//

#include "FrameRGB.h"


namespace RoposeGrabber
{
    FrameRGB::FrameRGB(std::string baseTopic, std::string infoTopic, std::string path) :
            FrameBase(baseTopic, infoTopic, path)
    {
        InitSaveing();
    }

    void FrameRGB::InitTopics(ros::NodeHandle* node)
    {
        FrameBase::InitTopics(node);
        //create camera topics with type etc.
        _mapSubscriber["image"] = node->subscribe(_name, 1000, &FrameRGB::ImageRawCallback, this);

    }

    void FrameRGB::ImageRawCallback(const sensor_msgs::Image &msg)
    {
        _latestImage = msg;
    }

    void FrameRGB::SaveCurrentState(int frameNr, ros::Time time)
    {
        std::string name = std::to_string(frameNr);

        _recorder->RecordImage(_latestImage, _path + "/" + name,
                               _latestImage.encoding);

    }

}
