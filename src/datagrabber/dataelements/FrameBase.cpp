//
// Created by thomas on 5/22/17.
//

#include "FrameBase.h"

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/replace.hpp>

namespace RoposeGrabber
{
    FrameBase::FrameBase(std::string baseTopic, std::string infoTopic, std::string path) :
            Grabelement(baseTopic, path), _infoTopic(infoTopic)
    {
        InitSaveing();
    }

    void FrameBase::InitTopics(ros::NodeHandle* node)
    {
        //create camera topics with type etc.
        _mapSubscriber["camera_info"] = node->subscribe(_infoTopic, 1000, &FrameBase::CameraInfoCallback, this);
    }

    void FrameBase::CameraInfoCallback(const sensor_msgs::CameraInfo &msg) {
        _latestCameraInfo = msg;

        if(!savedCameraInfo)
        {
            std::string cameraName = boost::replace_all_copy(_name, "/", "");

            savedCameraInfo = _recorder->CreateCameraInfo(_latestCameraInfo, _path, this->cameraName);

            //shutdown subscirber cause we need the CameraInfo just once
            _mapSubscriber[_name].shutdown();
        }
    }

    void FrameBase::SaveCurrentState(int frameNr, ros::Time time)
    {

    }

}
