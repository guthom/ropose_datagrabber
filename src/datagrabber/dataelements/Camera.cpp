//
// Created by thomas on 5/19/17.
//

#include "Camera.h"

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

namespace RoposeGrabber {

    Camera::Camera(std::string baseTopic, std::string path, std::string cameraName) :
            Grabelement(baseTopic, path), _cameraName(cameraName)
    {
        InitSaveing();
    }

    void Camera::AddFrame(FrameBase* element)
    {
        element->cameraName = _cameraName;
        _frames.push_back(element);
    }

    void Camera::InitTopics(ros::NodeHandle* node)
    {
        //Init topics for all frames in the frame collection
        for (int i = 0; i < _frames.size(); i++)
        {
            _frames[i]->InitTopics(node);
        }

    }

    void Camera::SaveCurrentState(int frameNr, ros::Time time)
    {
        using namespace std;

        std::string imageName = to_string(frameNr);

        for (int i = 0; i < _frames.size(); i++)
        {
            _frames[i]->SaveCurrentState(frameNr, time);
        }
    }
}
