//
// Created by thomas on 5/19/17.
//

#ifndef PROJECT_CAMERA_H
#define PROJECT_CAMERA_H

#include "FrameBase.h"
#include <map>
#include <vector>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/CameraInfo.h>
#include "Recorder.h"
#include "../../helper/TransformationHandler.h"

namespace RoposeGrabber
{
    class Camera : public Grabelement
    {
        public:
        Camera(std::string name, std::string path, std::string cameraName);

        void InitTopics(ros::NodeHandle* node) override;
        void SaveCurrentState(int frameNr, ros::Time time) override;

        void AddFrame(FrameBase* element);

        std::string _cameraName;


        protected:
        std::vector<Grabelement*> _frames;

    };
}
#endif //PROJECT_CAMERA_H
