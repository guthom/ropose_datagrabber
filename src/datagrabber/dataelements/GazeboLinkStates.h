//
// Created by thomas on 5/24/17.
//

#ifndef PROJECT_GAZEBOLINKSTATES_H
#define PROJECT_GAZEBOLINKSTATES_H

#include "Grabelement.h"
#include <ros/ros.h>
#include "gazebo_msgs/LinkStates.h"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <boost/functional/hash.hpp>

namespace RoposeGrabber {

    class GazeboLinkStates : public Grabelement {

    public:
        GazeboLinkStates(std::string name, std::string path, std::vector<std::string> transformNames);
        GazeboLinkStates(std::string name, std::string path, std::vector<std::string> transformNames,
                         std::map<std::string, std::string> cameraNames);
        GazeboLinkStates(std::string name, std::string path, std::vector<std::string> transformNames,
                         std::string linkStateTopic);
        GazeboLinkStates(std::string name, std::string path, std::vector<std::string> transformNames,
                         std::string linkStateTopic, std::map<std::string, std::string> cameraNames);
        ~GazeboLinkStates();

        virtual void InitTopics(ros::NodeHandle* node);
        virtual void SaveCurrentState(int frameNr, ros::Time time);
        virtual void InitSaveing();

    private:
        std::string _linkStateTopic = "/gazebo/link_states";
        std::vector<std::string> _transformNames;
        std::map<std::string, RoposeGrabber::Recorder*> _poseRecorder;

        //contains the camera topic (first) and the gazebo model Name (second)
        std::map<std::string, std::string> _cameraNames;
        bool _transformInCamPoses = false;
        void GazebomodelStateCallback(const gazebo_msgs::ModelStates &msg);
        void CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg, std::string camName);
        std::map<std::string, sensor_msgs::CameraInfo> _latestCameraInfos;
        std::map<std::string, geometry_msgs::Pose> _latestCamPoses;
        geometry_msgs::Pose GetPoseInCamera(std::string camera, geometry_msgs::Pose pose);

        gazebo_msgs::LinkStates _latestLinkStates;
        std::string _latestLinkString;
        void LinkStatesCallback(const gazebo_msgs::LinkStates &msg);
        void AddFirstEntry();
        std::string CreateFirstCameraEntry();
    };

}
#endif //PROJECT_GAZEBOLINKSTATES_H
