//
// Created by thomas on 5/19/17.
//

#include "Grabcollection.h"


#include "Camera.h"
#include "TFLogger.h"
#include "PointProjector.h"
#include "GazeboLinkStates.h"

namespace RoposeGrabber {

    Grabcollection::Grabcollection(std::string path) : _path(path) {}

    void Grabcollection::AddCamera(Camera* cam) {
        _elements[cam->_cameraName] = cam;
    }

    void Grabcollection::AddTFLogger(std::string name, std::vector<std::string> transformNames)
    {
        TFLogger *newPose = new TFLogger(name, _path + "/" + name, transformNames);
        _elements[name] = newPose;
    }

    void Grabcollection::AddTFLogger(std::string name, std::vector<std::string> transformNames, std::string fromFrame)
    {
        TFLogger *newPose = new TFLogger(name, _path + "/" + name, transformNames, fromFrame);
        _elements[name] = newPose;
    }

    void Grabcollection::AddPointProjector(std::string name, std::vector<std::string> transformNames,
                                           std::string fromFrame, Eigen::MatrixXd projectionMatrix, bool isOpticalFrame)
    {
        PointProjector *newPose = new PointProjector(name, _path + "/" + name, transformNames, fromFrame,
                                                     projectionMatrix, isOpticalFrame);
        _elements[name] = newPose;
    }

    void Grabcollection::AddGazeboLinkStates(std::string name, std::vector<std::string> transformNames)
    {
        GazeboLinkStates *newLinkState = new GazeboLinkStates(name, _path, transformNames);
        _elements[name] = newLinkState;
    }

    void Grabcollection::AddGazeboLinkStates(std::string name, std::vector<std::string> transformNames,
                                             std::map<std::string, std::string> cameraNames)
    {
        GazeboLinkStates *newLinkState = new GazeboLinkStates(name, _path, transformNames, cameraNames);
        _elements[name] = newLinkState;
    }

    void Grabcollection::InitTopics(ros::NodeHandle *node)
    {
        std::map<std::string, Grabelement*>::iterator it;

        //init topics for all entries
        for ( it = _elements.begin(); it != _elements.end(); it++ )
        {
            it->second->InitTopics(node);
        }
    }

    void Grabcollection::SaveCurrentState() {
        //trigger save state of all elements
        std::map<std::string, Grabelement*>::iterator it;
        try
        {
            ros::Time time = ros::Time::now();

            //TODO: Maybe use an extra thread for this action
            for ( it = _elements.begin(); it != _elements.end(); it++ )
            {
                it->second->SaveCurrentState(frameCounter, time);
            }
            frameCounter++;

            ROS_INFO_STREAM("Saved current states! Saved State Count = " << frameCounter);
        }
        catch(std::runtime_error& ex) {
            ROS_ERROR("Exception while saving State!: [%s]", ex.what());
        }
    }

}
