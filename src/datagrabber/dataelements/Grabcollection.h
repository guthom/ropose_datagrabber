//
// Created by thomas on 5/19/17.
//

#ifndef PROJECT_GRABCOLLECTION_H
#define PROJECT_GRABCOLLECTION_H

#include "map"

#include "Grabelement.h"
#include "Camera.h"
#include <ros/ros.h>
#include <vector>

namespace RoposeGrabber {

    class Grabcollection {

    private:
        std::map<std::string, Grabelement*> _elements;
        std::string _path = "";
        int frameCounter = 0;

    public:
        Grabcollection(std::string path);
        void InitTopics(ros::NodeHandle *node);
        void SaveCurrentState();

        //Add methods for elements
        void AddCamera(Camera* cam);

        void AddTFLogger(std::string name, std::vector<std::string> transformNames);
        void AddTFLogger(std::string name, std::vector<std::string> transformNames, std::string fromFrame);
        void AddPointProjector(std::string name, std::vector<std::string> transformNames, std::string fromFrame,
                               Eigen::MatrixXd  projectionMatrix, bool isOpticalFrame);
        void AddGazeboLinkStates(std::string name, std::vector<std::string> transformNames);
        void AddGazeboLinkStates(std::string name, std::vector<std::string> transformNames,
                                 std::map<std::string, std::string> cameraNames);

    };
}
#endif //PROJECT_GRABCOLLECTION_H
