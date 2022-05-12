//
// Created by thomas on 5/23/17.
//

#ifndef PROJECT_TFLOGGER_H
#define PROJECT_TFLOGGER_H

#include "Grabelement.h"
#include <ros/ros.h>
#include <vector>
#include <boost/property_tree/ptree.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>


namespace RoposeGrabber {
    class TFLogger : public Grabelement{

    public:
        TFLogger(std::string name, std::string path, std::vector<std::string> transformNames);
        TFLogger(std::string name, std::string path, std::vector<std::string> transformNames, std::string fromFrame);
        ~TFLogger();

        void LookupCurrentTF(ros::Time time);
        virtual void SaveCurrentState(int frameNr, ros::Time time);


    protected:
        tf2_ros::Buffer* _tfBuffer;
        tf2_ros::TransformListener* _tfListener;
        std::string _latestTransformString;
        std::vector<std::string> _transformNames;

    private:
        std::string _fromFrame = "world";
        boost::property_tree::ptree _currentEntry;
        std::map<std::string, std::string> _cameraNames;

    };
}
#endif //PROJECT_TFLOGGER_H
