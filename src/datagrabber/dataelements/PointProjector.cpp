//
// Created by thomas on 5/23/17.
//

#include "PointProjector.h"
#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "opencv2/calib3d/calib3d.hpp"

namespace RoposeGrabber {


    PointProjector::PointProjector(std::string name, std::string path, std::vector<std::string> transformNames,
                                   std::string fromFrame, Eigen::MatrixXd projectionMatrix, bool isOpticalFrame)
            : Grabelement(name, path), _transformNames(transformNames), _isOpticalFrame(isOpticalFrame)
    {
        _tfBuffer = new tf2_ros::Buffer();

        _tfListener = new tf2_ros::TransformListener(*_tfBuffer);
        InitSaveing();
        _fromFrame = fromFrame;

        _projectionMatrix = projectionMatrix;
    }

    void PointProjector::LookupCurrentTF(ros::Time rosTime)
    {
        using namespace std;
        using namespace boost::property_tree;

        //lookup transformations according to the tree of the given vector
        std::string compareString ="";

        ptree data;

        data.put("ros_time", rosTime);

        for (int i = 0; i < _transformNames.size(); i++)
        {
            geometry_msgs::TransformStamped transformStamped;
            try{

                if(_tfBuffer->_frameExists(_transformNames[i]))
                {
                    try
                    {
                        transformStamped = _tfBuffer->lookupTransform(_fromFrame, _transformNames[i], rosTime);
                    }
                    catch (tf2::TransformException &ex)
                    {
                        transformStamped = _tfBuffer->lookupTransform(_fromFrame, _transformNames[i], ros::Time(0.0));
                        ROS_WARN_STREAM("TFLogger: " << "Could not grab transforms at given time, and saved "
                                "the latest instead!");
                    }

                    Eigen::Vector3d point;
                    if(_isOpticalFrame)
                    {
                        //can use rawValues
                        point = Eigen::Vector3d(transformStamped.transform.translation.x,
                                                transformStamped.transform.translation.y,
                                                transformStamped.transform.translation.z);

                    } else
                    {
                        //apply to opencv turned coordinate system
                        point = Eigen::Vector3d(-transformStamped.transform.translation.y,
                                                -transformStamped.transform.translation.z,
                                                transformStamped.transform.translation.x);
                    }


                    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

                    //std::cout << std::endl << _projectionMatrix.format(CleanFmt) << std::endl;

                    Eigen::MatrixXd resPoint =  _projectionMatrix * point;

                    double resX = resPoint(0, 0);
                    double resY = resPoint(1, 0);
                    double resZ = resPoint(2, 0);

                    data.put( _transformNames[i] + ".translation.x", to_string(resX));
                    data.put( _transformNames[i] + ".translation.y", to_string(resY));
                    data.put( _transformNames[i] + ".valid", to_string(true));

                } else{
                    data.put( _transformNames[i] + ".translation.x", to_string(-1));
                    data.put( _transformNames[i] + ".translation.y", to_string(-1));
                    data.put( _transformNames[i] + ".valid", to_string(false));

                }

            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("Exception while lookup new transformations: %s",ex.what());
                data.put( _transformNames[i] + ".translation.x", to_string(-1));
                data.put( _transformNames[i] + ".translation.y", to_string(-1));
                data.put( _transformNames[i] + ".valid", to_string(false));
                continue;
            }

            //TODO: Maybe find a better solution for this redundancy check -> check for robots movements instead
            compareString += to_string(round(transformStamped.transform.translation.x*1000)/1000) +
                    to_string(round(transformStamped.transform.translation.y*1000)/1000) +
                                      to_string(round(transformStamped.transform.translation.z*1000)/1000);

        }
        if(_latestTransformString == compareString)
        {
            data.put("redundant", true);
        }
        else
        {
            _latestTransformString = compareString;
            data.put("redundant", false);
        }

        this->_currentEntry = data;
    }

    void PointProjector::SaveCurrentState(int frameNr, ros::Time time)
    {
        LookupCurrentTF(time);
        //_recorder->AppendLine(std::to_string(frameNr) + ";" + _currentEntry);
        _recorder->RecordGroundTruth_Json(_path, _currentEntry, frameNr);
    }


    PointProjector::~PointProjector()
    {
    }
}