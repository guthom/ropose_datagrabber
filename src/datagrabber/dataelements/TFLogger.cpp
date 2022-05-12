//
// Created by thomas on 5/23/17.
//

#include "TFLogger.h"
#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace RoposeGrabber {

    TFLogger::TFLogger(std::string name, std::string path, std::vector<std::string> transformNames)
            : Grabelement(name, path), _transformNames(transformNames)
    {
        _tfBuffer = new tf2_ros::Buffer();

        _tfListener = new tf2_ros::TransformListener(*_tfBuffer);
        InitSaveing();
    }

    TFLogger::TFLogger(std::string name, std::string path, std::vector<std::string> transformNames,
                       std::string fromFrame)
            : TFLogger(name, path, transformNames)
    {
        _fromFrame = fromFrame;
    }

    void TFLogger::LookupCurrentTF(ros::Time rosTime)
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

                    data.put( _transformNames[i] + ".translation.x", to_string(transformStamped.transform.translation.x));
                    data.put( _transformNames[i] + ".translation.y", to_string(transformStamped.transform.translation.y));
                    data.put( _transformNames[i] + ".translation.z", to_string(transformStamped.transform.translation.z));

                    data.put( _transformNames[i] + ".rotation.x", to_string(transformStamped.transform.rotation.x));
                    data.put( _transformNames[i] + ".rotation.y", to_string(transformStamped.transform.rotation.y));
                    data.put( _transformNames[i] + ".rotation.z", to_string(transformStamped.transform.rotation.z));
                    data.put( _transformNames[i] + ".rotation.w", to_string(transformStamped.transform.rotation.w));
                    data.put( _transformNames[i] + ".valid", to_string(true));
                }
                else
                {
                    data.put( _transformNames[i] + ".translation.x", to_string(transformStamped.transform.translation.x));
                    data.put( _transformNames[i] + ".translation.y", to_string(transformStamped.transform.translation.y));
                    data.put( _transformNames[i] + ".translation.z", to_string(transformStamped.transform.translation.z));

                    data.put( _transformNames[i] + ".rotation.x", to_string(transformStamped.transform.rotation.x));
                    data.put( _transformNames[i] + ".rotation.y", to_string(transformStamped.transform.rotation.y));
                    data.put( _transformNames[i] + ".rotation.z", to_string(transformStamped.transform.rotation.z));
                    data.put( _transformNames[i] + ".rotation.w", to_string(transformStamped.transform.rotation.w));
                    data.put( _transformNames[i] + ".valid", to_string(false));
                }
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("Exception while lookup new transformations: %s",ex.what());
                data.put( _transformNames[i] + ".translation.x", to_string(transformStamped.transform.translation.x));
                data.put( _transformNames[i] + ".translation.y", to_string(transformStamped.transform.translation.y));
                data.put( _transformNames[i] + ".translation.z", to_string(transformStamped.transform.translation.z));

                data.put( _transformNames[i] + ".rotation.x", to_string(transformStamped.transform.rotation.x));
                data.put( _transformNames[i] + ".rotation.y", to_string(transformStamped.transform.rotation.y));
                data.put( _transformNames[i] + ".rotation.z", to_string(transformStamped.transform.rotation.z));
                data.put( _transformNames[i] + ".rotation.w", to_string(transformStamped.transform.rotation.w));
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

    void TFLogger::SaveCurrentState(int frameNr, ros::Time time)
    {
        LookupCurrentTF(time);
        //_recorder->AppendLine(std::to_string(frameNr) + ";" + _currentEntry);
        _recorder->RecordGroundTruth_Json(_path, _currentEntry, frameNr);
    }


    TFLogger::~TFLogger()
    {
    }
}