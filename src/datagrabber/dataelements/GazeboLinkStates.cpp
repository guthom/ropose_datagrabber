//
// Created by thomas on 5/24/17.
//

#include "GazeboLinkStates.h"
#include <boost/filesystem.hpp>
#include <boost/bind.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <math.h>

namespace RoposeGrabber {

    GazeboLinkStates::GazeboLinkStates(std::string name, std::string path, std::vector<std::string> transformNames)
            : Grabelement(name, path), _transformNames(transformNames)
    {
        InitSaveing();
    }

    GazeboLinkStates::GazeboLinkStates(std::string name, std::string path, std::vector<std::string> transformNames,
                                       std::string linkStateTopic)
            : Grabelement(name, path), _linkStateTopic(linkStateTopic),  _transformNames(transformNames)

    {
        InitSaveing();
    }

    GazeboLinkStates::GazeboLinkStates(std::string name, std::string path, std::vector<std::string> transformNames,
                                       std::map<std::string, std::string> cameraNames)
            : Grabelement(name, path) , _cameraNames(cameraNames), _transformNames(transformNames)
    {
        _transformInCamPoses = true;
        InitSaveing();
    }

    GazeboLinkStates::GazeboLinkStates(std::string name, std::string path, std::vector<std::string> transformNames,
                                       std::string linkStateTopic, std::map<std::string, std::string> cameraNames)
            : Grabelement(name, path), _cameraNames(cameraNames), _linkStateTopic(linkStateTopic),
              _transformNames(transformNames)
    {
        _transformInCamPoses = true;
        InitSaveing();
    }

    GazeboLinkStates::~GazeboLinkStates() {}

    void GazeboLinkStates::InitSaveing()
    {
        using namespace boost::filesystem;

        //replace / in name because we don't want extra dirs
        _fileName = _path + "/" + _name + ".txt";
        create_directories(_path);

        _recorder->UseConstantFile(_fileName);
        AddFirstEntry();

        if (_transformInCamPoses)
        {
            //we want to save the extra camera coordinates as well
            typedef std::map<std::string, std::string>::iterator it_type;
            for (it_type iterator=_cameraNames.begin(); iterator != _cameraNames.end(); ++iterator)
            {
                RoposeGrabber::Recorder* recorder =  new RoposeGrabber::Recorder();
                std::string fileName =  _path + "/" + _name + "_in_" + iterator->second + ".txt";
                recorder->UseConstantFile(fileName);
                _poseRecorder[iterator->second] = recorder;
                recorder->AppendLine(CreateFirstCameraEntry());
            }
        }
    }

    void GazeboLinkStates::SaveCurrentState(int frameNr, ros::Time time)
    {
        using namespace std;

        //save the additional entries for the different cameras
        std::map <std::string, std::string> camEntries;
        if (_transformInCamPoses) {
            typedef std::map<std::string, std::string>::iterator it_type;
            for (it_type iterator = _cameraNames.begin(); iterator != _cameraNames.end(); ++iterator)
            {
                camEntries[iterator->second] = std::to_string(frameNr)+ ";";
            }
        }

        string currentEntry = std::to_string(frameNr)+ ";";
        string compareString ="";
        boost::hash<std::string> newHash;
        //create current entry from msg
        int looker = 0;
        for (int i = 0; i < _latestLinkStates.name.size(); i++)
        {
            if(_latestLinkStates.name[i] == _transformNames[looker])
            {
                geometry_msgs::Pose pose = _latestLinkStates.pose[i];
                std::string translation = "[";
                translation += to_string(pose.position.x) + ",";
                translation += to_string(pose.position.y) + ",";
                translation += to_string(pose.position.z) + "],";

                std::string rotation = "[";
                rotation += to_string(pose.orientation.x) + ",";
                rotation += to_string(pose.orientation.y) + ",";
                rotation += to_string(pose.orientation.z) + ",";
                rotation += to_string(pose.orientation.w) + "]";
                currentEntry += translation + " " + rotation + ";";

                //TODO: Maybe find a better solution for this redundancy check -> check for robots movements instead
                compareString += to_string(round(pose.position.x*1000)/1000) + to_string(round(pose.position.y*1000)/1000) +
                        to_string(round(pose.position.z*1000)/1000);

                if (_transformInCamPoses)
                {
                    typedef std::map<std::string, std::string>::iterator it_type;
                    for (it_type iterator = _cameraNames.begin(); iterator != _cameraNames.end(); ++iterator)
                    {
                        geometry_msgs::Pose poseInCam = GetPoseInCamera(iterator->second, pose);
                        camEntries[iterator->second] += "[" + to_string(poseInCam.position.x) + "," + to_string(poseInCam.position.y) + "];";
                    }
                }
                looker++;
            }
            if (looker > _transformNames.size())
                break;
        }
        if(_latestLinkString == compareString)
        {
            currentEntry += "REDUNDANT";
            typedef std::map<std::string, std::string>::iterator it_type;
            for (it_type iterator = _cameraNames.begin(); iterator != _cameraNames.end(); ++iterator)
            {
               camEntries[iterator->second] += " REDUNDANT";
            }
        }
        else
        {
            _latestLinkString = compareString;
        }

        _recorder->AppendLine(currentEntry);

        if (_transformInCamPoses) {
            typedef std::map<std::string, std::string>::iterator it_type;
            for (it_type iterator = _cameraNames.begin(); iterator != _cameraNames.end(); ++iterator)
            {
                _poseRecorder[iterator->second]->AppendLine(camEntries[iterator->second]);
            }
        }
    }

    geometry_msgs::Pose GazeboLinkStates::GetPoseInCamera(std::string camera, geometry_msgs::Pose pose)
    {
        geometry_msgs::Pose ret;

        sensor_msgs::CameraInfo cameraInfo = _latestCameraInfos[camera];
        geometry_msgs::Pose camPose = _latestCamPoses[camera];

        //TODO: Finish transformations method here it's done extrenaly at the moment
        //so use the Jointmapper-Tool for now


        return ret;
    }

    void GazeboLinkStates::AddFirstEntry()
    {
        _recorder->AppendLine(_name);
        std::string entry = "Kinematic Chain;";

        for (int i = 0; i < _transformNames.size(); i++)
        {
            entry += _transformNames[i] + ";";
        }
        entry += "[X,Y,Z] [x, y, z, w]";

        _recorder->AppendLine(entry);
    }

    std::string GazeboLinkStates::CreateFirstCameraEntry()
    {
        std::string entry = "Kinematic Chain: ";

        for (int i = 0; i < _transformNames.size(); i++)
        {
            entry += _transformNames[i] + ";";
        }
        entry += "[X,Y]";

        return entry;
    }

    void GazeboLinkStates::InitTopics(ros::NodeHandle* node)
    {
        _mapSubscriber[_linkStateTopic] = node->subscribe(_linkStateTopic, 1000,
                                                          &GazeboLinkStates::LinkStatesCallback, this);
        if(_transformInCamPoses)
        {
            std::string topicName = "/gazebo/model_states";
            _mapSubscriber[topicName] = node->subscribe(topicName, 1000, &GazeboLinkStates::GazebomodelStateCallback,
                                                        this);

            using namespace std;
            typedef map<string,string>::iterator it_type;

            for (it_type iterator=_cameraNames.begin(); iterator != _cameraNames.end(); ++iterator)
            {
                std::string topicName = iterator->first + "/camera_info";
                //use boost bind to extend the callback parameters
                const std::string name = iterator->second;

                ros::Subscriber subscriber = node->subscribe<sensor_msgs::CameraInfo>
                        (topicName, 1000, boost::bind(&GazeboLinkStates::CameraInfoCallback, this, _1, name));
                _mapSubscriber[topicName] = subscriber;
            }
        }
    }

    void GazeboLinkStates::GazebomodelStateCallback(const gazebo_msgs::ModelStates &msg)
    {
        using namespace std;
        gazebo_msgs::ModelStates currentStates = msg;

        typedef map<string,string>::iterator it_type;
        for (int i = 0; i < currentStates.name.size(); i++)
        {
            for (it_type iterator=_cameraNames.begin(); iterator != _cameraNames.end(); ++iterator)
            {
                if (currentStates.name[i] == iterator->second)
                {
                    _latestCamPoses[currentStates.name[i]] = currentStates.pose[i];
                    break;
                }
            }
        }
    }

    void GazeboLinkStates::CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg, std::string camName)
    {
        _latestCameraInfos[camName] = *msg;
    }

    void GazeboLinkStates::LinkStatesCallback(const gazebo_msgs::LinkStates &msg)
    {
        _latestLinkStates = msg;
    }
}
