//
// Created by thomas on 5/22/17.
//

#include "FramePC.h"

namespace RoposeGrabber
{
    FramePC::FramePC(std::string baseTopic, std::string path) :
            Grabelement(baseTopic, path)
    {
        InitSaveing();
    }

    void FramePC::InitTopics(ros::NodeHandle* node)
    {
        //create camera topics with type etc.
        _mapSubscriber["point_cloud"] = node->subscribe(_name, 1000, & FramePC::PointCloud2Callback, this);

    }

    void FramePC::PointCloud2Callback(const sensor_msgs::PointCloud2 &msg)
    {
        _latestPC = msg;
    }


    void FramePC::SaveCurrentState(int frameNr, ros::Time time)
    {
        std::string name = std::to_string(frameNr);

        _recorder->RecordPointCloud2(_latestPC, _path + "/" + name);

    }

}
