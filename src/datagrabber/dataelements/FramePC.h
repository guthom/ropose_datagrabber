//
// Created by thomas on 5/22/17.
//

#ifndef PROJECT_FRAMEPC_H
#define PROJECT_FRAMEPC_H

#include "Grabelement.h"

#include <sensor_msgs/PointCloud2.h>

namespace RoposeGrabber {

    class FramePC : public Grabelement {
    public:
        FramePC(std::string baseTopic, std::string path);

        void InitTopics(ros::NodeHandle* node) override;
        void SaveCurrentState(int frameNr, ros::Time time) override;

    protected:
        //callbacks
        void PointCloud2Callback(const sensor_msgs::PointCloud2 &msg);
        std::string _cameraName;

    private:
        sensor_msgs::PointCloud2 _latestPC;
    };

}
#endif //PROJECT_FRAMEPC_H
