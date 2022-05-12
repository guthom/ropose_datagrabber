//
// Created by thomas on 5/23/17.
//

#ifndef PROJECT_RECORDER_H
#define PROJECT_RECORDER_H

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <string.h>
#include <pcl/point_types.h>
#include <boost/property_tree/ptree.hpp>
#include <fstream>

namespace RoposeGrabber {
    class Recorder {

    public:
        Recorder();
        ~Recorder();

        bool RecordImage(sensor_msgs::Image imageMsg, std::string path,
                         std::string imageEncoding = sensor_msgs::image_encodings::BGR8);
        bool RecoordDepthImage(sensor_msgs::Image imageMsg, std::string path,
                          std::string imageEncoding = sensor_msgs::image_encodings::BGR8);
        bool CreateCameraInfo(sensor_msgs::CameraInfo info, std::string path, std::string name);
        bool RecordPointCloud(sensor_msgs::PointCloud imageMsg, std::string path);
        bool RecordPointCloud2(sensor_msgs::PointCloud2 pcMsg, std::string path);

        bool AppendLine(std::string entry);
        bool RecordGroundTruth(std::string path, std::string entry, int frameNr);
        bool RecordGroundTruth_Json(std::string path, boost::property_tree::ptree entry, int frameNr);
        void UseConstantFile(std::string pathToFile);

    private:
        std::string _pathToConstantfile;
    };

}
#endif //PROJECT_RECORDER_H
