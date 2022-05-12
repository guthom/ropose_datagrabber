//
// Created by thomas on 5/23/17.
//

#include "Recorder.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/ros/conversions.h>
#include <boost/property_tree/json_parser.hpp>

namespace RoposeGrabber {

    Recorder::Recorder()
    {

    };

    Recorder::~Recorder()
    {
    }

    bool Recorder::RecordImage(sensor_msgs::Image imageMsg, std::string path,
                               std::string imageEncoding)
    {
        using namespace std;
        cv_bridge::CvImagePtr cv_ptr;
        try {

            if(imageMsg.encoding != imageEncoding)
                cv_ptr = cv_bridge::toCvCopy(imageMsg, imageEncoding);
            else
                cv_ptr = cv_bridge::toCvCopy(imageMsg);

            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGB);

        } catch (cv_bridge::Exception& e)
        {
            ROS_WARN_STREAM("Exception while trying to save Image:" << e.what());
            return false;
        }

        try {
            std::stringstream sstream;
            sstream.exceptions(std::ifstream::badbit);
            sstream << path + ".png";

            cv::imwrite(sstream.str(),  cv_ptr->image );
        } catch (const ios_base::failure& e)
        {
            ROS_WARN_STREAM("Exception while trying to save Image:" << e.what());
            return false;
        }


        return true;
    }

    bool Recorder::RecoordDepthImage(sensor_msgs::Image imageMsg, std::string path,
                                std::string imageEncoding)
    {
        using namespace std;
        cv_bridge::CvImagePtr cv_ptr;
        try {

            if(imageMsg.encoding != imageEncoding)
                cv_ptr = cv_bridge::toCvCopy(imageMsg, imageEncoding);
            else
                cv_ptr = cv_bridge::toCvCopy(imageMsg);



        } catch (cv_bridge::Exception& e)
        {
            ROS_WARN_STREAM("Exception while trying to save Image:" << e.what());
            return false;
        }

        try {

            cv::FileStorage fs;
            fs.open(path + ".xml", cv::FileStorage::WRITE);
            fs << "depthImage" << cv_ptr->image;
            fs.release();

        } catch (const ios_base::failure& e)
        {
            ROS_WARN_STREAM("Exception while trying to save Image:" << e.what());
            return false;
        }


        return true;
    }

    bool Recorder::CreateCameraInfo(sensor_msgs::CameraInfo info, std::string path, std::string name)
    {
        using namespace std;
        using namespace boost::property_tree;

        try
        {
            std::ofstream filestream;
            filestream.exceptions(std::ifstream::badbit);
            std::string fileName = path +  "/CameraInfo.json";
            filestream.open(fileName, std::ios_base::app);
            //create strings for matrices
            std::string D = "[";
            for (auto& element : info.D)
                D.append(std::to_string(element) + ", ");
            D.pop_back();
            D.pop_back();
            D.append("]");

            std::string K = "[";
            for (auto& element : info.K)
                K.append(std::to_string(element) + ", ");
            K.pop_back();
            K.pop_back();
            K.append("]");

            std::string R = "[";
            for (auto& element : info.R)
                R.append(std::to_string(element) + ", ");
            R.pop_back();
            R.pop_back();
            R.append("]");

            std::string P = "[";
            for (auto& element : info.P)
                P.append(std::to_string(element) + ", ");
            P.pop_back();
            P.pop_back();
            P.append("]");

            boost::property_tree::ptree entry;

            entry.put("CameraName", name);
            entry.put("D", D);
            entry.put("K", K);
            entry.put("R", R);
            entry.put("P", P);

            entry.put("width", to_string(info.width));
            entry.put("height", to_string(info.height));
            entry.put("binning_x", to_string(info.binning_x));
            entry.put("binning_y", to_string(info.binning_y));


            write_json(filestream, entry, false);

            filestream.close();
        }
        catch (const ios_base::failure e)
        {
            ROS_WARN_STREAM("Expection while trying to save CameraInfo-File:" << e.what());
            return false;
        }

        ROS_INFO_STREAM("Saved CameraInfo to" << path);

        return true;
    }


    //bool Recorder::RecordPointCloud(sensor_msgs::PointCloud pcMsg, std::string path) {}

    bool Recorder::RecordPointCloud2(sensor_msgs::PointCloud2 pcMsg, std::string path)
    {
        using namespace std;

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(pcMsg,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
        try
        {

            pcl::io::savePCDFile(path + ".pcd", *temp_cloud);
        }
        catch (const ios_base::failure& e)
        {
            ROS_WARN_STREAM("Exception while trying to save PointCloud2:" << e.what());
            return false;
        }


        return true;
    }

    void Recorder::UseConstantFile(std::string pathToFile)
    {
        _pathToConstantfile = pathToFile;
    }


    bool Recorder::RecordGroundTruth(std::string path, std::string entry, int frameNr)
    {
        try {
            std::ofstream filestream;
            filestream.exceptions(std::ifstream::badbit);
            std::string fileName = path +  "/dataset_" + std::to_string(frameNr) + ".json";


            filestream.open(fileName, std::ios_base::app);
            filestream << entry << "\n";
            filestream.close();
        }
        catch (const std::ios_base::failure e)
        {
            ROS_WARN_STREAM("Expection while trying to append line to constant filestream:" << e.what());
            return false;
        }
        return true;
    }

    bool Recorder::RecordGroundTruth_Json(std::string path, boost::property_tree::ptree entry, int frameNr)
    {

        using namespace boost::property_tree;

        try {
            std::ofstream filestream;
            filestream.exceptions(std::ifstream::badbit);
            std::string fileName = path +  "/dataset_" + std::to_string(frameNr) + ".json";
            filestream.open(fileName, std::ios_base::app);

            write_json(filestream, entry, false);

            filestream.close();
        }
        catch (const std::ios_base::failure e)
        {
            ROS_WARN_STREAM("Expection while trying to append line to constant filestream:" << e.what());
            return false;
        }
        return true;
    }


    bool Recorder::AppendLine(std::string entry)
    {
        try {
            std::ofstream filestream;
            filestream.exceptions(std::ifstream::badbit);
            filestream.open(_pathToConstantfile, std::ios_base::app);
            filestream << entry << "\n";
            filestream.close();
        }
        catch (const std::ios_base::failure e)
        {
            ROS_WARN_STREAM("Expection while trying to append line to constant filestream:" << e.what());
            return false;
        }
        return true;
    }


}