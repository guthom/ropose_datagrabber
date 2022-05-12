//
// Created by thomas on 5/19/17.
//

#include "Grabelement.h"
#include <ros/node_handle.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/replace.hpp>

namespace RoposeGrabber {
    Grabelement::Grabelement(std::string name, std::string path) : _name(name), _path(path) {
        Init();
    }

    void Grabelement::Init()
    {
        _recorder = new Recorder();
        InitSaveing();

    }

    void Grabelement::InitTopics(ros::NodeHandle* node) {}
    void Grabelement::SaveCurrentState(int frameNr, ros::Time time) {}
    void Grabelement::InitSaveing()
    {
        using namespace boost::filesystem;
        create_directories(_path);
    }


}