//
// Created by thomas on 5/19/17.
//

#ifndef PROJECT_GRABELEMENT_H
#define PROJECT_GRABELEMENT_H

#include "string.h"
#include "map"
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/message.h>

#include "Recorder.h"

namespace RoposeGrabber {
    class Grabelement {

    private:
        void Init();

    protected:
        std::string _name = "";
        std::string _path = "";
        std::string _fileName = "";

        std::map<std::string, ros::Subscriber> _mapSubscriber;
        Recorder* _recorder;

        void InitSaveing();

    public:
        Grabelement(std::string name, std::string path);
        //leave this to the developer of the childclass
        virtual void InitTopics(ros::NodeHandle* node);
        virtual void SaveCurrentState(int frameNr, ros::Time time);
    };

}
#endif //PROJECT_GRABELEMENT_H
