//
// Created by thomas on 5/22/17.
//

#ifndef PROJECT_FRAMEDEPTH_H
#define PROJECT_FRAMEDEPTH_H

#include "FrameRGB.h"

namespace RoposeGrabber {

    class FrameDepth : public FrameRGB {
    public:
        FrameDepth(std::string baseTopic, std::string infoTopic, std::string path);
        void SaveCurrentState(int frameNr, ros::Time time) override;

    };

}
#endif //PROJECT_FRAMEDEPTH_H
