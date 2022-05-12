//
// Created by thomas on 5/22/17.
//

#include "FrameDepth.h"


namespace RoposeGrabber
{

    FrameDepth::FrameDepth(std::string baseTopic, std::string infoTopic, std::string path)
            : FrameRGB(baseTopic, infoTopic, path)
    {

    }

    void FrameDepth::SaveCurrentState(int frameNr, ros::Time time)
    {
        std::string name = std::to_string(frameNr);

        _recorder->RecoordDepthImage(_latestImage, _path + "/" + name,
                                sensor_msgs::image_encodings::TYPE_32FC1);

    }

}
