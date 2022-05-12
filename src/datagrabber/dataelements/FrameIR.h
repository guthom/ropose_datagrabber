//
// Created by thomas on 5/22/17.
//

#ifndef PROJECT_FRAMEIR_H
#define PROJECT_FRAMEIR_H

#include "FrameRGB.h"

namespace RoposeGrabber {

    class FrameIR : public FrameRGB {

    public:
        FrameIR(std::string baseTopic, std::string infoTopic, std::string path);
    };

}
#endif //PROJECT_FRAMEIR_H
