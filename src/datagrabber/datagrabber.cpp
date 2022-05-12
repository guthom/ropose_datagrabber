/*  TODO:
 * Autocreation of grabbing elements according to the given config in tha yaml file
 * Create files directly in the needed json format
 */


#include <ros/ros.h>
#include <string.h>
#include <vector>
#include <map>

#include <custom_parameter/parameterHandler.h>
#include <custom_parameter/parameter.h>

#include "dataelements/Grabcollection.h"
#include "dataelements/Camera.h"
#include "dataelements/FrameRGB.h"
#include "dataelements/FramePC.h"
#include "dataelements/FrameIR.h"
#include "dataelements/FrameDepth.h"

#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <cmath>
#include <random>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <ropose_datagrabber/SaveCurrentState.h>
#include "../helper/TransformationHandler.h"

#include <vector>
#include <tf/LinearMath/Quaternion.h>

//common stuff
std::string nodeName = "datagrabber";

//ros sutff
ros::NodeHandle* _node;

//parameter stuff
customparameter::ParameterHandler* parameterHandler;
customparameter::Parameter<std::string> paramSavePath;
customparameter::Parameter<float> paramRefreshRate;
customparameter::Parameter<float> paramMinRadius;
customparameter::Parameter<float> paramMaxRadius;
customparameter::Parameter<float> paramMaxPolAngle;
customparameter::Parameter<float> paramMinPolAngle;
customparameter::Parameter<float> paramMaxAziAngle;
customparameter::Parameter<float> paramMinAziAngle;
customparameter::Parameter<float> paramLookAtRange;
customparameter::Parameter<float> paramLookAtYRange;
customparameter::Parameter<bool> paramAutoGrabbing;
customparameter::Parameter<bool> paramSwitchCamPoses;
customparameter::Parameter<bool> paramForceCamPose;
customparameter::Parameter<bool> paramAdjustForcedCamPose;
customparameter::Parameter<std::vector<float>> paramForcedPose;

customparameter::Parameter<XmlRpc::XmlRpcValue> paramCameras;
customparameter::Parameter<std::vector<std::string>> paramTransformNames;

//cam pose Stuff
gazebo_msgs::SetModelState msgCameraModelState;
ros::ServiceClient setModelStateClient;
geometry_msgs::Pose forcedCamPose;

//organize all elements to grab
RoposeGrabber::Grabcollection* grabcollection;

helper::TransformationHandler* transformHandler;



geometry_msgs::Pose ListToPose(std::vector<double> vector)
{
    //set Pose
    geometry_msgs::Pose newPose;

    newPose.position.x = vector[0];
    newPose.position.y = vector[1];
    newPose.position.z = vector[2];

    tf::Quaternion quat;
    quat.setRPY(vector[3], vector[4], vector[5]);

    newPose.orientation.x = quat.x();
    newPose.orientation.y = quat.y();
    newPose.orientation.z = quat.z();
    newPose.orientation.w = quat.w();

    return newPose;
}

double GetRandomValue(double min, double max)
{
    //https://stackoverflow.com/questions/5289613/generate-random-float-between-two-floats/5289624
    double random = ((double) rand()) / (double) RAND_MAX;
    double range = max - min;
    return (random*range) + min;
}

Eigen::Vector3d  GetRandomCamPosition()
{
    double r = GetRandomValue(paramMinRadius.GetValue(), paramMaxRadius.GetValue());
    double polAng = GetRandomValue(paramMinPolAngle.GetValue(), paramMaxPolAngle.GetValue());
    double aziAng = GetRandomValue(paramMinAziAngle.GetValue(), paramMaxAziAngle.GetValue());
    Eigen::Vector3d ret;

    double a = r *cos(aziAng);
    ret[0] = a * cos(polAng);
    ret[1] = r * sin(aziAng);
    ret[2] = a * sin(polAng);

    return ret;
}


Eigen::Vector3d  GetRandomLookAtPosition()
{
    double max = paramLookAtRange.GetValue();
    double max_2 = max/2;
    Eigen::Vector3d ret;
    ret[0] = GetRandomValue(0.0f, max) - max_2;
    ret[1] = GetRandomValue(0.0f, max) - max_2;
    ret[2] = GetRandomValue(0.0f, max) - max_2;

    return ret;
}

Eigen::Vector3d  GetRandomYVector()
{
    float range = paramLookAtYRange.GetValue();

    Eigen::Vector3d ret;
    if (range < 0.0)
    {
        ret[0] = GetRandomValue(0.0f, 1.0f);
        ret[1] = GetRandomValue(0.0f, 1.0f);
        ret[2] = GetRandomValue(0.0f, 1.0f);
    }
    else
    {
        double randRange = GetRandomValue(-range, range);
        if (randRange < 0.0)
        {
            randRange += M_PI;
        }

        double x = 0.0f;
        double y = 0.0f;
        double z = 0.0f;

        if (randRange >= 0.0 && randRange <= M_PI_2)
        {
            y = sin(randRange);
            z = -cos(randRange);
        }
        else if (randRange >= M_PI_2 && randRange <= M_PI)
        {
            //quadrant 2
            y = sin(randRange);
            z = cos(randRange);
        }
        else if (randRange >= M_PI && randRange < 1.5 * M_PI)
        {
            //quadrant 3
            y = -sin(randRange);
            z = cos(randRange);
        }
        else if (randRange >=  1.5 * M_PI && randRange < 2 * M_PI)
        {
            //quadrant 4
            y = -sin(randRange);
            z = -cos(randRange);
        }

        ret[0] = x;
        ret[1] = y;
        ret[2] = z;

    }

    ret.normalize();
    return ret;
}


Eigen::Quaterniond GetLookAtRotation(Eigen::Vector3d  from, Eigen::Vector3d  to)
{
    using namespace Eigen;
    Eigen::Quaterniond quat;
    quat.w() = 1.0;

    if ( from != to )
    {
        //calculate unit vectors of new coordinate system
        Vector3d xVec, yVec, zVec;
        xVec = to - from;
        xVec.normalize();

        yVec = xVec.cross(GetRandomYVector());
        yVec.normalize();

        zVec = xVec.cross(yVec);
        zVec.normalize();


        Eigen::Matrix3d mat;

        mat <<  xVec[0],  yVec[0],  zVec[0],
                xVec[1],  yVec[1],  zVec[1],
                xVec[2],  yVec[2],  zVec[2];

        Eigen::Quaterniond quatSet(mat);

        quatSet.normalize();
        return quatSet;
    }
    return quat;
}


void InitParams() {
    using namespace std;
    std::string subNamespace = "datagrabber";
    //Standard params
    std::string homePath = boost::filesystem::path(getenv("HOME")).string();

    paramSavePath = parameterHandler->AddParameter("SavePath", "", homePath + "/ropose/ropose_data");
    paramRefreshRate = parameterHandler->AddParameter("RefreshRate", "", (float) 3);
    paramMinRadius = parameterHandler->AddParameter("SimCamMinRadius", "", 2.0f);
    paramMaxRadius = parameterHandler->AddParameter("SimCamMaxRadius", "", 10.0f);
    paramMaxPolAngle = parameterHandler->AddParameter("SimCamMaxPolarAng", "", float(M_PI));
    paramMinPolAngle = parameterHandler->AddParameter("SimCamMinPolarAng", "", float(M_PI/10));
    paramMaxAziAngle = parameterHandler->AddParameter("SimCamMaxAzimuthalAng", "", 0.0f);
    paramMinAziAngle = parameterHandler->AddParameter("SimCamMinAzimuthalAng", "", float(2*M_PI));
    paramLookAtRange = parameterHandler->AddParameter("SimCamLookAtRange", "", 2.0f);
    paramLookAtYRange = parameterHandler->AddParameter("SimCamLookAtYRange", "", -1.0f);
    paramAutoGrabbing = parameterHandler->AddParameter("AutoGrabbing", "", true);
    paramSwitchCamPoses = parameterHandler->AddParameter("SwitchCamPoses", "", false);
    paramForceCamPose = parameterHandler->AddParameter("ForceCamPose", "", false);
    paramAdjustForcedCamPose = parameterHandler->AddParameter("AdjustForcedCamPose", "", false);
    paramForcedPose = parameterHandler->AddParameter("ForcedPose", "",
                                                     vector<float>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}));
    //parameters to generate the grabbing units
    paramCameras = parameterHandler->AddParameter("Cameras", "", XmlRpc::XmlRpcValue());
    paramTransformNames = parameterHandler->AddParameter("TransformNames", "", vector<string>());

}

geometry_msgs::Pose GetRandomCamPose()
{
    geometry_msgs::Pose ret;

    Eigen::Vector3d  camPosition = GetRandomCamPosition();

    ret.position.x = camPosition[0];
    ret.position.y = camPosition[1];
    ret.position.z = camPosition[2];

    Eigen::Quaterniond  quat = GetLookAtRotation(camPosition, GetRandomLookAtPosition()).normalized();

    ret.orientation.x = quat.x();
    ret.orientation.y = quat.y();
    ret.orientation.z = quat.z();
    ret.orientation.w = quat.w();

    return ret;
}

void InitServices()
{
    ros::ServiceClient getModelStateClient = _node->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    setModelStateClient = _node->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    gazebo_msgs::GetModelState getMsg;
    getMsg.request.model_name ="depthcam1";
    getModelStateClient.call(getMsg);
    msgCameraModelState.request.model_state.pose = getMsg.response.pose;
    msgCameraModelState.request.model_state.model_name = "depthcam1";
}

void ToCvCam(geometry_msgs::Pose* pose)
{
    //Rotate pose to fit with the model used by openCV
    tf2::Quaternion origin(pose->orientation.x, pose->orientation.y, pose->orientation.z, pose->orientation.w);

    tf2::Quaternion rot1;
    rot1.setRPY( 0, 0, -M_PI_2);
    tf2::Quaternion rot2;
    rot2.setRPY( -M_PI_2, 0, 0 );

    origin *= rot1;
    origin *= rot2;

    pose->orientation.x = origin.x();
    pose->orientation.y = origin.y();
    pose->orientation.z = origin.z();
    pose->orientation.w = origin.w();
}


void FromCVCamPose(geometry_msgs::Pose* pose)
{
    //Rotate pose to fit with the model used by openCV
    tf2::Quaternion origin(pose->orientation.x, pose->orientation.y, pose->orientation.z, pose->orientation.w);

    tf2::Quaternion rot1;
    rot1.setRPY( 0, 0, M_PI_2);
    tf2::Quaternion rot2;
    rot2.setRPY( M_PI_2, 0, 0 );

    origin *= rot2;
    origin *= rot1;

    pose->orientation.x = origin.x();
    pose->orientation.y = origin.y();
    pose->orientation.z = origin.z();
    pose->orientation.w = origin.w();
}

void SwitchCamPose()
{
    geometry_msgs::Pose pose = GetRandomCamPose();

    msgCameraModelState.request.model_state.pose = pose;
    setModelStateClient.call(msgCameraModelState);

    ToCvCam(&pose);

    transformHandler->SendTransform(pose, "world", "depthcam1_link");
}

void AdjustForcedPose()
{
    Eigen::Vector3d  camPosition;

    camPosition[0] = forcedCamPose.position.x;
    camPosition[1] = forcedCamPose.position.y;
    camPosition[2] = forcedCamPose.position.z;

    Eigen::Quaterniond  quat = GetLookAtRotation(camPosition, GetRandomLookAtPosition()).normalized();

    forcedCamPose.orientation.x = quat.x();
    forcedCamPose.orientation.y = quat.y();
    forcedCamPose.orientation.z = quat.z();
    forcedCamPose.orientation.w = quat.w();

    msgCameraModelState.request.model_state.pose = forcedCamPose;
    setModelStateClient.call(msgCameraModelState);

    ToCvCam(&forcedCamPose);

    transformHandler->SendTransform(forcedCamPose, "world", "depthcam1_link");
}

void SetForcedCamPose()
{
    std::vector<float> forcedPose = paramForcedPose.GetValue();

    forcedCamPose.position.x = forcedPose[0];
    forcedCamPose.position.y = forcedPose[1];
    forcedCamPose.position.z = forcedPose[2];

    forcedCamPose.orientation.x = forcedPose[3];
    forcedCamPose.orientation.y = forcedPose[4];
    forcedCamPose.orientation.z = forcedPose[5];
    forcedCamPose.orientation.w = forcedPose[6];


    FromCVCamPose(&forcedCamPose);

    msgCameraModelState.request.model_state.pose = forcedCamPose;
    setModelStateClient.call(msgCameraModelState);

    ToCvCam(&forcedCamPose);

    transformHandler->SendTransform(forcedCamPose, "world", "depthcam1_link");
}


void CheckPath(std::string savePath)
{
    namespace fs = boost::filesystem;

    if (fs::exists(savePath))
    {
        //rename dir with timestamp
        boost::posix_time::ptime time = boost::posix_time::second_clock::local_time();
        using namespace std;
        std::string timestamp =  to_string(time.date().day()) + to_string(time.date().month()) +
                to_string(time.date().year()) + "_" + to_string(time.time_of_day().hours()) +
                to_string(time.time_of_day().minutes()) + to_string(time.time_of_day().seconds());

        fs::rename(savePath, savePath + "_" + timestamp);
    }
    else
    {
        fs::create_directories(savePath);
    }
}

sensor_msgs::CameraInfo* tempInfoObject = nullptr;

void CameraInfoCallback(const sensor_msgs::CameraInfo &msg)
{
    tempInfoObject = new sensor_msgs::CameraInfo(msg);
}

void InitGrabbElements()
{
    using namespace RoposeGrabber;
    using namespace std;
    using namespace XmlRpc;

    try
    {
        string savePath = paramSavePath.GetValue();
        CheckPath(savePath);
        grabcollection = new Grabcollection(savePath);

        //add robopose with needed tf-frames
        vector<string> transformNames = paramTransformNames.GetValue();

        //extract all the cameras
        XmlRpcValue cameras = paramCameras.GetValue();


        for (int i = 0; i < cameras.size(); i++) {
            XmlRpcValue camera = cameras[i];

            string name = camera["name"];
            string linkName = camera["linkName"];

            XmlRpcValue frames = camera["frames"];
            XmlRpcValue frameTopics = camera["frameTopics"];
            XmlRpcValue frameLinks = camera["frameLinks"];
            XmlRpcValue isOpticalFrame = camera["isOpticalFrame"];
            XmlRpcValue camInfoTopics = camera["camInfoTopics"];

            //create camera instances
            string camFullName = "/" + name + "/";
            string cameraSavePath = savePath + camFullName;

            Camera* cam = new Camera(camFullName, cameraSavePath, name);

            //counter for proper folder naming
            int rgbCounter = 0;
            int pcCounter = 0;
            int irCounter = 0;
            int depthCounter = 0;

            //generate frames
            for(int frameIndex = 0; frameIndex < frames.size(); frameIndex++)
            {
                string frameType = frames[frameIndex];
                string frameTopic = frameTopics[frameIndex];
                string camInfoTopic = camInfoTopics[frameIndex];
                string baseDirName = name + "/" + frameType + to_string(irCounter);

                auto subscriber = _node->subscribe(camInfoTopic, 1000, CameraInfoCallback);

                //extract camera info to initialize a tf loger to generate project the 2D points
                while(!tempInfoObject)
                {
                    ROS_INFO_STREAM("Waiting for camera info of: " + subscriber.getTopic());
                    sleep(1);
                }

                //array to projection matrix
                Eigen::MatrixXd newProjectionMatrix(3, 3);
                newProjectionMatrix.row(0) << tempInfoObject->K[0],  tempInfoObject->K[1], tempInfoObject->K[2];
                newProjectionMatrix.row(1) << tempInfoObject->K[3],  tempInfoObject->K[4], tempInfoObject->K[5];
                newProjectionMatrix.row(2) << tempInfoObject->K[6],  tempInfoObject->K[7], tempInfoObject->K[8];

                delete tempInfoObject;

                std::string frameLink = frameLinks[frameIndex];
                if (frameLink == "")
                {
                    std::string frameLink = linkName;
                }

                grabcollection->AddPointProjector(baseDirName + "/transforms_2D",
                                                  transformNames, frameLink, newProjectionMatrix,
                                                  isOpticalFrame[frameIndex]);

                if(frameType == "rgb" || frameType == "RGB" || frameType == "COLOR")
                {
                    cam->AddFrame(new FrameRGB(frameTopic, camInfoTopic, cameraSavePath + "rgb" + to_string(rgbCounter)));
                    rgbCounter++;
                }
                else if (frameType == "pointcloud" || frameType == "pc" || frameType == "PC")
                {
                    //cam->AddFrame(new FramePC(frameTopic, cameraSavePath + "pc" + to_string(pcCounter)));
                    pcCounter++;
                }
                else if (frameType == "infrared" || frameType == "ir" || frameType == "IR")
                {
                    cam->AddFrame(new FrameIR(frameTopic, camInfoTopic, cameraSavePath + "ir" + to_string(irCounter)));
                    irCounter++;
                }
                else if (frameType == "depth" || frameType == "DEPTH")
                {
                    cam->AddFrame(new FrameDepth(frameTopic, camInfoTopic,
                                                 cameraSavePath + "depth" + to_string(depthCounter)));
                    depthCounter++;
                }
            }

            grabcollection->AddCamera(cam);

            //also add camera transforms we need to reconstruct the poses
            transformNames.push_back(linkName);
            grabcollection->AddTFLogger(name + "/transforms", transformNames, linkName);
        }

        grabcollection->AddTFLogger("world_transforms", transformNames);

        //init all topics of each element
        grabcollection->InitTopics(_node);
        sleep(2);
    }
    catch(std::exception &e)
    {
        ROS_ERROR_STREAM("Cant parse grabbing elements for " + nodeName + " are you sure all parameters are set correct?");
    }
}

bool SaveCurrentState(ropose_datagrabber::SaveCurrentState::Request &req, ropose_datagrabber::SaveCurrentState::Response &res)
{
    grabcollection->SaveCurrentState();

    if (paramSwitchCamPoses.GetValue())
        SwitchCamPose();

    res.Success = true;
}

void RunAutoNode()
{
    ros::Rate rate(paramRefreshRate.GetValue());

    bool switchCamPose = paramSwitchCamPoses.GetValue();
    bool forcedCamPose = paramForceCamPose.GetValue();
    bool adjustForcedCamPose = paramAdjustForcedCamPose.GetValue();

    if (switchCamPose && !forcedCamPose)
    {
        SwitchCamPose();
        rate.sleep();
    }

    if (forcedCamPose)
    {
        SetForcedCamPose();
        rate.sleep();
    }

    while(_node->ok())
    {
        ros::spinOnce();
        grabcollection->SaveCurrentState();

        if (forcedCamPose)
        {
            if (adjustForcedCamPose)
            {
                AdjustForcedPose();
            }
        }
        else
        {
            if (switchCamPose)
                SwitchCamPose();
        }


        //trigger the ros callbacks and wait the needed time
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);
    _node = new ros::NodeHandle(nodeName);

    //init params
    parameterHandler = new customparameter::ParameterHandler(_node);
    transformHandler = new helper::TransformationHandler(_node, 200);
    InitParams();
    InitServices();
    InitGrabbElements();

    //init services
    ros::ServiceServer service = _node->advertiseService("SaveCurrentState", SaveCurrentState);

    if(paramSwitchCamPoses.GetValue() && !paramForceCamPose.GetValue())
    {
        ROS_WARN_STREAM("You have switch campose activated! Be sure you are running a simulation. If not this will"
                                "manipulate the base transformation for your camera and your dataset will be useless!");
    }

    if(paramForceCamPose.GetValue())
    {
        ROS_WARN_STREAM("You forced the simulated cam pose!");
    }

    bool autoGrabing = paramAutoGrabbing.GetValue();
    if(autoGrabing)
    {
        RunAutoNode();
    }
    else
    {
        ros::Rate rate(paramRefreshRate.GetValue());
        while(_node->ok())
        {
            //trigger the ros callbacks and wait the needed time
            ros::spinOnce();
            rate.sleep();
        }

        sleep(5);
    }

    return 0;
}
