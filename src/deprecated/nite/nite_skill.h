#ifndef NITE_SKILL_H
#define NITE_SKILL_H

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#include "utils/debug/debug.h"

#define CHECK_RC(nRetVal, what)  \
    if (nRetVal != XN_STATUS_OK) \
{                                \
    ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
    return nRetVal;              \
}

/*! \class  NiteSkill
 *
 */
class NiteSkill {
public:
    /*! constructor */
    NiteSkill() {
        maggieDebug2("ctor");
        init();
    }


    ////////////////////////////////////////////////////////////////////////////

    virtual void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId) {
        maggieDebug2("User_NewUser(%i)", nId);
    }

    ////////////////////////////////////////////////////////////////////////////

    virtual void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId) {
        maggieDebug2("User_LostUser(%i)", nId);
    }

    ////////////////////////////////////////////////////////////////////////////

    virtual void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId) {
        maggieDebug2("UserCalibration_CalibrationStart(%i)", nId);
    }

    ////////////////////////////////////////////////////////////////////////////

    virtual void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess) {
        maggieDebug2("UserCalibration_CalibrationEnd(%i)", nId);
    }

    ////////////////////////////////////////////////////////////////////////////

    virtual void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId) {
        maggieDebug2("UserPose_PoseDetected(%i)", nId);
    }

    ////////////////////////////////////////////////////////////////////////////

    virtual void do_stuff() {
        ROS_INFO_THROTTLE(5, "do_stuff()");
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

private:

    int init() {
        maggieDebug2("init()");
        g_bNeedPose = false;
        g_strPose[0] = '\0';

        std::string configFilename = ros::package::getPath("openni_tracker") + "/openni_tracker.xml";
        XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
        CHECK_RC(nRetVal, "InitFromXml");

        nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
        CHECK_RC(nRetVal, "Find depth generator");

        nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
        if (nRetVal != XN_STATUS_OK) {
            nRetVal = g_UserGenerator.Create(g_Context);
            CHECK_RC(nRetVal, "Find user generator");
        }

        if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
            ROS_INFO("Supplied user generator doesn't support skeleton");
            return 1;
        }

        XnCallbackHandle hUserCallbacks;
        g_UserGenerator.RegisterUserCallbacks(User_NewUser_static,
                                              User_LostUser_static,
                                              this,
                                              hUserCallbacks);

        XnCallbackHandle hCalibrationCallbacks;
        g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(
                    UserCalibration_CalibrationStart_static,
                    UserCalibration_CalibrationEnd_static,
                    this,
                    hCalibrationCallbacks);

        if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
            g_bNeedPose = TRUE;
            if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
                ROS_INFO("Pose required, but not supported");
                return 1;
            }

            XnCallbackHandle hPoseCallbacks;
            g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks
                    (UserPose_PoseDetected_static,
                     NULL,
                     this,
                     hPoseCallbacks);

            g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
        }

        g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

        nRetVal = g_Context.StartGeneratingAll();
        CHECK_RC(nRetVal, "StartGenerating");

        ros::Rate r(30);


        ros::NodeHandle pnh("~");
        std::string frame_id("openni_depth_frame");
        pnh.getParam("camera_frame_id", frame_id);

        while (ros::ok()) {
            g_Context.WaitAndUpdateAll();
            do_stuff();
            r.sleep();
        }

        g_Context.Shutdown();
    } // end init()

    ////////////////////////////////////////////////////////////////////////////

    static void XN_CALLBACK_TYPE User_NewUser_static(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
        maggieDebug3("User_NewUser_static(%i)", nId);
        NiteSkill* pThis = ((NiteSkill*) pCookie);
        if (pThis->g_bNeedPose)
            pThis->g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(pThis->g_strPose, nId);
        else
            pThis->g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
        // user callback
        pThis->User_NewUser(generator, nId);
    }

    ////////////////////////////////////////////////////////////////////////////

    static void XN_CALLBACK_TYPE User_LostUser_static(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
        maggieDebug3("User_LostUser_static(%i)", nId);
        ((NiteSkill*) pCookie)->User_LostUser(generator, nId);
    }

    ////////////////////////////////////////////////////////////////////////////

    static void XN_CALLBACK_TYPE UserCalibration_CalibrationStart_static(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
        maggieDebug3("UserCalibration_CalibrationStart_static(%i)", nId);
        ((NiteSkill*) pCookie)->UserCalibration_CalibrationStart(capability, nId);
    }

    ////////////////////////////////////////////////////////////////////////////

    static void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd_static(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
        maggieDebug3("UserCalibration_CalibrationEnd_static(%i)", nId);
        NiteSkill* pThis = ((NiteSkill*) pCookie);
        if (bSuccess) {
            ROS_INFO("Calibration complete, start tracking user %d", nId);
            pThis->g_UserGenerator.GetSkeletonCap().StartTracking(nId);
        }
        else {
            ROS_INFO("Calibration failed for user %d", nId);
            if (pThis->g_bNeedPose)
                pThis->g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(pThis->g_strPose, nId);
            else
                pThis->g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
        }
        // user callback
        pThis->UserCalibration_CalibrationEnd(capability, nId, bSuccess);
    }

    ////////////////////////////////////////////////////////////////////////////

    static void XN_CALLBACK_TYPE UserPose_PoseDetected_static(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
        maggieDebug3("UserPose_PoseDetected_static(%i)", nId);
        NiteSkill* pThis = ((NiteSkill*) pCookie);
        pThis->g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
        pThis->g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
        // user callback
        pThis->UserPose_PoseDetected(capability, strPose, nId);
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    ros::NodeHandle nh;
    xn::Context        g_Context;
    xn::DepthGenerator g_DepthGenerator;
    xn::UserGenerator  g_UserGenerator;

    XnBool g_bNeedPose;
    XnChar g_strPose[20];
};

#endif // NITE_SKILL_H
