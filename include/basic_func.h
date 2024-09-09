#ifndef __BASIC_FUNC_H__
#define __BASIC_FUNC_H__

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/geometry/Rot3.h>

gtsam::Pose3 fromGeometryPose(const geometry_msgs::Pose& msg)
{
    gtsam::Pose3 pose(gtsam::Rot3(gtsam::Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)), 
        gtsam::Vector3(msg.position.x, msg.position.y, msg.position.z));
    return pose;
}

geometry_msgs::Pose fromGtsamPose(const gtsam::Pose3& pose)
{
    geometry_msgs::Pose msg;
    msg.orientation.w = pose.rotation().toQuaternion().w();
    msg.orientation.x = pose.rotation().toQuaternion().x();
    msg.orientation.y = pose.rotation().toQuaternion().y();
    msg.orientation.z = pose.rotation().toQuaternion().z();

    msg.position.x    = pose.translation().x();
    msg.position.y    = pose.translation().y();
    msg.position.z    = pose.translation().z();
    return msg;
}

#endif