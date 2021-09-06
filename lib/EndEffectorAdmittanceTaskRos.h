#ifndef ENDEFFECTORADMITTANCETASKROS_H
#define ENDEFFECTORADMITTANCETASKROS_H

#include "EndEffectorAdmittanceTask.h"
#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace XBot { namespace Cartesian {

namespace ServerApi
{
class EndEffectorAdmittanceTaskRos;
}

/**
 * @brief The ServerApi::EndEffectorAdmittanceTaskRos class implements a ROS
 * interface for the task.
 */
class ServerApi::EndEffectorAdmittanceTaskRos : public ServerApi::TaskRos
{

public:

    CARTESIO_DECLARE_SMART_PTR(EndEffectorAdmittanceTaskRos)

    EndEffectorAdmittanceTaskRos(TaskDescription::Ptr task,
                       RosContext::Ptr ros_context);

    void run(ros::Time time) override;


private:

    void on_ref_recv(geometry_msgs::Vector3StampedConstPtr msg);

    ros::Subscriber _ref_sub;
    ros::Publisher _cur_ref_pub;

    EndEffectorAdmittanceTask::Ptr _ci_angmom;


};

} }

#endif // ENDEFFECTORADMITTANCETASKROS_H