#include "my_pkg/RobotControl.h"


bool RobotControl::activateActionNotificationService()
{
    // We need to call this service to activate the Action Notification on the kortex_driver node.
    service_client_activate_notif = m_node->serviceClient<kortex_driver::OnNotificationActionTopic>("/" + m_robotName + "/base/activate_publishing_of_action_topic");
    kortex_driver::OnNotificationActionTopic service_activate_notif;
    if (service_client_activate_notif.call(service_activate_notif))
    {
        ROS_INFO("Action notification activated!");
    }
    else 
    {
        std::string error_string = "Action notification publication failed";
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }
    ros::Duration(1.00).sleep();
    return true;
}

bool RobotControl::readRosParams()
{
    bool success = true;
    // Parameter robot_name
    if (!ros::param::get("~robot_name", m_robotName))
    {
        std::string error_string = "Parameter robot_name was not specified, defaulting to " + m_robotName + " as namespace";
        ROS_WARN("%s", error_string.c_str());
    }
    else 
    {
        std::string error_string = "Using robot_name " + m_robotName + " as namespace";
        ROS_INFO("%s", error_string.c_str());
        success = false;
    }
    return success;
}

void RobotControl::subscribeToActionTopic(void (*cb)(const kortex_driver::ActionNotification &))
{
    //actionNotifSub = m_node->subscribe("/" + m_robotName  + "/action_topic", 1000, cb);
}

void RobotControl::onNotificationCallback(const kortex_driver::ActionNotification& notif)
{
    m_lastActionNotificationEvent = notif.action_event;
    m_lastActionNotificationID = notif.handle.identifier;
}


bool RobotControl::homeRobot()
{
    ROS_INFO("Home start");
    ros::ServiceClient service_client_read_action = m_node->serviceClient<kortex_driver::ReadAction>("/" + m_robotName + "/base/read_action");
    kortex_driver::ReadAction service_read_action;

    // The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
    service_read_action.request.input.identifier = HOME_ACTION_IDENTIFIER;

    if (!service_client_read_action.call(service_read_action))
    {
        std::string error_string = "Failed to call ReadAction";
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }

    // We can now execute the Action that we read 
    service_client_execute_action = m_node->serviceClient<kortex_driver::ExecuteAction>("/" + m_robotName + "/base/execute_action");
    kortex_driver::ExecuteAction service_execute_action;

    service_execute_action.request.input = service_read_action.response.output;
    
    if (service_client_execute_action.call(service_execute_action))
    {
        ROS_INFO("The Home position action was sent to the robot.");
    }
    else
    {
        std::string error_string = "Failed to call ExecuteAction";
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }

    ROS_INFO("Home end");
    return true;
    //return waitActionEndOrAbort();
}

// RobotControl::RobotControl()
// {
// }

bool RobotControl::moveCartesian(const kortex_driver::ConstrainedPose& pose,
        const kortex_driver::CartesianSpeed& speed)
{
    ros::ServiceClient service_client_execute_action = m_node->serviceClient<kortex_driver::ExecuteAction>("/" + m_robotName + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;

  service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(pose);
  service_execute_action.request.input.name = "pose1";
  service_execute_action.request.input.handle.identifier = 1001;
  service_execute_action.request.input.handle.action_type = kortex_driver::ActionType::REACH_POSE;
  
  m_lastActionNotificationEvent = 0;
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("Pose 1 was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction on pose 1";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

}


bool RobotControl::relmoveCartesian(const kortex_driver::ConstrainedPose& pose,
        const kortex_driver::CartesianSpeed& speed)
{

    return false;
}


bool RobotControl::waitActionEndOrAbort()
{
    while (ros::ok())
    {
        if (m_lastActionNotificationEvent.load() == kortex_driver::ActionEvent::ACTION_END)
        {
            ROS_INFO("Received ACTION_END notification for action %d", m_lastActionNotificationID.load());
            return true;
        }
        else if (m_lastActionNotificationEvent.load() == kortex_driver::ActionEvent::ACTION_ABORT)
        {
            ROS_INFO("Received ACTION_ABORT notification for action %d", m_lastActionNotificationID.load());
            return false;
        }
        ros::spinOnce();
    }
    return false;
}

bool RobotControl::init(ros::NodeHandle& node)
{
    bool res = true;
    // if (m_node != nullptr) return true;
    m_node = &node;
    // Get parameters
    res &= readRosParams();
    // Start action notification service
    res &= activateActionNotificationService();
    return res;
}

RobotControl::~RobotControl()
{
}