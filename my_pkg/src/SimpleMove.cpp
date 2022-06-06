
#include <thread>
#include <atomic>

#include "ros/ros.h"
#include <kortex_driver/ActionNotification.h>
#include <kortex_driver/CartesianSpeed.h>
#include <kortex_driver/ExecuteAction.h>
#include <kortex_driver/SetCartesianReferenceFrame.h>
#include <kortex_driver/SendGripperCommand.h>
#include <kortex_driver/CartesianReferenceFrame.h>
#include <kortex_driver/GripperMode.h>
#include <kortex_driver/ActionEvent.h>
#include <kortex_driver/ActionType.h>
#include <kortex_driver/GetMeasuredCartesianPose.h>
#include <kortex_driver/OnNotificationActionTopic.h>
#include <kortex_driver/Base_ClearFaults.h>
#include <kortex_driver/BaseCyclic_Feedback.h>
#include <kortex_driver/ReadAction.h>

#define HOME_ACTION_IDENTIFIER 2

class RobotControl
{
    private:
    std::string m_robotName = "my_gen3";
    ros::NodeHandle* m_node = nullptr;
    std::atomic<int> m_lastActionNotificationEvent{0};
    std::atomic<int> m_lastActionNotificationID{0};

    private:
    bool readRosParams();
    bool activateActionNotificationService();
    public:
    RobotControl(){};
    void onNotificationCallback(const kortex_driver::ActionNotification& notif);
    void subscribeToActionTopic(void (*cb)(const kortex_driver::ActionNotification &));
    bool homeRobot();
    bool moveCartesian(const kortex_driver::ConstrainedPose& pose,
        const kortex_driver::CartesianSpeed& speed);
    bool waitActionEndOrAbort();

    void init(int& argc, char** argv, const std::string& node, uint32_t options = 0U);
    ~RobotControl();
};

inline bool RobotControl::activateActionNotificationService()
{
    // We need to call this service to activate the Action Notification on the kortex_driver node.
    ros::ServiceClient service_client_activate_notif = m_node->serviceClient<kortex_driver::OnNotificationActionTopic>("/" + m_robotName + "/base/activate_publishing_of_action_topic");
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

inline bool RobotControl::readRosParams()
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

inline void RobotControl::subscribeToActionTopic(void (*cb)(const kortex_driver::ActionNotification &))
{
    ros::Subscriber sub = m_node->subscribe("/" + m_robotName  + "/action_topic", 1000, cb);
}

inline void RobotControl::onNotificationCallback(const kortex_driver::ActionNotification& notif)
{
    m_lastActionNotificationEvent = notif.action_event;
    m_lastActionNotificationID = notif.handle.identifier;
}


inline bool RobotControl::homeRobot()
{
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
    ros::ServiceClient service_client_execute_action = m_node->serviceClient<kortex_driver::ExecuteAction>("/" + m_robotName + "/base/execute_action");
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

    return waitActionEndOrAbort();
}

// RobotControl::RobotControl()
// {
// }

inline bool RobotControl::moveCartesian(const kortex_driver::ConstrainedPose& pose,
        const kortex_driver::CartesianSpeed& speed)
{
    return false;
}

inline bool RobotControl::waitActionEndOrAbort()
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

inline void RobotControl::init(int& argc, char** argv, const std::string& node, uint32_t options)
{
    if (m_node != nullptr) return;
    // Init ROS
    ros::init(argc, argv, node, options);
    // Create node
    m_node = new ros::NodeHandle();
    // Get parameters
    readRosParams();
    // Start action notification service
    activateActionNotificationService();
}

inline RobotControl::~RobotControl()
{
    if (m_node)
        delete m_node;
}



RobotControl robot;



void NotificationCallback(const kortex_driver::ActionNotification& notif)
{
    robot.onNotificationCallback(notif);
}

int main(int argc, char** argv)
{
    robot.init(argc, argv, "SimpleMoveCPP");
    robot.subscribeToActionTopic(NotificationCallback);

    bool result = robot.homeRobot();
    return result;
}