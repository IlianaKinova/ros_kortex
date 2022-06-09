
#include "my_pkg/RobotControl.h"

std::atomic<int> m_lastActionNotificationEvent{0};
std::atomic<int> m_lastActionNotificationID{0};

RobotControl robot{m_lastActionNotificationEvent, m_lastActionNotificationID};

void spinThread()
{
    while (ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.2).sleep();
    }
}

void NotificationCallback(const kortex_driver::ActionNotification& notif)
{
    ROS_INFO("Notification callback called");
    robot.onNotificationCallback(notif);
}

bool wait_for_action_end_or_abort()
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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "SimpleMoveCPP");
    ros::NodeHandle n;
    robot.init(n);

    //auto spint = std::thread(spinThread);
    robot.subscribeToActionTopic(NotificationCallback);
    auto sub = n.subscribe("/my_gen3/action_topic", 1000, NotificationCallback);
    
    bool result = robot.homeRobot();
    
    result &= robot.waitActionEndOrAbort();
    return result;
}