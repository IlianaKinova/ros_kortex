#ifndef __ROBOTCONTROL_H__
#define __ROBOTCONTROL_H__

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
    std::atomic<int>& m_lastActionNotificationEvent;
    std::atomic<int>& m_lastActionNotificationID;
    ros::Subscriber actionNotifSub;
    ros::ServiceClient service_client_activate_notif;

    private:
    bool readRosParams();
    bool activateActionNotificationService();
    public:
    RobotControl(std::atomic<int>& lastActionNotificationEvent,
        std::atomic<int>& lastActionNotificationID) : 
            m_lastActionNotificationEvent(lastActionNotificationEvent),
            m_lastActionNotificationID(lastActionNotificationID)
    {}
    void onNotificationCallback(const kortex_driver::ActionNotification& notif);
    void subscribeToActionTopic(void (*cb)(const kortex_driver::ActionNotification &));
    bool homeRobot();
    bool moveCartesian(const kortex_driver::ConstrainedPose& pose,
        const kortex_driver::CartesianSpeed& speed);
    bool relmoveCartesian(const kortex_driver::ConstrainedPose& pose,
        const kortex_driver::CartesianSpeed& speed);
    bool waitActionEndOrAbort();



    bool init(ros::NodeHandle& node);
    ~RobotControl();
};


#endif // __ROBOTCONTROL_H__