/*
 * Author: Thomas Izycki <thomas.izycki2@hs-augsburg.de> 
 *
 */

#ifndef _GAZEBO_MR_PLUGIN_HH_
#define _GAZEBO_MR_PLUGIN_HH_

#include <thread>
#include <mutex>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/Range.h>

#include <random>

#include <sdf/sdf.hh>
#include <common.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include "multi_ranger_sensor_data.pb.h"

namespace gazebo
{

class GAZEBO_VISIBLE MrPlugin : public ModelPlugin
{
public:
	MrPlugin();
	virtual ~MrPlugin();

    void OnRosMsg(const sensor_msgs::RangeConstPtr &_msg, const int tollerInt);
    // void OnRosMsg(const sensor_msgs::RangeConstPtr &_msg);

protected:
	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
	virtual void OnUpdate(const common::UpdateInfo&);

private:
    void CreatePubsAndSubs();
    void IncSensorQueueThread();

    // std::mutex sensorDataMutex;

    std::string namespace_;

    bool pubs_and_subs_created_;

    physics::ModelPtr model_;
    physics::WorldPtr world_;
    event::ConnectionPtr updateConnection_;

    transport::NodePtr node_handle_;

    const int sensorIdFront = 0;
    const int sensorIdBack  = 1;
    const int sensorIdLeft  = 2;
    const int sensorIdRight = 3;

	multi_ranger_sensor_data::msgs::MrSensorData mrMsg;


    bool sensorDataRdy[4] = {false, false, false, false};
    bool sendSensorData = false;

    // Gazebo publisher
	transport::PublisherPtr mr_pub_;

    // A node use for ROS transport
    std::unique_ptr<ros::NodeHandle> rosNode;

    // A ROS subscriber
    ros::Subscriber rosSubSensorFront;
    ros::Subscriber rosSubSensorBack;
    ros::Subscriber rosSubSensorLeft;
    ros::Subscriber rosSubSensorRight;

    // A ROS callback queue that helps process messages
    ros::CallbackQueue rosQueue;

    // A thread the keeps running the rosQueue
    std::thread rosQueueThread;

};

} // namespace gazebo

#endif //_GAZEBO_MR_PLUGIN_HH_
