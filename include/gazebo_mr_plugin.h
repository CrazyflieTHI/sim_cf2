/*
 * Copyright 2023 Thomas Izycki, THA, Augsburg
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _GAZEBO_MR_PLUGIN_HH_
#define _GAZEBO_MR_PLUGIN_HH_

#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"
#include <sensor_msgs/msg/range.hpp>

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

    void OnRosMsg(const sensor_msgs::msg::Range::SharedPtr _msg, const int sensorId);

protected:
	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
	virtual void OnUpdate(const common::UpdateInfo&);

private:
    void CreatePubsAndSubs();
    void rosHelper();

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

	transport::PublisherPtr mr_pub_;

    std::shared_ptr<rclcpp::Node> rosNode;

    // ROS2 subscriber
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rosSubSensorFront;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rosSubSensorBack;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rosSubSensorLeft;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rosSubSensorRight;

    std::thread rosRangeMsgThread;

};

} // namespace gazebo

#endif //_GAZEBO_MR_PLUGIN_HH_
