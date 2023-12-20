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

#include "gazebo_mr_plugin.h"


namespace gazebo {
    GZ_REGISTER_MODEL_PLUGIN(MrPlugin)

    MrPlugin::MrPlugin()
    : ModelPlugin()
    { }

    MrPlugin::~MrPlugin()
    {
        updateConnection_->~Connection();
    }

    void MrPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        model_ = _model;
        world_ = model_->GetWorld();

		namespace_.clear();

        if (_sdf->HasElement("robotNamespace"))
            namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        else
            gzerr << "[gazebo_imu_plugin] Please specify a robotNamespace.\n";
        node_handle_ = transport::NodePtr(new transport::Node());
        node_handle_->Init(namespace_);

		updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&MrPlugin::OnUpdate, this, _1));

        rosNode = std::make_shared<rclcpp::Node>(namespace_ + "_mr_plugin_ros2_node");

        rosSubSensorFront = rosNode->create_subscription<sensor_msgs::msg::Range>(
                                "/" + namespace_ + "/front/range",
                                1,
                                [this](const sensor_msgs::msg::Range::SharedPtr _msg) {
                                OnRosMsg(_msg, sensorIdFront);
        });

        rosSubSensorBack = rosNode->create_subscription<sensor_msgs::msg::Range>(
                                "/" + namespace_ + "/back/range",
                                1,
                                [this](const sensor_msgs::msg::Range::SharedPtr _msg) {
                                OnRosMsg(_msg, sensorIdBack);
        });

        rosSubSensorLeft = rosNode->create_subscription<sensor_msgs::msg::Range>(
                                "/" + namespace_ + "/left/range",
                                1,
                                [this](const sensor_msgs::msg::Range::SharedPtr _msg) {
                                OnRosMsg(_msg, sensorIdLeft);
        });

        rosSubSensorRight = rosNode->create_subscription<sensor_msgs::msg::Range>(
                                "/" + namespace_ + "/right/range",
                                1,
                                [this](const sensor_msgs::msg::Range::SharedPtr _msg) {
                                OnRosMsg(_msg, sensorIdRight);
        });

        this->rosRangeMsgThread = std::thread(std::bind(&MrPlugin::rosHelper, this));

	    std::cout << "Multi-ranger plugin loaded." << std::endl;
    }

    /* Handle incoming range messages from ROS */
    void MrPlugin::OnRosMsg(const sensor_msgs::msg::Range::SharedPtr _msg, const int sensorId)
    {
        // std::cout << sensorId << ": " << _msg->range << std::endl;
        if(_msg->range > 4.0)
            _msg->range = 4.0;

        if(!sensorDataRdy[sensorId])
        {
            if(sensorId == 0)
            {
                mrMsg.set_front(_msg->range);
            }
            else if(sensorId == 1)
            {
                mrMsg.set_back(_msg->range);
            }
            else if(sensorId == 2)
            {
                mrMsg.set_left(_msg->range);
            } 
            else if(sensorId == 3)
            {
                mrMsg.set_right(_msg->range);
            }
            sensorDataRdy[sensorId] = true;

            for(int i=0; i < 4; i++) {
                if(!sensorDataRdy[i])
                    return;
            }
            sendSensorData = true;
        }
    }

    void MrPlugin::rosHelper()
    {
        rclcpp::spin(rosNode);
        rclcpp::shutdown();
    }

    void MrPlugin::OnUpdate(const common::UpdateInfo&)
    {
        if (!pubs_and_subs_created_)
        {
            CreatePubsAndSubs();
            pubs_and_subs_created_ = true;
        }

        if(!sendSensorData)
            return;

		mr_pub_->Publish(mrMsg);

        for(int i=0; i < 4; i++)
        {
            sensorDataRdy[i] = false;
        }
        sendSensorData = false;
    }

    void MrPlugin::CreatePubsAndSubs()
    {
        std::cout << "Created pubs and subs " << std::endl;

		mr_pub_ = node_handle_->Advertise<multi_ranger_sensor_data::msgs::MrSensorData>(
            namespace_ + "/gazebo/mr", 1);
    }
}
