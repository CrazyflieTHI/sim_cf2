#include "gazebo_mr_plugin.h"
#include <ros/ros.h>

// USER HEADERS
#include "ConnectGazeboToRosTopic.pb.h"

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

        node_handle_ = transport::NodePtr(new transport::Node());
		node_handle_->Init(namespace_);

		// Listen to the update event. This event is broadcast every simulation iteration.
		updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&MrPlugin::OnUpdate, this, _1));

        // From https://classic.gazebosim.org/tutorials?tut=guided_i6
        // Create our ROS node. This acts in a similar manner to the Gazebo node
        this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

        // Create named topics for the four range sensors, and subscribe to them
        ros::SubscribeOptions mrSensorFront =
        ros::SubscribeOptions::create<sensor_msgs::Range>(
            "/" + this->model_->GetName() + "/ros/sensor_front",
            1,
            boost::bind(&MrPlugin::OnRosMsg, this, _1, sensorIdFront),
            ros::VoidPtr(),
            &this->rosQueue);
        this->rosSubSensorFront = this->rosNode->subscribe(mrSensorFront);

        ros::SubscribeOptions mrSensorBack =
        ros::SubscribeOptions::create<sensor_msgs::Range>(
            "/" + this->model_->GetName() + "/ros/sensor_back",
            1,
            boost::bind(&MrPlugin::OnRosMsg, this, _1, sensorIdBack),
            ros::VoidPtr(),
            &this->rosQueue);
        this->rosSubSensorBack = this->rosNode->subscribe(mrSensorBack);

        ros::SubscribeOptions mrSensorLeft =
        ros::SubscribeOptions::create<sensor_msgs::Range>(
            "/" + this->model_->GetName() + "/ros/sensor_left",
            1,
            boost::bind(&MrPlugin::OnRosMsg, this, _1, sensorIdLeft),
            ros::VoidPtr(),
            &this->rosQueue);
        this->rosSubSensorLeft = this->rosNode->subscribe(mrSensorLeft);

        ros::SubscribeOptions mrSensorRight =
        ros::SubscribeOptions::create<sensor_msgs::Range>(
            "/" + this->model_->GetName() + "/ros/sensor_right",
            1,
            boost::bind(&MrPlugin::OnRosMsg, this, _1, sensorIdRight),
            ros::VoidPtr(),
            &this->rosQueue);
        this->rosSubSensorRight = this->rosNode->subscribe(mrSensorRight);

        // Spin up the queue helper thread
        this->rosQueueThread =
        std::thread(std::bind(&MrPlugin::IncSensorQueueThread, this));

	    ROS_INFO("Multi-ranger plugin loaded.");
    }

    // Handle an incoming message from ROS
    void MrPlugin::OnRosMsg(const sensor_msgs::RangeConstPtr &_msg, const int sensorId)
    {
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

    // ROS helper function that processes messages
    void MrPlugin::IncSensorQueueThread()
    {
        static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
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
	    ROS_INFO("Multi-ranger: Created subscriber.");
        // ROS_INFO_STREAM(model_->GetName());
		mr_pub_ = node_handle_->Advertise<multi_ranger_sensor_data::msgs::MrSensorData>(model_->GetName() + "/gazebo/mr", 1);

    }

}
