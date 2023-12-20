/*
 * Copyright 2018 Eric Goubault, Cosynus, LIX, France
 * Copyright 2018 Sylve Putot, Cosynus, LIX, France
 * Copyright 2018 Franck Djeumou, Cosynus, LIX, France
 *
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

#include <iostream>
#include <mutex>
#include <atomic>
#include <atomicops.h>
#include <readerwriterqueue.h>
#include <math.h>
#include <deque>
#include <stdio.h>
#include <sdf/sdf.hh>

#include <boost/bind.hpp>
#include <Eigen/Eigen>

#include <Eigen/Core>
#include "Imu.pb.h"
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include "common.h"

#include "gazebo_pressure_plugin.h"
#include <CommandMotorSpeed.pb.h>
#include <Imu.pb.h>
#include <MagneticField.pb.h>
#include <FluidPressure.pb.h>
#include <Vector3dStamped.pb.h>
#include "CrtpUtils.h"
#include "posixmq_wrapper.h"
#include "crtp.h"
#include "multi_ranger_sensor_data.pb.h"

const unsigned long IPC_QUEUE_LENGTH = 10ul;
const unsigned long IPC_MSG_LENGTH = 32ul;
const unsigned long IPC_QUEUE_MSG_PRIO = 31ul;
const unsigned long IPC_TIMEOUT_NS = 1000000ul; // 1ms
const unsigned long IPC_TIMEOUT_NS_LONG = 100000000ul; // 100ms

namespace gazebo {

typedef const boost::shared_ptr<const gz_mav_msgs::CommandMotorSpeed> CommandMotorSpeedMsgPtr;
typedef const boost::shared_ptr<const gz_sensor_msgs::Imu> ImuMsgPtr;
typedef const boost::shared_ptr<const gz_sensor_msgs::MagneticField> MagneticFieldMsgPtr;
typedef const boost::shared_ptr<const gz_sensor_msgs::FluidPressure> FluidPressureMsgPtr;
typedef const boost::shared_ptr<const gz_geometry_msgs::Vector3dStamped> LpsMsgPtr;
typedef const boost::shared_ptr<const multi_ranger_sensor_data::msgs::MrSensorData> MrMsgPtr;

static const std::string defaultNamespace = "";
static const std::string motorVelocityReferencePubTopic = "/gazebo/command/motor_speed";
static const std::string imuTopic = "/gazebo/imu";
static const std::string magneticFieldTopic = "/gazebo/magnetic_field";
static const std::string fluidPressureTopic = "/gazebo/air_pressure";
static const std::string lpsTopic = "/gazebo/lps";
static const std::string mrTopic = "/gazebo/mr";
static const std::string rxIpcCrtpQueueNameBase = "/rxgazebocrtpmq";
static const std::string txIpcCrtpQueueNameBase = "/txgazebocrtpmq";

class GazeboCfHandlerPlugin : public ModelPlugin {
public:

    GazeboCfHandlerPlugin() :
        namespace_(defaultNamespace),
        imu_topic_(imuTopic),
        motor_velocity_reference_pub_topic_(motorVelocityReferencePubTopic),
        magnetic_field_topic_(magneticFieldTopic),
        fluid_pressure_topic_(fluidPressureTopic),
        lps_topic_(lpsTopic),
        mr_topic_(mrTopic),
        world_(nullptr),
        model_{},
        isInit(false),
        isPluginOn(true),
        sitlConnected(false)
        {}
    ~GazeboCfHandlerPlugin();

    void imuCb(ImuMsgPtr& _msg);
protected:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void OnUpdate(const common::UpdateInfo&);

private:
    std::string namespace_;
    std::string imu_topic_;
    std::string motor_velocity_reference_pub_topic_;
    std::string magnetic_field_topic_;
    std::string fluid_pressure_topic_;
    std::string lps_topic_;
    std::string mr_topic_;
    std::string frame_id_;
    std::string link_name_;

    transport::NodePtr node_handle_;

    // Pointer to the world
    physics::WorldPtr world_;
    // Pointer to the model
    physics::ModelPtr model_;
    // Pointer to the link
    physics::LinkPtr link_;
    // Pointer to the update event connection
    event::ConnectionPtr updateConnection_;

    bool isInit;
    bool isPluginOn;

    moodycamel::BlockingReaderWriterQueue<crtpPacket_t> m_queueSend;

    bool sitlConnected;
    std::string rxQueueName;
    std::string txQueueName;
    mqd_t rxIpcQueue;
    mqd_t txIpcQueue;
    // bool send(const uint8_t* data , uint32_t length);
    int send(crtpPacket_t& crtpPacket, uint32_t length);

    ignition::math::Vector3d gravity_W_;
    ignition::math::Vector3d velocity_prev_W_;

    Eigen::Vector3d gyroscope_bias_;
    Eigen::Vector3d accelerometer_bias_;

    Eigen::Vector3d gyroscope_turn_on_bias_;
    Eigen::Vector3d accelerometer_turn_on_bias_;

    void ImuCallback(ImuMsgPtr& imu_msg);
    void MagneticFieldCallback(MagneticFieldMsgPtr& mag_msg);
    void FluidPressureCallback(FluidPressureMsgPtr& press_msg);
    void LpsCallback(LpsMsgPtr& lps_msg);
    void MrCallback(MrMsgPtr& mr_msg);

    // mutex and messages for motors command
    gz_mav_msgs::CommandMotorSpeed m_motor_speed;
    std::mutex motors_mutex;

    struct MotorsCommand {
        float m1;
        float m2;
        float m3;
        float m4;
    } m_motor_command_;
    transport::PublisherPtr motor_velocity_reference_pub_;
    void writeMotors();
    void handleMotorsMessage(const uint8_t* data);

    std::thread rxIpcThread;
    std::thread txIpcThread;
    void receiveIpc();
    void transmitIpc();
    void initializeSubsAndPub();

    gazebo::transport::SubscriberPtr imu_sub_;
    gazebo::transport::SubscriberPtr magnetic_field_sub_;
    gazebo::transport::SubscriberPtr fluid_pressure_sub_;
    gazebo::transport::SubscriberPtr lps_sub_;
    gazebo::transport::SubscriberPtr mr_sub_;

    int getCrazyflieNumberFromNamespace();

};
}
