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

#include <gazebo/common/common.hh>
#include <gazebo_cf_handler_plugin.h>
#include <thread>
#include <chrono>
#include <string.h>

#define CRTP_HEADER(port, channel) (((port & 0x0F) << 4) | (channel & 0x0F))

namespace gazebo
{

GazeboCfHandlerPlugin::~GazeboCfHandlerPlugin()
{
    this->updateConnection_.reset();

    isPluginOn = false;
    sitlConnected = true;
    rxIpcThread.join();
    txIpcThread.join();

    int errnum;
    if(closeMqPosix(rxIpcQueue, &errnum) == 0)
    {
        // std::cout << namespace_ << " POSIX RX Queue closed queue!" << std::endl;
    } else {
        std::cout << namespace_ << " Error closing POSIX RX Queue!" << std::endl;
    }
    if(unlinkMqPosix(rxQueueName.c_str(), &errnum)== 0)
    {
        std::cout << namespace_ << " POSIX RX Queue deleted queue!" << std::endl;
    } else {
        std::cout << namespace_ << " Error deleting POSIX RX Queue!" << std::endl;
    }

    if(closeMqPosix(txIpcQueue, &errnum) == 0)
    {
        // std::cout << namespace_ << " POSIX TX Queue closed queue!" << std::endl;
    } else {
        std::cout << namespace_ << " Error closing POSIX TX Queue!" << std::endl;
    }
    if(unlinkMqPosix(txQueueName.c_str(), &errnum)== 0)
    {
        std::cout << namespace_ << " POSIX TX Queue deleted queue!" << std::endl;
    } else {
        std::cout << namespace_ << " Error deleting POSIX TX Queue!" << std::endl;
    }
}

void GazeboCfHandlerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
gzdbg << __FUNCTION__ << "() called." << std::endl;

model_ = _model;

world_ = model_->GetWorld();

namespace_.clear();
if(_sdf->HasElement("robotNamespace")){
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    gzdbg << "namespace_ = \"" << namespace_ << "\"." << std::endl;
} else {
    gzerr << "[gazebo_cf_handler_plugin] Please specify a robotNamespace.\n";
}

node_handle_ = transport::NodePtr(new transport::Node());
node_handle_->Init(namespace_);

// Listen to the update event. This event is broadcast every simulation iteration.
updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboCfHandlerPlugin::OnUpdate, this, _1));

int crazyflieId = getCrazyflieNumberFromNamespace();
if (crazyflieId < 0)
{
    std::cout << namespace_ << " ERROR: Cf handler problem with crazyflieId" << std::endl;
    return;
}

std::stringstream strStream;
strStream << std::hex << std::setw(2) << std::setfill('0') << crazyflieId;
std::string cfIdString = strStream.str();

rxQueueName = rxIpcCrtpQueueNameBase + cfIdString;
txQueueName = txIpcCrtpQueueNameBase + cfIdString;
int errnum;

rxIpcQueue = openMqPosixNonblock(rxQueueName.c_str(), IPC_QUEUE_LENGTH, IPC_MSG_LENGTH, &errnum);
if(rxIpcQueue == -1) {
    std::cout << namespace_ << "Couldn't open POSIX mq RX - errnum " <<  errnum << std::endl;
    return;
} else {
    std::cout << namespace_ << " Opened RX POSIX mq: " << rxQueueName << std::endl;
}

txIpcQueue = openMqPosixNonblock(txQueueName.c_str(), IPC_QUEUE_LENGTH, IPC_MSG_LENGTH, &errnum);
if(txIpcQueue == -1) {
    std::cout << namespace_ << "Couldn't open POSIX mq TX - errnum " <<  errnum << std::endl;
    return;
} else {
    std::cout << namespace_ << " Opened TX POSIX mq: " << txQueueName << std::endl;
}

// int ret = -1;
// ret = flushMqPosix(rxIpcQueue, &errnum);
// if(ret == -1) {
//     std::cout << namespace_ << " Couldn't FLUSH POSIX mq RX - errnum " <<  errnum << std::endl;
//     return;
// } else {
//     std::cout << namespace_ << " FLUSHED RX POSIX mq: " << std::endl;
// }

// ret = flushMqPosix(txIpcQueue, &errnum);
// if(ret == -1) {
//     std::cout << namespace_ << " Couldn't FLUSH POSIX mq TX - errnum " <<  errnum << std::endl;
//     return;
// } else {
//     std::cout << namespace_ << " FLUSHED TX POSIX mq: " << std::endl;
// }

/* Set initial motor speed to 0 */
m_motor_command_.m1 = 0;
m_motor_command_.m2 = 0;
m_motor_command_.m3 = 0;
m_motor_command_.m4 = 0;

m_queueSend = moodycamel::BlockingReaderWriterQueue<crtpPacket_t>(20);

rxIpcThread = std::thread(std::bind(&GazeboCfHandlerPlugin::receiveIpc, this));
txIpcThread = std::thread(std::bind(&GazeboCfHandlerPlugin::transmitIpc, this));
}

void GazeboCfHandlerPlugin::OnUpdate(const common::UpdateInfo&)
{
    writeMotors(); // Should reduce this frequency of motor updates
}

void GazeboCfHandlerPlugin::receiveIpc()
{
    // std::cout << namespace_ << " Started receiver IPC thread!" << std::endl;
    /* Set a timeout for dequeue to check regularly for isPluginOn */
    size_t msgLength = IPC_MSG_LENGTH;
    unsigned rxPrio = 31;
    int errnum = 0;
    int ret = -1;
    uint8_t msgBuffer[IPC_MSG_LENGTH];

    /* First, establish a connection to the SITL */
    while(!sitlConnected)
    {
        if((ret = rcvMqPosixTimed(rxIpcQueue, (char*)&msgBuffer[0], msgLength,
                                  &rxPrio, IPC_TIMEOUT_NS_LONG, &errnum)) != -1)
        {
            if(msgBuffer[1] == 0xF3)
            {
                ret = -1;
                std::cout << namespace_ << " Got SYN msg from SITL ..." << std::endl;
                while(ret == -1)
                {
                    ret = sendMqPosixTimed(txIpcQueue, (char*)msgBuffer, msgLength, IPC_QUEUE_MSG_PRIO, IPC_TIMEOUT_NS, &errnum);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                sitlConnected = true;
                std::cout << namespace_ << " Finished sync with SITL!" << std::endl;
            } else {
                std::cout << namespace_ << " Getting something from SITL ... " << msgBuffer[1] << std::endl;
            }
        }
    }

    while(isPluginOn)
    {
        if((ret = rcvMqPosixTimed(rxIpcQueue, (char*)&msgBuffer[0], msgLength,
                                  &rxPrio, IPC_TIMEOUT_NS, &errnum)) != -1)
        {
            /* Motor command message */
            if(crtp(msgBuffer[1]) == crtp(0x09, 0))
                handleMotorsMessage(&msgBuffer[1]);
        }
    }
}

void GazeboCfHandlerPlugin::transmitIpc()
{
    // std::cout << namespace_ << " Started transmitter IPC thread!" << std::endl;
    /* Set a timeout for dequeue to check regularly for isPluginOn */
    auto timeout = std::chrono::milliseconds(100);

    while(!sitlConnected)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

    initializeSubsAndPub();

    crtpPacket_t p;
    while(isPluginOn)
    {
        if(m_queueSend.wait_dequeue_timed(p, timeout) == true)
        {
            send(p, sizeof(p));
        }
    }

    std::cout << namespace_ << " Transmit IPC thread terminated" << std::endl;
}

int GazeboCfHandlerPlugin::send(crtpPacket_t &crtpPacket, uint32_t length)
{
    int errnum = 0;
    uint8_t msgBuffer[IPC_MSG_LENGTH];

    memcpy(&msgBuffer, reinterpret_cast<const void*>(&crtpPacket), length);

    return sendMqPosixTimed(txIpcQueue, (char*)msgBuffer, IPC_MSG_LENGTH, IPC_QUEUE_MSG_PRIO, IPC_TIMEOUT_NS, &errnum);
}

void GazeboCfHandlerPlugin::handleMotorsMessage(const uint8_t* data)
{
	crtpMotorsDataResponse* motorsData = (crtpMotorsDataResponse*)data;
	{
		std::unique_lock<std::mutex> mlock(motors_mutex);
		m_motor_command_.m1 = PWM2OMEGA(motorsData->m1);
		m_motor_command_.m2 = PWM2OMEGA(motorsData->m2);
		m_motor_command_.m3 = PWM2OMEGA(motorsData->m3);
		m_motor_command_.m4 = PWM2OMEGA(motorsData->m4);
	}
}

void GazeboCfHandlerPlugin::initializeSubsAndPub()
{
    imu_sub_ = node_handle_->Subscribe(namespace_ + imu_topic_, &GazeboCfHandlerPlugin::ImuCallback , this);
    magnetic_field_sub_ = node_handle_->Subscribe(namespace_ + magnetic_field_topic_, &GazeboCfHandlerPlugin::MagneticFieldCallback, this);
    fluid_pressure_sub_ = node_handle_->Subscribe(namespace_ + fluid_pressure_topic_, &GazeboCfHandlerPlugin::FluidPressureCallback, this);
    lps_sub_ = node_handle_->Subscribe(namespace_ + lps_topic_, &GazeboCfHandlerPlugin::LpsCallback, this);
    mr_sub_ = node_handle_->Subscribe(namespace_ + mr_topic_, &GazeboCfHandlerPlugin::MrCallback, this);

    motor_velocity_reference_pub_ = node_handle_->Advertise<gz_mav_msgs::CommandMotorSpeed>(namespace_ + motor_velocity_reference_pub_topic_, 1);

    isInit = true;
}

void GazeboCfHandlerPlugin::ImuCallback(ImuMsgPtr& imu_msg)
{
    crtpPacket_t p;

    p.header = CRTP_HEADER(CRTP_PORT_SETPOINT_SIM, 0);
    p.data[0] = SENSOR_GYRO_ACC_SIM;

    Axis3i16 acc = {static_cast<int16_t>(imu_msg->linear_acceleration().x() / SENSORS_G_PER_LSB_CFG / GRAVITY_MAGNITUDE_CF),
                    static_cast<int16_t>(imu_msg->linear_acceleration().y() / SENSORS_G_PER_LSB_CFG / GRAVITY_MAGNITUDE_CF),
                    static_cast<int16_t>(imu_msg->linear_acceleration().z() / SENSORS_G_PER_LSB_CFG / GRAVITY_MAGNITUDE_CF)};
    Axis3i16 gyro = {static_cast<int16_t>(imu_msg->angular_velocity().x() / SENSORS_DEG_PER_LSB_CFG / DEG_TO_RAD_CF),
                     static_cast<int16_t>(imu_msg->angular_velocity().y() / SENSORS_DEG_PER_LSB_CFG / DEG_TO_RAD_CF),
                     static_cast<int16_t>(imu_msg->angular_velocity().z() / SENSORS_DEG_PER_LSB_CFG	/ DEG_TO_RAD_CF)};

    mempcpy(&p.data[1], &acc, sizeof(Axis3i16));
    mempcpy(&p.data[7], &gyro, sizeof(Axis3i16));

    m_queueSend.enqueue(p);
}

void GazeboCfHandlerPlugin::MagneticFieldCallback(MagneticFieldMsgPtr& mag_msg)
{
    crtpPacket_t p;

    p.header = CRTP_HEADER(CRTP_PORT_SETPOINT_SIM, 0);
    p.data[0] = SENSOR_MAG_SIM;

    /* Need to be convert in gauss */
    Axis3i16 mag = {static_cast<int16_t>(mag_msg->magnetic_field().x() * 10000.0 * MAG_GAUSS_PER_LSB),
                    static_cast<int16_t>(mag_msg->magnetic_field().y() * 10000.0 * MAG_GAUSS_PER_LSB),
                    static_cast<int16_t>(mag_msg->magnetic_field().z() * 10000.0 * MAG_GAUSS_PER_LSB)};

    mempcpy(&p.data[1], &mag, sizeof(Axis3i16));

    m_queueSend.enqueue(p);
}

void GazeboCfHandlerPlugin::FluidPressureCallback(FluidPressureMsgPtr& press_msg)
{
    crtpPacket_t p;

    p.header = CRTP_HEADER(CRTP_PORT_SETPOINT_SIM, 0);
    p.data[0] = SENSOR_BARO_SIM;

	double temperature_at_altitude_kelvin = kSeaLevelTempKelvin * exp(- log(press_msg->fluid_pressure() / kPressureOneAtmospherePascals)/kAirConstantDimensionless);
	double height_geopotential_m = (kSeaLevelTempKelvin - temperature_at_altitude_kelvin)/kTempLapseKelvinPerMeter;
	double height_geometric_m = height_geopotential_m * kEarthRadiusMeters / (kEarthRadiusMeters - height_geopotential_m);

	float pressure = static_cast<float>(press_msg->fluid_pressure() * 0.01); // Convert in mBar
	float temperature = static_cast<float>(temperature_at_altitude_kelvin - 273.15);
	float asl = static_cast<float>(height_geometric_m);

    mempcpy(&p.data[1], &pressure, sizeof(float));
    mempcpy(&p.data[5], &temperature, sizeof(float));
    mempcpy(&p.data[9], &asl, sizeof(float));

	m_queueSend.enqueue(p);
}

void GazeboCfHandlerPlugin::LpsCallback(LpsMsgPtr& lps_msg)
{
    crtpPacket_t p;

    /* The position data is not treated as sensor data in the firmware.
       Therefore is is not handled im sensors_sim.c but in crtp_localization_service.c
       and identified by the port (CRTP_PORT_LOCALIZATION) it gets transmitted on */
    p.header = CRTP_HEADER(CRTP_PORT_LOCALIZATION, 0);

	// Just need to transfer to the fcu the external position
    Axis3f pos = {static_cast<float>(lps_msg->position().x()),
                  static_cast<float>(lps_msg->position().y()),
                  static_cast<float>(lps_msg->position().z())};

    mempcpy(&p.data[0], &pos, sizeof(Axis3f));
	m_queueSend.enqueue(p);
}

void GazeboCfHandlerPlugin::MrCallback(MrMsgPtr& mr_msg)
{
    crtpPacket_t p;

    p.header = CRTP_HEADER(CRTP_PORT_SETPOINT_SIM, 0);
    p.data[0] = SENSOR_MR_SIM;

	float mr[4] = {
		static_cast<float>(mr_msg->front()),
		static_cast<float>(mr_msg->back()),
		static_cast<float>(mr_msg->left()),
		static_cast<float>(mr_msg->right())
	};

    mempcpy(&p.data[1], &mr, sizeof(mr));
    m_queueSend.enqueue(p);
}

void GazeboCfHandlerPlugin::writeMotors()
{
    if(!isInit)
        return;

    m_motor_speed.clear_motor_speed();
    motors_mutex.lock();
    m_motor_speed.add_motor_speed(m_motor_command_.m1); // 0
    m_motor_speed.add_motor_speed(m_motor_command_.m2); // 1
    m_motor_speed.add_motor_speed(m_motor_command_.m3); // 2
    m_motor_speed.add_motor_speed(m_motor_command_.m4); // 3
    motors_mutex.unlock();
    motor_velocity_reference_pub_->Publish(m_motor_speed);
}

int GazeboCfHandlerPlugin::getCrazyflieNumberFromNamespace()
{
    std::string prefix = "cf";
    size_t prefixLength = prefix.length();
    int number = -1;

    if (namespace_.substr(0, prefixLength) == prefix) {
        // Extract the number part
        std::string numberStr = namespace_.substr(prefixLength);

        try {
            // Convert hexadecimal string to integer
            number = std::stoi(numberStr, nullptr, 16);
            // std::cout << "Extracted Number: " << number << std::endl;
        } catch (const std::invalid_argument& e) {
            std::cerr << "Invalid number format: " << e.what() << std::endl;
        } catch (const std::out_of_range& e) {
            std::cerr << "Number out of range: " << e.what() << std::endl;
        }
    } else {
        std::cerr << "Invalid namespace format." << std::endl;
    }

    return number;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboCfHandlerPlugin)
}  // namespace gazebo
