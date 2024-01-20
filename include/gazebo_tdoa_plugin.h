

#ifndef _GAZEBO_TDOA_PLUGIN_HH_
#define _GAZEBO_TDOA_PLUGIN_HH_

#include <random>

#include <sdf/sdf.hh>
#include <common.h>

#include "Tdoa.pb.h"
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include <Vector3dStamped.pb.h>

namespace gazebo
{

// Would be good to set it through a config or launch file
static constexpr double tdoa_std_dev = 0.15;
static constexpr double tdoa_measurement_step = 0.01;
static constexpr int min_anchor_id = 0;
static constexpr int max_anchor_id = 7;

struct Coordinates {
    double x, y, z;
};

class GAZEBO_VISIBLE TdoaPlugin : public ModelPlugin
{
public:
    TdoaPlugin();
    virtual ~TdoaPlugin();

protected:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void OnUpdate(const common::UpdateInfo&);

private:
    void CreatePubsAndSubs();
    double calculateDistance(const ignition::math::Vector3d& tagPosition,
                             const Coordinates& anchorPosition);
    double calculateDistanceDifference(ignition::math::Vector3d& tagPosition_,
                                       const int anchorIdA, const int anchorIdB);
    int getRandomIndex(int excludeIndex);

    std::string namespace_;

    // Array to store 3D coordinates of anchors
    std::array<Coordinates, 8> anchors;

    bool pubs_and_subs_created_;

    bool tdoa_noise_;
    double tdoa_std_dev_;

    physics::ModelPtr model_;
    physics::WorldPtr world_;
    event::ConnectionPtr updateConnection_;

    transport::NodePtr node_handle_;
    transport::PublisherPtr tdoa_pub_;

    gz_sensor_msgs::Tdoa tdoa_msg;

    double tdoa_delay_;
    common::Time last_time_;

    // For the moment we just consider white noise
    ignition::math::Vector3d noise_tdoa_pos_;

    // tdoa noise parameter
    std::default_random_engine rand_;
    std::normal_distribution<float> randn_;

    // Generation of random anchor IDs
    std::mt19937 gen_;
    std::uniform_int_distribution<int> distribution_;

    int anchorIdA, anchorIdB;
};
}
#endif //_GAZEBO_TDOA_PLUGIN_HH_
