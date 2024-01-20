/*
 * Copyright 2024 Thomas Izycki, THA, Augsburg
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

#include "gazebo_tdoa_plugin.h"
#include "ConnectGazeboToRosTopic.pb.h"


namespace gazebo {
    GZ_REGISTER_MODEL_PLUGIN(TdoaPlugin)

    TdoaPlugin::TdoaPlugin() : ModelPlugin()
    ,anchors {{
        // Example setup for anchors
        {-3.5, -4.5, 0.2},
        {-3.5,  4.5, 3.0},
        { 3.5,  4.5, 0.2},
        { 3.5, -4.5, 3.0},
        {-3.5, -4.5, 3.0},
        {-3.5,  4.5, 0.2},
        { 3.5,  4.5, 3.0},
        { 3.5, -4.5, 0.2}
    }}
    ,tdoa_std_dev_(tdoa_std_dev)
    ,tdoa_delay_(tdoa_measurement_step)
    ,gen_(std::random_device{}())
    ,distribution_(min_anchor_id, max_anchor_id)
    {}

    TdoaPlugin::~TdoaPlugin()
    {
        updateConnection_->~Connection();
    }

    void TdoaPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        model_ = _model;
        world_ = model_->GetWorld();

        if (_sdf->HasElement("tdoaStdDev")){
            getSdfParam<double>(_sdf, "tdoaStdDev", tdoa_std_dev_, 0);
            tdoa_noise_ = tdoa_std_dev_ > 0;
        } else {
            tdoa_noise_ = false;
        }
        namespace_.clear();
        if (_sdf->HasElement("robotNamespace")){
            namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        } else {
            gzerr << "[gazebo_tdoa_plugin] Please specify a robotNamespace.\n";
        }

        node_handle_ = transport::NodePtr(new transport::Node());
        node_handle_->Init(namespace_);

        // Listen to the update event. This event is broadcast every simulation iteration.
        updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&TdoaPlugin::OnUpdate, this, _1));

        // Populate static part of the tdoa message
        tdoa_msg.mutable_header()->set_frame_id("tdoa_link");

        last_time_ = world_->SimTime();

        pubs_and_subs_created_ = false;

        anchorIdA = getRandomIndex(-1);
        anchorIdB = getRandomIndex(anchorIdA);
    }

    void TdoaPlugin::OnUpdate(const common::UpdateInfo&)
    {
        if (!pubs_and_subs_created_){
            CreatePubsAndSubs();
            pubs_and_subs_created_ =  true;
        }

        common::Time current_time = world_->SimTime();

        double dt = (current_time - last_time_).Double();

        if (dt < tdoa_delay_)
            return;

        ignition::math::Pose3d T_W_I = model_->WorldPose();

        ignition::math::Vector3d& pos_W_I = T_W_I.Pos();

        anchorIdA = getRandomIndex(-1);
        anchorIdB = getRandomIndex(anchorIdA);

        tdoa_msg.mutable_header()->mutable_stamp()->set_sec(current_time.sec);
        tdoa_msg.mutable_header()->mutable_stamp()->set_nsec(current_time.nsec);

        gazebo::msgs::Vector3d* anchor_coord_A = new gazebo::msgs::Vector3d();
        anchor_coord_A->set_x(anchors[anchorIdA].x);
        anchor_coord_A->set_y(anchors[anchorIdA].y);
        anchor_coord_A->set_z(anchors[anchorIdA].z);
        tdoa_msg.set_allocated_anchor_position_a(anchor_coord_A);

        gazebo::msgs::Vector3d* anchor_coord_B = new gazebo::msgs::Vector3d();
        anchor_coord_B->set_x(anchors[anchorIdB].x);
        anchor_coord_B->set_y(anchors[anchorIdB].y);
        anchor_coord_B->set_z(anchors[anchorIdB].z);

        tdoa_msg.set_allocated_anchor_position_b(anchor_coord_B);

        double distDiff = calculateDistanceDifference(pos_W_I, anchorIdA, anchorIdB);
        double distDiffWithNoise = distDiff + tdoa_std_dev_ * randn_(rand_);
        tdoa_msg.set_distance_difference(distDiffWithNoise);
        tdoa_msg.set_anchor_id_a(anchorIdA);
        tdoa_msg.set_anchor_id_b(anchorIdB);

        // std::cout << namespace_ << " Distance diff: " << distDiff << std::endl;

        tdoa_pub_->Publish(tdoa_msg);
        last_time_ =  current_time;
    }

    // Function to calculate the Euclidean distance between two 3D points
    double TdoaPlugin::calculateDistance(const ignition::math::Vector3d& tagPosition, const Coordinates& anchorPosition)
    {
        return std::sqrt(
            std::pow(tagPosition.X() - anchorPosition.x, 2) +
            std::pow(tagPosition.Y() - anchorPosition.y, 2) +
            std::pow(tagPosition.Z() - anchorPosition.z, 2)
        );
    }

    // Function to calculate the difference between distances to two anchors
    double TdoaPlugin::calculateDistanceDifference(ignition::math::Vector3d& tagPosition_, const int anchorIdA, const int anchorIdB)
    {
        double distanceToAnchorA = calculateDistance(tagPosition_, anchors[anchorIdA]);
        double distanceToAnchorB = calculateDistance(tagPosition_, anchors[anchorIdB]);
        return distanceToAnchorB - distanceToAnchorA;
    }

    void TdoaPlugin::CreatePubsAndSubs()
    {
        tdoa_pub_ = node_handle_->Advertise<gz_sensor_msgs::Tdoa>(namespace_ + "/gazebo/tdoa", 1);
        // std::cout << "Created TDOA PUB " << namespace_ + "/gazebo/tdoa" << std::endl;
    }

    int TdoaPlugin::getRandomIndex(int excludeIndex)
    {
        int randomIndex;
        do {
            randomIndex = distribution_(gen_);
        } while (randomIndex == excludeIndex);

        return randomIndex;
    }

}
