#include "depth_cone_map/GtsamWrapper.hpp"

GtsamWrapper::GtsamWrapper() : pose_idx(0), first_pose(true){

    odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 0.002, 0.002, 0.002,  // Incertezza rotazionale per passo
                                  0.02,  0.02,  0.02   // Incertezza traslazionale per passo
            ).finished()
    );

    // infinitesima perchè deve essere stabile
    prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6,   // Rotazione roll, pitch, yaw (rad)
                                  1e-6, 1e-6, 1e-6   // Traslazione x, y, z (metri)
            ).finished()
    );
    landmark_noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.345),
            gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.05, 0.05, 0.3))
    );
}

void GtsamWrapper::savePose(gtsam::Pose3 pose){
    current_pose = pose;
    if (first_pose){
        auto key = getCurrentPoseKey();
        new_factors.addPrior(key, pose, this->prior_noise);
        new_values.insert(key, pose);
        last_pose = pose;
        first_pose = false;
    }
    else {
        pose_idx++;
        auto key = getCurrentPoseKey();
        gtsam::Pose3 relative_pose = last_pose.between(pose);
        new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            gtsam::Symbol('p', pose_idx - 1),
            key,
            relative_pose,
            this->odom_noise
        );
        new_values.insert(key, pose);
        last_pose = pose;
    }
}

void GtsamWrapper::saveValue(const Cone* cone, int cone_idx){
    new_values.insert(
        gtsam::Symbol('l', cone_idx),
        gtsam::Point3(cone->position_world_frame.x, cone->position_world_frame.y, cone->position_world_frame.z)
    );
}

void GtsamWrapper::saveBearingRangeFactor(const Cone* cone, int cone_idx){
    gtsam::Point3 cone_point(cone->position_world_frame.x, cone->position_world_frame.y, cone->position_world_frame.z);
    double range = current_pose.range(cone_point);
    gtsam::Unit3 bearing = current_pose.bearing(cone_point);
    new_factors.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3>>(
        getCurrentPoseKey(),
        gtsam::Symbol('l', cone_idx),
        bearing,
        range,
        this->landmark_noise
    );
}

gtsam::Values GtsamWrapper::updateAndCalculate(){
    isam2.update(new_factors, new_values);
    new_factors.resize(0);
    new_values.clear();
    return isam2.calculateEstimate();
}
