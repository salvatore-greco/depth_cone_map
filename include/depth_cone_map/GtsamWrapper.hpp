#ifndef GTSAMWRAPPER_HPP
#define GTSAMWRAPPER_HPP

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/BearingRange.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "Cone.hpp"

class GtsamWrapper{
    public:
        GtsamWrapper();
        void savePose(gtsam::Pose3 pose);
        void saveValue(const Cone* cone, int cone_idx);
        void saveBearingRangeFactor(const Cone* cone, int cone_idx);
        gtsam::Values updateAndCalculate();
    private:
        gtsam::NonlinearFactorGraph new_factors;
        gtsam::Values new_values;
        gtsam::Pose3 last_pose;
        gtsam::Pose3 current_pose;
        gtsam::noiseModel::Diagonal::shared_ptr prior_noise;
        gtsam::noiseModel::Diagonal::shared_ptr odom_noise;
        gtsam::noiseModel::Robust::shared_ptr landmark_noise;
        gtsam::ISAM2 isam2;
        int pose_idx;
        bool first_pose;

        gtsam::Symbol getCurrentPoseKey(){
            return gtsam::Symbol('p', pose_idx);
        }
};

#endif
