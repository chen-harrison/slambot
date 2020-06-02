#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>


ActionModel::ActionModel(void)
: alpha1_(0.08f)
, alpha2_(0.0008f)
, alpha3_(0.005f)
, alpha4_(0.0002f)
, initialized_(false)
, numberGenerator_(rd_())
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if(!initialized_)
    {
        previousOdometry_ = odometry;
        initialized_ = true;
    }

    float deltaX = odometry.x - previousOdometry_.x;
    float deltaY = odometry.y - previousOdometry_.y;
    dRot1_ = angle_diff(std::atan2(deltaY, deltaX), previousOdometry_.theta);
    dTrans_ = std::sqrt(std::pow(deltaX, 2) + std::pow(deltaY, 2));

    if(std::abs(dTrans_) <= 0.001 && std::abs(angle_diff(odometry.theta, previousOdometry_.theta)) <= 0.001)
    {
        return false;
    }
    else if(std::abs(dTrans_) <= 0.001)
    {
        dRot1_ = 0.0;
    }
    else if(std::abs(dRot1_) > M_PI_2)
    {
        dRot1_ = angle_diff(dRot1_, M_PI);
        dTrans_ *= -1.0;
    }

    dRot2_ = wrap_to_pi(odometry.theta - previousOdometry_.theta - dRot1_);
    
    stdRot1_ = std::sqrt(alpha1_*std::pow(dRot1_, 2) + alpha2_*std::pow(dTrans_, 2));
    stdTrans_ = std::sqrt(alpha3_*std::pow(dTrans_, 2) + alpha4_*std::pow(dRot1_, 2) + alpha4_*std::pow(dRot2_, 2));
    stdRot2_ = std::sqrt(alpha1_*std::pow(dRot2_, 2) + alpha2_*std::pow(dTrans_, 2));

    // need to set previousOdometry_ to current one before finishing? + update utime_
    previousOdometry_ = odometry;
    utime_ = odometry.utime;

    return true;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

    double sampledRot1 = std::normal_distribution<>(dRot1_, stdRot1_)(numberGenerator_);
    double sampledTrans= std::normal_distribution<>(dTrans_, stdTrans_)(numberGenerator_);
    double sampledRot2 = std::normal_distribution<>(dRot2_, stdRot2_)(numberGenerator_);

    particle_t newSample = sample;
    newSample.parent_pose = sample.pose;
    newSample.pose.utime = utime_;

    newSample.pose.x += sampledTrans * std::cos(sample.pose.theta + sampledRot1);
    newSample.pose.y += sampledTrans * std::sin(sample.pose.theta + sampledRot1);
    newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampledRot1 + sampledRot2);

    return newSample;
}
