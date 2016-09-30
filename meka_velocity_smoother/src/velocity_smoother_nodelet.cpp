/**
 * @file /src/velocity_smoother_nodelet.cpp
 *
 * @brief Velocity smoother implementation.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_velocity_smoother/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <dynamic_reconfigure/server.h>
#include <meka_velocity_smoother/paramsConfig.h>

#include "meka_velocity_smoother/velocity_smoother_nodelet.hpp"

#include <thread>

#include <ReflexxesAPI.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>


/*****************************************************************************
 ** Preprocessing
 *****************************************************************************/

#define PERIOD_RECORD_SIZE    5
#define ZERO_VEL_COMMAND      geometry_msgs::Twist();
#define IS_ZERO_VEOCITY(a)   ((a.linear.x == 0.0) && (a.angular.z == 0.0))

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace meka_velocity_smoother {

/*********************
 ** Implementation
 **********************/

VelocitySmoother::VelocitySmoother(const std::string &name) :
        name(name), quiet(false), shutdown_req(false), input_active(false), pr_next(0), dynamic_reconfigure_server(
                NULL), last_acc_vx(0.0), last_acc_vy(0.0), last_acc_w(0.0) {
};

void VelocitySmoother::reconfigCB(meka_velocity_smoother::paramsConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure request : %f %f %f %f %f %f %f",
            config.speed_lim_v, config.speed_lim_w, config.accel_lim_v, config.accel_lim_w,
            config.jerk_lim_v, config.jerk_lim_w, config.decel_factor);

    speed_lim_v = config.speed_lim_v;
    speed_lim_w = config.speed_lim_w;
    accel_lim_v = config.accel_lim_v;
    accel_lim_w = config.accel_lim_w;
    jerk_lim_v = config.jerk_lim_v;
    jerk_lim_w = config.jerk_lim_w;
    decel_factor = config.decel_factor;
}

void VelocitySmoother::velocityCB(const geometry_msgs::Twist::ConstPtr& msg) {
    // Estimate commands frequency; we do continuously as it can be very different depending on the
    // publisher type, and we don't want to impose extra constraints to keep this package flexible
    if (period_record.size() < PERIOD_RECORD_SIZE) {
        period_record.push_back((ros::Time::now() - last_cb_time).toSec());
    } else {
        period_record[pr_next] = (ros::Time::now() - last_cb_time).toSec();
    }

    pr_next++;
    pr_next %= period_record.size();
    last_cb_time = ros::Time::now();

    if (period_record.size() <= PERIOD_RECORD_SIZE / 2) {
        // wait until we have some values; make a reasonable assumption (10 Hz) meanwhile
        cb_avg_time = 0.1;
    } else {
        // enough; recalculate with the latest input
        cb_avg_time = median(period_record);
    }

    input_active = true;

    // Bound speed with the maximum values
    target_vel.linear.x =
            msg->linear.x > 0.0 ? std::min(msg->linear.x, speed_lim_v) : std::max(msg->linear.x, -speed_lim_v);
    target_vel.linear.y =
            msg->linear.y > 0.0 ? std::min(msg->linear.y, speed_lim_v) : std::max(msg->linear.y, -speed_lim_v);
    target_vel.angular.z =
            msg->angular.z > 0.0 ? std::min(msg->angular.z, speed_lim_w) : std::max(msg->angular.z, -speed_lim_w);
}

void VelocitySmoother::odometryCB(const nav_msgs::Odometry::ConstPtr& msg) {
    if (robot_feedback == ODOMETRY)
        current_vel = msg->twist.twist;

    // ignore otherwise
}

void VelocitySmoother::robotVelCB(const geometry_msgs::Twist::ConstPtr& msg) {
    if (robot_feedback == COMMANDS)
        current_vel = *msg;

    // ignore otherwise
}

void VelocitySmoother::spin() {
        double period = 1.0 / frequency;
        ros::Rate spin_rate(frequency);
        int dof = 3;

        // create Reflexxes API for <dof> DOF actuator
        ReflexxesAPI * reflexxes_api = new ReflexxesAPI(dof, period);
        RMLPositionInputParameters * reflexxes_position_input = new RMLPositionInputParameters(dof);
        RMLPositionOutputParameters * reflexxes_position_output = new RMLPositionOutputParameters(dof);
        RMLPositionFlags reflexxes_motion_flags;

        while (!shutdown_req && ros::ok()) {
            if ((input_active == true) && (cb_avg_time > 0.0)
                    && ((ros::Time::now() - last_cb_time).toSec() > std::min(3.0 * cb_avg_time, 0.5))) {
                // Velocity input no active anymore; normally last command is a zero-velocity one, but reassure
                // this, just in case something went wrong with our input, or he just forgot good manners...
                // Issue #2, extra check in case cb_avg_time is very big, for example with several atomic commands
                // The cb_avg_time > 0 check is required to deal with low-rate simulated time, that can make that
                // several messages arrive with the same time and so lead to a zero median
                input_active = false;
                if (IS_ZERO_VEOCITY(target_vel) == false) {
                    ROS_WARN_STREAM(
                            "Velocity Smoother : input got inactive leaving us a non-zero target velocity (" << target_vel.linear.x << ", " << target_vel.linear.y << ", " << target_vel.angular.z << "), zeroing...[" << name << "]");
                    target_vel = ZERO_VEL_COMMAND;
                }
            }

            geometry_msgs::TwistPtr cmd_vel;
            cmd_vel.reset(new geometry_msgs::Twist(target_vel));

            reflexxes_position_input->TargetPositionVector->VecData[0] = target_vel.linear.x;
            reflexxes_position_input->SelectionVector->VecData[0] = true;
            reflexxes_position_input->MaxVelocityVector->VecData[0] = target_vel.linear.x != 0 ? accel_lim_v : accel_lim_v * decel_factor;
            reflexxes_position_input->MaxAccelerationVector->VecData[0] = target_vel.linear.x != 0 ? jerk_lim_v : jerk_lim_v * decel_factor;
            reflexxes_position_input->TargetVelocityVector->VecData[0] = 0.0;

            reflexxes_position_input->TargetPositionVector->VecData[1] = target_vel.linear.y;
            reflexxes_position_input->SelectionVector->VecData[1] = true;
            reflexxes_position_input->MaxVelocityVector->VecData[1] = target_vel.linear.y != 0 ? accel_lim_v : accel_lim_v * decel_factor;
            reflexxes_position_input->MaxAccelerationVector->VecData[1] = target_vel.linear.y != 0 ? jerk_lim_v : jerk_lim_v * decel_factor;
            reflexxes_position_input->TargetVelocityVector->VecData[1] = 0.0;

            reflexxes_position_input->TargetPositionVector->VecData[2] = target_vel.angular.z;
            reflexxes_position_input->SelectionVector->VecData[2] = true;
            reflexxes_position_input->MaxVelocityVector->VecData[2] = target_vel.angular.z != 0 ? accel_lim_w : accel_lim_w * decel_factor;
            reflexxes_position_input->MaxAccelerationVector->VecData[2] = target_vel.angular.z != 0 ? jerk_lim_w : jerk_lim_w * decel_factor;
            reflexxes_position_input->TargetVelocityVector->VecData[2] = 0.0;


            int res = reflexxes_api->RMLPosition(*reflexxes_position_input,
                    reflexxes_position_output, reflexxes_motion_flags);

            if (res < 0) {
                if (res == ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES) {
                    printf("> ReflexxesMotionGenerator --> RML_ERROR_INVALID_INPUT_VALUES error\n");
                } else {
                    printf("> ReflexxesMotionGenerator --> UNKNOWN_ERROR: reflexxes error %d\n", res);
                }
            }

            // feed back values
            for (int i = 0; i < dof; i++) {
                reflexxes_position_input->CurrentPositionVector->VecData[i] =
                        reflexxes_position_output->NewPositionVector->VecData[i];

                reflexxes_position_input->CurrentVelocityVector->VecData[i] =
                        reflexxes_position_output->NewVelocityVector->VecData[i];

                reflexxes_position_input->CurrentAccelerationVector->VecData[i] =
                        reflexxes_position_output->NewAccelerationVector->VecData[i];
            }

            cmd_vel->linear.x = reflexxes_position_output->NewPositionVector->VecData[0];
            cmd_vel->linear.y = reflexxes_position_output->NewPositionVector->VecData[1];
            cmd_vel->angular.z = reflexxes_position_output->NewPositionVector->VecData[2];

            smooth_vel_pub.publish(cmd_vel);

            spin_rate.sleep();
    }
}

/**
 * Initialise from a nodelet's private nodehandle.
 * @param nh : private nodehandle
 * @return bool : success or failure
 */
bool VelocitySmoother::init(ros::NodeHandle& nh) {
    // Dynamic Reconfigure
    dynamic_reconfigure_callback = boost::bind(&VelocitySmoother::reconfigCB, this, _1, _2);

    dynamic_reconfigure_server = new dynamic_reconfigure::Server<meka_velocity_smoother::paramsConfig>(nh);
    dynamic_reconfigure_server->setCallback(dynamic_reconfigure_callback);

    // Optional parameters
    int feedback;
    nh.param("frequency", frequency, 20.0);
    nh.param("quiet", quiet, quiet);
    nh.param("decel_factor", decel_factor, 1.0);
    nh.param("robot_feedback", feedback, (int) NONE);

    if ((int(feedback) < NONE) || (int(feedback) > COMMANDS)) {
        ROS_WARN("Invalid robot feedback type (%d). Valid options are 0 (NONE, default), 1 (ODOMETRY) and 2 (COMMANDS)",
                feedback);
        feedback = NONE;
    }

    robot_feedback = static_cast<RobotFeedbackType>(feedback);

    // Mandatory parameters
    if ((nh.getParam("speed_lim_v", speed_lim_v) == false) || (nh.getParam("speed_lim_w", speed_lim_w) == false)) {
        ROS_ERROR("Missing velocity limit parameter(s)");
        return false;
    }

    if ((nh.getParam("accel_lim_v", accel_lim_v) == false) || (nh.getParam("accel_lim_w", accel_lim_w) == false)) {
        ROS_ERROR("Missing acceleration limit parameter(s)");
        return false;
    }

    if ((nh.getParam("jerk_lim_v", jerk_lim_v) == false) || (nh.getParam("jerk_lim_w", jerk_lim_w) == false)) {
        ROS_ERROR("Missing jerk limit parameter(s)");
        return false;
    }

    // Publishers and subscribers
    odometry_sub = nh.subscribe("odometry", 1, &VelocitySmoother::odometryCB, this);
    current_vel_sub = nh.subscribe("robot_cmd_vel", 1, &VelocitySmoother::robotVelCB, this);
    raw_in_vel_sub = nh.subscribe("raw_cmd_vel", 1, &VelocitySmoother::velocityCB, this);
    smooth_vel_pub = nh.advertise<geometry_msgs::Twist>("smooth_cmd_vel", 1);

    return true;
}

/*********************
 ** Nodelet
 **********************/

class VelocitySmootherNodelet: public nodelet::Nodelet {
public:
    VelocitySmootherNodelet() {
    }
    ~VelocitySmootherNodelet() {
        NODELET_DEBUG("Velocity Smoother : waiting for worker thread to finish...");
        vel_smoother_->shutdown();
        worker_thread_.join();
    }

    std::string unresolvedName(const std::string &name) const {
        size_t pos = name.find_last_of('/');
        return name.substr(pos + 1);
    }

    virtual void onInit() {
        ros::NodeHandle ph = getPrivateNodeHandle();
        std::string resolved_name = ph.getUnresolvedNamespace(); // this always returns like /robosem/goo_arm - why not unresolved?
        std::string name = unresolvedName(resolved_name); // unresolve it ourselves
        NODELET_DEBUG_STREAM("Velocity Smoother : initialising nodelet...[" << name << "]");
        vel_smoother_.reset(new VelocitySmoother(name));
        if (vel_smoother_->init(ph)) {
            NODELET_DEBUG_STREAM("Velocity Smoother : nodelet initialised [" << name << "]");
            worker_thread_ = std::thread(&VelocitySmoother::spin, vel_smoother_);
        } else {
            NODELET_ERROR_STREAM("Velocity Smoother : nodelet initialisation failed [" << name << "]");
        }
    }

private:
    boost::shared_ptr<VelocitySmoother> vel_smoother_;
    std::thread worker_thread_;
};

} // namespace meka_velocity_smoother

PLUGINLIB_EXPORT_CLASS(meka_velocity_smoother::VelocitySmootherNodelet, nodelet::Nodelet);
