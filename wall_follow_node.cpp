#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>
#include <algorithm>
class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // TODO: create ROS subscribers and publishers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "drive", 10);
        
    }

private:
    // PID CONTROL PARAMS
    double kp = 1.0;
    double kd = 0.005;
    double ki = 0.1;
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;
    double deadzone = 0.05; // error smaller = on target, empty integral
    double i_limit = 0.1; // most steering I can ever add

    // Scan Info
    double angle_min_ = 0.0;
    double angle_max_ = 0.0;
    double angle_increment_ = 0.0;
    double range_min_ = 0.0;
    double range_max_ = 0.0;
    double desired_dist = 1.0; // dist from wall
    double lookahead_distance = 1.0; // lookahead
    double max_steer = 0.4189;
    int num_ranges_ = 0;

    // Time Step
    double prev_time_ = 0.0;
    double dt_ = 0.0;
    bool have_prev_time_ = false;

    // Doorway Detection
    double corner_front_dist = 4.0; // wall closer than straight ahead = corner coming
    double doorway_error = 0.12; // error bigger = sudden jump
    double lookahead_angle = 20.0 * std::acos(-1.0) / 180.0; // look-ahead beam, 20 degrees left
    double continue_tol = 0.25; // how close (m) counts as same wall


    

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;



    double get_range(const float* range_data, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        // angle must be inside lidar sweep
        if (angle < angle_min_ || angle > angle_max_) {
            return -1.0;
        }
        
        // index calcs
        int index = static_cast<int>(std::round((angle - angle_min_) / (angle_increment_)));

        if (index < 0 || index >= num_ranges_) { // filter bad indexes
            return -1.0;
        }
        float ranges_data = range_data[index];
        
        // return reading, filtering nan/inf
        if (!std::isfinite(ranges_data) || ranges_data < range_min_ || ranges_data > range_max_) {
            return -1.0;
        }
        return ranges_data;
    }

    double get_error(const float* range_data, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        // TODO:implement
        
        // construct angle a & b
        double theta = std::acos(-1.0) / 4;
        double angle_b  = std::acos(-1.0) / 2;
        double angle_a = angle_b - theta;
        

        // find distances
        double b = get_range(range_data, angle_b);
        double a = get_range(range_data, angle_a);
        
        // discard garbage distances
        if ( b < 0 || a < 0) {
            return 0.0;
        }

        // look straight ahead, and ahead-left along the wall
        double front = get_range(range_data, 0.0);
        double r_ahead = get_range(range_data, lookahead_angle);
        double sideways_ahead = r_ahead * std::sin(lookahead_angle);

        // doorway checks
        bool wall_ahead = (front > 0.0 && front < corner_front_dist);
        bool wall_continues = (r_ahead > 0.0 && std::abs(sideways_ahead - dist) <continue_tol);
        
        // define angle alpha
        double alpha = std::atan((a * std::cos(theta) - b) / (a * std::sin(theta)));

        // create distances now, future, and lookahead distance (L)
        double L = lookahead_distance; // lookahead distance
        double dist_now = b * std::cos(alpha);
        double dist_future = dist_now + L * std::sin(alpha);

        // compute error based on distance from wall

        double error = dist_future - dist; // left wall: too close --> negative error --> steer right (away)

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, // repeating print 
            "front: %.2f r_ahead: %.2f, sideways: %.2f error: %.2f ahead: %d continues: %d integral: %.2f dt: %.2f",
            front, r_ahead, sideways_ahead, error, wall_ahead, wall_continues, integral, dt_);

        // doorway: left side jumped, no wall ahead, but wall continues -> ignore
        if (error > doorway_error && !wall_ahead && wall_continues) {
            return 0.0;
        }

        return error;
    }

    void pid_control(double error, double velocity)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
        double angle = 0.0;

        // TODO: Use kp, ki & kd to implement a PID controller
        
        // related parameters, per second (not scan)
        double rate = 0.0;
        if (dt_ > 0.0) {
            rate = (error - prev_error) / dt_;
            integral = integral + error * dt_;
        }

        // windup guard 1: on target -> forget old error
        if (std::abs(error) < deadzone) {
            integral = 0.0;
        }

        // windup guard 2: cap the integral so ki never passes i_limit
        if (ki >0.0) {
            double integral_max = i_limit / ki;
            if (integral > integral_max) {
                integral = integral_max;
            } else if (integral < -integral_max) {
                integral = -integral_max;
            }
        }

        // angle = kp x error + ki x integral + kd x rate
        angle = kp * error + ki * integral + kd * rate;

        // limit steering angle
        if (angle > max_steer) {
            angle = max_steer;
        } else if (angle < -max_steer) {
            angle = -max_steer;
        }

        // save scan's error for next time
        prev_error = error;
        
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // TODO: fill in drive message and publish
        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.speed = velocity;
        drive_pub_->publish(drive_msg); // publish
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {

        angle_min_ = scan_msg->angle_min;
        angle_max_ = scan_msg->angle_max;
        angle_increment_ = scan_msg->angle_increment;
        range_min_ = scan_msg->range_min;
        range_max_ = scan_msg->range_max;
        num_ranges_ = scan_msg->ranges.size();
        double now = rclcpp::Time(scan_msg->header.stamp).seconds(); // grab time now

        // check time
        if (have_prev_time_ == true) {
            dt_ = now - prev_time_;
            prev_time_ = now; // set prev_time_
        } else if (have_prev_time_ == false) {
            prev_time_ = now;
            have_prev_time_ = true;
        }


        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.
        
        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        double error = get_error(scan_msg->ranges.data(), desired_dist); 
        double velocity = 0.0; 
        
        double est_rate = 0.0;
        if (dt_ > 0.0) {
            est_rate = (error - prev_error) / dt_;
        }
        double est_angle = kp * error + ki * integral + kd * est_rate;

        if (std::abs(est_angle) < 10 * std::acos(-1.0) / 180.0) { // < 10 degrees
            velocity = 3.0;
        } else if (std::abs(est_angle) < 20 * std::acos(-1.0) / 180.0) { // < 20 degrees
            velocity = 2.0;
        } else {
            velocity = 1.0; // > 20 degrees
        }
        // TODO: actuate the car with PID
        pid_control(error, velocity);


    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}