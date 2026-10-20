#pragma once

#include "grid.hpp"

// Where the robot is: position in meters (map frame) and heading in radians.
// Heading 0 = facing +x (right), counter-clockwise is positive.
struct Pose {
    double x = 0;
    double y = 0;
    double theta = 0;
};

// A velocity command: forward speed (m/s) and turn rate (rad/s).
// The same idea as ROS's geometry_msgs/Twist on the /cmd_vel topic.
struct Twist {
    double linear = 0;
    double angular = 0;
};

// Physical properties, roughly a TurtleBot3-sized robot.
struct DiffDriveConfig {
    double wheelBase = 0.30;     // distance between the two wheels, m
    double radius = 0.18;        // body radius, used for collisions, m
    double maxWheelSpeed = 0.6;  // top speed of each wheel, m/s
    double maxWheelAccel = 1.2;  // motors cannot change speed instantly, m/s^2
};

// Wraps any angle into [-pi, pi).
double normalizeAngle(double a);

// True if a circle of `radius` at `centre` overlaps a wall cell
// (outside the map counts as wall).
bool collides(const Grid& grid, Point centre, double radius);

// A robot with two driven wheels on one axle, steering by speed difference.
class DiffDriveRobot {
public:
    DiffDriveRobot(const DiffDriveConfig& config, Pose start);

    // What we would like the robot to do. Converted to wheel speeds
    // (inverse kinematics); the wheels then ramp towards them.
    void setCommand(Twist cmd);

    // Advances the simulation by dt seconds (forward kinematics).
    // Returns false if the move would hit a wall: the robot then stays put
    // and its wheels stop, like a real bumper switch.
    bool step(double dt, const Grid& grid);

    const Pose& pose() const { return pose_; }
    const DiffDriveConfig& config() const { return config_; }
    double leftSpeed() const { return left_; }
    double rightSpeed() const { return right_; }
    Twist velocity() const;            // actual v and omega right now
    double distanceTravelled() const { return distance_; }  // odometer, m

private:
    DiffDriveConfig config_;
    Pose pose_;
    double targetLeft_ = 0;
    double targetRight_ = 0;
    double left_ = 0;   // current wheel speeds, m/s
    double right_ = 0;
    double distance_ = 0;
};
