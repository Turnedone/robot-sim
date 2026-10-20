#pragma once

#include "robot.hpp"

// Dead reckoning: the robot's own estimate of where it is, from the wheel
// encoders alone. It integrates the measured wheel speeds with exactly the
// same kinematics as the robot, so without slip it matches the truth; with
// slip every small error adds up and the estimate drifts away for ever.
class Odometry {
public:
    Odometry(double wheelBase, const Pose& start) : wheelBase_(wheelBase), pose_(start) {}

    // Call once per control step with what the encoders report.
    void update(double leftSpeed, double rightSpeed, double dt) {
        pose_ = integrateArc(pose_, (leftSpeed + rightSpeed) / 2, (rightSpeed - leftSpeed) / wheelBase_, dt);
    }

    const Pose& pose() const { return pose_; }
    void reset(const Pose& pose) { pose_ = pose; }

private:
    double wheelBase_;
    Pose pose_;
};
