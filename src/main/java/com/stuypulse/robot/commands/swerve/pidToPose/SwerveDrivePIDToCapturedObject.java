/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.swerve.pidToPose;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveDrivePIDToCapturedObject extends SwerveDrivePIDToPose {
    Pose2d pose;

    public SwerveDrivePIDToCapturedObject(Supplier<Pose2d> targetPose) {
        super(targetPose);
    }

    public SwerveDrivePIDToCapturedObject withTolerance(double x, double y, Rotation2d theta) {
        super.withTolerance(x, y, theta);
        return this;
    }

    public SwerveDrivePIDToCapturedObject withTranslationalConstraints(double maxVelocity, double maxAcceleration) {
        super.withTranslationalConstraints(maxVelocity, maxAcceleration);
        return this;
    }

    public SwerveDrivePIDToCapturedObject withoutMotionProfile() {
        super.withoutMotionProfile();
        return this;
    }

    public SwerveDrivePIDToCapturedObject withCanEnd(Supplier<Boolean> canEnd) {
        super.withCanEnd(canEnd);
        return this;
    }
}