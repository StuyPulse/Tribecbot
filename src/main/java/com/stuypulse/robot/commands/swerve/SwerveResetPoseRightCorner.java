package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;

public class SwerveResetPoseRightCorner extends SwerveResetPose {
    public SwerveResetPoseRightCorner() {
        super(new Pose2d(0, 0, CommandSwerveDrivetrain.getInstance().getPose().getRotation()));
    }    
}
