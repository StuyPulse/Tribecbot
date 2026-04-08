package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;

public class SwerveResetPoseLeftCorner extends SwerveResetPose {
    public SwerveResetPoseLeftCorner() {
        super(new Pose2d(0, Field.WIDTH, CommandSwerveDrivetrain.getInstance().getPose().getRotation()));
    }    
}
