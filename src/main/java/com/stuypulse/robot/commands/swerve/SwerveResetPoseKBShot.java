package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SwerveResetPoseKBShot extends SwerveResetPose {
    public SwerveResetPoseKBShot() {
        super(new Pose2d(
            Field.HUB_CENTER.getX() - Field.HUB_RADIUS - Units.inchesToMeters(23.5), 
            Field.WIDTH / 2 - Units.inchesToMeters(6.5), 
            Rotation2d.kZero));     
    } 
}
