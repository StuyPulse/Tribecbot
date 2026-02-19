/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.hoodedshooter;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.HoodedShooter.AngleInterpolation;
import com.stuypulse.robot.subsystems.hoodedshooter.HoodedShooter;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.hoodedshooter.ShotCalculator.AlignAngleSolution;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Supplier;

public class HoodAngleCalculator {
    public static InterpolatingDoubleTreeMap distanceAngleInterpolator;

    static {
        distanceAngleInterpolator = new InterpolatingDoubleTreeMap();
        for (double[] pair : AngleInterpolation.distanceAngleInterpolationValues) {
            distanceAngleInterpolator.put(pair[0], pair[1]);
        }
    }

    public static Supplier<Rotation2d> interpolateHoodAngle() {
        return () -> {
            CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

            Translation2d hubPose = Field.getHubPose().getTranslation();
            Translation2d currentPose = swerve.getPose().getTranslation();

            double distanceMeters = hubPose.getDistance(currentPose);

            SmartDashboard.putNumber("HoodedShooter/Distance to Hub", distanceMeters);

            return Rotation2d.fromRadians(distanceAngleInterpolator.get(distanceMeters));
        };
    }

    public static Supplier<Rotation2d> calculateHoodAngleSOTM() {
        return () -> {
            CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
            HoodedShooter hdsr = HoodedShooter.getInstance();
            
            Pose2d currentPose = swerve.getPose();
            
            ChassisSpeeds robotRelSpeeds = swerve.getChassisSpeeds();
            ChassisSpeeds fieldRelSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelSpeeds, 
                currentPose.getRotation()
            );
            
            Pose3d targetPose = Field.hubCenter3d;
            double axMetersPerSecondSquared = swerve.getPigeon2().getAccelerationX().getValueAsDouble();
            double ayMetersPerSecondSquared = swerve.getPigeon2().getAccelerationY().getValueAsDouble();
            
            double shooterRPS = hdsr.getTargetRPM() / 60.0;
            
            AlignAngleSolution sol = ShotCalculator.solveShootOnTheFly(
                new Pose3d(currentPose.plus(Constants.Turret.TURRET_OFFSET)),
                targetPose,
                axMetersPerSecondSquared,
                ayMetersPerSecondSquared,
                fieldRelSpeeds, // current speeds
                shooterRPS,
                Constants.Align.MAX_ITERATIONS,
                Constants.Align.TIME_TOLERANCE
            );
            
            return sol.launchPitchAngle();
        };
    }
}