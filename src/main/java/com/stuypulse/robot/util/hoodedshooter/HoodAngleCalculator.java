/************************ PROJECT ALPHA *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.hoodedshooter;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.HoodedShooter.AngleInterpolation;
import com.stuypulse.robot.constants.Settings.HoodedShooter.FerryRPMInterpolation;
import com.stuypulse.robot.constants.Settings.HoodedShooter.RPMInterpolation;
import com.stuypulse.robot.subsystems.hoodedshooter.HoodedShooter;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.hoodedshooter.ShotCalculator.AlignAngleSolution;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Supplier;

public class HoodAngleCalculator {
    public static InterpolatingDoubleTreeMap distanceAngleInterpolator;
    public static InterpolatingDoubleTreeMap distanceRPMInterpolator;
    public static InterpolatingDoubleTreeMap ferryingDistanceRPMInterpolator;
 
    static {
        distanceAngleInterpolator = new InterpolatingDoubleTreeMap();
        for (double[] pair : AngleInterpolation.distanceAngleInterpolationValues) {
            distanceAngleInterpolator.put(pair[0], pair[1]);
        }
    }

    static {
        distanceRPMInterpolator = new InterpolatingDoubleTreeMap();
        for (double[] pair : RPMInterpolation.distanceRPMInterpolationValues) {
            distanceRPMInterpolator.put(pair[0], pair[1]);
        }
    }

    static {
        ferryingDistanceRPMInterpolator = new InterpolatingDoubleTreeMap();
        for(double[] pair: FerryRPMInterpolation.distanceRPMInterpolationValues) {
            ferryingDistanceRPMInterpolator.put(pair[0], pair[1]);
        }
    }

    public static Supplier<Rotation2d> interpolateHoodAngle() {
        return () -> {
            CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

            Translation2d hubPose = Field.getHubPose().getTranslation();
            Translation2d currentPose = swerve.getTurretPose().getTranslation();

            double distanceMeters = hubPose.getDistance(currentPose);

            Rotation2d targetAngle = Rotation2d.fromRadians(distanceAngleInterpolator.get(distanceMeters));

            SmartDashboard.putNumber("HoodedShooter/Interpolated Target Angle", targetAngle.getDegrees());

            return targetAngle;
        };
    }

    public static Supplier<Double> interpolateShooterRPM() {
        return () -> {
            CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

            Translation2d hubPose = Field.getHubPose().getTranslation();
            Translation2d currentPose = swerve.getTurretPose().getTranslation();

            double distanceMeters = hubPose.getDistance(currentPose);

            double targetRPM = distanceRPMInterpolator.get(distanceMeters);

            SmartDashboard.putNumber("HoodedShooter/Interpolated RPM", targetRPM);
            
            return targetRPM;
        };
    }

    public static Supplier<Double> interpolateFerryingRPM() {
        return () -> {
            CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

            Translation2d currentPose = swerve.getTurretPose().getTranslation();
            Translation2d cornerPose = Field.getFerryZonePose(currentPose).getTranslation();

            double distanceMeters = cornerPose.getDistance(currentPose);

            double targetRPM = ferryingDistanceRPMInterpolator.get(distanceMeters);

            SmartDashboard.putNumber("HoodedShooter/Interpolated Ferrying RPM", targetRPM);
            
            return targetRPM;
        };
    }

    public static Supplier<Rotation2d> calculateHoodAngleSOTM() {
        return () -> {
            CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
            HoodedShooter hdsr = HoodedShooter.getInstance();
            
            Pose2d currentPose = swerve.getPose();
            Pose2d turretPose = swerve.getTurretPose();
            
            ChassisSpeeds robotRelSpeeds = swerve.getChassisSpeeds();
            ChassisSpeeds fieldRelSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelSpeeds, 
                currentPose.getRotation()
            );
            
            Pose3d targetPose = Field.hubCenter3d;
            Pose3d turretPose3d = new Pose3d(new Translation3d(turretPose.getX(), turretPose.getY(), Settings.Turret.Constants.TURRET_HEIGHT), new Rotation3d());

            double axMetersPerSecondSquared = swerve.getPigeon2().getAccelerationX().getValueAsDouble();
            double ayMetersPerSecondSquared = swerve.getPigeon2().getAccelerationY().getValueAsDouble();
            
            double shooterRPS = hdsr.getTargetRPM() / 60.0;
            
            AlignAngleSolution sol = ShotCalculator.solveShootOnTheFly(
                turretPose3d,
                targetPose,
                axMetersPerSecondSquared,
                ayMetersPerSecondSquared,
                fieldRelSpeeds, // current speeds
                shooterRPS,
                Settings.ShootOnTheFly.MAX_ITERATIONS,
                Settings.ShootOnTheFly.TIME_TOLERANCE
            );
            
            return sol.launchPitchAngle();
        };
    }
}