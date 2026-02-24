/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.hoodedshooter;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.HoodedShooter.AngleInterpolation;
import com.stuypulse.robot.constants.Settings.HoodedShooter.FerryRPMInterpolation;
import com.stuypulse.robot.constants.Settings.HoodedShooter.RPMInterpolation;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.hoodedshooter.ShotCalculator.SOTMSolution;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Supplier;

public class HoodAngleCalculator {

    public static SOTMSolution sol;

    private static FieldObject2d hubPose2d;
    private static FieldObject2d virtualHubPose2d;
    private static FieldObject2d futureTurretPose2d;

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

            Pose2d hubPose = Field.getHubPose();
            Pose2d turretPose = swerve.getTurretPose();

            double targetRPM = ShotCalculator.solveInterpolation(turretPose, hubPose).targetRPM();

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

    public static void updateSOTMSolution() {

        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
        
        Pose2d robotPose = swerve.getPose();
        Pose2d hubPose = Field.getHubPose();
        
        ChassisSpeeds robotRelativeSpeeds = swerve.getChassisSpeeds();
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds, 
            robotPose.getRotation()
        );

        Pose2d futureTurretPose = swerve.getTurretPose().exp(
            new Twist2d(
                robotRelativeSpeeds.vxMetersPerSecond * Settings.ShootOnTheFly.UPDATE_DELAY.doubleValue(),
                robotRelativeSpeeds.vyMetersPerSecond * Settings.ShootOnTheFly.UPDATE_DELAY.doubleValue(),
                0
            )
        );
        

        SOTMSolution solution = ShotCalculator.solveShootOnTheMove(
            futureTurretPose,
            robotPose,
            hubPose,
            fieldRelativeSpeeds,
            Settings.ShootOnTheFly.MAX_ITERATIONS,
            Settings.ShootOnTheFly.TIME_TOLERANCE
        );

        sol = solution;

        hubPose2d.setPose(Robot.isBlue() ? hubPose : Field.transformToOppositeAlliance(hubPose));
        virtualHubPose2d.setPose((Robot.isBlue() ? sol.virtualPose() : Field.transformToOppositeAlliance(sol.virtualPose())));
        futureTurretPose2d.setPose((Robot.isBlue() ? futureTurretPose : Field.transformToOppositeAlliance(futureTurretPose)));
  
  
        SmartDashboard.putNumber("HoodedShooter/SOTM/calculated turret angle", sol.targetTurretAngle().getDegrees());
        SmartDashboard.putNumber("HoodedShooter/SOTM/calculated hood angle", sol.targetHoodAngle().getDegrees());
        SmartDashboard.putNumber("HoodedShooter/SOTM/calculated flight time", sol.flightTime());
        SmartDashboard.putNumber("HoodedShooter/SOTM/turret dist to virtual pose", futureTurretPose.getTranslation().getDistance(sol.virtualPose().getTranslation()));
    }

    public static Supplier<Rotation2d> calculateHoodAngleSOTM() {
        return () -> sol.targetHoodAngle();
    }

    public static Supplier<Rotation2d> calculateTurretAngleSOTM() {
        return () -> sol.targetTurretAngle();
    }
}