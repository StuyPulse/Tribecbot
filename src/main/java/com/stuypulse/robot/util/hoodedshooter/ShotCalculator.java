/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.hoodedshooter;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.HoodedShooter.AngleInterpolation;
import com.stuypulse.robot.constants.Settings.HoodedShooter.RPMInterpolation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class ShotCalculator {
    public static final double g = 9.81;

    public static InterpolatingDoubleTreeMap distanceAngleInterpolator;
    public static InterpolatingDoubleTreeMap distanceRPMInterpolator;

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

    public record StationarySolution(
        Rotation2d targetHoodAngle,
        double targetRPM,
        double flightTimeSeconds) {
    }

    public static StationarySolution solveInterpolation(Pose2d turretPose, Pose2d targetPose) {
        
        double distanceMeters = turretPose.getTranslation().getDistance(targetPose.getTranslation());

        // Interpolated Angle
        Rotation2d targetHoodAngle = Rotation2d.fromRadians(distanceAngleInterpolator.get(distanceMeters));

        // Interpolated RPM
        double targetRPM = distanceRPMInterpolator.get(distanceMeters);

        // Physics-based TOF
        double launchSpeed = 0.5 * targetRPM * (2 * Math.PI / 60.0) * Settings.HoodedShooter.Shooter.FLYWHEEL_RADIUS; 
        Rotation2d launchAngle = Rotation2d.kCCW_Pi_2.minus(targetHoodAngle);

        double v_x = launchSpeed * Math.cos(launchAngle.getRadians());
        double flightTime = distanceMeters / v_x;
        
        return new StationarySolution(
            targetHoodAngle,
            targetRPM,
            flightTime
        );
    }

    public record SOTMSolution(
        Rotation2d targetHoodAngle,
        Rotation2d targetTurretAngle,
        Pose2d virtualPose,
        double flightTime) {
    }

    public static SOTMSolution solveShootOnTheMove(
        Pose2d turretPose,
        Pose2d robotPose,
        Pose2d targetPose,
        ChassisSpeeds fieldRelativeSpeeds,
        int maxIterations,
        double timeTolerance) {

        /*
         *   Start with v_ball * flightTime = distanceToTargetPose.
         *    
         *   We know that v_ball = v_robot + v_shooter, so 
         *   (v_robot + v_shooter) * flightTime = distanceToTargetPose
         *    
         *   Rearranging, we can get
         *   (v_shooter) * flight_time = distanceToTargetPose - v_robot * flightTime
         * 
         *   So we can instead shoot at a virtual pose and treat the robot as stationary:
         *   distanceToVirtualPose = distanceToTargetPose - v_robot * flightTime
         *   (v_shooter) * flight_time = distanceToVirtualPose
         * 
         *   Looking at the first equation, we can find the virtual pose with the flight time, 
         *   but looking at the second equation, to get the flight time we need to solveBallisticWithSpeed()
         *   using the virtual pose, so we have a circular dependence.
         * 
         *   Thus, we can make an initial guess for the flight time: the flight time if the robot were stationary
         *   We want our guess to converge such that the left side equals the right side:
         *   (v_shooter) * t_guess = distanceToVirtualPose = distance - v_robot * t_guess, which would make t_guess = flightTime
         * 
         *   We do the right side first using our inital guess, and then update t_guess with a new guess by 
         *   calculating the flightTime to that virtualPose.
         * 
         *   The pose is that the flightTime converges within maxIterations.
         */

        StationarySolution sol = solveInterpolation(
            turretPose,
            targetPose
        );

        double t_guess = sol.flightTimeSeconds();
        
        Pose2d virtualPose = targetPose;

             
        for (int i = 0; i < maxIterations; i++) {

            double dx = fieldRelativeSpeeds.vxMetersPerSecond * t_guess;
            double dy = fieldRelativeSpeeds.vyMetersPerSecond * t_guess;

            virtualPose = new Pose2d(
                targetPose.getX() - dx,
                targetPose.getY() - dy,
                targetPose.getRotation());

            StationarySolution newSol = solveInterpolation(
                turretPose,
                virtualPose
            );

            if (Math.abs(newSol.flightTimeSeconds() - t_guess) < timeTolerance) {
                break;
            }

            t_guess = newSol.flightTimeSeconds();

            sol = newSol;
        }
        
        Translation2d virtualTranslation = virtualPose.getTranslation();
        Translation2d turretTranslation = turretPose.getTranslation();

        double yaw = Math.atan2(
            virtualTranslation.getY() - turretTranslation.getY(),
            virtualTranslation.getX() - turretTranslation.getX() 
        ); 

        Rotation2d targetTurretAngle = Robot.isReal() ? 
            Rotation2d.fromRadians(-yaw).plus(robotPose.getRotation()) :
            Rotation2d.fromRadians(yaw).minus(robotPose.getRotation());

        return new SOTMSolution(
            sol.targetHoodAngle(),
            targetTurretAngle,
            virtualPose,
            sol.flightTimeSeconds()
        );
    }
}