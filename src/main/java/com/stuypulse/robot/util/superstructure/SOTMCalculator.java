/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.superstructure;


import com.ctre.phoenix6.hardware.Pigeon2;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.hood.Hood;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.superstructure.InterpolationCalculator.InterpolatedFerryInfo;
import com.stuypulse.robot.util.superstructure.InterpolationCalculator.InterpolatedShotInfo;
import com.stuypulse.stuylib.network.SmartBoolean;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

public class SOTMCalculator {
    public static SmartBoolean accountForAccel = new SmartBoolean("Superstructure/SOTM/account for accel", false);

    public static final double g = 9.81;

    public static MoveSolution hubSol;
    public static MoveSolution ferrySol;

    private static FieldObject2d hubPose2d;
    private static FieldObject2d virtualHubPose2d;
    private static FieldObject2d futureTurretPose2d;

    private static FieldObject2d ferryPose2d;
    private static FieldObject2d virtualFerryPose2d;

    public record MoveSolution(
        Rotation2d targetHoodAngle,
        Rotation2d targetTurretAngle,
        double targetShooterRPM,
        Pose2d virtualPose,
        double flightTime) {
    }

    static {
        hubSol = new MoveSolution(
            Hood.getInstance().getAngle(),
            Turret.getInstance().getAngle(),
            Shooter.getInstance().getRPM(), 
            Field.getHubPose(), 
            0.0
        );

        ferrySol = new MoveSolution(
            Hood.getInstance().getAngle(),
            Turret.getInstance().getAngle(),
            Shooter.getInstance().getRPM(), 
            Field.getFerryZonePose(CommandSwerveDrivetrain.getInstance().getPose().getTranslation()), 
            0.0
        );

        hubPose2d = Field.FIELD2D.getObject("hubPose");
        virtualHubPose2d = Field.FIELD2D.getObject("virtualHubPose");

        ferryPose2d = Field.FIELD2D.getObject("ferryPose");
        virtualFerryPose2d = Field.FIELD2D.getObject("virtualFerryPose");

        futureTurretPose2d = Field.FIELD2D.getObject("futureTurretPose");
    }


    public static MoveSolution solveSOTM(
        Pose2d turretPose,
        Pose2d targetPose,
        Rotation2d robotHeading,
        double vTurretX,
        double vTurretY,
        int maxIterations,
        double timeTolerance) {

        /*
        Start with v_ball * flightTime = distanceToTargetPose.
        
        We know that v_ball = v_robot + v_shooter, so 
        (v_robot + v_shooter) * flightTime = distanceToTargetPose

        Rearranging, we can get
        (v_shooter) * flight_time = distanceToTargetPose - v_robot * flightTime

        So we can instead shoot at a virtual pose and treat the robot as stationary:
        distanceToVirtualPose = distanceToTargetPose - v_robot * flightTime
        (v_shooter) * flight_time = distanceToVirtualPose

        Looking at the first equation, we can find the virtual pose with the flight time, 
        but looking at the second equation, to get the flight time we need to solveBallisticWithSpeed()
        using the virtual pose, so we have a circular dependence.

        Thus, we can make an initial guess for the flight time: the flight time if the robot were stationary
        We want our guess to converge such that the left side equals the right side:
        (v_shooter) * t_guess = distanceToVirtualPose = distance - v_robot * t_guess, which would make t_guess = flightTime

        We do the right side first using our inital guess, and then update t_guess with a new guess by 
        calculating the flightTime to that virtualPose.

        The pose is that the flightTime converges within maxIterations.
        */
        

        InterpolatedShotInfo sol = InterpolationCalculator.interpolateShotInfo(turretPose, targetPose);
        InterpolatedShotInfo newSol;
        
        double t_guess = sol.flightTimeSeconds();
        
        Pose2d virtualPose = targetPose;

             
        for (int i = 0; i < maxIterations; i++) {

            // SmartDashboard.putNumber("Superstructure/SOTM/iteration #", i);

            double dx = vTurretX * t_guess;
            double dy = vTurretY * t_guess;

            virtualPose = new Pose2d(
                targetPose.getX() - dx,
                targetPose.getY() - dy,
                targetPose.getRotation());

            newSol = InterpolationCalculator.interpolateShotInfo(turretPose, virtualPose);

            if (Math.abs(newSol.flightTimeSeconds() - t_guess) < timeTolerance) {
                break;
            }

            t_guess = newSol.flightTimeSeconds();

            sol = newSol;

        }
        
        Translation2d virtualTranslation = virtualPose.getTranslation();
        Translation2d turretTranslation = turretPose.getTranslation();

        // double yaw = Math.atan2(
        //     virtualTranslation.getY() - turretTranslation.getY(),
        //     virtualTranslation.getX() - turretTranslation.getX() 
        // );
        
        // Rotation2d targetTurretAngle = Robot.isReal() ? 
        //     Rotation2d.fromRadians(-yaw).plus(robotPose.getRotation()) :
        //     Rotation2d.fromRadians(yaw).minus(robotPose.getRotation());

        return new MoveSolution(
            sol.targetHoodAngle(),
            TurretAngleCalculator.getPointAtTargetAngle(virtualTranslation, turretTranslation, robotHeading),
            sol.targetRPM(),
            virtualPose,
            sol.flightTimeSeconds()
        );
    }

    public static MoveSolution solveFOTM(
        Pose2d turretPose,
        Pose2d targetPose,
        Rotation2d robotHeading,
        double vTurretX,
        double vTurretY,
        int maxIterations,
        double timeTolerance) {
            
        InterpolatedFerryInfo sol = InterpolationCalculator.interpolateFerryingInfo(turretPose, targetPose);

        
        double t_guess = sol.flightTimeSeconds();
        
        Pose2d virtualPose = targetPose;

             
        for (int i = 0; i < maxIterations; i++) {

            // SmartDashboard.putNumber("Superstructure/FOTM/iteration #", i);

            double dx = vTurretX * t_guess;
            double dy = vTurretY * t_guess;

            virtualPose = new Pose2d(
                targetPose.getX() - dx,
                targetPose.getY() - dy,
                targetPose.getRotation());

  
            InterpolatedFerryInfo newSol = InterpolationCalculator.interpolateFerryingInfo(turretPose, virtualPose);

            if (Math.abs(newSol.flightTimeSeconds() - t_guess) < timeTolerance) {
                break;
            }

            t_guess = newSol.flightTimeSeconds();

            sol = newSol;

        }

        if (virtualPose.getY() >= Field.WIDTH) {
            virtualPose = new Pose2d(virtualPose.getX(), Field.WIDTH, virtualPose.getRotation());
        }

        else if (virtualPose.getY() <= 0) {
            virtualPose = new Pose2d(virtualPose.getX(), 0, virtualPose.getRotation());
        }

        Translation2d virtualTranslation = virtualPose.getTranslation();
        Translation2d turretTranslation = turretPose.getTranslation();

        return new MoveSolution(
            sol.targetHoodAngle(),
            TurretAngleCalculator.getPointAtTargetAngle(virtualTranslation, turretTranslation, robotHeading),
            sol.targetRPM(),
            virtualPose,
            sol.flightTimeSeconds()
        );
    }

    public static void updateSOTMSolution() {

        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
        
        Pose2d turretPose = swerve.getTurretPose();
        Pose2d robotPose = swerve.getPose();
        Pose2d hubPose = Field.getHubPose();
        
        ChassisSpeeds robotRelativeSpeeds = swerve.getChassisSpeeds();
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds, 
            robotPose.getRotation()
        );
        
        // Pigeon2 pigeon = swerve.getPigeon2();
        // double ax = pigeon.getAccelerationX().getValueAsDouble() * g;
        // double ay = pigeon.getAccelerationY().getValueAsDouble() * g;

        Transform2d robotToTurret = turretPose.minus(robotPose);
        double omega = robotRelativeSpeeds.omegaRadiansPerSecond;

        /*
        There is a delay dt between the target turret angle and the actual turret angle. That is, by the time the
        turret reaches its target angle, the robot would already be at a future position and rotation.
        To account for this, we simply use the robot's pose and rotation dt time in the future instead of the current robot pose and rotation
        to calculate the turret angle, shooter rpm, and hood angle.
        That way, when we reach tolerance and fire at the future pose and rotation, the parameters will be correct.
        */ 

        double t = Settings.Superstructure.SOTM.UPDATE_DELAY.doubleValue();

        Pose2d futureRobotPose = robotPose.exp(
            new Twist2d (
                robotRelativeSpeeds.vxMetersPerSecond * t,
                robotRelativeSpeeds.vyMetersPerSecond * t,
                omega * t
            )
        );

        // if (accountForAccel.get()) {
        //     futureRobotPose = robotPose.exp(
        //         new Twist2d (
        //             robotRelativeSpeeds.vxMetersPerSecond * t + 0.5 * ax * t*t,
        //             robotRelativeSpeeds.vyMetersPerSecond * t + 0.5 * ay * t*t,
        //             omega * t
        //         )
        //     );
        // }

        Pose2d futureTurretPose = futureRobotPose.transformBy(robotToTurret);


        Translation2d r = turretPose.getTranslation().minus(robotPose.getTranslation());
        // not only does the ball exit with the xy velocity of the turret, but also the tangential velocity from the robot's rotation (r * omega)
        double vTurretX = fieldRelativeSpeeds.vxMetersPerSecond - omega * r.getY();
        double vTurretY = fieldRelativeSpeeds.vyMetersPerSecond + omega * r.getX();

        // if (accountForAccel.get()) {
        //     Translation2d fieldAccel = new Translation2d(ax, ay)
        //         .rotateBy(robotPose.getRotation());
                
        //     vTurretX += fieldAccel.getX() * t;
        //     vTurretY += fieldAccel.getY() * t;
        // }


        /*
        this part simply shifts where we're aiming from the hub's center to the corresponding rim of the hub
        based on our field relative robot velocity.
        i.e. if we are moving up torwards the hub, we shift our aim down to the bottom rim of the hub.
        */
        
        // Vector2D oppositeDirection = new Vector2D(new Translation2d(
        //     -fieldRelativeSpeeds.vxMetersPerSecond,
        //     -fieldRelativeSpeeds.vyMetersPerSecond
        // ));

        // if (oppositeDirection.magnitude() < Settings.Swerve.MODULE_VELOCITY_DEADBAND_M_PER_S) {
        //     oppositeDirection = Vector2D.kOrigin;
        // }
        // else {
        //     oppositeDirection = oppositeDirection.normalize();
        // }

        // hubPose = hubPose.plus(
        //     new Transform2d(
        //         oppositeDirection.x * Field.HUB_RADIUS,
        //         oppositeDirection.y * Field.HUB_RADIUS,
        //         Rotation2d.kZero
        //     )
        // );

        hubSol = solveSOTM(
            futureTurretPose,
            hubPose,
            futureRobotPose.getRotation(),
            vTurretX,
            vTurretY,
            Settings.Superstructure.SOTM.MAX_ITERATIONS,
            Settings.Superstructure.SOTM.TIME_TOLERANCE
        );

        hubPose2d.setPose(Robot.isBlue() ? hubPose : Field.transformToOppositeAlliance(hubPose));
        virtualHubPose2d.setPose((Robot.isBlue() ? hubSol.virtualPose() : Field.transformToOppositeAlliance(hubSol.virtualPose())));
        futureTurretPose2d.setPose((Robot.isBlue() ? futureTurretPose : Field.transformToOppositeAlliance(futureTurretPose)));
  
  
        // SmartDashboard.putNumber("Superstructure/SOTM/calculated turret angle", hubSol.targetTurretAngle().getDegrees());
        // SmartDashboard.putNumber("Superstructure/SOTM/calculated hood angle", hubSol.targetHoodAngle().getDegrees());
        // SmartDashboard.putNumber("Superstructure/SOTM/calculated flight time", hubSol.flightTime());
        
        DogLog.log("Superstructure/SOTM/virtual pose", virtualHubPose2d.getPose());
        DogLog.log("Superstructure/SOTM/turret dist to virtual pose", futureTurretPose.getTranslation().getDistance(hubSol.virtualPose().getTranslation()));
    }

    public static void updateFOTMSolution() {
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
        
        Pose2d turretPose = swerve.getTurretPose();
        Pose2d robotPose = swerve.getPose();
        Pose2d ferryPose = Field.getFerryZonePose(turretPose.getTranslation());
        
        ChassisSpeeds robotRelativeSpeeds = swerve.getChassisSpeeds();
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds, 
            robotPose.getRotation()
        );

        double omega = robotRelativeSpeeds.omegaRadiansPerSecond;
        double t = Settings.Superstructure.SOTM.UPDATE_DELAY.doubleValue();

        Pose2d futureRobotPose = robotPose.exp(
            new Twist2d (
                robotRelativeSpeeds.vxMetersPerSecond * t,
                robotRelativeSpeeds.vyMetersPerSecond * t,
                omega * t
            )
        );

        Transform2d robotToTurret = turretPose.minus(robotPose);
        Pose2d futureTurretPose = futureRobotPose.transformBy(robotToTurret);


        Translation2d r = turretPose.getTranslation().minus(robotPose.getTranslation());

        double vTurretX = fieldRelativeSpeeds.vxMetersPerSecond - omega * r.getY();
        double vTurretY = fieldRelativeSpeeds.vyMetersPerSecond + omega * r.getX();

        ferrySol = solveFOTM(
            futureTurretPose,
            ferryPose,
            futureRobotPose.getRotation(),
            vTurretX,
            vTurretY,
            Settings.Superstructure.SOTM.MAX_ITERATIONS,
            Settings.Superstructure.SOTM.TIME_TOLERANCE
        );



        ferryPose2d.setPose(Robot.isBlue() ? ferryPose : Field.transformToOppositeAlliance(ferryPose));
        virtualFerryPose2d.setPose((Robot.isBlue() ? ferrySol.virtualPose() : Field.transformToOppositeAlliance(ferrySol.virtualPose())));
        futureTurretPose2d.setPose((Robot.isBlue() ? futureTurretPose : Field.transformToOppositeAlliance(futureTurretPose)));

        // SmartDashboard.putNumber("Superstructure/FOTM/calculated turret angle", ferrySol.targetTurretAngle().getDegrees());
        // SmartDashboard.putNumber("Superstructure/FOTM/calculated hood angle", ferrySol.targetHoodAngle().getDegrees());
        // SmartDashboard.putNumber("Superstructure/FOTM/calculated flight time", ferrySol.flightTime());
        // SmartDashboard.putNumber("Superstructure/FOTM/turret dist to ferry pose", futureTurretPose.getTranslation().getDistance(ferrySol.virtualPose().getTranslation()));
    }

    public static Rotation2d calculateHoodAngleSOTM() {
        return hubSol.targetHoodAngle();
    }
    
    public static Rotation2d calculateTurretAngleSOTM() {
        return hubSol.targetTurretAngle();
    }
    
    public static double calculateShooterRPMSOTM() {
        return hubSol.targetShooterRPM();
    }

    public static Rotation2d calculateHoodAngleFOTM() {
        //TODO: don't forget to change this back to the solution!!
        // return ferrySol.targetHoodAngle();
        // return Rotation2d.fromDegrees(40);
        return Settings.Superstructure.Hood.Angles.FERRY_ANGLE;
    }
    
    public static Rotation2d calculateTurretAngleFOTM() {
        return ferrySol.targetTurretAngle();
    }
    
    public static double calculateShooterRPMFOTM() {
        return ferrySol.targetShooterRPM();
    }
}