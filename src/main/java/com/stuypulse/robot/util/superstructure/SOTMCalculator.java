package com.stuypulse.robot.util.superstructure;

import java.util.function.Supplier;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.hood.Hood;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret;
import com.stuypulse.robot.util.superstructure.InterpolationCalculator.InterpolatedShotInfo;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SOTMCalculator {
    public static final double g = 9.81;

    public static SOTMSolution sol;

    private static FieldObject2d hubPose2d;
    private static FieldObject2d virtualHubPose2d;
    private static FieldObject2d futureTurretPose2d;

    public record SOTMSolution(
        Rotation2d targetHoodAngle,
        Rotation2d targetTurretAngle,
        double targetShooterRPM,
        Pose2d virtualPose,
        double flightTime) {
    }

    static {
        sol = new SOTMSolution(
            Hood.getInstance().getAngle(),
            Turret.getInstance().getAngle(),
            Shooter.getInstance().getRPM(), 
            Field.getHubPose(), 
            0.0
        );

        hubPose2d = Field.FIELD2D.getObject("hubPose");
        virtualHubPose2d = Field.FIELD2D.getObject("virtualHubPose");
        futureTurretPose2d = Field.FIELD2D.getObject("futureTurretPose");
    }


    public static SOTMSolution solveShootOnTheMove(
        Pose2d turretPose,
        Pose2d robotPose,
        Pose2d targetPose,
        ChassisSpeeds fieldRelativeSpeeds,
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
        

        InterpolatedShotInfo sol = InterpolationCalculator.interpolateShotInfo();

        
        double t_guess = sol.flightTimeSeconds();
        
        Pose2d virtualPose = targetPose;

             
        for (int i = 0; i < maxIterations; i++) {

            SmartDashboard.putNumber("SuperStructure/SOTM/iteration #", i);

            double dx = fieldRelativeSpeeds.vxMetersPerSecond * t_guess;
            double dy = fieldRelativeSpeeds.vyMetersPerSecond * t_guess;

            virtualPose = new Pose2d(
                targetPose.getX() - dx,
                targetPose.getY() - dy,
                targetPose.getRotation());

  
            InterpolatedShotInfo newSol = InterpolationCalculator.interpolateShotInfo(virtualPose);

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

        return new SOTMSolution(
            sol.targetHoodAngle(),
            TurretAngleCalculator.getPointAtTargetAngle(virtualTranslation, turretTranslation),
            sol.targetRPM(),
            virtualPose,
            sol.flightTimeSeconds()
        );
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
                robotRelativeSpeeds.vxMetersPerSecond * Settings.Superstructure.SOTM.UPDATE_DELAY.doubleValue(),
                robotRelativeSpeeds.vyMetersPerSecond * Settings.Superstructure.SOTM.UPDATE_DELAY.doubleValue(),
                0
            )
        );

        Vector2D oppositeDirection = new Vector2D(new Translation2d(
            -robotRelativeSpeeds.vxMetersPerSecond,
            -robotRelativeSpeeds.vyMetersPerSecond
        ));

        if (!oppositeDirection.equals(Vector2D.kOrigin)) {
            oppositeDirection = oppositeDirection.normalize();
        }

        hubPose = hubPose.exp(
            new Twist2d(
                oppositeDirection.x * Field.HUB_RADIUS,
                oppositeDirection.y * Field.HUB_RADIUS,
                0
            )
        );
        

        SOTMSolution solution = solveShootOnTheMove(
            futureTurretPose,
            robotPose,
            hubPose,
            fieldRelativeSpeeds,
            Settings.Superstructure.SOTM.MAX_ITERATIONS,
            Settings.Superstructure.SOTM.TIME_TOLERANCE
        );

        sol = solution;

        hubPose2d.setPose(Robot.isBlue() ? hubPose : Field.transformToOppositeAlliance(hubPose));
        virtualHubPose2d.setPose((Robot.isBlue() ? sol.virtualPose() : Field.transformToOppositeAlliance(sol.virtualPose())));
        futureTurretPose2d.setPose((Robot.isBlue() ? futureTurretPose : Field.transformToOppositeAlliance(futureTurretPose)));
  
  
        SmartDashboard.putNumber("SuperStructure/SOTM/calculated turret angle", sol.targetTurretAngle().getDegrees());
        SmartDashboard.putNumber("SuperStructure/SOTM/calculated hood angle", sol.targetHoodAngle().getDegrees());
        SmartDashboard.putNumber("SuperStructure/SOTM/calculated flight time", sol.flightTime());
        SmartDashboard.putNumber("SuperStructure/SOTM/turret dist to virtual pose", futureTurretPose.getTranslation().getDistance(sol.virtualPose().getTranslation()));
    }

    public static Rotation2d calculateHoodAngleSOTM() {
        return sol.targetHoodAngle();
    }
    
    public static Rotation2d calculateTurretAngleSOTM() {
        return sol.targetTurretAngle();
    }
    
    public static Double calculateShooterRPMSOTM() {
        return sol.targetShooterRPM();
    }
}