package com.stuypulse.robot.util.superstructure;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings.Superstructure.AngleInterpolation;
import com.stuypulse.robot.constants.Settings.Superstructure.FerryRPMInterpolation;
import com.stuypulse.robot.constants.Settings.Superstructure.RPMInterpolation;
import com.stuypulse.robot.constants.Settings.Superstructure.TOFInterpolation;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class InterpolationCalculator {

    public static InterpolatingDoubleTreeMap distanceAngleInterpolator;
    public static InterpolatingDoubleTreeMap distanceRPMInterpolator;
    public static InterpolatingDoubleTreeMap distanceTOFInterpolator;

    public static InterpolatingDoubleTreeMap ferryingDistanceRPMInterpolator;

    static {
        distanceAngleInterpolator = new InterpolatingDoubleTreeMap();
        for (double[] pair : AngleInterpolation.distanceAngleInterpolationValues) {
            distanceAngleInterpolator.put(pair[0], pair[1]);
        }

        distanceRPMInterpolator = new InterpolatingDoubleTreeMap();
        for (double[] pair : RPMInterpolation.distanceRPMInterpolationValues) {
            distanceRPMInterpolator.put(pair[0], pair[1]);
        }

        distanceTOFInterpolator = new InterpolatingDoubleTreeMap();
        for (double[] pair : TOFInterpolation.distanceTOFInterpolationValues) {
            distanceTOFInterpolator.put(pair[0], pair[1]);
        }

        ferryingDistanceRPMInterpolator = new InterpolatingDoubleTreeMap();
        for(double[] pair: FerryRPMInterpolation.distanceRPMInterpolationValues) {
            ferryingDistanceRPMInterpolator.put(pair[0], pair[1]);
        }
    }
    
    
    public record InterpolatedShotInfo(
        Rotation2d targetHoodAngle,
        double targetRPM,
        double flightTimeSeconds) {
    }

    public static InterpolatedShotInfo interpolateShotInfo(){
        return interpolateShotInfo(Field.getHubPose());
    }

    public static InterpolatedShotInfo interpolateShotInfo(Pose2d targetPose) {
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        Translation2d hubPose = targetPose.getTranslation();
        Translation2d turretPose = swerve.getTurretPose().getTranslation();

        double distanceMeters = turretPose.getDistance(hubPose);

        Rotation2d targetAngle = Rotation2d.fromRadians(distanceAngleInterpolator.get(distanceMeters));
        double targetRPM = distanceRPMInterpolator.get(distanceMeters);
        double flightTime = distanceTOFInterpolator.get(distanceMeters);
        

        SmartDashboard.putNumber("SuperStructure/Interpolated Target Angle", targetAngle.getDegrees());
        SmartDashboard.putNumber("SuperStructure/Interpolated RPM", targetRPM);
        SmartDashboard.putNumber("SuperStructure/Interpolated TOF", flightTime);

        return new InterpolatedShotInfo(
            targetAngle, 
            targetRPM, 
            flightTime
        );
    }

    public static Supplier<Double> interpolateFerryingRPM() {
        return () -> {
            CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

            Translation2d currentPose = swerve.getTurretPose().getTranslation();
            Translation2d cornerPose = Field.getFerryZonePose(currentPose).getTranslation();

            double distanceMeters = cornerPose.getDistance(currentPose);

            double targetRPM = ferryingDistanceRPMInterpolator.get(distanceMeters);

            SmartDashboard.putNumber("SuperStructure/Interpolated Ferrying RPM", targetRPM);
            
            return targetRPM;
        };
    }

    
}