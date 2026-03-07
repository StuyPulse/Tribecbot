package com.stuypulse.robot.util.superstructure;

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

    public record InterpolatedShotInfo(
        Rotation2d targetHoodAngle,
        double targetRPM,
        double flightTimeSeconds) {
    }


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
        for(double[] pair: FerryRPMInterpolation.ferryDistanceRPMInterpolation) {
            ferryingDistanceRPMInterpolator.put(pair[0], pair[1]);
        }
    }
    
    public static InterpolatedShotInfo interpolateShotInfo(){
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        return interpolateShotInfo(swerve.getTurretPose(), Field.getHubPose());
    }


    public static InterpolatedShotInfo interpolateShotInfo(Pose2d turretPose, Pose2d targetPose) {
        Translation2d hubPose = targetPose.getTranslation();
        Translation2d currentPose = turretPose.getTranslation();

        double distanceMeters = currentPose.getDistance(hubPose);

        Rotation2d targetAngle = Rotation2d.fromRadians(distanceAngleInterpolator.get(distanceMeters));
        double targetRPM = distanceRPMInterpolator.get(distanceMeters);
        double flightTime = distanceTOFInterpolator.get(distanceMeters);
        

        SmartDashboard.putNumber("InterpolationTesting/Interpolated Target Angle", targetAngle.getDegrees());
        SmartDashboard.putNumber("InterpolationTesting/Interpolated RPM", targetRPM);
        SmartDashboard.putNumber("InterpolationTesting/Interpolated TOF", flightTime);

        return new InterpolatedShotInfo(
            targetAngle, 
            targetRPM, 
            flightTime
        );
    }

    public static double interpolateFerryingRPM() {
        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        Translation2d currentPose = swerve.getTurretPose().getTranslation();
        Translation2d cornerPose = Field.getFerryZonePose(currentPose).getTranslation();

        double distanceMeters = cornerPose.getDistance(currentPose);

        double targetRPM = ferryingDistanceRPMInterpolator.get(distanceMeters);

        SmartDashboard.putNumber("Superstructure/Interpolated Ferrying RPM", targetRPM);
        
        return targetRPM;
    }

    
}