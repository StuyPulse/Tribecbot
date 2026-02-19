/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.stuylib.network.SmartBoolean;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.vision.LimelightHelpers;
import com.stuypulse.robot.util.vision.LimelightHelpers.PoseEstimate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightVision extends SubsystemBase{

    private static final LimelightVision instance;

    static {
        instance = new LimelightVision();
    }

    public static LimelightVision getInstance() {
        return instance;
    }

    private String[] names;
    private SmartBoolean enabled;
    private SmartBoolean[] camerasEnabled;
    private MegaTagMode megaTagMode;

    public enum MegaTagMode {
        MEGATAG1,
        MEGATAG2 
    }
    private int maxTagCount;

    public LimelightVision() {
        names = new String[Cameras.LimelightCameras.length];
        for (int i = 0; i < Cameras.LimelightCameras.length; i++) {
            names[i] = Cameras.LimelightCameras[i].getName();
            Pose3d robotRelativePose = Cameras.LimelightCameras[i].getLocation();
            LimelightHelpers.setCameraPose_RobotSpace(
                names[i], 
                robotRelativePose.getX(), 
                robotRelativePose.getY(), 
                robotRelativePose.getZ(), 
                Rotation2d.fromRadians(robotRelativePose.getRotation().getX()).getDegrees(), 
                Rotation2d.fromRadians(robotRelativePose.getRotation().getY()).getDegrees(), 
                Rotation2d.fromRadians(robotRelativePose.getRotation().getZ()).getDegrees()
            );
        }

        camerasEnabled = new SmartBoolean[Cameras.LimelightCameras.length];
        
        for (int i = 0; i < camerasEnabled.length; i++) {
            camerasEnabled[i] = new SmartBoolean("Vision/" + names[i] + " Is Enabled", true);
            LimelightHelpers.SetIMUMode(names[i], 0);
            SmartDashboard.putBoolean("Vision/" + names[i] + " Has Data", false);
        }

        enabled = new SmartBoolean("Vision/Is Enabled", true);
        megaTagMode = MegaTagMode.MEGATAG1;

        maxTagCount = 0;
    }

    public void setTagWhitelist(int... ids) {
        for (String name : names) {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, ids);
        }
    }

    public void enable() {
        enabled.set(true);
    }

    public void disable() {
        enabled.set(false);
    }

    public void setCameraEnabled(String name, boolean enabled) {
        for (int i = 0; i < names.length; i++) {
            if (names[i].equals(name)) {
                camerasEnabled[i].set(enabled);
            }
        }
    }

    public void setMegaTagMode(MegaTagMode mode) {
        this.megaTagMode = mode;
    }

    public void setIMUMode(int mode) {
        for (String name : names) {
            LimelightHelpers.SetIMUMode(name, mode);
        }
    }

    public int getMaxTagCount() {
        return this.maxTagCount;
    }

    @Override
    public void periodic() {
        this.maxTagCount = 0;

        if (enabled.get()) {
            for (int i = 0; i < names.length; i++) {
                if (camerasEnabled[i].get()) {
                    String limelightName = names[i];

                    // Seed robot heading (used by MT2)
                    LimelightHelpers.SetRobotOrientation(
                        limelightName, 
                        (CommandSwerveDrivetrain.getInstance().getPose().getRotation().getDegrees() + (Robot.isBlue() ? 0 : 180)) % 360, 
                        0, 
                        0, 
                        0, 
                        0, 
                        0
                    );


                    PoseEstimate poseEstimate;

                    // MegaTag switching
                    if (megaTagMode == MegaTagMode.MEGATAG1) {
                        poseEstimate = Robot.isBlue() 
                            ? LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName)
                            : LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName);
                    } else {
                        poseEstimate = Robot.isBlue() 
                            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName)
                            : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
                    }
                    
                    // Adding to pose estimator
                    if (poseEstimate != null && poseEstimate.tagCount > 0) {
                        Pose2d robotPose = poseEstimate.pose;
                        double timestamp = poseEstimate.timestampSeconds;

                        CommandSwerveDrivetrain.getInstance().addVisionMeasurement(robotPose, timestamp, Settings.Vision.MT1_STDEVS);

                        SmartDashboard.putNumber("Vision/Pose X Component", robotPose.getX());
                        SmartDashboard.putNumber("Vision/Pose Y Component", robotPose.getY());
                        SmartDashboard.putNumber("Vision/Pose Theta (Degrees)", robotPose.getRotation().getDegrees());

                        SmartDashboard.putBoolean("Vision/" + names[i] + " Has Data", true);

                        maxTagCount = Math.max(maxTagCount, poseEstimate.tagCount);
                    }
                    else {
                        SmartDashboard.putBoolean("Vision/" + names[i] + " Has Data", false);
                    }

                    SmartDashboard.putString("Vision/MegaTag Mode", megaTagMode.toString());
                }
            }
        }
    }
}