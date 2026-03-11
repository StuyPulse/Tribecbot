/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.vision.LimelightHelpers;
import com.stuypulse.robot.util.vision.LimelightHelpers.IMUData;
import com.stuypulse.robot.util.vision.LimelightHelpers.PoseEstimate;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
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
    // private SmartBoolean[] camerasEnabled;
    private MegaTagMode megaTagMode;
    private Pose2d[] arrayOfLimelightPoses;
    private final StructArrayPublisher<Pose2d> arrayPublisher;

    public enum MegaTagMode {
        MEGATAG1,
        MEGATAG2 
    }

    public LimelightVision() {
        arrayPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();

        arrayOfLimelightPoses = new Pose2d[Cameras.LimelightCameras.length];
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

            arrayOfLimelightPoses[i] = new Pose2d();
        }

        // camerasEnabled = new SmartBoolean[Cameras.LimelightCameras.length];
        
        // for (int i = 0; i < camerasEnabled.length; i++) {
        //     camerasEnabled[i] = new SmartBoolean("Vision/" + names[i] + " Is Enabled", true);
        //     LimelightHelpers.SetIMUMode(names[i], Settings.Vision.INTERNAL_EXTERNAL_ASSIST_INDEX);
        //     SmartDashboard.putBoolean("Vision/" + names[i] + " Has Data", false);
        // }

        enabled = new SmartBoolean("Vision/Is Enabled", true);
        megaTagMode = MegaTagMode.MEGATAG1;
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

    public void setMegaTagMode(MegaTagMode mode) {
        this.megaTagMode = mode;
    }

    public void setIMUMode(int mode) {
        for (String name : names) {
            LimelightHelpers.SetIMUMode(name, mode);
        }
    }

    /**
     * Allows you to set the convergence speed of the internal LL IMU and robot gyro. 
     *
     * @param assistValue, an double that sets the correction speed of the complementary filter for the IMU. IMU Mode 4 
     * uses the fusing of the internal IMU (1khz) with the external gyro reading as well. Higher values ranging towards 1 
     * indicate a faster convergence of internal IMU to the robot IMU mode. Defaults to 0.001.
     */
    public void setIMUAssistValue(double assistValue) {
        for (String name : names) {
            LimelightHelpers.SetIMUAssistAlpha(name, assistValue);
        }
    }

    public IMUData[] getIMUData() {
        IMUData[] data = new IMUData[Cameras.LimelightCameras.length];

        for (int i = 0; i < Cameras.LimelightCameras.length; i++) {
            data[i] = LimelightHelpers.getIMUData(Cameras.LimelightCameras[i].getName());
        }

        return data;
    }
    

    @Override
    public void periodic() {
        if (enabled.get()) {
            for (int i = 0; i < names.length; i++) {
                if (Cameras.LimelightCameras[i].isEnabled()) {
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
                        
                        if (megaTagMode == MegaTagMode.MEGATAG1) {
                            CommandSwerveDrivetrain.getInstance().addVisionMeasurement(robotPose, timestamp, Settings.Vision.MT1_STDEVS);
                        } else {
                            CommandSwerveDrivetrain.getInstance().addVisionMeasurement(robotPose, timestamp, Settings.Vision.MT2_STDEVS);
                        }

                        arrayOfLimelightPoses[i] = robotPose;
                        
                        SmartDashboard.putNumber("Vision/Pose X Component", robotPose.getX());
                        SmartDashboard.putNumber("Vision/Pose Y Component", robotPose.getY());
                        SmartDashboard.putNumber("Vision/Pose Theta (Degrees)", robotPose.getRotation().getDegrees());

                        SmartDashboard.putBoolean("Vision/" + names[i] + " Has Data", true);
                    } else {
                        SmartDashboard.putBoolean("Vision/" + names[i] + " Has Data", false);
                    }

                    SmartDashboard.putString("Vision/MegaTag Mode", megaTagMode.toString());
                    // this yaw is seems to be the robot yaw passed into the LL
                    SmartDashboard.putNumber("Vision/Limelight Robot Yaw", LimelightHelpers.getIMUData(limelightName).robotYaw);
                    // this is just the yaw of the internal imu 
                    SmartDashboard.putNumber("Vision/Limelight Yaw", LimelightHelpers.getIMUData(limelightName).Yaw);

                }
                if (Settings.DEBUG_MODE) {
                    String limelightName = names[i];
                    SmartDashboard.putString("Vision/MegaTag Mode", megaTagMode.toString());
                        // this yaw is seems to be the robot yaw passed into the LL
                    SmartDashboard.putNumber("Vision/Limelight Robot Yaw " + limelightName, LimelightHelpers.getIMUData(limelightName).robotYaw);
                        // this is just the yaw of the internal imu 
                    SmartDashboard.putNumber("Vision/Limelight Yaw " + limelightName, LimelightHelpers.getIMUData(limelightName).Yaw);
                    SmartDashboard.putNumber("Vision/Limelight Robot Yaw Passed in", (CommandSwerveDrivetrain.getInstance().getPose().getRotation().getDegrees() + (Robot.isBlue() ? 0 : 180)) % 360);
                }
            }
            if (Settings.DEBUG_MODE) {
                arrayPublisher.set(arrayOfLimelightPoses);
            }
        }
    }
}