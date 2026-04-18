/** ********************** PROJECT TRIBECBOT ************************ */
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
 /* Use of this source code is governed by an MIT-style license */
 /* that can be found in the repository LICENSE file.           */
/** ************************************************************ */
package com.stuypulse.robot.subsystems.vision;

import java.util.Arrays;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Cameras;
import com.stuypulse.robot.constants.Cameras.Camera;
import com.stuypulse.robot.constants.Cameras.Camera.Pipeline;
import com.stuypulse.robot.constants.Cameras.Camera.RejectionValue;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.vision.LimelightHelpers;
import com.stuypulse.robot.util.vision.LimelightHelpers.IMUData;
import com.stuypulse.robot.util.vision.LimelightHelpers.PoseEstimate;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightVision extends SubsystemBase {

    private static final LimelightVision instance;

    static {
        instance = new LimelightVision();
    }

    public static LimelightVision getInstance() {
        return instance;
    }

    private String[] names;
    private SmartBoolean enabled;
    private int maxTagCount;
    private MegaTagMode megaTagMode;

    private Pose2d[] limelightPoseArray;

    // private StructPublisher<Pose2d> leftLimelightPosePublisher;
    // private StructPublisher<Pose2d> rightLimelightPosePublisher;
    // private StructPublisher<Pose2d> backLimelightPosePublisher;

    private boolean hasData;
    private BStream debouncedHasData;

    private Timer hdrTimer = new Timer();
    private boolean lastHdrEnabledVal = false;

    public enum MegaTagMode {
        MEGATAG1,
        MEGATAG2
    }

    public void setPipeline(Pipeline pipeline) {
        for(Camera camera: Cameras.LimelightCameras) {
            camera.setPipeline(pipeline);
        }
    }

    public LimelightVision() {
        limelightPoseArray = new Pose2d[Cameras.LimelightCameras.length];
        // leftLimelightPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Limelight/Pose Left", Pose2d.struct).publish();
        // rightLimelightPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Limelight/Pose Right", Pose2d.struct).publish();
        // backLimelightPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Limelight/Pose Back", Pose2d.struct).publish();

        names = new String[Cameras.LimelightCameras.length];

        maxTagCount = 0;

        for (int i = 0; i < Cameras.LimelightCameras.length; i++) {
            names[i] = Cameras.LimelightCameras[i].getName();
            Pose3d robotRelativePose = Cameras.LimelightCameras[i].getLocation();
            LimelightHelpers.setCameraPose_RobotSpace(
                    names[i],
                    robotRelativePose.getX(),
                    robotRelativePose.getY(),
                    robotRelativePose.getZ(),
                    Units.radiansToDegrees(robotRelativePose.getRotation().getX()),
                    Units.radiansToDegrees(robotRelativePose.getRotation().getY()),
                    Units.radiansToDegrees(robotRelativePose.getRotation().getZ())
            );

            limelightPoseArray[i] = new Pose2d();
        }

        enabled = new SmartBoolean("Vision/Is Enabled", true);
        megaTagMode = MegaTagMode.MEGATAG1;
        setIMUMode(1);

        hdrTimer.reset();
        hdrTimer.start();

        debouncedHasData = BStream.create(
                () -> hasData)
                .filtered(new BDebounce.Both(Settings.Vision.BUZZ_DEBOUNCE));

        setPipeline(Pipeline.NO_SUN);
    }

    public void setAllLTagWhitelist(int... ids) {
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

    public int getMaxTagCount() {
        return this.maxTagCount;
    }

    public void setIMUMode(int mode) {
        for (String name : names) {
            LimelightHelpers.SetIMUMode(name, mode);
        }
    }

    /**
     * Allows you to set the convergence speed of the internal LL IMU and robot
     * gyro.
     *
     * @param assistValue, an double that sets the correction speed of the
     * complementary filter for the IMU. IMU Mode 4 uses the fusing of the
     * internal IMU (1khz) with the external gyro reading as well. Higher values
     * ranging towards 1 indicate a faster convergence of internal IMU to the
     * robot IMU mode. Defaults to 0.001.
     */
    public void setIMUAssistValue(double assistValue) {
        for (String name : names) {
            LimelightHelpers.SetIMUAssistAlpha(name, assistValue);
        }
    }

    /**
     * Allows all tags except the specified ones by setting blacklisted tag
     * indexes to -1 in the full tag list before applying the new list.
     *
     * @param tagsToBlacklist array of tag IDs to exclude from detection
     * @param limelight the name of the Limelight camera to configure
     */
    public void setTagBlacklist(int[] tagsToBlacklist, String limelight) {
        int[] allTags = Field.ALL_TAGS.clone();
        
        for (int i = 0; i < tagsToBlacklist.length; i++) {
            allTags[tagsToBlacklist[i] - 1] = -1;
        }

        int[] validTags = new int[allTags.length - tagsToBlacklist.length];
        
        int counter = 0;
        for (int i = 0; i < allTags.length; i++) {
            if (allTags[i] != -1) {
                validTags[counter] = allTags[i];
                counter++;
            }
        }

        System.out.println(Arrays.toString(validTags));
        DogLog.log("Allowlisted Tags for " + limelight, Arrays.toString(validTags));

        LimelightHelpers.SetFiducialIDFiltersOverride(limelight, validTags);
    }

    /**
     * Restricts detection to only the specified tags by passing them directly
     * as the whitelist.
     *
     * @param tagsToWhitelist array of tag IDs to allow for detection
     * @param limelight the name of the Limelight camera to configure
     */
    public void setTagWhitelist(int[] tagsToWhitelist, String limelight) {
        LimelightHelpers.SetFiducialIDFiltersOverride(limelight, tagsToWhitelist);
    }

    public IMUData[] getIMUData() {
        IMUData[] data = new IMUData[Cameras.LimelightCameras.length];

        for (int i = 0; i < Cameras.LimelightCameras.length; i++) {
            data[i] = LimelightHelpers.getIMUData(Cameras.LimelightCameras[i].getName());
        }

        return data;
    }

    public boolean hasData() {
        return debouncedHasData.get();
    }

    public void periodicAfterScheduler() {
        if (enabled.get()) {
            hasData = false;

        this.maxTagCount = 0;

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
                    boolean notNull = false;
                    boolean withinAngularVelocityTolerance = false;
                    boolean withinInvalidPositionTolerance = false;
                    // boolean withinTargetAreaTolerance = false;

                    if (poseEstimate != null && poseEstimate.tagCount > 0)  {
                        notNull = true;

                        if (poseEstimate.pose.getTranslation().getDistance(Settings.Vision.INVALID_POSITION) < Settings.Vision.INVALID_POSITION_TOLERANCE_M){
                            withinInvalidPositionTolerance = true;
                        } else {
                            Cameras.LimelightCameras[i].incrementRejection(RejectionValue.INVALID_POSITION);
                        }

                        if (CommandSwerveDrivetrain.getInstance().getChassisSpeeds().omegaRadiansPerSecond < Settings.Vision.MAX_ANGULAR_VELOCITY_RAD_SEC) {
                            withinAngularVelocityTolerance = true;
                        } else {
                            Cameras.LimelightCameras[i].incrementRejection(RejectionValue.ANGULAR_VELOCITY);
                        }

                        // if (poseEstimate.avgTagArea >= Settings.Vision.MIN_TAG_AREA) {
                        //     withinTargetAreaTolerance = true;
                        // } else {
                        //     Cameras.LimelightCameras[i].incrementRejection(RejectionValue.TARGET_AREA);
                        // }

                        Pose2d robotPose = poseEstimate.pose;
                        double timestamp = poseEstimate.timestampSeconds;

                        boolean isAcceptablePose = notNull && withinAngularVelocityTolerance && !withinInvalidPositionTolerance;// && withinTargetAreaTolerance;

                        if (megaTagMode == MegaTagMode.MEGATAG1 && isAcceptablePose) {
                            CommandSwerveDrivetrain.getInstance().addVisionMeasurement(robotPose, timestamp, Settings.Vision.MT1_STDEVS);
                            hasData = true;
                        } else if (megaTagMode == MegaTagMode.MEGATAG2 && isAcceptablePose) {
                            CommandSwerveDrivetrain.getInstance().addVisionMeasurement(robotPose, timestamp, Settings.Vision.MT2_STDEVS);
                            hasData = true;
                        }

                        DogLog.log("Vision/Within Invalid Position Tolerance", withinInvalidPositionTolerance);
                        DogLog.log("Vision/Within Angular Velocity Tolerance", withinAngularVelocityTolerance);
                        DogLog.log("Vision/Not Null", notNull);

                        DogLog.log("Vision/Pose X Component", robotPose.getX());
                        DogLog.log("Vision/Pose Y Component", robotPose.getY());
                        DogLog.log("Vision/Pose Theta (Degrees)", robotPose.getRotation().getDegrees());
                        DogLog.log("Vision/Pose Estimate X " + limelightName, poseEstimate.pose.getX());
                        DogLog.log("Vision/Pose Estimate Y " + limelightName, poseEstimate.pose.getY());
                        DogLog.log("Vision/Pose Estimate Theta " + limelightName, poseEstimate.pose.getRotation().getDegrees());

                        // switch (limelightName) {
                        //     case "limelight-right" ->
                        //         rightLimelightPosePublisher.set(robotPose);
                        //     case "limelight-left" ->
                        //         leftLimelightPosePublisher.set(robotPose);
                        //     case "limelight-back" ->
                        //         backLimelightPosePublisher.set(robotPose);
                        // }

                        DogLog.log("Vision/" + names[i] + " Has Data", true);
                        
                    } else {
                        DogLog.log("Vision/" + names[i] + " Has Data", false);
                        Cameras.LimelightCameras[i].incrementRejection(RejectionValue.NOT_NULL);
                    }

                    DogLog.log("Vision/MegaTag Mode", megaTagMode.toString());
                    // this yaw is seems to be the robot yaw passed into the LL
                    DogLog.log("Vision/Limelight Robot Yaw", LimelightHelpers.getIMUData(limelightName).robotYaw);
                    // this is just the yaw of the internal imu 
                    DogLog.log("Vision/Limelight Yaw", LimelightHelpers.getIMUData(limelightName).Yaw);

                    //Rejection counters
                    Cameras.LimelightCameras[i].log();
                }
            }

            // Alternating pipelines for hdr
            if (lastHdrEnabledVal != Settings.Vision.HDR_ENABLED.get()) {
                setPipeline(Pipeline.NO_SUN);
                if(!lastHdrEnabledVal) {
                    Cameras.LimelightCameras[2].setPipeline(Pipeline.HIGH_SUN);
                }
            }

            lastHdrEnabledVal = Settings.Vision.HDR_ENABLED.get();

            if(Settings.Vision.HDR_ENABLED.get()) {
                if(hdrTimer.hasElapsed(Settings.Vision.HDR_TIMEOUT_SEC)) {
                    for(Camera camera: Cameras.LimelightCameras) {
                        camera.performHDR();
                    }
                    hdrTimer.reset();
                }
            }

            DogLog.log("Vision/Has Data", hasData);
        }
    }
}
