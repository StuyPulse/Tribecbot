/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.util.vision.LimelightHelpers;
import com.stuypulse.robot.util.vision.LimelightHelpers.LimelightResults;
import com.stuypulse.robot.util.vision.LimelightHelpers.RawFiducial;
import com.stuypulse.stuylib.network.SmartBoolean;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/** This interface stores information about each camera. */
public interface Cameras {

    public Camera[] LimelightCameras = {
            new Camera("limelight-right", //.381, .2325, .2069592 // this ll bugging 
                    new Pose3d(Units.inchesToMeters( -9.149), Units.inchesToMeters(15.080), Units.inchesToMeters(8.088),
                    // new Pose3d(0.0,0.0,0.0, 
                    new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(28.0), Units.degreesToRadians(-80.203885))), // Yaw edited by Yunus Gubrud who is indicating this by the request of dear leader Kalimul on april 1'st circa 2026.
                    RobotContainer.EnabledSubsystems.RIGHT_LIMELIGHT),
            new Camera("limelight-left", 
                    new Pose3d(Units.inchesToMeters(-2.490), Units.inchesToMeters(-14.8620), Units.inchesToMeters(5.676), 
                    // new Pose3d(0.0,0.0,0.0,
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(14.955812), Units.degreesToRadians(71.5))), //289
                    RobotContainer.EnabledSubsystems.LEFT_LIMELIGHT),
            new Camera("limelight-back",   //11.0845u
                    new Pose3d(Units.inchesToMeters(-10.676), Units.inchesToMeters(-12.969), Units.inchesToMeters(8.753), 
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(27.875), Units.degreesToRadians(185.155825))), 
                    RobotContainer.EnabledSubsystems.BACK_LIMELIGHT)// 31 // 26.965
                    //THIS ROTATION IS NOT EXACTLY 180 DEGREES...
                    
    };

    public static class Camera {
        private String name;
        private Pose3d location;
        private SmartBoolean isEnabled;
        private String keyName;

        private int rejectedCounterNotNull;
        private int rejectedCounterAngularVelocity;
        private int rejectedCounterInvalidPosition;
        private int rejectedCounterTargetArea;
        private LimelightResults result;

        private Pipeline currentPipeline;

        public Camera(String name, Pose3d location, SmartBoolean isEnabled) {
            this.name = name;
            this.location = location;
            this.isEnabled = isEnabled;
            this.keyName = "Vision/" + name + "/";
            this.result = LimelightHelpers.getLatestResults(name);
        }

        public enum Pipeline {
            NO_SUN,
            LOW_SUN,
            MED_SUN,
            HIGH_SUN
        }
        
        private int getCurrentPipelineID() {
            return switch(this.currentPipeline) {
                case NO_SUN -> 3;
                case LOW_SUN -> 2;
                case MED_SUN -> 1;
                case HIGH_SUN -> 0;
            };
        }

        public enum RejectionValue {
            NOT_NULL,
            ANGULAR_VELOCITY,
            INVALID_POSITION,
            TARGET_AREA
        };

        public void setPipeline(Pipeline pipeline) {
            this.currentPipeline = pipeline;
            LimelightHelpers.setPipelineIndex(name, getCurrentPipelineID());
        }

        public void performHDR() {
            Pipeline nextHdrPipeline = Pipeline.NO_SUN;

            if (currentPipeline == Pipeline.NO_SUN) {
                nextHdrPipeline = Pipeline.HIGH_SUN;
            }

            setPipeline(nextHdrPipeline);
        }

        public void incrementRejection(RejectionValue rejectionValue) {
            switch (rejectionValue) {
                case NOT_NULL:
                    rejectedCounterNotNull++;
                    break;
                case ANGULAR_VELOCITY:
                    rejectedCounterAngularVelocity++;
                    break;
                case INVALID_POSITION:
                    rejectedCounterInvalidPosition++;
                    break;
                case TARGET_AREA:
                    rejectedCounterTargetArea++;
                    break;
            }
        }

        public void log() {
            DogLog.log(keyName + "# Rejected Not Null", rejectedCounterNotNull);
            DogLog.log(keyName + "# Rejected Target Area", rejectedCounterTargetArea);
            DogLog.log(keyName + "# Rejected Angular Velocity", rejectedCounterAngularVelocity);
            DogLog.log(keyName + "# Rejected Invalid Position", rejectedCounterInvalidPosition);

            DogLog.log(keyName + "Heartbeat", LimelightHelpers.getHeartbeat(name));
            DogLog.log(keyName + "Temp (C)", LimelightHelpers.getLimelightDoubleArrayEntry(name, "hw").get());
            DogLog.log(keyName + "Pose MT1", (Robot.isBlue()
                                ? LimelightHelpers.getBotPoseEstimate_wpiBlue(name).pose
                                : LimelightHelpers.getBotPoseEstimate_wpiRed(name).pose));
            DogLog.log(keyName + "Pose MT2", (Robot.isBlue()
                                ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).pose
                                : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(name).pose));
            DogLog.log(keyName + "Pipeline", LimelightHelpers.getCurrentPipelineIndex(name));
    
            result = LimelightHelpers.getLatestResults(name);
            DogLog.log(keyName + "Time since last boot", result.timestamp_LIMELIGHT_publish / 1000.0, "Seconds");
            
            RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(name);
            for(Integer i = 0; i < rawFiducials.length; i++) {
                DogLog.log(keyName + "Raw Fiducials[" + i.toString() + "]", rawFiducials[i].id);
            }
        }

        public String getName() {
            return name;
        }

        public Pose3d getLocation() {
            return location;
        }

        public boolean isEnabled() {
            return isEnabled.get();
        }

        public void setEnabled(boolean enabled) {
            this.isEnabled.set(enabled);
        }
    }
}