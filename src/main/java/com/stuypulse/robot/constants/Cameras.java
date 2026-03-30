/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;

import com.stuypulse.robot.RobotContainer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/** This interface stores information about each camera. */
public interface Cameras {

    public Camera[] LimelightCameras = {
            new Camera("limelight-right", //.381, .2325, .2069592
                    new Pose3d(Units.inchesToMeters( -9.149), Units.inchesToMeters(15.080), Units.inchesToMeters(8.088), //TODO: check this offset
                    // new Pose3d(0.0,0.0,0.0, 
                    new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(28.0), Units.degreesToRadians(-90.203885))),
                    RobotContainer.EnabledSubsystems.RIGHT_LIMELIGHT),
            new Camera("limelight-left", 
                    new Pose3d(Units.inchesToMeters(-2.490), Units.inchesToMeters(-14.8620), Units.inchesToMeters(5.676), 
                    // new Pose3d(0.0,0.0,0.0,
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(14.955812), Units.degreesToRadians(71.5))), //289
                    RobotContainer.EnabledSubsystems.LEFT_LIMELIGHT),
            new Camera("limelight-back",   //11.0845u
                    new Pose3d(Units.inchesToMeters(-11.274), Units.inchesToMeters(-13.218), Units.inchesToMeters(8.452), 
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(27.875), Units.degreesToRadians(185.155825))), 
                    RobotContainer.EnabledSubsystems.BACK_LIMELIGHT)// 31 // 26.965
                    //THIS ROTATION IS NOT EXACTLY 180 DEGREES...
                    
    };

    public static class Camera {
        private String name;
        private Pose3d location;
        private SmartBoolean isEnabled;

        public Camera(String name, Pose3d location, SmartBoolean isEnabled) {
            this.name = name;
            this.location = location;
            this.isEnabled = isEnabled;
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