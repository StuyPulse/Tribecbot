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
            new Camera("limelight-climber",
                    new Pose3d(-0.233, 0.375959, 0.20368,
                    new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(31.08), Units.degreesToRadians(-80))),
                    RobotContainer.EnabledSubsystems.LIMELIGHT),
            // new Camera("limelight-shooter", 
            //         new Pose3d(Units.inchesToMeters(2.490), Units.inchesToMeters(14.8620), Units.inchesToMeters(5.920157), 
            //         new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(15.0), Units.degreesToRadians(251.))), 
            //         RobotContainer.EnabledSubsystems.LIMELIGHT),
            new Camera("limelight-module",   //11.0845u
                    new Pose3d(Units.inchesToMeters(-10.768823), Units.inchesToMeters(12.717519), Units.inchesToMeters(8.331714), 
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(31.0), Units.degreesToRadians(185.0))), 
                    RobotContainer.EnabledSubsystems.LIMELIGHT)// 31 // 26.965
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