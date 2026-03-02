/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

public interface DriverConstants {

    public interface Driver {
        double BUZZ_TIME = 1.0;
        double BUZZ_INTENSITY = 1.0;

        public interface Drive {
            double DEADBAND = 0.05;
            double RC = 0.05;
            int POWER = 2;
        }
        public interface Turn {
            double DEADBAND = 0.05;
            double RC = 0.05;
            int POWER = 2;
        }
    }
}