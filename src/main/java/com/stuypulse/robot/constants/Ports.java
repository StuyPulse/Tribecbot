/************************ PROJECT 2026 ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }

    public interface Climber {
        int CLIMBER = 3;
    }

    public interface Feeder {
        int FEEDER1 = 4;
        int FEEDER2 = 5;
    }

    public interface HoodedShooter {
        int HOOD = 5;

        int SHOOTER_LEADER = 6;
        int SHOOTER_FOLLOWER = 7;
    }

    public interface Intake {
        int PIVOT = 8;
        int ROLLER_LEADER = 9;
        int ROLLER_FOLLOWER = 10;
    }

    public interface Spindexer {
        int SPINDEXER = 11;
    }

    public interface Turret {
        int TURRET = 12;
        int ENCODER_17 = 13;
        int ENCODER_18 = 14;
    }
}
