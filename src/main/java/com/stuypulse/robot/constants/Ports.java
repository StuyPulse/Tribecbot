/************************ PROJECT 2026 ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.ctre.phoenix6.CANBus;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    // TODO: Get bus name
    public CANBus bus = new CANBus("rio");
    
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }

    public interface Climber {
        int CLIMBER = 3;
    }

    public interface Feeder {
        int FEEDER_LEADER = 4;
        int FEEDER_FOLLOWER= 5;
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
        int ABSOLUTE_ENCODER = 11;
    }

    public interface Spindexer {
        int SPINDEXER_LEAD_MOTOR = 11;
        int SPINDEXER_FOLLOW_MOTOR = 12;
    }

    public interface Turret {
        int MOTOR = 50;
        int ENCODER17T = 38;
        int ENCODER18T = 21;
    }
}
