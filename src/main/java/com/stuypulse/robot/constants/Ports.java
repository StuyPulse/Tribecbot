/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
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

    public interface ClimberHopper {
        int CLIMBER_HOPPER = 3;
    }

    public interface Handoff {
        int HANDOFF = 4;
    }

    public interface HoodedShooter {
        // all these ports are copied from alphabot
        public interface Hood {
            int MOTOR = 25;
            int THROUGHBORE_ENCODER = 37;
        }

        public interface Shooter {
            int MOTOR_LEAD = 17;
            int MOTOR_FOLLOW = 14;
        }
    }

    public interface Intake {
        int PIVOT = 8;
        int ROLLER_LEADER = 9;
        int ROLLER_FOLLOWER = 10;
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

    public interface LED {
        int LED_PORT = 9;
    }
}
