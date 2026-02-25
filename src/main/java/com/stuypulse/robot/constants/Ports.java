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
    public CANBus RIO = new CANBus("rio");
    public CANBus CANIVORE = new CANBus("CANIVORE");
    
    public interface Gamepad {
        int DRIVER = 0;
    }

    public interface ClimberHopper {
        int CLIMBER_HOPPER = 60;
    }

    public interface Handoff {
        int HANDOFF = 43;
    }

    public interface HoodedShooter {
        public interface Hood {
            int MOTOR = 45;
            int THROUGHBORE_ENCODER = 44;
        }

        public interface Shooter {
            int MOTOR_LEAD = 47;
            int MOTOR_FOLLOW = 46;
        }
    }

    public interface Intake {
        int PIVOT = 20;
        int ROLLER_LEADER = 21;
        int ROLLER_FOLLOWER = 22;
    }

    public interface Spindexer {
        int SPINDEXER_LEAD_MOTOR = 30;
        int SPINDEXER_FOLLOW_MOTOR = 31;
    }

    public interface Turret {
        int MOTOR = 40;
        int ENCODER17T = 42;
        int ENCODER18T = 41;
    }

    public interface LED {
        int LED_PORT = 9;
    }
}
