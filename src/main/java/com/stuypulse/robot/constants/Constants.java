/************************ PROJECT ALPHA *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public interface Constants {
    public interface HoodedShooter {
        public interface Hood {
            public final double GEAR_RATIO = 1290300.0 / 5967.0; 
            public final double SENSOR_TO_HOOD_RATIO = 360.0 / 36.0;
            public final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(7);
            public final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(36.80);
            public final Rotation2d HUB_ANGLE = Rotation2d.fromDegrees(12); // TBD
            public final Rotation2d LEFT_CORNER_ANGLE = Rotation2d.fromDegrees(10); // TBD
            public final Rotation2d RIGHT_CORNER_ANGLE = Rotation2d.fromDegrees(10); // TBD

            public final Rotation2d ENCODER_OFFSET = Rotation2d.fromDegrees(20.0 * 10.0);//(0.332);//(0.325);
        }
        public interface Shooter {
            public final double GEAR_RATIO = 1.0;
        }
    }
  
    public interface Align {
        int MAX_ITERATIONS = 5;
        double TIME_TOLERANCE = 0.01;
    }

    public interface Spindexer {
        public final double GEAR_RATIO = 40.0 / 12.0;
    }

    public interface Turret {
        double RANGE = 180.0; // 420.0;
        Transform2d TURRET_OFFSET = new Transform2d(Units.inchesToMeters(-2.50), Units.inchesToMeters(11.19), Rotation2d.kZero);
        double TURRET_HEIGHT = Units.inchesToMeters(10.984); // TODO: get value
        public interface Encoder18t {
            public final int TEETH = 18;
            public final Rotation2d OFFSET = new Rotation2d();
        }
        
        public interface Encoder17t {
            public final int TEETH = 17;
            public final Rotation2d OFFSET = new Rotation2d();
        }

        public interface BigGear {
            public final int TEETH = 95;
        }

        public interface SoftwareLimit {
            public final double FORWARD_MAX_ROTATIONS = 1.5;
            public final double BACKWARDS_MAX_ROTATIONS = 1.5;
        } 

        public final double GEAR_RATIO_MOTOR_TO_MECH = 1425.0 / 36.0;
    }

    public interface Feeder {
        public final double GEAR_RATIO = 1.0; 
    }
}