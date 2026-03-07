/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

import com.pathplanner.lib.config.PIDConstants;

public class Gains {

    public interface Climber {
        SmartNumber kP = new SmartNumber("Climber/Gains/kP", 1.0);
        SmartNumber kI = new SmartNumber("Climber/Gains/kI", 0.0);
        SmartNumber kD = new SmartNumber("Climber/Gains/kD", 0.20);

        SmartNumber kS = new SmartNumber("Climber/Gains/kS", 0.0);
        SmartNumber kV = new SmartNumber("Climber/Gains/kV", 0.123);
        SmartNumber kA = new SmartNumber("Climber/Gains/kA", 0.0);

        // double kP = 1.0;
        // double kI = 0.0;
        // double kD = 0.20;
 
        // double kS = 0.0;
        // double kV = 0.123;
        // double kA = 0.0;
    }

    public interface Superstructure {
        public interface Shooter {
            SmartNumber kP = new SmartNumber("Superstructure/Shooter/Gains/kP", 0.45);
            SmartNumber kI = new SmartNumber("Superstructure/Shooter/Gains/kI", 0.0);
            SmartNumber kD = new SmartNumber("Superstructure/Shooter/Gains/kD", 0.0);

            SmartNumber kS = new SmartNumber("Superstructure/Shooter/Gains/kS", 0.0);
            SmartNumber kV = new SmartNumber("Superstructure/Shooter/Gains/kV", 0.123);
            SmartNumber kA = new SmartNumber("Superstructure/Shooter/Gains/kA", 0.0);

            // double kP = 0.45;
            // double kI = 0.0;
            // double kD = 0.0;

            // double kS = 0.0;
            // double kV = 0.123;
            // double kA = 0.0;
        }

        public interface Hood {
            double kP = 250.0;   //300.0;
            double kI = 0.0;
            double kD = 2.0;     //0.5

            double kS = 0.25;
            double kV = 0.0;
            double kA = 0.0;
        }

        public interface Turret {
            public interface slot0 {
                double kP = 1300.0;
                double kI = 0.0;
                double kD = 140.0;

                double kS = 0.23;   // FOUND ON 2/25 PD 8
                double kV = 0.0;
                double kA = 0.0;
            }
            
            public interface slot1 {
                double kP = 0.0;
                double kI = 0.0;
                double kD = 0.0;
            
                double kS = 0.0;
                double kV = 0.0;
                double kA = 0.0;
            }
        }
    }

    public interface Spindexer {
        double kP = 1.2;
        double kI = 0.0;
        double kD = 10.0;

        double kS = 0.25;
        double kV = 1.2; //0.9413
        double kA = 0.010876;
    }

    public interface Intake {
        public interface Pivot {
            double kP = 100.0;
            double kI = 0.0;
            double kD = 10.0;

            double kS = 0.0;
            double kV = 0.12;
            double kA = 0.0;

            double kG = 0.5;
        }
    }

    public interface Handoff {
        double kP = 0.00015508; // 0.016973 from sysid
        double kI = 0.0;
        double kD = 0.0;

        double kS = 0.1728;
        double kV = 0.12;
        double kA = 0.00284;
    }

    public interface Swerve {
        public interface Drive {
            double kP = 0.3838;
            double kI = 0.0;
            double kD = 0.0;

            double kS = 0.20896;
            double kV = 0.12464;
            double kA = 0.014877;
        }

        public interface Turn {
            double kP = 100.0;
            double kI = 0.0;
            double kD = 0.5;

            double kS = 0.1;
            double kV = 2.66;
            double kA = 0.0;
        }

        public interface Alignment {
            public interface Rotation {
                double kp = 112.3;
                double ki = 0.0;
                double kd = 2.3758;
                double ks = 0.31395;
                double kv = 0.10969;
                double ka = 0.026589;
            }

            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;
            double akP = 0.0;
            double akI = 0.0;
            double akD = 0.0;

            PIDConstants XY = new PIDConstants(3.0, 0.0, 0.2);
            PIDConstants THETA = new PIDConstants(13.0, 0.0, 0.5);
        }
    }
}
