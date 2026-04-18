/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

import com.pathplanner.lib.config.PIDConstants;

public class Gains {

    public interface Superstructure {
        public interface Shooter {
            // VTC PID
            SmartNumber kP = new SmartNumber("Superstructure/Shooter/Gains/kP",10.5);
            SmartNumber kI = new SmartNumber("Superstructure/Shooter/Gains/kI", 0.0);
            SmartNumber kD = new SmartNumber("Superstructure/Shooter/Gains/kD", 0.0);

            SmartNumber kS = new SmartNumber("Superstructure/Shooter/Gains/kS", 2.47);
            SmartNumber kV = new SmartNumber("Superstructure/Shooter/Gains/kV", 0.01775);
            SmartNumber kA = new SmartNumber("Superstructure/Shooter/Gains/kA", 0.0);
        }

        public interface Hood {
            double kP = 250.0;
            double kI = 0.0;
            double kD = 2.0;

            double kS = 0.25;
            double kV = 0.0;
            double kA = 0.0;
        }

        public interface Turret {
            public interface slot0 {
                double kP = 200.0;
                double kI = 0.0;
                double kD = 0.0;
            
                double kS = 0.4775;
                double kV = 0.0;
                double kA = 0.0;
            }
            
            public interface slot1 {
                SmartNumber kP = new SmartNumber("Superstructure/Turret/Gains/kP", 150.0); // 80
                SmartNumber kI = new SmartNumber("Superstructure/Turret/Gains/kI", 0.0);
                SmartNumber kD = new SmartNumber("Superstructure/Turret/Gains/kD", 3.0); // 10

                SmartNumber kS = new SmartNumber("Superstructure/Turret/Gains/kS", 0.4775);
                SmartNumber kV = new SmartNumber("Superstructure/Turret/Gains/kV", 0.0);
                SmartNumber kA = new SmartNumber("Superstructure/Turret/Gains/kA", 0.0);
            }

            SmartNumber kOmega = new SmartNumber("Superstructure/Turret/Gains/kOmega", 3.43);
            SmartNumber kTranslation = new SmartNumber("Superstructure/Turret/Gains/kTranslation", 0.0);
        }
    }

    public interface Spindexer {
        double kP = 1.2;
        double kI = 0.0;
        double kD = 10.0;

        double kS = 0.25;
        double kV = 1.2;
        double kA = 0.010876;
    }

    public interface Intake {
        public interface Pivot {
            SmartNumber kP = new SmartNumber("Intake/Pivot/Gains/kP", 200.0);
            SmartNumber kI = new SmartNumber("Intake/Pivot/Gains/kI", 0.0);
            SmartNumber kD = new SmartNumber("Intake/Pivot/Gains/kD", 10.0);
            
            SmartNumber kS = new SmartNumber("Intake/Pivot/Gains/kS", 0.0);
            SmartNumber kV = new SmartNumber("Intake/Pivot/Gains/kV", 0.12);
            SmartNumber kA = new SmartNumber("Intake/Pivot/Gains/kA", 0.0);

            double kG = 0.5;
        }
    }

    public interface Handoff {
        double kP = 0.00015508;
        double kI = 0.0;
        double kD = 0.0;

        double kS = 0.1728;
        double kV = 0.12;
        double kA = 0.00284;
    }

    public interface Swerve {
        public interface Drive {
            double kP = 0.10224;
            double kI = 0.0;
            double kD = 0.0;

            double kS = 0.19896;
            double kV = 0.12528;
            double kA = 0.011662;
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
            PIDConstants XY = new PIDConstants(5.0, 0, 0);
            PIDConstants THETA = new PIDConstants(4.0, 0, 0.0);
        }
    }
}
