/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

import com.pathplanner.lib.config.PIDConstants;

public class Gains {

    public interface ClimberHopper {
        SmartNumber kP = new SmartNumber("ClimberHopper/Gains/kP", 1.0);
        SmartNumber kI = new SmartNumber("ClimberHopper/Gains/kI", 0.0);
        SmartNumber kD = new SmartNumber("ClimberHopper/Gains/kD", 0.20);

        SmartNumber kS = new SmartNumber("ClimberHopper/Gains/kS", 0.0);
        SmartNumber kA = new SmartNumber("ClimberHopper/Gains/kA", 0.123);
        SmartNumber kV = new SmartNumber("ClimberHopper/Gains/kV", 0.0);

        // double kP = 1.0;
        // double kI = 0.0;
        // double kD = 0.20;
 
        // double kS = 0.0;
        // double kV = 0.123;
        // double kA = 0.0;
    }

    public interface HoodedShooter {
        public interface Shooter {
            SmartNumber kP = new SmartNumber("HoodedShooter/Shooter/Gains/kP", 0.45);
            SmartNumber kI = new SmartNumber("HoodedShooter/Shooter/Gains/kI", 0.0);
            SmartNumber kD = new SmartNumber("HoodedShooter/Shooter/Gains/kD", 0.0);

            SmartNumber kS = new SmartNumber("HoodedShooter/Shooter/Gains/kS", 0.0);
            SmartNumber kA = new SmartNumber("HoodedShooter/Shooter/Gains/kA", 0.0);
            SmartNumber kV = new SmartNumber("HoodedShooter/Shooter/Gains/kV", 0.123);

            // double kP = 0.45;
            // double kI = 0.0;
            // double kD = 0.0;

            // double kS = 0.0;
            // double kV = 0.123;
            // double kA = 0.0;
        }

        public interface Hood {
            SmartNumber kP = new SmartNumber("HoodedShooter/Hood/Gains/kP", 250.0); //300.0);
            SmartNumber kI = new SmartNumber("HoodedShooter/Hood/Gains/kI", 0.0);
            SmartNumber kD = new SmartNumber("HoodedShooter/Hood/Gains/kD", 2.0); //0.5);

            SmartNumber kS = new SmartNumber("HoodedShooter/Hood/Gains/kS", 0.25);
            SmartNumber kA = new SmartNumber("HoodedShooter/Hood/Gains/kA", 0.0);
            SmartNumber kV = new SmartNumber("HoodedShooter/Hood/Gains/kV", 0.0);

            // double kP = 300.0;
            // double kI = 0.0;
            // double kD = 0.0;

            // double kS = 0.0;
            // double kV = 0.0;
            // double kA = 0.0;
        }

    }

    public interface Spindexer {
        SmartNumber kP = new SmartNumber("Spindexer/Gains/kP", 1.20);
        SmartNumber kI = new SmartNumber("Spindexer/Gains/kI", 0.0);
        SmartNumber kD = new SmartNumber("Spindexer/Gains/kD", 0.0);

        SmartNumber kS = new SmartNumber("Spindexer/Gains/kS", 0.25);
        SmartNumber kA = new SmartNumber("Spindexer/Gains/kA", 0.010876);
        SmartNumber kV = new SmartNumber("Spindexer/Gains/kV", 1.20); //0.9413);
    }

    public interface Intake {
        public interface Pivot {
            SmartNumber kP = new SmartNumber("Intake/Pivot/Gains/kP", 100.0);
            SmartNumber kI = new SmartNumber("Intake/Pivot/Gains/kI", 0.0);
            SmartNumber kD = new SmartNumber("Intake/Pivot/Gains/kD", 10.0);

            SmartNumber kS = new SmartNumber("Intake/Pivot/Gains/kS", 0.0);
            SmartNumber kV = new SmartNumber("Intake/Pivot/Gains/kA", 0.12);
            SmartNumber kA = new SmartNumber("Intake/Pivot/Gains/kV", 0.0);

            // double kP = 100.0; // 0.016973 from sysid
            // double kI = 0.0;
            // double kD = 10.0;

            // double kS = 0.0; // 0.1728 from alpha
            // double kA = 0.12;
            // double kV = 0.0;

            double kG = 0.5;
        }
    }

    public interface Handoff {
        SmartNumber kP = new SmartNumber("Handoff/Gains/kP", 0.000155808);
        SmartNumber kI = new SmartNumber("Handoff/Gains/kI", 0.0);
        SmartNumber kD = new SmartNumber("Handoff/Gains/kD", 0.0);
        
        SmartNumber kS = new SmartNumber("Handoff/Gains/kS", 0.1728);
        SmartNumber kV = new SmartNumber("Handoff/Gains/kV", 0.12);
        SmartNumber kA = new SmartNumber("Handoff/Gains/kA", 0.00284);

        // double kP = 0.00015508; // 0.016973 from sysid
        // double kI = 0.0;
        // double kD = 0.0;

        // double kS = 0.21149; // 0.1728 from alpha
        // double kA = 0.016329;
        // double kV = 0.3652;
    }

    public interface Turret {
        public interface slot0 {
            SmartNumber kP = new SmartNumber("Turret/Gains/kP", 1300.0);
            SmartNumber kI = new SmartNumber("Turret/Gains/kI", 0.0);
            SmartNumber kD = new SmartNumber("Turret/Gains/kD", 140.0);
            
            SmartNumber kS = new SmartNumber("Turret/Gains/kS", 0.23);
            SmartNumber kV = new SmartNumber("Turret/Gains/kV", 0.0);
            SmartNumber kA = new SmartNumber("Turret/Gains/kA", 0.0);

            // double kP = 1300.0;
            // double kI = 0.0;
            // double kD = 140.0;

            // double kS = 0.23; // FOUND ON 2/25 PD 8
            // double kV = 0.0;
            // double kA = 0.0;
        }

        public interface slot1 {
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;

            double kS = 0.0; // FOUND ON 2/25 PD 8
            double kV = 0.0;
            double kA = 0.0;
        }
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
