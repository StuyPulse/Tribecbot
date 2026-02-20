/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

import com.pathplanner.lib.config.PIDConstants;

public class Gains {
    
    public interface ClimberHopper {
        double kP = 1.0;
        double kI = 0.0;
        double kD = 0.20;

        double kS = 0.0;
        double kV = 0.123;
        double kA = 0.0;
    }

    public interface HoodedShooter {
        public interface Shooter {
            double kP = 0.45;
            double kI = 0.0;
            double kD = 0.0;

            double kS = 0.0;
            double kV = 0.123;
            double kA = 0.0;
        }

        public interface Hood {
            double kP = 300.0;
            double kI = 0.0;
            double kD = 0.0;

            double kS = 0.0;
            double kV = 0.0;
            double kA = 0.0;
        }
        
    }

    public interface Spindexer {
        double kP = 1.20;
        double kI = 0.0;
        double kD = 0.0;
        
        double kS = 0.019444;
        double kA = 0.010876;
        double kV = 0.38546;
    }

    public interface Intake {
        public interface Pivot {
            double kP = 1.0;
            double kI = 0.0;
            double kD = 0.0;

            double kS = 0.0;
            double kV = 0.0;
            double kA = 0.0;

            double kG = 0.0; 
        }
    }

    public interface Handoff {
        double kP = 0.00015508;
        double kI = 0.0;
        double kD = 0.0;
        
        double kS = 0.1728;
        double kA = 0.0028428;
        double kV = 0.11725;
    }

    public interface Turret {
        double kS = 0.179;
        double kV = 0.0;
        double kA = 0.0;

        double kP = 1300.0;
        double kI = 0.0;
        double kD = 140.0;
    }

    public interface Swerve {
        public interface Drive {
            double kS = 0.0;
            double kV = 0.124;
            double kA = 0.0;
            double kP = 0.1;
            double kI = 0.0;
            double kD = 0.0;
        }

        public interface Turn {
            double kS = 0.1;
            double kV = 2.66;
            double kA = 0.0;
            double kP = 100.0;
            double kI = 0.0;
            double kD = 0.5;
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
            PIDConstants THETA = new PIDConstants(3.0, 0.0, 0.2);
        }
    }
}
