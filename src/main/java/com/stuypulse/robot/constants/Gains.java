/************************ PROJECT 2026 ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import com.pathplanner.lib.config.PIDConstants;

public class Gains {

    public interface Climber {
        double kP = 1.0;
        double kI = 0.0;
        double kD = 0.20;

        double kS = 0.1;
        double kV = 0.2;
        double kA = 0.01;
    }

    public interface HoodedShooter {
        public interface Hood {
            double kP = 1.0;
            double kI = 0.0;
            double kD = 0.20;

            double kS = 0.1;
            double kV = 0.2;
            double kA = 0.01;
        }

        public interface Shooter {
            double kP = 1.0;
            double kI = 0.0;
            double kD = 0.20;

            double kS = 0.1;
            double kV = 0.2;
            double kA = 0.01;
        }
    }

    public interface Intake {
        public interface Pivot {
            double kP = 1.0;
            double kI = 0.0;
            double kD = 0.25;

            double kS = 0.0;
            double kV = 0.2;
            double kA = 0.01;
        }
    }

    public interface Turret {
        double kP = 1.0;
        double kI = 0.0;
        double kD = 0.25;

        double kS = 0.0;
        double kV = 0.2;
        double kA = 0.01;
    }

    public interface Spindexer {
        double kP = 1.0;
        double kI = 0.0;
        double kD = 0.20;

        double kS = 0.1;
        double kV = 0.2;
        double kA = 0.01;
    }

    public interface Swerve {
        public interface Drive {
            double kS = 0.17608;
            double kV = 0.11448;
            double kA = 0.0059131;
            double kP = 0.096506;
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
        public interface Motion {
            PIDConstants XY = new PIDConstants(2.0, 0, 0.25);
            PIDConstants THETA = new PIDConstants(5.0, 0, 0.2);
        }
        public interface Alignment {
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;
            double akP = 0.0;
            double akI = 0.0;
            double akD = 0.0;

            PIDConstants XY = new PIDConstants(0.0, 0.0, 0.0);
            PIDConstants THETA = new PIDConstants(0.0, 0.0, 0.0);
        }
    }
}
