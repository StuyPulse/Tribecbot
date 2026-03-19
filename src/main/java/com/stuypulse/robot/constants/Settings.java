/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/*-
 * File containing constants and tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */

public interface Settings {
    public final double DT = 0.020;
    public final int LOGGING_FREQUENCY = 2;
    public final double SECONDS_IN_A_MINUTE = 60.0;
    public final SmartBoolean DEBUG_MODE = new SmartBoolean("Robot/DebugMode", true);
    public final CANBus CANIVORE = new CANBus("canivore", "./logs/example.hoot");

    public interface Handoff {
        public final double GEAR_RATIO = 3.0 / 1.0;

        double HANDOFF_STOP = 0.0;
        double HANDOFF_MAX = 6000.0;
        double HANDOFF_REVERSE = -500.0;
        double RPM_TOLERANCE = 2200.0;
        double RPM_SOTM_TOLERANCE = 700.0;
        SmartNumber HANDOFF_RPM = new SmartNumber("Handoff/Target RPM", HANDOFF_MAX);

        SmartNumber HANDOFF_STALL_CURRENT = new SmartNumber("Handoff/Stall Current Limit for Reverse", 30.0);
    }

    public interface Intake {
        Rotation2d PIVOT_STOW_ANGLE = Rotation2d.fromDegrees(90.0); 
        Rotation2d PIVOT_DEPLOY_ANGLE = Rotation2d.fromDegrees(0.0);

        Rotation2d PIVOT_ANGLE_TOLERANCE = Rotation2d.fromDegrees(5.0); 

        Rotation2d PIVOT_MAX_ANGLE = Rotation2d.fromDegrees(88.0);
        Rotation2d PIVOT_MIN_ANGLE = Rotation2d.fromDegrees(-10.0);

        Rotation2d THRESHOLD_TO_START_ROLLERS = Rotation2d.fromDegrees(10.0);

        Rotation2d ANGLE_THRESHOLD_FOR_HOLDING_VOLTAGE = Rotation2d.fromDegrees(15.0);
        double HOMING_VOLTAGE = 3.0;
        
        double PUSHDOWN_VOLTAGE = 3.0;

        double GEAR_RATIO = 37.93;
        
        double STALL_CURRENT_LIMIT = 0; //TODO: set value
        double STALL_DEBOUNCE = 1.0; //TODO: VERIFY
    }

    public interface Spindexer {
        double FORWARD_SPEED = 4500.0;
        double REVERSE_SPEED = -4500.0;
        double STOP_SPEED = 0.0;

        double RPM_TOLERANCE = 800.0;
        double TOLERANCE_TO_START_INTAKE_ROLLERS_DURING_SCORING_ROUTINE = 1500.0;
        double STALL_CURRENT_LIMIT = 40.0; // random number as of 3/9

        /* CONSTANTS */
        double GEAR_RATIO = 8.0 / 1.0;
    }
    
    public interface Superstructure {
        public final double SHOOTER_TOLERANCE_RPM = 100.0;
        public final Rotation2d HOOD_TOLERANCE = Rotation2d.fromDegrees(0.2);

        public final double SHOOTER_SOTM_TOLERANCE_RPM = 350.0;
        public final double SHOOTER_FOTM_TOLERANCE_RPM = 250.0;
        public final Rotation2d HOOD_SOTM_TOLERANCE = Rotation2d.fromDegrees(3.0);

        public interface AngleInterpolation {
            double[][] distanceAngleInterpolationValues = {
                {1.22, Units.degreesToRadians(22.5)},           //BLAY-APPROVED (ALMOST AGAINST HUB), LOCKED IN
                {2.15, Units.degreesToRadians(27)},             //BLAY-APPROVED
                {3.38, Units.degreesToRadians(37)},             //BLAY-APPROVED   
                {4.43, Units.degreesToRadians(39)},             //BLAY-APPROVED
                {5.66, Units.degreesToRadians(39)}              //KEVIN-APPROVED
            };
        }

        public interface RPMInterpolation{
            double[][] distanceRPMInterpolationValues = {
                {1.22, 2670.0},                                         //BLAY-APPROVED, LOCKED IN
                {2.15, 2880.0},                                         //BLAY-APPROVED
                {3.38, 3200},                                           //BLAY-APPROVED
                {4.43, 3500.0},                                         //BLAY-APPROVED
                {5.66, 3900.0}                                          //KEVIN-APPROVED
            };
        }

        public interface TOFInterpolation{
            double[][] distanceTOFInterpolationValues = {
                {1.22, 0.965}, // seconds
                // {2.15, },
                {3.38, 1.32},  
                {4.43, 1.125},
                {5.66, 1.29}
            };
        }

        public interface FerryRPMInterpolation {
            double[][] ferryDistanceRPMInterpolation = {
                {5.16, 3500.0},
                {6.94, 3700.0},
                {7.87, 4000.0},
                {9.77, 4500.0},
                {10.694, 4795.0},       //STARTING FROM HERE THE DATA IS UNRELIABLE!!!
                {11.516, 4950.0},
                {12.416, 5100.0},
                {13.316, 5250.0},
                {14.216, 5375.0},
                {15.148, 5400.0},
                {16.54, 5500}           //FIELD LENGTH
            };
        }

        public interface FerryTOFInterpolation {
            double [][] FerryTOFInterpolationInterpolation = {
                // {5.16, },
                // {6.94, },
                // {7.87, },
                // {9.77, },
            };
        }

        public interface Shooter {
            
            public final double GEAR_RATIO = 1.0;
            public final double FLYWHEEL_RADIUS = Units.inchesToMeters(3.965 / 2);
            
            public interface RPM {
                public final SmartBoolean OVERRIDEN = new SmartBoolean("Superstructure/Shooter/RPM Override enabled?", false);
                public final SmartNumber OVERRIDE_VALUE = new SmartNumber("Superstructure/Shooter/RPM Override value", 0.0);

                public final SmartNumber SHOOT = new SmartNumber("InterpolationTesting/Shoot State Target RPM", 3500.0);
                public final SmartNumber FERRY = new SmartNumber("InterpolationTesting/Ferry State Target RPM", 2000.0);

                public final double REVERSE = 0.0;
                public final double KB = 2720.0;
                public final double LEFT_CORNER = 3850.0;
                public final double RIGHT_CORNER = 3850.0;
            }
        }

        public interface Hood {
            /**
             *
             * The absolute encoder is mounted on a 10.67:1 gear reduction relative to the
             * hood mechanism. This means:
             *
             *  - The encoder rotates 10.67 times for every 1 full rotation of the hood.
             *  - The hood's physical range of motion is only 33 degrees.
             *
             * Because 33° * 10.67 = ~352°, the encoder will never exceed 360° over the
             * entire hood travel. Therefore, the absolute encoder reading (0–360°)
             * uniquely maps to the hood’s 0–33° mechanical range without any ambiguity.
             *
             */
            public final double GEAR_RATIO = 1064.0 / 9.0;
            public final double ENCODER_TO_MECH = 32.0 / 3.0;
            public final double HOOD_HOMING_VOLTAGE = 2.0;

            public final Rotation2d ENCODER_OFFSET = Rotation2d.fromRotations(0.795);

            public final Rotation2d FORWARD_SOFT_LIMIT = Rotation2d.fromDegrees(40.0);
            public final Rotation2d REVERSE_SOFT_LIMIT = Rotation2d.fromDegrees(20.0);
            public final Rotation2d MIN_FROM_HORIZON = Rotation2d.fromDegrees(7.0);
            public final Rotation2d MAX_FROM_HORIZON = Rotation2d.fromDegrees(40.0);

            public final double STALL_CURRENT_LIMIT = 20.0;
            public final double STALL_DEBOUNCE = 0.5;

            public interface Angles {
                public final SmartBoolean OVERRIDEN = new SmartBoolean("Superstructure/Hood/Angle Override enabled?", false);
                public final SmartNumber OVERRIDE_VALUE_DEG = new SmartNumber("Superstructure/Hood/Angle Override value", 0.0);

                public final SmartNumber SHOOT = new SmartNumber("InterpolationTesting/Shoot State Target Angle (deg)", 20.0);
                public final SmartNumber FERRY = new SmartNumber("InterpolationTesting/Ferry State Target Angle (deg)", 20.0);
                public final Rotation2d FERRY_ANGLE = Rotation2d.fromDegrees(40.0);
                public final Rotation2d MIN = Rotation2d.fromDegrees(20.0);
                public final Rotation2d MAX = Rotation2d.fromDegrees(40.0);

                public final Rotation2d STOW = Rotation2d.fromDegrees(21.0);
                public final Rotation2d KB = Rotation2d.fromDegrees(22.0);
                public final Rotation2d LEFT_CORNER = Rotation2d.fromDegrees(38.0);
                public final Rotation2d RIGHT_CORNER = Rotation2d.fromDegrees(38.0);
            }
        }

        public interface Turret {
            public final Rotation2d MAX_VEL = new Rotation2d(Units.degreesToRadians(600.0));
            public final Rotation2d MAX_ACCEL = new Rotation2d(Units.degreesToRadians(600.0));
            public final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2.0);
            public final Rotation2d SOTM_TOLERANCE = Rotation2d.fromDegrees(5.0);
            
            public final Rotation2d KB = Rotation2d.fromDegrees(0.0);
            public final Rotation2d LEFT_CORNER = Rotation2d.fromDegrees(-233.0);
            public final Rotation2d RIGHT_CORNER = Rotation2d.fromDegrees(53.0);
            
            double RESOLUTION_OF_ABSOLUTE_ENCODER = 0.1;
            double WRAP_DEBOUNCE = 0.5;
            Rotation2d MAX_THEORETICAL_ROTATION = Rotation2d.fromDegrees(612);
            Rotation2d MIN_THEORETICAL_ROTATION = Rotation2d.fromDegrees(-612);
            
            /* CONSTANTS */
            public final double RANGE_LEFT = -360.0;
            public final double RANGE_RIGHT = 85.0;
        
            public final Rotation2d GAIN_SWITCHING_THRESHOLD = Rotation2d.fromDegrees(30);
        
            public final Transform2d TURRET_OFFSET = new Transform2d(Units.inchesToMeters(-4.0), Units.inchesToMeters(8.0), Rotation2d.kZero);
            public final double TURRET_HEIGHT = Units.inchesToMeters(0.0);
        
            public final double GEAR_RATIO_MOTOR_TO_MECH = (60.0 / 9.0) * (95.0 / 12.0); //1425.0 / 36.0;
        
            public interface BigGear {
                public final int TEETH = 95;
            }
        
            public interface Encoder17t {
                public final int TEETH = 17;
                public final Rotation2d OFFSET = Rotation2d.fromRotations(-0.716);
            }
        
            public interface Encoder18t {
                public final int TEETH = 18;
                public final Rotation2d OFFSET = Rotation2d.fromRotations(-0.559);
            }
        
            public interface SoftwareLimit {
                public final double FORWARD_MAX_ROTATIONS = 210.0 / 360.0;
                public final double BACKWARDS_MAX_ROTATIONS = -210.0 / 360.0;
            }
        }

        public interface SOTM {
            public final int MAX_ITERATIONS = 10;
            double TIME_TOLERANCE = 1e-5;
            SmartNumber UPDATE_DELAY = new SmartNumber("Superstructure/SOTM/update delay", 0.15);
        }
    }
    

    public interface Swerve {
        public final double MODULE_VELOCITY_DEADBAND_M_PER_S = 0.1;
        public final double ROTATIONAL_DEADBAND_RAD_PER_S = 0.1;

        public interface Constraints {
            public final double MAX_VELOCITY_M_PER_S = 4.93; 
            public final double MAX_VELOCITY_SOTM_M_PER_S = 1.5;
            public final double MAX_VELOCITY_FOTM_M_PER_S = 3.00;

            public final double MAX_ANGULAR_VEL_RAD_PER_S = Units.degreesToRadians(300.0);
            public final double MAX_ANGULAR_VEL_SOTM_RAD_PER_S = Units.degreesToRadians(75.0);
            public final double MAX_ANGULAR_VEL_FOTM_RAD_PER_S = Units.degreesToRadians(75.0);

            public final double MAX_ACCEL_M_PER_S_SQUARED = 15.0;
            public final double MAX_ACCEL_M_PER_S_SQUARED_SOTM = 4.0;
            public final double MAX_ACCEL_M_PER_S_SQUARED_FOTM = 10.0;
            public final double MAX_ANGULAR_ACCEL_RAD_PER_S_SQUARED = Units.degreesToRadians(900.0);

            public final PathConstraints DEFAULT_CONSTRAINTS =
                new PathConstraints(
                    MAX_VELOCITY_M_PER_S,
                    MAX_ACCEL_M_PER_S_SQUARED,
                    MAX_ANGULAR_VEL_RAD_PER_S,
                    MAX_ANGULAR_ACCEL_RAD_PER_S_SQUARED);
        }

        public interface Alignment {

            public interface Targets {}

            public interface Tolerances {
                public final double X_TOLERANCE = Units.inchesToMeters(2.0);
                public final double Y_TOLERANCE = Units.inchesToMeters(2.0);
                public final double THETA_TOLERANCE_DEG = 3.0;

                public final Pose2d POSE_TOLERANCE = new Pose2d(
                    Units.inchesToMeters(2.0),
                    Units.inchesToMeters(2.0),
                    Rotation2d.fromDegrees(2.0));

                public final double MAX_VELOCITY_WHEN_ALIGNED = 0.15;

                public final double ALIGNMENT_DEBOUNCE = 0.15;
            }
        }
    }

    public interface Vision {
        public final Vector<N3> MT1_STDEVS = VecBuilder.fill(0.5, 0.5, 1.0);
        public final Vector<N3> MT2_STDEVS = VecBuilder.fill(0.7, 0.7, 694694.0);
        public final int RESET_IMU_INDEX = 1;
        public final int INTERNAL_EXTERNAL_ASSIST_INDEX = 4;
        public final Translation2d INVALID_POSITION = new Translation2d(8.2705, 4.0345);
        public final double INVALID_POSITION_TOLERANCE_M = 0.05;
        public final double MAX_ANGULAR_VELOCITY_RAD_SEC = 2 * Math.PI;

        public final double BUZZ_DEBOUNCE = 0.25;
    }
}