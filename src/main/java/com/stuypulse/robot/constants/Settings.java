/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
    public final double SECONDS_IN_A_MINUTE = 60.0;
    public final boolean DEBUG_MODE = true;
    public final CANBus CANIVORE = new CANBus("canivore", "./logs/example.hoot");

    public interface HubDATA {
        public final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
        public final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};
        
    }

    public interface ClimberHopper {
        /* CONSTANTS */
        public final double GEAR_RATIO = 45.0;


        public final double MIN_ROTATIONS = 0.0;
        public final double MAX_HEIGHT_METERS = 0.2032;
        public final double MAX_ROTATIONS = 1.75;



        public final double MASS_KG = 1.0;

        public final double DRUM_RADIUS_METERS = Units.inchesToMeters(0.75);
        /* CONSTANTS */

        public final double CLIMBER_UP_ROTATIONS = MAX_ROTATIONS - 0.1; // TODO: FIND
        public final double CLIMBER_DOWN_ROTATIONS = 0.0;

        public final double STALL = 10.0;

        public final double DEBOUNCE = 0.25;


        // public final double HEIGHT_TOLERANCE_METERS = 0.015;
        public final double TOLERANCE_ROTATIONS = 0.1; // TODO: tune tolerance for 12V

        public final double RAMP_RATE = 50.0;

        public final double MOTOR_VOLTAGE = 12.0; 
    }

    public interface Handoff {
        public final double GEAR_RATIO = 3.0 / 1.0;

        double HANDOFF_STOP = 0.0;
        double HANDOFF_MAX = 4800.0;
        double HANDOFF_REVERSE = -500.0;
        double RPM_TOLERANCE = 200.0;
        SmartNumber HANDOFF_RPM = new SmartNumber("Handoff/Target RPM", HANDOFF_MAX);
    }

    public interface Intake {
        Rotation2d PIVOT_STOW_ANGLE = Rotation2d.fromDegrees(90.0); 
        Rotation2d PIVOT_DEPLOY_ANGLE = Rotation2d.fromDegrees(0.0);

        Rotation2d PIVOT_ANGLE_TOLERANCE = Rotation2d.fromDegrees(15.0); 

        Rotation2d PIVOT_MAX_ANGLE = Rotation2d.fromDegrees(90.0);
        Rotation2d PIVOT_MIN_ANGLE = Rotation2d.fromDegrees(0.0);

        Rotation2d PIVOT_MAX_VEL_DEPLOY = Rotation2d.fromDegrees(720.0);
        Rotation2d PIVOT_MAX_ACCEL_DEPLOY = Rotation2d.fromDegrees(1440.0);

        Rotation2d PIVOT_MAX_VEL_STOW = Rotation2d.fromDegrees(360.0);
        Rotation2d PIVOT_MAX_ACCEL_STOW = Rotation2d.fromDegrees(600.0);

        Rotation2d THRESHHOLD_TO_START_ROLLERS = Rotation2d.fromDegrees(10.0);

        Rotation2d ARBITRARY_VOLTAGE_THRESHOLD = Rotation2d.fromDegrees(15.0);
        
        double PUSHDOWN_VOLTAGE = 3.0;

        double GEAR_RATIO = 37.93;
        
        double debugVoltage = 0; //TODO: set value
        double STALL_CURRENT_LIMIT = 0; //TODO: set value
        double STALL_DEBOUNCE = 1.0; //TODO: VERIFY
    }

    public interface Spindexer {
        double FORWARD_SPEED = 4500.0;
        double REVERSE_SPEED = -4500.0;
        double STOP_SPEED = 0.0;

        double RPM_TOLERANCE = 400.0;

        /* CONSTANTS */
        double GEAR_RATIO = 8.0 / 1.0;
    }
    
    public interface Superstructure {
        public final double SHOOTER_TOLERANCE_RPM = 100.0;
        public final Rotation2d HOOD_TOLERANCE = Rotation2d.fromDegrees(0.5);

public interface AngleInterpolation {
            double[][] distanceAngleInterpolationValues = {
                {1.22, Units.degreesToRadians(20)},             //AGAINST THE HUB
                {1.43, Units.degreesToRadians(21.0)},           //meters, radians
                {2.15, Units.degreesToRadians(23.23)},          //KEVIN-APPROVED
                {2.864967, Units.degreesToRadians(27)},         //KEVIN-APPROVED
                {3.65, Units.degreesToRadians(28.0)},           //KEVIN-APPROVED
                {4.43, Units.degreesToRadians(33.5)},           //KEVIN-APPROVED
                {5.66, Units.degreesToRadians(39)}              //KEVIN-APPROVED
            };
        }
        public interface RPMInterpolation{
            double[][] distanceRPMInterpolationValues = {
                {1.22, 2950.0},                                         //KEVIN-APPROVED
                {1.43, 3000.0}, // meters, RPM 
                {2.15, 3050.0},                                         //KEVIN-APPROVED
                {2.864967, 3150},                                       //KEVIN-APPROVED
                {3.65, 3400.0},                                         //KEVIN-APPROVED
                {4.43, 3600.0},                                         //KEVIN-APPROVED
                {5.66, 3900.0}                                          //KEVIN-APPROVED
            };
        }

        public interface TOFInterpolation{
            // double[][] distanceTOFInterpolationValues = {
            //     {1.30, 0.0}, // seconds
            //     {1.43, 0.0},
            //     {2.15, 0.0},
            //     {2.864967, 0.0},
            //     {3.65, 0.0},
            //     {4.43, 0.0},
            //     {5.32, 0.0}
            // };

            double[][] distanceTOFInterpolationValues = {
                {1.30, 1.0}, // seconds
                {1.43, 1.0},
                {2.15, 1.0},
                {2.864967, 1.0},
                {3.65, 1.0},
                {4.43, 1.0},
                {5.32, 1.0}
            };
        }

        // public interface AngleInterpolation {
        //     public final double[][] distanceAngleInterpolationValues = {
        //         {1.22, Units.degreesToRadians(20)},             //AGAINST THE HUB
        //         {1.43, Units.degreesToRadians(21.0)},
        //         {2.15, Units.degreesToRadians(23.23)},

        //         //NEW
        //         {2.864967, Units.degreesToRadians(25.460189)},
        //         {3.65, Units.degreesToRadians(28.0)},
        //         {4.43, Units.degreesToRadians(30.65)},
        //         {5.32, Units.degreesToRadians(33.5)}
        //     };
        // }

        // public interface RPMInterpolation {
        //     public final double[][] distanceRPMInterpolationValues = {
        //         {1.22, 2850.0},                                         //AGAINST THE HUB
        //         {1.43, 3000.0},
        //         {2.15, 3050.0},

        //         //NEW
        //         {2.864967, 3215.271125},
        //         {3.65, 3400.0},
        //         {4.43, 3650.0},
        //         {5.32, 3950.0}
        //     };
        // }

        // public interface TOFInterpolation{
        //     public final double[][] distanceTOFInterpolationValues = { // COLLECT THESE
        //         // {1.22, 0.0},
        //         {1.30, 1.01}, // seconds
        //         // {1.43, 0.0},
        //         // {2.15, 0.0},
        //         {2.864967, 1.1},
        //         // {3.65, 0.0},
        //         {4.43, 1.234},
        //         {5.32, 1.267}
        //     };
        // }

        public interface FerryRPMInterpolation {
            public final double[][] distanceRPMInterpolationValues = {
                {3.79, 3450.0}
            };
        }

        public interface Shooter {
            public final double GEAR_RATIO = 1.0;
            public final double FLYWHEEL_RADIUS = Units.inchesToMeters(3.965 / 2);

            public interface RPM {
                public final SmartNumber SHOOT = new SmartNumber("InterpolationTesting/Shoot State Target RPM", 3500.0);
                public final SmartNumber FERRY = new SmartNumber("InterpolationTesting/Ferry State Target RPM", 2000.0);
                public final double REVERSE = 0.0;
                public final double KB = 0.0;
                public final double LEFT_CORNER = 0.0;
                public final double RIGHT_CORNER = 0.0;
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

            public final Rotation2d ENCODER_OFFSET = Rotation2d.fromRotations(-0.043);

            public final Rotation2d FORWARD_SOFT_LIMIT = Rotation2d.fromDegrees(39.0);
            public final Rotation2d REVERSE_SOFT_LIMIT = Rotation2d.fromDegrees(20.0);
            public final Rotation2d MIN_FROM_HORIZON = Rotation2d.fromDegrees(7.0);

            public final double STALL_CURRENT_LIMIT = 20.0;
            public final double STALL_DEBOUNCE = 0.5;

            public interface Angles {
                public final SmartNumber SHOOT = new SmartNumber("InterpolationTesting/Shoot State Target Angle (deg)", 15.0);
                public final SmartNumber FERRY = new SmartNumber("InterpolationTesting/Ferry State Target Angle (deg)", 20.0);

                public final Rotation2d MIN = Rotation2d.fromDegrees(20.0);
                public final Rotation2d MAX = Rotation2d.fromDegrees(40.0);

                public final Rotation2d STOW = Rotation2d.fromDegrees(11.0);
                public final Rotation2d KB = Rotation2d.fromDegrees(12.0);
                public final Rotation2d LEFT_CORNER = Rotation2d.fromDegrees(10.0);
                public final Rotation2d RIGHT_CORNER = Rotation2d.fromDegrees(10.0);
            }
        }

        public interface Turret {
            public final Rotation2d MAX_VEL = new Rotation2d(Units.degreesToRadians(600.0));
            public final Rotation2d MAX_ACCEL = new Rotation2d(Units.degreesToRadians(600.0));
            public final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2.0);
            
            public final Rotation2d KB = Rotation2d.fromDegrees(0.0);
            public final Rotation2d LEFT_CORNER = Rotation2d.fromDegrees(0.0);
            public final Rotation2d RIGHT_CORNER = Rotation2d.fromDegrees(0.0);
            
            double RESOLUTION_OF_ABSOLUTE_ENCODER = 0.1;
            double WRAP_DEBOUNCE = 0.5;
            Rotation2d MAX_THEORETICAL_ROTATION = Rotation2d.fromDegrees(612);
            Rotation2d MIN_THEORETICAL_ROTATION = Rotation2d.fromDegrees(-612);
            
            /* CONSTANTS */
            public final double RANGE_LEFT = -45;
            public final double RANGE_RIGHT = 390;
        
            public final double SLOT_SWITCHING_THRESHOLD_ROT = .5;
        
            public final Transform2d TURRET_OFFSET = new Transform2d(Units.inchesToMeters(-4.0), Units.inchesToMeters(8.0), Rotation2d.kZero);
            public final double TURRET_HEIGHT = Units.inchesToMeters(0.0);
        
            public final double GEAR_RATIO_MOTOR_TO_MECH = (60.0 / 9.0) * (95.0 / 12.0); //1425.0 / 36.0;
        
            public interface BigGear {
                public final int TEETH = 95;
            }
        
            public interface Encoder17t {
                public final int TEETH = 17;
                public final Rotation2d OFFSET = Rotation2d.fromRotations(-0.279541015625);//(-0.86962890625); //0.6787109375
            }
        
            public interface Encoder18t {
                public final int TEETH = 18;
                public final Rotation2d OFFSET = Rotation2d.fromRotations(-0.42822265625);//(-0.700927734375); //0.53564453125
            }
        
            public interface SoftwareLimit {
                public final double FORWARD_MAX_ROTATIONS = 210.0 / 360.0;
                public final double BACKWARDS_MAX_ROTATIONS = -210.0 / 360.0;
            }
        }

        public interface SOTM {
            public final int MAX_ITERATIONS = 5;
            public final double TIME_TOLERANCE = 0.01;
            public final SmartNumber UPDATE_DELAY = new SmartNumber("Superstructure/SOTM/Update Delay", 0.00);
        }
    }
    

    public interface Swerve {
        public final double MODULE_VELOCITY_DEADBAND_M_PER_S = 0.1;
        public final double ROTATIONAL_DEADBAND_RAD_PER_S = 0.1;

        public interface Constraints {
            public final double MAX_VELOCITY_M_PER_S = 4.93; // 4.3p
            public final double MAX_ACCEL_M_PER_S_SQUARED = 15.0;
            public final double MAX_ANGULAR_VEL_RAD_PER_S = Units.degreesToRadians(600.0);
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
    }
}