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
import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;

public interface Settings {
    public final double DT = 0.020;
    public final double SECONDS_IN_A_MINUTE = 60.0;
    public final boolean DEBUG_MODE = true;
    public final CANBus CANIVORE = new CANBus("canivore", "./logs/example.hoot");

    public interface ClimberHopper {
        public interface Constants {
            public final double GEAR_RATIO = 45.0;

            public final double MIN_HEIGHT_METERS = 0.0;
            public final double MAX_HEIGHT_METERS = 1.0;

            public final double MASS_KG = 1.0;

            public final double NUM_ROTATIONS_TO_REACH_TOP = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / (0.480 / 13.0);
            public final double POSITION_CONVERSION_FACTOR = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / NUM_ROTATIONS_TO_REACH_TOP;
            public final double VELOCITY_CONVERSION_FACTOR = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / NUM_ROTATIONS_TO_REACH_TOP / 60.0;

            public final double DRUM_RADIUS_METERS = ((MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / (NUM_ROTATIONS_TO_REACH_TOP / GEAR_RATIO)) / 2.0 / Math.PI;
        }

        public final double CLIMBER_UP_HEIGHT_METERS = Constants.MAX_HEIGHT_METERS;
        public final double CLIMBER_DOWN_HEIGHT_METERS = 0.2;
        public final double HOPPER_DOWN_HEIGHT_METERS = Constants.MIN_HEIGHT_METERS;
        public final double HOPPER_UP_HEIGHT_METERS = 0.5;

        public final double STALL = 10.0;

        public final double ROTATIONS_AT_BOTTOM = 0.0;

        public final double DEBOUNCE = 0.25;

        public final double GYRO_TOLERANCE = 0.0;

        public final double HEIGHT_TOLERANCE_METERS = 0.02;

        public final double RAMP_RATE = 50.0;

        public final double MOTOR_VOLTAGE = 3.5;
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
        Rotation2d PIVOT_INTAKE_OUTAKE_ANGLE = Rotation2d.fromDegrees(0.0);

        Rotation2d PIVOT_ANGLE_TOLERANCE = Rotation2d.fromDegrees(3.0); 
        double FORWARD_MAX_ROTATIONS = -30.0 / 360.0;
        double BACKWARDS_MAX_ROTATIONS = 90.0 / 360.0;

        Rotation2d PIVOT_ANGLE_OFFSET = new Rotation2d();
        Rotation2d PIVOT_MAX_ANGLE = Rotation2d.fromDegrees(190);
        Rotation2d PIVOT_MIN_ANGLE = Rotation2d.fromDegrees(80);

        Rotation2d PIVOT_MAX_VEL = Rotation2d.fromDegrees(300.0);
        Rotation2d PIVOT_MAX_ACCEL = Rotation2d.fromDegrees(300.0);

        double GEAR_RATIO = 48.0;
    }

    public interface Spindexer {
        double FORWARD_SPEED = 6000.0;
        double REVERSE_SPEED = -6000.0;
        double STOP_SPEED = 0.0;

        double RPM_TOLERANCE = 400.0;

        public interface Constants {
            double GEAR_RATIO = 8.0 / 1.0;
        }
    }
    
    public interface HoodedShooter {
        public final double SHOOTER_TOLERANCE_RPM = 50.0;
        public final double HOOD_TOLERANCE_DEG = 0.5;

        public interface RPMs {
            public final SmartNumber SHOOT_RPM = new SmartNumber("HoodedShooter/Shoot State Target RPM", 3400.0);
            public final SmartNumber FERRY_RPM = new SmartNumber("HoodedShooter/Ferry State Target RPM", 2000.0);
            public final double REVERSE = 0.0;
            public final double KB_RPM = 0.0;
            public final double LEFT_CORNER_RPM = 0.0;
            public final double RIGHT_CORNER_RPM = 0.0;
        }
        public interface Angles {
            public final SmartNumber SHOOT_ANGLE = new SmartNumber("HoodedShooter/Shoot State Target Angle (deg)", 15.0);
            public final SmartNumber FERRY_ANGLE = new SmartNumber("HoodedShooter/Ferry State Target Angle (deg)", 20.0);

            public final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(7.0);
            public final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(40.0);
            public final Rotation2d KB_ANGLE = Rotation2d.fromDegrees(12.0);
            public final Rotation2d LEFT_CORNER_ANGLE = Rotation2d.fromDegrees(10.0);
            public final Rotation2d RIGHT_CORNER_ANGLE = Rotation2d.fromDegrees(10.0);
        }

        public interface AngleInterpolation {
            public final double[][] distanceAngleInterpolationValues = {
                {1.30, Units.degreesToRadians(16.5)},
                {1.43, Units.degreesToRadians(21.0)},
                {2.15, Units.degreesToRadians(23.23)},
                {2.864967, Units.degreesToRadians(25.460189)},
                {3.65, Units.degreesToRadians(28.0)},
                {4.43, Units.degreesToRadians(30.65)},
                {5.32, Units.degreesToRadians(33.5)}
            };
        }

        public interface RPMInterpolation {
            public final double[][] distanceRPMInterpolationValues = {
                {1.30, 3000.0},
                {1.43, 3000.0},
                {2.15, 3050.0},
                {2.864967, 3215.271125},
                {3.65, 3400.0},
                {4.43, 3650.0},
                {5.32, 3950.0}
            };
        }

        public interface FerryRPMInterpolation {
            public final double[][] distanceRPMInterpolationValues = {
                {3.79, 3450.0}
            };
        }

        public interface Shooter {
            public final double GEAR_RATIO = 1.0;
            public final double FLYWHEEL_RADIUS = Units.inchesToMeters(3.965 / 2);
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
            public final double GEAR_RATIO = 17024.0 / 135.0;
            public final double ENCODER_TO_MECH = 32.0 / 3.0;

            public final Rotation2d ENCODER_OFFSET = Rotation2d.fromDegrees(0.0);
        }
    }
    public interface ShootOnTheFly {
        public final int MAX_ITERATIONS = 5;
        public final double TIME_TOLERANCE = 0.01;
        public final SmartNumber UPDATE_DELAY = new SmartNumber("HoodedShooter/SOTM/Update Delay", 0.00);
    }

    public interface Swerve {
        public final double MODULE_VELOCITY_DEADBAND_M_PER_S = 0.1;
        public final double ROTATIONAL_DEADBAND_RAD_PER_S = 0.1;

        public interface Alignment {
            public interface Constraints {
                public final double DEFAULT_MAX_VELOCITY = 4.3;
                public final double DEFAULT_MAX_ACCELERATION = 15.0;
                public final double DEFAULT_MAX_ANGULAR_VELOCITY = Units.degreesToRadians(400.0);
                public final double DEFAULT_MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(900.0);
            }

            public interface Targets {}

            public interface Tolerances {
                public final double X_TOLERANCE = Units.inchesToMeters(2.0);
                public final double Y_TOLERANCE = Units.inchesToMeters(2.0);
                public final double THETA_TOLERANCE_DEG = 2.0;

                public final Pose2d POSE_TOLERANCE = new Pose2d(
                    Units.inchesToMeters(2.0),
                    Units.inchesToMeters(2.0),
                    Rotation2d.fromDegrees(2.0));

                public final double MAX_VELOCITY_WHEN_ALIGNED = 0.15;

                public final double ALIGNMENT_DEBOUNCE = 0.15;
            }
        }

        public interface Constraints {
            public final double MAX_VELOCITY_M_PER_S = 4.3;
            public final double MAX_ACCEL_M_PER_S_SQUARED = 15.0;
            public final double MAX_ANGULAR_VEL_RAD_PER_S = Units.degreesToRadians(400.0);
            public final double MAX_ANGULAR_ACCEL_RAD_PER_S = Units.degreesToRadians(900.0);

            public final PathConstraints DEFAULT_CONSTRAINTS =
                new PathConstraints(
                    MAX_VELOCITY_M_PER_S,
                    MAX_ACCEL_M_PER_S_SQUARED,
                    MAX_ANGULAR_VEL_RAD_PER_S,
                    MAX_ANGULAR_ACCEL_RAD_PER_S);
        }
    }

    public interface Turret {
        public final Rotation2d MAX_VEL = new Rotation2d(Units.degreesToRadians(600.0));
        public final Rotation2d MAX_ACCEL = new Rotation2d(Units.degreesToRadians(600.0));
        public final double TOLERANCE_DEG = 2.0;

        public final Rotation2d HUB = Rotation2d.fromDegrees(0.0);
        public final Rotation2d LEFT_CORNER = Rotation2d.fromDegrees(0.0);
        public final Rotation2d RIGHT_CORNER = Rotation2d.fromDegrees(0.0);

        double RESOLUTION_OF_ABSOLUTE_ENCODER = 0.1;
        Rotation2d MAX_THEORETICAL_ROTATION = Rotation2d.fromDegrees(612);
        Rotation2d MIN_THEORETICAL_ROTATION = Rotation2d.fromDegrees(-612);

        public interface Constants {
            public final double RANGE = 210.0;

            public final Transform2d TURRET_OFFSET = new Transform2d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Rotation2d.kZero);
            public final double TURRET_HEIGHT = Units.inchesToMeters(0.0);

            public final double GEAR_RATIO_MOTOR_TO_MECH = 1425.0 / 36.0;

            public interface BigGear {
                public final int TEETH = 95;
            }

            public interface Encoder17t {
                public final int TEETH = 17;
                public final Rotation2d OFFSET = new Rotation2d();
            }

            public interface Encoder18t {
                public final int TEETH = 18;
                public final Rotation2d OFFSET = new Rotation2d();
            }

            public interface SoftwareLimit {
                public final double FORWARD_MAX_ROTATIONS = 210.0 / 360.0;
                public final double BACKWARDS_MAX_ROTATIONS = -210.0 / 360.0;
            }
        }
    }

    public interface Vision {
        public final Vector<N3> MT1_STDEVS = VecBuilder.fill(0.5, 0.5, 1.0);
        public final Vector<N3> MT2_STDEVS = VecBuilder.fill(0.7, 0.7, 694694.0);
    }
}