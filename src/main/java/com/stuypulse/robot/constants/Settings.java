/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

import java.util.concurrent.TimeUnit;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;

/*-
 * File containing constants and tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    double DT = 0.020;
    double SECONDS_IN_A_MINUTE = 60.0;
    boolean DEBUG_MODE = true;
    CANBus CANIVORE = new CANBus("canivore", "./logs/example.hoot");

    public interface Handoff {
        double GEAR_RATIO = 1.0;

        double HANDOFF_STOP = 0.0;
        double HANDOFF_MAX = 4800.0;
        double HANDOFF_REVERSE = -500.0;
        double RPM_TOLERANCE = 200.0;
        public final SmartNumber HANDOFF_RPM = new SmartNumber("Handoff/Target RPM", HANDOFF_MAX);
    }

    public interface Intake {
        Rotation2d PIVOT_STOW_ANGLE = Rotation2d.fromDegrees(90.0); 
        Rotation2d PIVOT_INTAKE_OUTAKE_ANGLE = Rotation2d.fromDegrees(0.0);

        public final Rotation2d PIVOT_ANGLE_TOLERANCE = Rotation2d.fromDegrees(3.0); 
        public final double FORWARD_MAX_ROTATIONS = -30.0 / 360.0;
        public final double BACKWARDS_MAX_ROTATIONS = 90.0 / 360.0;

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
        double SHOOTER_TOLERANCE_RPM = 50.0;
        double HOOD_TOLERANCE_DEG = 0.5;

        public interface AngleInterpolation {
            double[][] distanceAngleInterpolationValues = {
                {1.43, Units.degreesToRadians(21.0)}, // meters, radians
                {3.65, Units.degreesToRadians(28.0)},
                {5.32, Units.degreesToRadians(33.5)}
            };
        }
        public interface RPMInterpolation{
            double[][] distanceRPMInterpolationValues = {
                {1.43, 3000.0}, // meters, RPM 
                {3.65, 3400.0},
                {5.32, 3850.0}
            };
        }

        public interface RPMs {
            SmartNumber SHOOT_RPM = new SmartNumber("HoodedShooter/Shoot State Target RPM", 3400.0);
            SmartNumber FERRY_RPM = new SmartNumber("HoodedShooter/Ferry State Target RPM", 2000.0);
            public final double REVERSE = -0.0;
            public final double HUB_RPM = 0.0; 
            public final double LEFT_CORNER_RPM = 0.0; // TBD
            public final double RIGHT_CORNER_RPM = 0.0; // TBD
            public final double STOW = 0.0; // TBD
        }

        public interface Angles {
            SmartNumber SHOOT_ANGLE = new SmartNumber("HoodedShooter/Shoot State Target Angle (deg)", 15.0);
            SmartNumber FERRY_ANGLE = new SmartNumber("HoodedShooter/Ferry State Target Angle (deg)", 20.0);

            public final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(7);
            public final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(36.80);
            public final Rotation2d HUB_ANGLE = Rotation2d.fromDegrees(12);
            public final Rotation2d LEFT_CORNER_ANGLE = Rotation2d.fromDegrees(10); 
            public final Rotation2d RIGHT_CORNER_ANGLE = Rotation2d.fromDegrees(10);
        }

        public interface Hood {
            public final double GEAR_RATIO = 1290300.0 / 5967.0; 
            public final double SENSOR_TO_HOOD_RATIO = 360.0 / 36.0;

            public final Rotation2d ENCODER_OFFSET = Rotation2d.fromDegrees(0.0);
        }
        public interface Shooter {
            public final double GEAR_RATIO = 1.0;
        }
    }
    
    public interface Turret {
        Rotation2d MAX_VEL = new Rotation2d(Units.degreesToRadians(600.0));
        Rotation2d MAX_ACCEL = new Rotation2d(Units.degreesToRadians(600.0));        
        double TOLERANCE_DEG = 2.0;

        Rotation2d HUB = Rotation2d.fromDegrees(0.0);
        Rotation2d LEFT_CORNER = Rotation2d.fromDegrees(0.0);
        Rotation2d RIGHT_CORNER = Rotation2d.fromDegrees(0.0);

        public interface Constants {
            double RANGE = 210.0;

            Transform2d TURRET_OFFSET = new Transform2d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Rotation2d.kZero);
            double TURRET_HEIGHT = Units.inchesToMeters(0.0);
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
                public final double FORWARD_MAX_ROTATIONS = 210.0 / 360.0;
                public final double BACKWARDS_MAX_ROTATIONS = -210.0 / 360.0;
            } 

            public final double GEAR_RATIO_MOTOR_TO_MECH = 1425.0 / 36.0;
        }
    }

    public interface ClimberHopper {
        public interface Constants {
            double GEAR_RATIO = 45.0;

            double MIN_HEIGHT_METERS = 0;
            double MAX_HEIGHT_METERS = 1;

            double MASS_KG = 1;

            double NUM_ROTATIONS_TO_REACH_TOP = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / (0.480 / 13); // Number of rotations that the motor has to spin, NOT the gear
            double POSITION_CONVERSION_FACTOR = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / NUM_ROTATIONS_TO_REACH_TOP;
            double VELOCITY_CONVERSION_FACTOR = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / NUM_ROTATIONS_TO_REACH_TOP / 60;

            double DRUM_RADIUS_METERS = ((MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / (NUM_ROTATIONS_TO_REACH_TOP / GEAR_RATIO)) / 2 / Math.PI;
        }

        double CLIMBER_UP_HEIGHT_METERS = Constants.MAX_HEIGHT_METERS;
        double CLIMBER_DOWN_HEIGHT_METERS = 0.2;
        double HOPPER_DOWN_HEIGHT_METERS = Constants.MIN_HEIGHT_METERS;
        double HOPPER_UP_HEIGHT_METERS = 0.5;

        double STALL = 10.0;

        double ROTATIONS_AT_BOTTOM = 0.0;

        double DEBOUNCE = 0.25;

        double GYRO_TOLERANCE = 0.0;

        double HEIGHT_TOLERANCE_METERS = 0.02;

        double RAMP_RATE = 50.0;

        double MOTOR_VOLTAGE = 3.5;
    }

    public interface Vision {
        Vector<N3> MT1_STDEVS = VecBuilder.fill(0.5, 0.5, 1.0);
        Vector<N3> MT2_STDEVS = VecBuilder.fill(0.7, 0.7, 694694);
    }

    public interface ShootOnTheFly {
        int MAX_ITERATIONS = 5;
        double TIME_TOLERANCE = 0.01;
    }

    public interface Swerve {
        double MODULE_VELOCITY_DEADBAND_M_PER_S = 0.1;
        double ROTATIONAL_DEADBAND_RAD_PER_S = 0.1;
        public interface Constraints {    
            double MAX_VELOCITY_M_PER_S = 4.3;
            double MAX_ACCEL_M_PER_S_SQUARED = 15.0;
            double MAX_ANGULAR_VEL_RAD_PER_S = Units.degreesToRadians(400);
            double MAX_ANGULAR_ACCEL_RAD_PER_S = Units.degreesToRadians(900);

            PathConstraints DEFAULT_CONSTRAINTS =
                new PathConstraints(
                    MAX_VELOCITY_M_PER_S,
                    MAX_ACCEL_M_PER_S_SQUARED,
                    MAX_ANGULAR_VEL_RAD_PER_S,
                    MAX_ANGULAR_ACCEL_RAD_PER_S);
        }

        public interface Alignment {
            public interface Constraints {
                double DEFAULT_MAX_VELOCITY = 4.3;
                double DEFAULT_MAX_ACCELERATION = 15.0;
                double DEFUALT_MAX_ANGULAR_VELOCITY = Units.degreesToRadians(400);
                double DEFAULT_MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(900);
            }

            public interface Tolerances {
                double X_TOLERANCE = Units.inchesToMeters(2.0); 
                double Y_TOLERANCE = Units.inchesToMeters(2.0);
                double THETA_TOLERANCE_DEG = 2.0;

                Pose2d POSE_TOLERANCE = new Pose2d(
                    Units.inchesToMeters(2.0), 
                    Units.inchesToMeters(2.0), 
                    Rotation2d.fromDegrees(2.0));

                double MAX_VELOCITY_WHEN_ALIGNED = 0.15;

                double ALIGNMENT_DEBOUNCE = 0.15;
            }

            public interface Targets {}
        }
    }

    public interface LEDS {
        double DESIRED_TAGS_WHEN_DISABLED = 2;

        public enum LEDState {
            PRESSED_TOP_BUTTON(LEDPattern.solid(Color.kLightBlue)),
            PRESSED_LEFT_BUTTON(LEDPattern.solid(Color.kSkyBlue)),
            PRESSED_RIGHT_BUTTON(LEDPattern.solid(Color.kBlueViolet)),
            PRESSED_BOT_BUTTON(LEDPattern.solid(Color.kAliceBlue)),
            PRESSED_LEFT_TRIGGER(LEDPattern.solid(Color.kDarkBlue)),
            PRESSED_RIGHT_TRIGGER(LEDPattern.solid(Color.kCadetBlue)),
            PRESSED_LEFT_BUMPER(LEDPattern.solid(Color.kFirstBlue)),
            PRESSED_RIGHT_BUMPER(LEDPattern.solid(Color.kRoyalBlue)),
            PRESSED_LEFT_DPAD(LEDPattern.solid(Color.kSlateBlue)),
            PRESSED_RIGHT_DPAD(LEDPattern.solid(Color.kSteelBlue)),
            PRESSED_DOWN_DPAD(LEDPattern.solid(Color.kDodgerBlue)),
            PRESSED_TOP_DPAD(LEDPattern.solid(Color.kCornflowerBlue)),
            TRENCH_PASS(LEDPattern.solid(Color.kGreen).breathe(Seconds.of(0.4))),
            TRENCH_LOWERING(LEDPattern.solid(Color.kLightGoldenrodYellow)),
            TRENCH_E_STOP(LEDPattern.solid(Color.kRed)),
            CLIMBING(LEDPattern.solid(Color.kLightSkyBlue)),
            CLIMB_IS_ALIGNED(LEDPattern.solid(Color.kMediumBlue)),
            APPROACHING_LEFT_CORNER(LEDPattern.solid(Color.kWheat)),
            LEFT_CORNER(LEDPattern.solid(Color.kAntiqueWhite)),
            APPROACHING_RIGHT_CORNER(LEDPattern.solid(Color.kBlanchedAlmond)),
            RIGHT_CORNER(LEDPattern.solid(Color.kAqua)),
            DISABLED_ALIGNED(LEDPattern.solid(Color.kPurple)),
            SHOOTING_ON_THE_MOVE(LEDPattern.solid(null)),
            VEL_HIGH_SHOOTING_PAUSED(LEDPattern.solid(null)),
            SHOOTING_MODE(LEDPattern.solid(null)),
            FERRYING_MODE(LEDPattern.solid(null)),
            DEFAULT_SETTING(LEDPattern.kOff);
            
            public LEDPattern pattern;

            private LEDState(LEDPattern pattern) {
                this.pattern = pattern;
            }

        }

        //buttons 
        // LEDPattern PRESSED_TOP_BUTTON = LEDPattern.solid(Color.kLightBlue);
        // LEDPattern PRESSED_LEFT_BUTTON = LEDPattern.solid(Color.kSkyBlue);
        // LEDPattern PRESSED_RIGHT_BUTTON = LEDPattern.solid(Color.kBlueViolet);
        // LEDPattern PRESSED_BOT_BUTTON = LEDPattern.solid(Color.kAliceBlue);
        // LEDPattern PRESSED_LEFT_TRIGGER = LEDPattern.solid(Color.kDarkBlue);
        // LEDPattern PRESSED_RIGHT_TRIGGER = LEDPattern.solid(Color.kCadetBlue);
        // LEDPattern PRESSED_LEFT_BUMPER = LEDPattern.solid(Color.kFirstBlue);
        // LEDPattern PRESSED_RIGHT_BUMPER = LEDPattern.solid(Color.kRoyalBlue);
        // LEDPattern PRESSED_LEFT_DPAD = LEDPattern.solid(Color.kSlateBlue);
        // LEDPattern PRESSED_RIGHT_DPAD = LEDPattern.solid(Color.kSteelBlue);
        // LEDPattern PRESSED_DOWN_DPAD = LEDPattern.solid(Color.kDodgerBlue);
        // LEDPattern PRESSED_TOP_DPAD = LEDPattern.solid(Color.kCornflowerBlue);
    }

    public interface Driver {
        double BUZZ_TIME = 1.0;
        double BUZZ_INTENSITY = 1.0;

        public interface Drive {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Drive/Deadband", 0.05);

            SmartNumber RC = new SmartNumber("Driver Settings/Drive/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Drive/Power", 2);
        }
        public interface Turn {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Turn/Deadband", 0.05);

            SmartNumber RC = new SmartNumber("Driver Settings/Turn/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Turn/Power", 2);
        }
    }  
}
