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
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

/*-
 * File containing constants and tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */

public interface Settings {
    public final double DT = 0.020;
    public final double WATCHDOG_TIMEOUT = 0.2;
    public final int LOGGING_FREQUENCY = 5;
    public final double SECONDS_IN_A_MINUTE = 60.0;
    public final SmartBoolean DEBUG_MODE = new SmartBoolean("Robot/DebugMode", true);
    public final CANBus CANIVORE = new CANBus("canivore", "./logs/example.hoot");
    public final double LOOP_OVERRUN_WARNING_TIME_SEC = 1; 
    SmartBoolean ENABLE_DISTANCE_CHECK = new SmartBoolean("Robot/Enable Distance Check?", false);
    SmartBoolean ENABLE_OUT_OF_FIELD_CHECK = new SmartBoolean("Robot/Enable out of field check", true);

    public interface Handoff {
        public final double GEAR_RATIO = 3.0 / 1.0;

        double HANDOFF_STOP = 0.0;
        double HANDOFF_MAX = 4800.0;
        double HANDOFF_REVERSE = -500.0;
        double RPM_TOLERANCE = 2200.0;
        double REVERSE_TIME = 2.0;
        double RPM_SOTM_TOLERANCE = 700.0;
        SmartNumber HANDOFF_RPM = new SmartNumber("Handoff/Target RPM", HANDOFF_MAX);

        double IS_EMPTY_AMPERAGE = 8; //TODO: update IS EMPTY VALUE

        double FORWARD_DUTY_CYCLE = 1.0;
        double REVERSE_DUTY_CYCLE = -1.0;

        SmartNumber HANDOFF_STALL_CURRENT = new SmartNumber("Handoff/Stall Current Limit for Reverse", 30.0);
        double HANDOFF_STALL_DEBOUNCE_SEC = 0.5;
    }

    public interface Intake {
        Rotation2d PIVOT_STOW_ANGLE = Rotation2d.fromDegrees(71.0); 
        Rotation2d PIVOT_DEPLOY_ANGLE = Rotation2d.fromDegrees(-10.0);
        Rotation2d PIVOT_DIGEST_ANGLE = Rotation2d.fromDegrees(30);

        Rotation2d PIVOT_ANGLE_TOLERANCE = Rotation2d.fromDegrees(5.0); 

        Rotation2d PIVOT_MAX_ANGLE = Rotation2d.fromDegrees(76.4);
        Rotation2d PIVOT_MIN_ANGLE = Rotation2d.fromDegrees(-10.0);

        Rotation2d THRESHOLD_TO_START_ROLLERS = Rotation2d.fromDegrees(10.0);

        Rotation2d ANGLE_THRESHOLD_FOR_HOLDING_VOLTAGE = Rotation2d.fromDegrees(15.0);
        double HOMING_VOLTAGE = 3.0;
        
        double PUSHDOWN_VOLTAGE = -3.0;
        double PUSHDOWN_CURRENT_TELEOP = -75.0;//new SmartNumber("Intake/Pushdown Current", -65.0); //TODO: GET ACTUAL TYTY
        double PUSHDOWN_CURRENT_AUTON = -80.0;

        double GEAR_RATIO = 32.0/20.0 * 64.0/18.0 * 60.0/8.0;
        
        double PIVOT_STALL_CURRENT = 0; //TODO: set value
        double PIVOT_STALL_DEBOUNCE = 1.0; //TODO: VERIFY

        double ROLLER_STALL_DEBOUNCE = 0.05; //TODO: VERIFY
        double ROLLER_STALL_CURRENT = 50.0;
    }

    public interface Spindexer {
        double FORWARD_DUTY_CYCLE = 1.0;
        double ANTI_POPCORN_DUTY_CYCLE = 0.2;
        double REVERSE_DUTY_CYCLE = -1.0;
        double STOP_SPEED = 0.0;
        double REVERSE_TIME = 2.0;
        double ANTI_POPCORN_FREQ = 100;
        double ANTI_POPCORN_LENGTH = 10;

        double RPM_TOLERANCE = 800.0;
        double TOLERANCE_TO_START_INTAKE_ROLLERS_DURING_SCORING_ROUTINE = 1500.0;
        double STALL_CURRENT_LIMIT = 40.0; // random number as of 3/9

        double IS_EMPTY_AMPERAGE = 10; //TODO: update IS EMPTY VALUE

        /* CONSTANTS */
        double GEAR_RATIO = 11.04 / 1.0;
    }
    
    public interface Superstructure {
        public final double SHOOTER_TOLERANCE_RPM_HIGH = 50.0;
        public final double SHOOTER_TOLERANCE_RPM_LOW = 80.0;        
        public final double SHOOTER_SOTM_TOLERANCE_RPM_HIGH = 100.0;
        public final double SHOOTER_SOTM_TOLERANCE_RPM_LOW = 100.0;
        public final double SHOOTER_FOTM_TOLERANCE_RPM_HIGH = 150.0;
        public final double SHOOTER_FOTM_TOLERANCE_RPM_LOW = 250.0;

        public final double IS_EMPTY_RPM_TOLERANCE = 150; //TODO: update IS EMPTY VALUE
        public final double IS_EMPTY_DEBOUNCE_TIME = 0.4; //TODO: update IS EMPTY VALUE
        
        public final Rotation2d HOOD_TOLERANCE = Rotation2d.fromDegrees(0.5);
        public final Rotation2d HOOD_SOTM_TOLERANCE = Rotation2d.fromDegrees(2);

        public interface AngleInterpolation {
            double[][] distanceAngleInterpolationValues = {
                {0.96, Units.degreesToRadians(15)},
                {1.22, Units.degreesToRadians(20)},
                {2.15, Units.degreesToRadians(27)},
                {3.38, Units.degreesToRadians(34)}, 
                {4.43, Units.degreesToRadians(39)},
                {5.66, Units.degreesToRadians(39)},
                {6.44, Units.degreesToRadians(44)}
            };
        }

        public interface RPMInterpolation{
            double[][] distanceRPMInterpolationValues = {
                {0.96, 2800},
                {1.22, 2600.0},
                {2.15, 2805.0},
                {3.38, 3075},
                {4.43, 3350.0},
                {5.66, 3650.0},
                {6.44, 3800.0},
                {10.0, 5766.0} // THIS POINT IS AN EXTRAPOLATION
            };
        }

        public interface TOFInterpolation{
            double[][] distanceTOFInterpolationValues = {
                {0.96, 1.055},
                {1.22, 0.965}, // seconds
                {2.15, 1.01},
                {3.38, 1.02},  
                {4.43, 1.165},
                {5.50, 1.21},
                {6.44, 1.255},
                {10.0, 1.46} // THIS POINT IS AN EXTRAPOLATION
            };
        }

        public interface FerryRPMInterpolation {
            double[][] ferryDistanceRPMInterpolation = {
                //Lab
                {1, 2000},
                {5.16, 3300.0},
                {6.94, 3600.0},
                {7.87, 3800.0},
                {9.77, 4300.0},
                {10.694, 4700.0},       //STARTING FROM HERE THE DATA IS EXTRAPOLATED!!!
                {11.516, 4900.0}
                // {11.516, 5200.0},
                // {12.416, 5500.0},       // AFTER OPP ALLIANCE ZONE, RPM SHOULD BE AT 5500 -blay
                // {13.316, 5500.0},
                // {14.216, 5600.0}
            };
        }

        public interface FerryTOFInterpolation {
            double [][] FerryTOFInterpolationInterpolation = {
                {5.16, 1.16},
                {6.94, 1.37},
                {7.87, 1.57},
                {9.77, 1.64},
                {10.694, 1.765},  // extrapolated
                {11.516, 1.838},  // extrapolated
                {12.416, 1.914},  // extrapolated
                {13.316, 1.988},  // extrapolated
                {14.216, 2.060},  // extrapolated
                {15.148, 2.131},  // extrapolated
                {16.54, 2.234},  // extrapolated (field length)
            };
        }

        public interface Shooter {
            
            public final double IS_SHOOTING_CURRENT = 25.0;
            
            public final double GEAR_RATIO = 1.0;
            public final double FLYWHEEL_RADIUS = Units.inchesToMeters(3.965 / 2.0);
            
            public interface RPM {
                public final SmartNumber MANUAL_OVERRIDE = new SmartNumber("InterpolationTesting/Shoot State Target RPM", 3500.0);

                public final double REVERSE = 0.0;
                public final double KB = 2675.0;
                public final double LEFT_CORNER = 3650.0;
                public final double RIGHT_CORNER = 3650.0;
            }
        }

        public interface Hood {
            /**
             * DISCLAIMER: THERE IS NO ABS ENCODER ON THE BOT RN
             * The absolute encoder is mounted on a 11:1 gear reduction relative to the
             * hood mechanism. This means:
             *
             *  - The encoder rotates 11 times for every 1 full rotation of the hood.
             *  - The hood's physical range of motion is only 30 degrees.
             *
             * Because 30° * 11 = 330°, the encoder will never exceed 360° over the
             * entire hood travel. Therefore, the absolute encoder reading (0–330°)
             * uniquely maps to the hood’s 0–30° mechanical range without any ambiguity.
             *
             */
            public final double GEAR_RATIO = 125.4;
            public final double ENCODER_TO_MECH = 11.0;
            public final double HOOD_HOMING_VOLTAGE = 0.5;

            public final Rotation2d ENCODER_OFFSET = Rotation2d.fromRotations(0.795);

            public final Rotation2d MAX_FROM_HORIZON = Rotation2d.fromDegrees(45.0);
            public final Rotation2d MIN_FROM_HORIZON = Rotation2d.fromDegrees(15.0);
            public final Rotation2d SOFT_LIMIT = Rotation2d.fromDegrees(.25);
            public final Rotation2d FORWARD_SOFT_LIMIT = MAX_FROM_HORIZON.minus(SOFT_LIMIT);
            public final Rotation2d REVERSE_SOFT_LIMIT = MIN_FROM_HORIZON.plus(SOFT_LIMIT);

            public final double STALL_CURRENT_LIMIT = 0.55;
            public final double STALL_DEBOUNCE = 0.5;

            public interface Angles {
                public final SmartNumber MANUAL_OVERRIDE = new SmartNumber("InterpolationTesting/Shoot State Target Angle (deg)", 20.0);
                public final Rotation2d MAX = FORWARD_SOFT_LIMIT;
                public final Rotation2d MIN = REVERSE_SOFT_LIMIT;
                public final Rotation2d FERRY_ANGLE = MAX;//Rotation2d.fromDegrees(44.0);

                public final Rotation2d STOW = Rotation2d.fromDegrees(21.0);
                public final Rotation2d KB = Rotation2d.fromDegrees(20.0);
                public final Rotation2d LEFT_CORNER = Rotation2d.fromDegrees(39.0);
                public final Rotation2d RIGHT_CORNER = Rotation2d.fromDegrees(39.0);
            }
        }

        public interface Turret {
            public final Rotation2d MAX_VEL = new Rotation2d(Units.degreesToRadians(600.0));
            public final Rotation2d MAX_ACCEL = new Rotation2d(Units.degreesToRadians(600.0));
            public final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2.0);
            public final SmartNumber SOTM_TOLERANCE_THRESHOLD_METERS = new SmartNumber("Superstructure/Turret/SOTM Tolerance Dist Threshold (Meters)", 1.75);
            public final SmartNumber SOTM_TOLERANCE_CLOSE = new SmartNumber("Superstructure/Turret/SOTM Tolerance (Close)", 10.0);
            public final SmartNumber SOTM_TOLERANCE_FAR = new SmartNumber("Superstructure/Turret/SOTM Tolerance (Far)", 6.0);//Rotation2d.fromDegrees(10.0);
            public final Rotation2d FOTM_TOLERANCE = Rotation2d.fromDegrees(10.0);
            
            public final Rotation2d KB = Rotation2d.fromDegrees(0.0);
            public final Rotation2d LEFT_CORNER = Rotation2d.fromDegrees(-233.0);
            public final Rotation2d RIGHT_CORNER = Rotation2d.fromDegrees(53.0);
            
            double RESOLUTION_OF_ABSOLUTE_ENCODER = 0.1;
            double WRAP_DEBOUNCE = 0.5;
            double SETPOINT_FILTER_THRESHOLD_DEG = 0.5;

            Rotation2d MAX_THEORETICAL_ROTATION = Rotation2d.fromDegrees(612);
            Rotation2d MIN_THEORETICAL_ROTATION = Rotation2d.fromDegrees(-612);
            
            /* CONSTANTS */
            public final double RANGE_CW = 90.0;//-360.0;
            public final double RANGE_CCW = -360.0;//85.0; // -397.0 is further
        
            public final Rotation2d GAIN_SWITCHING_THRESHOLD_START = Rotation2d.fromDegrees(30);
            public final Rotation2d GAIN_SWITCHING_THRESHOLD_END = Rotation2d.fromDegrees(3);

            public final Transform2d TURRET_OFFSET = new Transform2d(Units.inchesToMeters(-4.0), Units.inchesToMeters(8.0), Rotation2d.kZero);
            public final double TURRET_HEIGHT = Units.inchesToMeters(0.0);
        
            public final double GEAR_RATIO_MOTOR_TO_MECH = (60.0 / 9.0) * (95.0 / 12.0); //1425.0 / 36.0;

            // public final SmartNumber ARBITRARY_kA_TERM = new SmartNumber("Superstructure/Turret/Gains/arbitrary kA", 1.5);
        
            public interface BigGear {
                public final int TEETH = 95;
            }
        
            public interface Encoder17t {
                public final int TEETH = 17;
                public final Rotation2d OFFSET = Rotation2d.fromRotations(-0.185);
            }
            
            public interface Encoder18t {
                public final int TEETH = 18;
                public final Rotation2d OFFSET = Rotation2d.fromRotations(-0.814);
            }
        
            public interface SoftwareLimit {
                public final double FORWARD_MAX_ROTATIONS = 210.0 / 360.0;
                public final double BACKWARDS_MAX_ROTATIONS = -210.0 / 360.0;
            }
        }

        public interface SOTM {
            public final int MAX_ITERATIONS = 10;
            double TIME_TOLERANCE = 1e-3;
            SmartNumber UPDATE_DELAY = new SmartNumber("Superstructure/SOTM/update delay", 0.05);
        }
    }
    

    public interface Swerve {
        public final double MODULE_VELOCITY_DEADBAND_M_PER_S = 0.1;
        public final double ROTATIONAL_DEADBAND_RAD_PER_S = 0.1;
        double MAX_ACCEPTABLE_POSE_DELTA_METERS = Math.sqrt(Field.LENGTH * Field.LENGTH + Field.WIDTH * Field.WIDTH); //TODO: Might wanna make this smaller.
        double MAX_ACCEPTABLE_VISION_DEVIATION_METERS = 1.0;

        public interface Constraints {
            public final double MAX_VELOCITY_M_PER_S = 4.16; 
            public final double MAX_VELOCITY_SOTM_M_PER_S = 1.75;
            public final double MAX_VELOCITY_FOTM_M_PER_S = 4.16;

            public final double MAX_ANGULAR_VEL_RAD_PER_S = Units.degreesToRadians(300.0);
            public final double MAX_ANGULAR_VEL_SOTM_RAD_PER_S = Units.degreesToRadians(75.0);
            public final double MAX_ANGULAR_VEL_FOTM_RAD_PER_S = Units.degreesToRadians(150.0);

            public final double MAX_ACCEL_M_PER_S_SQUARED = 15.0;
            public final double MAX_ACCEL_M_PER_S_SQUARED_SOTM = 4.0;
            public final double MAX_ACCEL_M_PER_S_SQUARED_FOTM = 15.0;
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

    public interface LED {

        LEDPattern PASSING_TRENCH = LEDPattern.solid(Color.kRed);
        LEDPattern IS_BEHIND_HUB = LEDPattern.solid(Color.kRed);

        // LEDPattern CLIMB_ALIGNING = LEDPattern.solid(Color.kYellow);
        // LEDPattern CLIMB_ALIGNED = LEDPattern.solid(Color.kGreen);
        // LEDPattern CLIMBING = LEDPattern.solid(Color.kRed);

        LEDPattern TURRET_WRAPPING = LEDPattern.solid(Color.kRed);
        LEDPattern LEFT_WARNING = LEDPattern.solid(Color.kBlack); // TBD
        LEDPattern RIGHT_WARNING = LEDPattern.solid(Color.kBlack); // TBD

        LEDPattern SHOOT_IN_PLACE = LEDPattern.solid(Color.kPurple);

        LEDPattern SOTM_ON = LEDPattern.solid(Color.kCyan);
        LEDPattern FOTM_ON = LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(1 / 120.0));

        LEDPattern LEFT_CORNER = LEDPattern.solid(Color.kPurple);
        LEDPattern RIGHT_CORNER = LEDPattern.solid(Color.kBlue);
        
        LEDPattern KB_DISTANCE = LEDPattern.solid(Color.kPink);

        LEDPattern REVERSE = LEDPattern.solid(Color.kWhite);
        LEDPattern STOP_ROLLERS = LEDPattern.solid(Color.kYellow);

        LEDPattern RESET_HEADING = LEDPattern.solid(Color.kYellow);
        LEDPattern X_WHEELS = LEDPattern.solid(Color.kRed);

        LEDPattern INTAKE_STOW = LEDPattern.solid(Color.kBrown);        //broken
        LEDPattern INTAKE_DEPLOYED = LEDPattern.solid(Color.kOrange);   //broken

        LEDPattern DISABLED_ALIGNED = LEDPattern.solid(Color.kGreen);
        // LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed, Color.kWhite).scrollAtRelativeSpeed(Percent.per(Second).of(25));

        public final int DESIRED_TAGS_WHEN_DISABLED = 2;
        public final int LED_LENGTH = 9; // TBA

    }

    public interface Vision {
        public final Vector<N3> MT1_STDEVS = VecBuilder.fill(0.5, 0.5, 1.0);
        public final Vector<N3> MT2_STDEVS = VecBuilder.fill(0.7, 0.7, 694694.0);
        public final int RESET_IMU_INDEX = 1;
        public final int INTERNAL_EXTERNAL_ASSIST_INDEX = 4;
        public final Translation2d INVALID_POSITION = new Translation2d(8.2705, 4.0345);
        public final double INVALID_POSITION_TOLERANCE_M = 0.05;
        public final double MAX_ANGULAR_VELOCITY_RAD_SEC = 2 * Math.PI;
        double MIN_TAG_AREA = 5; //TODO: MAKE SURE THIS IS A GOOD VALUE!!!

        SmartBoolean HDR_ENABLED = new SmartBoolean("Vision/HDR Enabled?", false);
        double HDR_TIMEOUT_SEC = 0.25;

        public final double BUZZ_DEBOUNCE = 0.25;
    }
}