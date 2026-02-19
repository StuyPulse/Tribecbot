/************************ PROJECT 2026 ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.path.PathConstraints;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    double DT = 0.020;
    boolean DEBUG_MODE = true;
    CANBus CANIVORE = new CANBus("canivore", "./logs/example.hoot");

    public interface EnabledSubsystems {
        SmartBoolean SWERVE = new SmartBoolean("Enabled Subsystems/Swerve Is Enabled", true);
        SmartBoolean TURRET = new SmartBoolean("Enabled Subsystems/Turret Is Enabled", false);
        SmartBoolean FEEDER = new SmartBoolean("Enabled Subsystems/Turret Is Enabled", false);
        SmartBoolean SPINDEXER = new SmartBoolean("Enabled Subsystems/Turret Is Enabled", false);
        SmartBoolean CLIMBER = new SmartBoolean("Enabled Subsystems/Turret Is Enabled", false);
        SmartBoolean HOOD = new SmartBoolean("Enabled Subsystems/Superstructure Is Enabled", false);
        SmartBoolean SHOOTER = new SmartBoolean("Enabled Subsystems/Turret Is Enabled", false);
        SmartBoolean LIMELIGHT = new SmartBoolean("Enabled Subsystems/Limelight Is Enabled", false);
    }

    public interface ClimberHopper {
        // TODO: GET THESE
        // Voltages
        double CLIMBER_UP = 2;
        double CLIMBER_DOWN = -3;
        double HOPPER_DOWN = -3;
        double HOPPER_UP = 2;

        double MASS_KG = 1;
        
        double STALL = 10;
        double EXTENDED = 0;
        double RETRACTED = 0;

        double ROTATIONS_AT_BOTTOM = 0;
        // TODO: get these limits
        double MIN_HEIGHT_METERS = 0;
        double MAX_HEIGHT_METERS = 10;

        double DEBOUNCE = 0.25;

        double GYRO_TOLERANCE = 0;
        double HEIGHT_TOLERANCE = 1;

        double RAMP_RATE = 50;

        double DRUM_RADIUS_METERS = ((MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / (Encoders.NUM_ROTATIONS_TO_REACH_TOP / Encoders.GEARING)) / 2 / Math.PI;

        public interface Encoders {
            // TODO: get these
            double GEARING = 52.0/12.0;

            double NUM_ROTATIONS_TO_REACH_TOP = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / (0.480 / 13); // Number of rotations that the motor has to spin, NOT the gear
            double POSITION_CONVERSION_FACTOR = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / NUM_ROTATIONS_TO_REACH_TOP;
            double VELOCITY_CONVERSION_FACTOR = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / NUM_ROTATIONS_TO_REACH_TOP / 60;
        }
    }

    public interface Feeder {

    }

    public interface Intake {

    }
    public interface Spindexeer {

    }
    public interface HoodedShooter {

    }
    public interface Turret {
        
    }

    public interface Swerve {
        double MODULE_VELOCITY_DEADBAND_M_PER_S = 0.1;
        double ROTATIONAL_DEADBAND_RAD_PER_S = 0.1;
        public interface Motion {
            SmartNumber MAX_VELOCITY = new SmartNumber("Swerve/Motion/Max Velocity", 2.5);
            SmartNumber MAX_ACCELERATION = new SmartNumber("Swerve/Motion/Max Acceleration", 2.5);
            SmartNumber MAX_ANGULAR_VELOCITY = new SmartNumber("Swerve/Motion/Max Angular Velocity", Units.degreesToRadians(540));
            SmartNumber MAX_ANGULAR_ACCELERATION = new SmartNumber("Swerve/Motion/Max Angular Acceleration", Units.degreesToRadians(720));

            PathConstraints DEFAULT_CONSTRAINTS =
                new PathConstraints(
                    MAX_VELOCITY.get(),
                    MAX_ACCELERATION.get(),
                    MAX_ANGULAR_VELOCITY.get(),
                    MAX_ANGULAR_ACCELERATION.get());
        }

        public interface Turn {
            // boolean INVERTED = true;
            // double GEAR_RATIO = (150.0 / 7.0); // 21.4285714286
        }

        public interface Drive {
            double L2 = ((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)); // 6.74607175
            double L3 = ((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)); // 6.12244898
            double L4 = ((50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0)); // 5.35714285714

            // double WHEEL_DIAMETER = 4;
            // double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

            // double GEAR_RATIO = Swerve.Drive.L4;
        }
        
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
                SmartNumber THETA_TOLERANCE = new SmartNumber("Angle Tolerance", 2);

                Pose2d POSE_TOLERANCE = new Pose2d(
                    Units.inchesToMeters(2.0), 
                    Units.inchesToMeters(2.0), 
                    Rotation2d.fromDegrees(2.0));

                double MAX_VELOCITY_WHEN_ALIGNED = 0.15;

                double ALIGNMENT_DEBOUNCE = 0.15;
            }

            public interface Targets {

            }
        }
    }
   
    public interface Vision {
        Vector<N3> MT1_STDEVS = VecBuilder.fill(0.5, 0.5, 1.0);
        Vector<N3> MT2_STDEVS = VecBuilder.fill(0.7, 0.7, 694694);
    }

    public interface Driver {
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
