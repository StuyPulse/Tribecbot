/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public interface Motors {

    public interface ClimberHopper {
        public final TalonFXConfig MOTOR = new TalonFXConfig()
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            .withCurrentLimitAmps(50.0)
            .withSupplyCurrentLimitAmps(50.0)
            .withRampRate(Settings.ClimberHopper.RAMP_RATE);

        public final SoftwareLimitSwitchConfigs SOFT_LIMITS = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(Settings.ClimberHopper.ROTATIONS_AT_BOTTOM + Settings.ClimberHopper.Constants.NUM_ROTATIONS_TO_REACH_TOP)
            .withReverseSoftLimitThreshold(Settings.ClimberHopper.ROTATIONS_AT_BOTTOM);
    }

    public interface Handoff {
        public final TalonFXConfig HANDOFF = new TalonFXConfig()
            .withCurrentLimitAmps(80.0)
            .withRampRate(0.25)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInvertedValue(InvertedValue.CounterClockwise_Positive)
            .withFFConstants(Gains.Handoff.kS, Gains.Handoff.kV, Gains.Handoff.kA, 0)
            .withPIDConstants(Gains.Handoff.kP, Gains.Handoff.kI, Gains.Handoff.kD, 0)
            .withSensorToMechanismRatio(Settings.Handoff.GEAR_RATIO);
    }

    public interface HoodedShooter {
        public interface Hood {
            public final TalonFXConfig HOOD = new TalonFXConfig()
                .withCurrentLimitAmps(80.0)
                .withRampRate(0.25)
                .withNeutralMode(NeutralModeValue.Brake)
                .withInvertedValue(InvertedValue.Clockwise_Positive)
                .withPIDConstants(Gains.HoodedShooter.Hood.kP, Gains.HoodedShooter.Hood.kI, Gains.HoodedShooter.Hood.kD, 0)
                .withFFConstants(Gains.HoodedShooter.Hood.kS, Gains.HoodedShooter.Hood.kV, Gains.HoodedShooter.Hood.kA, 0)
                .withSensorToMechanismRatio(Settings.HoodedShooter.Hood.GEAR_RATIO);

            public final Slot0Configs SLOT_0 = new Slot0Configs()
                .withKP(Gains.HoodedShooter.Hood.kP)
                .withKI(Gains.HoodedShooter.Hood.kI)
                .withKD(Gains.HoodedShooter.Hood.kD)
                .withKS(Gains.HoodedShooter.Hood.kS)
                .withKV(Gains.HoodedShooter.Hood.kV)
                .withKA(Gains.HoodedShooter.Hood.kA)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

            public final SoftwareLimitSwitchConfigs SOFT_LIMITS = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(Settings.HoodedShooter.Angles.MAX_ANGLE.getRotations())
                .withReverseSoftLimitThreshold(Settings.HoodedShooter.Angles.MIN_ANGLE.getRotations());

            public final CANcoderConfiguration HOOD_ENCODER = new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                    .withAbsoluteSensorDiscontinuityPoint(1.0)
                    .withMagnetOffset(Settings.HoodedShooter.Hood.ENCODER_OFFSET.getRotations()));
        }

        public interface Shooter {
            public final TalonFXConfig SHOOTER = new TalonFXConfig()
                .withCurrentLimitEnable(false)
                .withNeutralMode(NeutralModeValue.Coast)
                .withInvertedValue(InvertedValue.CounterClockwise_Positive)
                .withPIDConstants(Gains.HoodedShooter.Shooter.kP, Gains.HoodedShooter.Shooter.kI,
                        Gains.HoodedShooter.Shooter.kD, 0)
                .withFFConstants(Gains.HoodedShooter.Shooter.kS, Gains.HoodedShooter.Shooter.kV,
                        Gains.HoodedShooter.Shooter.kA, 0)
                .withSensorToMechanismRatio(Settings.HoodedShooter.Shooter.GEAR_RATIO);
        }
    }

    public interface Intake {
        public final TalonFXConfig ROLLER = new TalonFXConfig()
            .withCurrentLimitAmps(40.0)
            .withRampRate(0.25)
            .withNeutralMode(NeutralModeValue.Coast)
            .withInvertedValue(InvertedValue.Clockwise_Positive);

        public final TalonFXConfig PIVOT = new TalonFXConfig()
            .withCurrentLimitAmps(40.0)
            .withRampRate(0.25)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withPIDConstants(Gains.Intake.Pivot.kP, Gains.Intake.Pivot.kI, Gains.Intake.Pivot.kD, 0)
            .withFFConstants(Gains.Intake.Pivot.kS, Gains.Intake.Pivot.kV, Gains.Intake.Pivot.kA, Gains.Intake.Pivot.kG, 0)
            .withMotionProfile(Settings.Intake.PIVOT_MAX_VEL.getRotations(), Settings.Intake.PIVOT_MAX_ACCEL.getRotations());
    }

    public interface Spindexer {
        public final TalonFXConfig SPINDEXER = new TalonFXConfig()
            .withCurrentLimitEnable(false)
            .withRampRate(0.25)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withFFConstants(Gains.Spindexer.kS, Gains.Spindexer.kV, Gains.Spindexer.kA, 0)
            .withPIDConstants(Gains.Spindexer.kP, Gains.Spindexer.kI, Gains.Spindexer.kD, 0)
            .withSensorToMechanismRatio(Settings.Spindexer.Constants.GEAR_RATIO);
    }

    public interface Turret {
        public final TalonFXConfig TURRET = new TalonFXConfig()
            .withRampRate(0.25)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withPIDConstants(Gains.Turret.kP, 0.0, Gains.Turret.kD, 0)
            .withFFConstants(Gains.Turret.kS, 0.0, 0.0, 0)
            .withSensorToMechanismRatio(Settings.Turret.Constants.GEAR_RATIO_MOTOR_TO_MECH);

        public final CANCoderConfig ENCODER_17T = new CANCoderConfig()
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(1.0);

        public final CANCoderConfig ENCODER_18T = new CANCoderConfig()
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(1.0);

        public final SoftwareLimitSwitchConfigs SOFT_LIMIT = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(Settings.Turret.Constants.SoftwareLimit.FORWARD_MAX_ROTATIONS)
            .withReverseSoftLimitThreshold(Settings.Turret.Constants.SoftwareLimit.BACKWARDS_MAX_ROTATIONS);
    }

    public static class CANCoderConfig {
        private final CANcoderConfiguration configuration = new CANcoderConfiguration();
        private final MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();

        public void configure(CANcoder encoder) {
            CANcoderConfiguration defaultConfig = new CANcoderConfiguration();
            encoder.getConfigurator().apply(defaultConfig);

            encoder.getConfigurator().apply(configuration);
        }

        public CANcoderConfiguration getConfiguration() {
            return this.configuration;
        }

        // MAGNET SENSOR CONFIGS

        public CANCoderConfig withSensorDirection(SensorDirectionValue sensorDirection) {
            magnetSensorConfigs.SensorDirection = sensorDirection;

            configuration.withMagnetSensor(magnetSensorConfigs);

            return this;
        }

        public CANCoderConfig withAbsoluteSensorDiscontinuityPoint(double discontinuityPoint) {
            magnetSensorConfigs.AbsoluteSensorDiscontinuityPoint = discontinuityPoint;

            configuration.withMagnetSensor(magnetSensorConfigs);

            return this;
        }

        public CANCoderConfig withMagnetOffset(double magnetOffset) {
            magnetSensorConfigs.MagnetOffset = magnetOffset;

            configuration.withMagnetSensor(magnetSensorConfigs);

            return this;
        }
    }

    public static class TalonFXConfig {
        private final TalonFXConfiguration configuration = new TalonFXConfiguration();
        private final Slot0Configs slot0Configs = new Slot0Configs();
        private final Slot1Configs slot1Configs = new Slot1Configs();
        private final Slot2Configs slot2Configs = new Slot2Configs();
        private final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        private final ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        private final OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
        private final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        private final FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        private final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

        public void configure(TalonFX motor) {
            TalonFXConfiguration defaultConfig = new TalonFXConfiguration();
            motor.getConfigurator().apply(defaultConfig);

            motor.getConfigurator().apply(configuration);
        }

        // SLOT CONFIGS

        public TalonFXConfig withPIDConstants(double kP, double kI, double kD, int slot) {
            switch (slot) {
                case 0:
                    slot0Configs.kP = kP;
                    slot0Configs.kI = kI;
                    slot0Configs.kD = kD;
                    configuration.withSlot0(slot0Configs);
                    break;
                case 1:
                    slot1Configs.kP = kP;
                    slot1Configs.kI = kI;
                    slot1Configs.kD = kD;
                    configuration.withSlot1(slot1Configs);
                    break;
                case 2:
                    slot2Configs.kP = kP;
                    slot2Configs.kI = kI;
                    slot2Configs.kD = kD;
                    configuration.withSlot2(slot2Configs);
                    break;
            }
            return this;
        }

        public TalonFXConfig withFFConstants(double kS, double kV, double kA, int slot) {
            return withFFConstants(kS, kV, kA, 0.0, slot);
        }

        public TalonFXConfig withFFConstants(double kS, double kV, double kA, double kG, int slot) {
            switch (slot) {
                case 0:
                    slot0Configs.kS = kS;
                    slot0Configs.kV = kV;
                    slot0Configs.kA = kA;
                    slot0Configs.kG = kG;
                    configuration.withSlot0(slot0Configs);
                    break;
                case 1:
                    slot1Configs.kS = kS;
                    slot1Configs.kV = kV;
                    slot1Configs.kA = kA;
                    slot1Configs.kG = kG;
                    configuration.withSlot1(slot1Configs);
                    break;
                case 2:
                    slot2Configs.kS = kS;
                    slot2Configs.kV = kV;
                    slot2Configs.kA = kA;
                    slot2Configs.kG = kG;
                    configuration.withSlot2(slot2Configs);
                    break;
            }
            return this;
        }

        public TalonFXConfig withGravityType(GravityTypeValue gravityType) {
            slot0Configs.GravityType = gravityType;
            slot1Configs.GravityType = gravityType;
            slot2Configs.GravityType = gravityType;

            configuration.withSlot0(slot0Configs);
            configuration.withSlot1(slot1Configs);
            configuration.withSlot2(slot2Configs);

            return this;
        }

        // MOTOR OUTPUT CONFIGS

        public TalonFXConfig withInvertedValue(InvertedValue invertedValue) {
            motorOutputConfigs.Inverted = invertedValue;

            configuration.withMotorOutput(motorOutputConfigs);

            return this;
        }

        public TalonFXConfig withNeutralMode(NeutralModeValue neutralMode) {
            motorOutputConfigs.NeutralMode = neutralMode;

            configuration.withMotorOutput(motorOutputConfigs);

            return this;
        }

        // RAMP RATE CONFIGS

        public TalonFXConfig withRampRate(double rampRate) {
            closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = rampRate;
            closedLoopRampsConfigs.TorqueClosedLoopRampPeriod = rampRate;
            closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = rampRate;

            openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = rampRate;
            openLoopRampsConfigs.TorqueOpenLoopRampPeriod = rampRate;
            openLoopRampsConfigs.VoltageOpenLoopRampPeriod = rampRate;

            configuration.withClosedLoopRamps(closedLoopRampsConfigs);
            configuration.withOpenLoopRamps(openLoopRampsConfigs);

            return this;
        }

        // CURRENT LIMIT CONFIGS

        public TalonFXConfig withCurrentLimitAmps(double currentLimitAmps) {
            currentLimitsConfigs.StatorCurrentLimit = currentLimitAmps;
            currentLimitsConfigs.StatorCurrentLimitEnable = true;

            configuration.withCurrentLimits(currentLimitsConfigs);

            return this;
        }

        public TalonFXConfig withLowerLimitSupplyCurrent(double currentLowerLimitAmps) {
            currentLimitsConfigs.SupplyCurrentLowerLimit = currentLowerLimitAmps;
            currentLimitsConfigs.StatorCurrentLimitEnable = true;

            configuration.withCurrentLimits(currentLimitsConfigs);

            return this;
        }

        public TalonFXConfig withSupplyCurrentLimitAmps(double currentLimitAmps) {
            currentLimitsConfigs.SupplyCurrentLimit = currentLimitAmps;
            currentLimitsConfigs.SupplyCurrentLimitEnable = true;

            configuration.withCurrentLimits(currentLimitsConfigs);

            return this;
        }

        public TalonFXConfig withCurrentLimitEnable(boolean enabled) {
            currentLimitsConfigs.SupplyCurrentLimitEnable = enabled;
            currentLimitsConfigs.StatorCurrentLimitEnable = enabled;

            configuration.withCurrentLimits(currentLimitsConfigs);

            return this;
        }

        // MOTION MAGIC CONFIGS

        public TalonFXConfig withMotionProfile(double maxVelocity, double maxAcceleration) {
            motionMagicConfigs.MotionMagicCruiseVelocity = maxVelocity;
            motionMagicConfigs.MotionMagicAcceleration = maxAcceleration;

            configuration.withMotionMagic(motionMagicConfigs);

            return this;
        }

        // FEEDBACK CONFIGS

        public TalonFXConfig withRemoteSensor(
                int ID, FeedbackSensorSourceValue source, double rotorToSensorRatio) {
            feedbackConfigs.FeedbackRemoteSensorID = ID;
            feedbackConfigs.FeedbackSensorSource = source;
            feedbackConfigs.RotorToSensorRatio = rotorToSensorRatio;

            configuration.withFeedback(feedbackConfigs);

            return this;
        }

        public TalonFXConfig withSensorToMechanismRatio(double sensorToMechanismRatio) {
            feedbackConfigs.SensorToMechanismRatio = sensorToMechanismRatio;

            configuration.withFeedback(feedbackConfigs);

            return this;
        }
    }
}
