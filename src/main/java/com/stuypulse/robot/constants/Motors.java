/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

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
import com.ctre.phoenix6.configs.SlotConfigs;
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
        private final SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();

        private final double[] lastKP = new double[3];
        private final double[] lastKI = new double[3];
        private final double[] lastKD = new double[3];
        private final double[] lastKS = new double[3];
        private final double[] lastKV = new double[3];
        private final double[] lastKA = new double[3];

        public void configure(TalonFX motor) {
            TalonFXConfiguration defaultConfig = new TalonFXConfiguration();
            motor.getConfigurator().apply(defaultConfig);

            motor.getConfigurator().apply(configuration);
        }

        public TalonFXConfiguration getConfiguration() {
            return this.configuration;
        }

        // SMARTNUMBER TUNABLE GAINS FOR TALONFX MOTOR CONTROLLERS
        // Note that this should ONLY be used during testing/debugging and not for competition code
        public void updateGainsConfig(TalonFX motor, int slot, SmartNumber kP, SmartNumber kI, SmartNumber kD, SmartNumber kS, SmartNumber kV, SmartNumber kA) {
            if (slot != 0 && slot != 1 && slot != 2) {
                return;
            }

            double currentKP = kP.getAsDouble();
            double currentKI = kI.getAsDouble();
            double currentKD = kD.getAsDouble();
            double currentKS = kS.getAsDouble();
            double currentKV = kV.getAsDouble();
            double currentKA = kA.getAsDouble();

            boolean changed =
                currentKP != lastKP[slot] ||
                currentKI != lastKI[slot] ||
                currentKD != lastKD[slot] ||
                currentKS != lastKS[slot] ||
                currentKV != lastKV[slot] ||
                currentKA != lastKA[slot];

            if (!changed) {
                return;
            }

            SlotConfigs gainConfig = new SlotConfigs()
                .withKP(currentKP)
                .withKI(currentKI)
                .withKD(currentKD)
                .withKS(currentKS)
                .withKV(currentKV)
                .withKA(currentKA);

            gainConfig.SlotNumber = slot;

            motor.getConfigurator().apply(gainConfig);

            lastKP[slot] = currentKP;
            lastKI[slot] = currentKI;
            lastKD[slot] = currentKD;
            lastKS[slot] = currentKS;
            lastKV[slot] = currentKV;
            lastKA[slot] = currentKA;

            switch (slot) {
                case 0:
                    motor.getConfigurator().refresh(this.getConfiguration().Slot0);
                    break;
                case 1:
                    motor.getConfigurator().refresh(this.getConfiguration().Slot1);
                    break;
                case 2:
                    motor.getConfigurator().refresh(this.getConfiguration().Slot2);
                    break;
            }
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

        public TalonFXConfig withStaticFeedforwardSign(StaticFeedforwardSignValue staticFeedforwardSign, int slot) {
            switch (slot) {
                case 0:
                    slot0Configs.StaticFeedforwardSign = staticFeedforwardSign;
                    configuration.withSlot0(slot0Configs);
                    break;
                case 1:
                    slot1Configs.StaticFeedforwardSign = staticFeedforwardSign;
                    configuration.withSlot1(slot1Configs);
                    break;
                case 2:
                    slot2Configs.StaticFeedforwardSign = staticFeedforwardSign;
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

        // SOFTWARE LIMIT CONFIGS

        public TalonFXConfig withSoftLimits(boolean forwardEnable, boolean reverseEnable, double forwardThreshold, double reverseThreshold) {
            softwareLimitSwitchConfigs.ForwardSoftLimitEnable = forwardEnable;
            softwareLimitSwitchConfigs.ReverseSoftLimitEnable = reverseEnable;
            softwareLimitSwitchConfigs.ForwardSoftLimitThreshold = forwardThreshold;
            softwareLimitSwitchConfigs.ReverseSoftLimitThreshold = reverseThreshold;

            configuration.withSoftwareLimitSwitch(softwareLimitSwitchConfigs);

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
