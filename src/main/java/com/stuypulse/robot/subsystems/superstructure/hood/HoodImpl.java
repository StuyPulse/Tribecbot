/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.hood;

import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.RobotMode;
import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import java.util.Optional;

public class HoodImpl extends Hood {
    private final Motors.TalonFXConfig hoodConfig;
    // private final Motors.CANCoderConfig hoodEncoderConfig;

    private final TalonFX hoodMotor;
    // private final CANcoder hoodEncoder;

    private final PositionVoltage controller;

    private Optional<Double> voltageOverride;

    private boolean hasUsedAbsoluteEncoder;

    private final BStream isStalling;

    private StatusSignal<Angle> hoodMotorPosition;
    private StatusSignal<Voltage> hoodMotorVoltage;
    private StatusSignal<Current> hoodMotorSupplyCurrent;
    private StatusSignal<Current> hoodMotorStatorCurrent;
    private StatusSignal<Double> hoodMotorClosedLoopError;
    private BaseStatusSignal[] signals;

    public HoodImpl() {
        hoodConfig = new Motors.TalonFXConfig()
                .withInvertedValue(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
                .withSupplyCurrentLimitAmps(80.0)
                .withStatorCurrentLimitEnabled(false)
                .withRampRate(0.25)
                .withPIDConstants(Gains.Superstructure.Hood.kP, Gains.Superstructure.Hood.kI,
                        Gains.Superstructure.Hood.kD, 0)
                .withFFConstants(Gains.Superstructure.Hood.kS, Gains.Superstructure.Hood.kV,
                        Gains.Superstructure.Hood.kA, 0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign, 0)
                .withSensorToMechanismRatio(Settings.Superstructure.Hood.GEAR_RATIO)
                .withSoftLimits(
                        true, true,
                        Settings.Superstructure.Hood.FORWARD_SOFT_LIMIT.getRotations(),
                        Settings.Superstructure.Hood.REVERSE_SOFT_LIMIT.getRotations());

        // hoodEncoderConfig = new Motors.CANCoderConfig()
        // .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        // .withAbsoluteSensorDiscontinuityPoint(1.0)
        // .withMagnetOffset(Settings.Superstructure.Hood.ENCODER_OFFSET.getRotations());

        hoodMotor = new TalonFX(Ports.Superstructure.Hood.MOTOR, Ports.RIO);
        // hoodEncoder = new CANcoder(Ports.Superstructure.Hood.THROUGHBORE_ENCODER,
        // Ports.RIO);

        hoodConfig.configure(hoodMotor);
        // hoodEncoderConfig.configure(hoodEncoder);

        controller = new PositionVoltage(getTargetAngle().getRotations())
                .withEnableFOC(true);

        voltageOverride = Optional.empty();

        isStalling = BStream
                .create(() -> hoodMotor.getSupplyCurrent()
                        .getValueAsDouble() > Settings.Superstructure.Hood.STALL_CURRENT_LIMIT) // TODO: update value in Settings after testing
                .filtered(new BDebounce.Both(Settings.Superstructure.Hood.STALL_DEBOUNCE));

        hoodMotorPosition = hoodMotor.getPosition();
        hoodMotorVoltage = hoodMotor.getMotorVoltage();
        hoodMotorSupplyCurrent = hoodMotor.getSupplyCurrent();
        hoodMotorStatorCurrent = hoodMotor.getStatorCurrent();
        hoodMotorClosedLoopError = hoodMotor.getClosedLoopError();
        signals = new BaseStatusSignal[] { hoodMotorPosition, hoodMotorVoltage, hoodMotorSupplyCurrent,
                hoodMotorStatorCurrent, hoodMotorClosedLoopError };
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(hoodMotorPosition.getValueAsDouble());
    }

    @Override
    public boolean isStalling() {
        return isStalling.getAsBoolean();
    }

    /*
     * Example:
     * Let's say the hood rotates 0.1 rotations. Then, the encoder has rotated 0.1 * 10.67 rotations
     * To convert the encoder reading to the mechanism position, we simply do (0.1 * 10.67) / 10.67 = 0.1
     */
    @Override
    public void seedHood() {
        // hoodMotor.setPosition(getAbsoluteHoodAngleDeg() / 360.0);
    }

    @Override
    public void seedHoodAtUpperHardStop() {
        hoodMotor.setPosition(Rotation2d.fromDegrees(40).getRotations());
    }

    @Override
    public void seedHoodAtLowerHardStop() {
        hoodMotor.setPosition(Rotation2d.fromDegrees(7.7).getRotations()); //empirically found with our current seed at top, then going down manually
    }

    private double getAbsoluteHoodAngleDeg() {
        return 0.0; // TODO:change back
        // return Settings.Superstructure.Hood.MIN_FROM_HORIZON.getDegrees() + hoodEncoder.getAbsolutePosition().getValueAsDouble() * 360.0 / Settings.Superstructure.Hood.ENCODER_TO_MECH;
    }

    @Override
    public void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(signals);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!hasUsedAbsoluteEncoder) {
            // seedHood();
            seedHoodAtUpperHardStop();
            hasUsedAbsoluteEncoder = true;
        }

        if (isStalling() && getState() == HoodState.HOMINGUPPER) {
            seedHoodAtUpperHardStop();
            setState(HoodState.IDLE);
            SmartDashboard.putBoolean("Superstructure/Hood/SUCCESFULLY HOMED UPPER", true);
        }
        if (isStalling() && getState() == HoodState.HOMINGLOWER) {
            seedHoodAtLowerHardStop();
            setState(HoodState.IDLE);
            SmartDashboard.putBoolean("Superstructure/Hood/SUCCESFULLY HOMED LOWER", true);
        }

        if (EnabledSubsystems.HOOD.get()) {
            if (voltageOverride.isPresent()) {
                hoodMotor.setVoltage(voltageOverride.get());
            } else if (getState() == HoodState.HOMINGUPPER && !isStalling()) {
                hoodMotor.setVoltage(Settings.Superstructure.Hood.HOOD_HOMING_VOLTAGE);
            } else if (getState() == HoodState.HOMINGLOWER && !isStalling()) {
                hoodMotor.setVoltage(-Settings.Superstructure.Hood.HOOD_HOMING_VOLTAGE);
            } else {
                hoodMotor.setControl(controller.withPosition(getTargetAngle().getRotations()));
            }
        } else {
            hoodMotor.stopMotor();
        }

        SmartDashboard.putBoolean("Superstructure/Hood/Has Used Absolute Encoder", hasUsedAbsoluteEncoder);

        SmartDashboard.putBoolean("Prematch Checks/Hood at Top?", getAngle().getDegrees() > 39.0);
        SmartDashboard.putNumber("Superstructure/Hood/Correct Hood Angle (deg)", getAbsoluteHoodAngleDeg());
        SmartDashboard.putNumber("Superstructure/Hood/Closed Loop Error (deg)",
                hoodMotorClosedLoopError.getValueAsDouble() * 360.0);

        if (Settings.DEBUG_MODE.get()) {
            SmartDashboard.putNumber("Superstructure/Hood/Applied Voltage (amps)", hoodMotorVoltage.getValueAsDouble());
            SmartDashboard.putNumber("Superstructure/Hood/Supply Current (amps)",
                    hoodMotorSupplyCurrent.getValueAsDouble());
            SmartDashboard.putNumber("Superstructure/Hood/Stator Current (amps)",
                    hoodMotorStatorCurrent.getValueAsDouble());
            SmartDashboard.putNumber("Superstructure/Hood/Raw Motor Encoder Value",
                    hoodMotorStatorCurrent.getValueAsDouble());

            if (Robot.getMode() == RobotMode.DISABLED && !DriverStation.isFMSAttached()) {
                SmartDashboard.putBoolean("Robot/CAN/Canivore/Hood Motor Connected? (ID "
                        + String.valueOf(Ports.Superstructure.Hood.MOTOR) + ")", hoodMotor.isConnected());
                // SmartDashboard.putBoolean("Robot/CAN/Canivore/Hood Encoder Connected? (ID " + String.valueOf(hoodEncoder.getDeviceID()) + ")", hoodEncoder.isConnected());
            }
        }
    }

    private void setVoltageOverride(Optional<Double> voltageOverride) {
        this.voltageOverride = voltageOverride;
    }

    @Override
    public SysIdRoutine getHoodSysIdRoutine() {
        return SysId.getRoutine(
                .45,
                2,
                "Hood",
                voltage -> setVoltageOverride(Optional.of(voltage)),
                () -> hoodMotor.getPosition().getValueAsDouble(),
                () -> hoodMotor.getVelocity().getValueAsDouble(),
                () -> hoodMotor.getMotorVoltage().getValueAsDouble(),
                getInstance());
    }

    // TODO: Implementation as of 3/3 but not yet tested
    // Should ONLY be called at the lower hardstop!
    @Override
    public void zeroHoodEncoderAtUpperHardstop() {
        // hoodEncoder.getConfigurator().refresh(hoodEncoderConfig.getConfiguration().MagnetSensor);

        // double currentOffset =
        // hoodEncoderConfig.getConfiguration().MagnetSensor.MagnetOffset;

        // double positionWithCurrentOffset =
        // hoodEncoder.getPosition().getValueAsDouble();
        // double newOffset = -((positionWithCurrentOffset - currentOffset) -
        // Settings.Superstructure.Hood.Angles.MAX.getRotations());

        // hoodEncoderConfig.withMagnetOffset(newOffset);

        // hoodEncoderConfig.configure(hoodEncoder);
    }

    @Override
    public void zeroHoodEncodersAfterSeed() { // only use if you are seeded -> might add a boolean to double check that
                                              // we are in seed at Upper Hardstop ^^
        // hoodEncoder.getConfigurator().refresh(hoodEncoderConfig.getConfiguration().MagnetSensor);

        // double currentOffset =
        // hoodEncoderConfig.getConfiguration().MagnetSensor.MagnetOffset;

        // double encoderPositionWithCurrentOffset =
        // hoodEncoder.getPosition().getValueAsDouble();
        // double encoderPositionWithOutOffset = encoderPositionWithCurrentOffset -
        // currentOffset;

        // //double newOffset = -((positionWithCurrentOffset - currentOffset) -
        // Settings.Superstructure.Hood.Angles.MAX.getRotations());
        // double newOffset = encoderPositionWithOutOffset -
        // hoodMotor.getPosition().getValueAsDouble();

        // hoodEncoderConfig.withMagnetOffset(newOffset);

        // hoodEncoderConfig.configure(hoodEncoder);
    }

    @Override
    public double getCurrentDraw() {
        return Math.abs(hoodMotorSupplyCurrent.getValueAsDouble());
    }
}