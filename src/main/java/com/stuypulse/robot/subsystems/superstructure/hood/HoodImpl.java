/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.hood;

import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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
    private final Motors.CANCoderConfig hoodEncoderConfig;

    private final TalonFX hoodMotor;
    private final CANcoder hoodEncoder;

    private final PositionVoltage controller;
    
    private Optional<Double> voltageOverride;
    
    private boolean hasUsedAbsoluteEncoder;

    private final BStream isStalling;

    public HoodImpl() {
        hoodConfig = new Motors.TalonFXConfig()
            .withInvertedValue(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            
            .withSupplyCurrentLimitAmps(80.0)
            .withStatorCurrentLimitEnabled(false)
            .withRampRate(0.25)
            
            .withPIDConstants(Gains.Superstructure.Hood.kP, Gains.Superstructure.Hood.kI, Gains.Superstructure.Hood.kD, 0)
            .withFFConstants(Gains.Superstructure.Hood.kS, Gains.Superstructure.Hood.kV, Gains.Superstructure.Hood.kA, 0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign, 0)
            
            .withSensorToMechanismRatio(Settings.Superstructure.Hood.GEAR_RATIO)

            .withSoftLimits(
                true, true, 
                Settings.Superstructure.Hood.FORWARD_SOFT_LIMIT.getRotations(),
                Settings.Superstructure.Hood.REVERSE_SOFT_LIMIT.getRotations());

        hoodEncoderConfig = new Motors.CANCoderConfig()
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(1.0)
            .withMagnetOffset(Settings.Superstructure.Hood.ENCODER_OFFSET.getRotations());

        hoodMotor = new TalonFX(Ports.Superstructure.Hood.MOTOR, Ports.RIO);
        hoodEncoder = new CANcoder(Ports.Superstructure.Hood.THROUGHBORE_ENCODER, Ports.RIO);

        hoodConfig.configure(hoodMotor);
        hoodEncoderConfig.configure(hoodEncoder);

        controller = new PositionVoltage(getTargetAngle().getRotations())
            .withEnableFOC(true);

        voltageOverride = Optional.empty();

        isStalling = BStream.create(() -> hoodMotor.getSupplyCurrent().getValueAsDouble() > Settings.Superstructure.Hood.STALL_CURRENT_LIMIT) //TODO: update value in Settings after testing
            .filtered(new BDebounce.Both(Settings.Superstructure.Hood.STALL_DEBOUNCE));
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(hoodMotor.getPosition().getValueAsDouble());
    }

    //TODO: Implementation as of 3/3 but not yet tested
    // Should ONLY be called at the lower hardstop!
    @Override
    public void zeroHoodEncoderAtLowerHardstop() {        
        hoodEncoder.getConfigurator().refresh(hoodEncoderConfig.getConfiguration().MagnetSensor);

        double currentOffset = hoodEncoderConfig.getConfiguration().MagnetSensor.MagnetOffset;

        double positionWithCurrentOffset = hoodEncoder.getPosition().getValueAsDouble();
        double newOffset = -((positionWithCurrentOffset - currentOffset) - Settings.Superstructure.Hood.Angles.MIN.getRotations());

        hoodEncoderConfig.withMagnetOffset(newOffset);
        
        hoodEncoderConfig.configure(hoodEncoder);
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
        hoodMotor.setPosition(getAbsoluteHoodAngleDeg() / 360.0);
    }

    private double getAbsoluteHoodAngleDeg() {
        return Settings.Superstructure.Hood.Angles.MIN.getDegrees() + hoodEncoder.getPosition().getValueAsDouble() * 360.0 / Settings.Superstructure.Hood.ENCODER_TO_MECH;
    }

    @Override 
    public void periodic() {
        super.periodic();

        if (!hasUsedAbsoluteEncoder) {
            seedHood();
            hasUsedAbsoluteEncoder = true;
        }

        if (EnabledSubsystems.HOOD.get()) {
            if (voltageOverride.isPresent()) {
                hoodMotor.setVoltage(voltageOverride.get());
            } else {
                if (Superstructure.getInstance().isHoodUnderTrench()) {
                    setState(HoodState.STOW);
                }
                hoodMotor.setControl(controller.withPosition(getTargetAngle().getRotations()));
            }
        } else {
            hoodMotor.stopMotor();
        }

        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("Superstructure/Hood/Correct Hood Angle (deg)", getAbsoluteHoodAngleDeg());

            SmartDashboard.putNumber("Superstructure/Hood/Applied Voltage", hoodMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Superstructure/Hood/Supply Current", hoodMotor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Superstructure/Hood/Stator Current", hoodMotor.getStatorCurrent().getValueAsDouble());

            SmartDashboard.putNumber("Superstructure/Hood/Closed Loop Error (deg)", hoodMotor.getClosedLoopError().getValueAsDouble() * 360.0);
            SmartDashboard.putBoolean("Superstructure/Hood/Has Used Absolute Encoder", hasUsedAbsoluteEncoder);

            SmartDashboard.putNumber("Superstructure/Hood/Raw Motor Encoder Value", hoodMotor.getPosition().getValueAsDouble());
            
            SmartDashboard.putNumber("InterpolationTesting/Hood Applied Voltage", hoodMotor.getMotorVoltage().getValueAsDouble());

            SmartDashboard.putNumber("Current Draws/Hood (amps)", hoodMotor.getSupplyCurrent().getValueAsDouble());
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
            getInstance()
        );
    }
    
}