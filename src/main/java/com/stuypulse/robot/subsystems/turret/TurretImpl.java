/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.turret;

import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Turret.Constants;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.robot.util.turret.TurretAngleCalculator;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import java.util.Optional;

public class TurretImpl extends Turret {
    private final Motors.TalonFXConfig turretConfig;
    private final Motors.CANCoderConfig encoder17tConfig;
    private final Motors.CANCoderConfig encoder18tConfig;

    private final TalonFX motor;
    private final CANcoder encoder17t;
    private final CANcoder encoder18t;

    private boolean hasUsedAbsoluteEncoder;
    private Optional<Double> voltageOverride;
    private final PositionVoltage controller;

    // private boolean wrapping;
    // private BStream isWrapping;

    public TurretImpl() {
        turretConfig = new Motors.TalonFXConfig()
            .withRampRate(0.25)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            
            .withPIDConstants(Gains.Turret.slot0.kP, Gains.Turret.slot0.kI, Gains.Turret.slot0.kD, 0)
            .withFFConstants(Gains.Turret.slot0.kS, Gains.Turret.slot0.kV, Gains.Turret.slot0.kA, 0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign, 0)
            
            .withPIDConstants(Gains.Turret.slot1.kP, Gains.Turret.slot1.kI, Gains.Turret.slot1.kD, 1)
            .withFFConstants(Gains.Turret.slot1.kS, Gains.Turret.slot1.kV, Gains.Turret.slot1.kA, 1)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign, 1)
            
            .withSensorToMechanismRatio(Settings.Turret.Constants.GEAR_RATIO_MOTOR_TO_MECH)
            // .withGainSchedBehavior(GainSchedBehaviorValue.UseSlot0, Settings.Turret.Constants.SLOT_SWITCHING_THRESHOLD_ROT, 1)
            .withSoftLimits(
                false, false,
                Settings.Turret.Constants.SoftwareLimit.FORWARD_MAX_ROTATIONS,
                Settings.Turret.Constants.SoftwareLimit.BACKWARDS_MAX_ROTATIONS);

        encoder17tConfig = new Motors.CANCoderConfig()
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(1.0);

        encoder18tConfig = new Motors.CANCoderConfig()
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(1.0);

        motor = new TalonFX(Ports.Turret.MOTOR, Ports.RIO);
        encoder17t = new CANcoder(Ports.Turret.ENCODER17T, Ports.RIO);
        encoder18t = new CANcoder(Ports.Turret.ENCODER18T, Ports.RIO);

        turretConfig.configure(motor);
        encoder17tConfig.configure(encoder17t);
        encoder18tConfig.configure(encoder18t);

        hasUsedAbsoluteEncoder = false;
        voltageOverride = Optional.empty();
        controller = new PositionVoltage(getTargetAngle().getRotations())
        .withEnableFOC(true);

        // isWrapping = BStream
        //     .create(() -> checkForWrapping())
        //     .filtered(new BDebounce.Both(Settings.Turret.WRAP_DEBOUNCE));
    }
    
    private Rotation2d getEncoderPos17t() {
        return Rotation2d.fromRotations(this.encoder17t.getAbsolutePosition().getValueAsDouble());
    }
    
    private Rotation2d getEncoderPos18t() {
        return Rotation2d.fromRotations(this.encoder18t.getAbsolutePosition().getValueAsDouble());
    }
    
    public Rotation2d getVectorSpaceAngle() {
        return TurretAngleCalculator.getAbsoluteAngle(getEncoderPos17t().getDegrees(), getEncoderPos18t().getDegrees());
    }
    
    public void zeroEncoders() {
        double encoderPos17T = encoder17t.getAbsolutePosition().getValueAsDouble();
        double encoderPos18T = encoder18t.getAbsolutePosition().getValueAsDouble();
        
        encoder17t.getConfigurator().refresh(encoder17tConfig.getConfiguration().MagnetSensor);
        encoder18t.getConfigurator().refresh(encoder18tConfig.getConfiguration().MagnetSensor);
        
        double currentOffset17T = encoder17tConfig.getConfiguration().MagnetSensor.MagnetOffset;
        double currentOffset18T = encoder18tConfig.getConfiguration().MagnetSensor.MagnetOffset;

        double newOffset17T = currentOffset17T - encoderPos17T;
        double newOffset18T = currentOffset18T - encoderPos18T;

        encoder17tConfig.withMagnetOffset(newOffset17T);
        encoder18tConfig.withMagnetOffset(newOffset18T);
        
        encoder17tConfig.configure(encoder17t);
        encoder18tConfig.configure(encoder18t);
    }

    public void seedTurret() {
        motor.setPosition(0); //TODO: SEED USING CRT INSTEAD OF TS, TS IS TEMP
        // motor.setPosition(getVectorSpaceAngle().getRotations());
    }
    
    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(motor.getPosition().getValueAsDouble());
    }
    
    @Override
    public boolean atTargetAngle() {
        return Math.abs(getAngle().minus(getTargetAngle()).getDegrees() + 180.0) < Settings.Turret.TOLERANCE_DEG;
    }
    
    private double getDelta(double target, double current) {
        double delta = (target - current) % 360;

        SmartDashboard.putNumber("Turret/Debugging: Delta", delta);
        SmartDashboard.putNumber("Turret/Debugging: Target", target);
        SmartDashboard.putNumber("Turret/Debugging: Current", current);
        
        if (delta > 180.0) delta -= 360;
        else if (delta < -180) delta += 360;

        if (Math.abs(current + delta) < Constants.RANGE) return delta;

        return delta < 0 ? delta + 360 : delta - 360;
    }

    // private boolean checkForWrapping() {
    //     double currentAngle = getAngle().getDegrees();
    //     double actualTargetDeg = currentAngle + getDelta(getTargetAngle().getDegrees(), currentAngle);
        
    //     if(!isWrapping.get()) { 
    //         return (Math.abs(currentAngle - actualTargetDeg) > 180.0);
    //     } else if(atTargetAngle()) { 
    //         return false;
    //     }

    //     return isWrapping.get();
    // }

    @Override
    public void periodic() {
        super.periodic();

        if (!hasUsedAbsoluteEncoder) {
            seedTurret();
            hasUsedAbsoluteEncoder = true;
            System.out.println("Absolute Encoder Reset triggered");
        }

        double currentAngle = getAngle().getDegrees();
        double actualTargetDeg = currentAngle + getDelta(getTargetAngle().getDegrees(), currentAngle);

        SmartDashboard.putNumber("Turret/Delta (deg)", getDelta(getTargetAngle().getDegrees(), getAngle().getDegrees()));
        SmartDashboard.putNumber("Turret/Actual Target (deg)", actualTargetDeg);
        SmartDashboard.putNumber("Turret/Closed Loop Slot" , motor.getClosedLoopSlot().getValue());

        if (EnabledSubsystems.TURRET.get()) {
            if (voltageOverride.isPresent()) {
                motor.setVoltage(voltageOverride.get());
            } else {
                // if(getState() == TurretState.TESTING) {
                //     motor.setControl(controller.withPosition(currentAngle + 1.0).withSlot(1));
                // } else
                // if (isWrapping.getAsBoolean()) {
                //     motor.setControl(controller.withPosition(actualTargetDeg / 360.0).withSlot(1));
                // } else {
                    motor.setControl(controller.withPosition(actualTargetDeg / 360.0).withSlot(0));
                // }
            }
        } else {
            motor.stopMotor();
        }

        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("Turret/Closed Loop Error (deg)", motor.getClosedLoopError().getValueAsDouble() * 360.0);
            SmartDashboard.putNumber("Turret/Wrap Point +210", Settings.Turret.Constants.RANGE);
            SmartDashboard.putNumber("Turret/Wrap Point -210", -Settings.Turret.Constants.RANGE);
            SmartDashboard.putNumber("Turret/Encoder18t Abs Position (Rot)", encoder18t.getAbsolutePosition().getValueAsDouble());
            SmartDashboard.putNumber("Turret/Encoder17t Abs Position (Rot)", encoder17t.getAbsolutePosition().getValueAsDouble());
            SmartDashboard.putNumber("Turret/Vector Space Position (Rot)", getVectorSpaceAngle().getRotations());
            SmartDashboard.putNumber("Turret/Vector Space Position (Deg)", getVectorSpaceAngle().getDegrees());
            SmartDashboard.putNumber("Turret/Relative Encoder Position (Rot)", motor.getPosition().getValueAsDouble() * 360.0);
            SmartDashboard.putNumber("Turret/Voltage", motor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Turret/Error", motor.getClosedLoopError().getValueAsDouble() * 360.0);
            SmartDashboard.putNumber("Turret/Stator Current", motor.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Turret/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
        }
    }

    @Override
    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
                2,
                6,
                "Turret",
                voltage -> setVoltageOverride(Optional.of(voltage)),
                () -> this.motor.getPosition().getValueAsDouble(),
                () -> this.motor.getVelocity().getValueAsDouble(),
                () -> this.motor.getMotorVoltage().getValueAsDouble(),
                getInstance());
    }

    private void setVoltageOverride(Optional<Double> volts) {
        this.voltageOverride = volts;
    }
}