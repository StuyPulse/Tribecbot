/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.turret;

import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.robot.util.superstructure.TurretAngleCalculator;

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

public class TurretImpl extends Turret {
    private final Motors.TalonFXConfig turretConfig;
    private final Motors.CANCoderConfig encoder17tConfig;
    private final Motors.CANCoderConfig encoder18tConfig;

    private final TalonFX turretMotor;
    private final CANcoder encoder17t;
    private final CANcoder encoder18t;

    private boolean hasUsedAbsoluteEncoder;
    private Optional<Double> voltageOverride;
    private final PositionVoltage controller;

    private boolean isWrapping;

    public TurretImpl() {
        turretConfig = new Motors.TalonFXConfig()
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            
            .withSupplyCurrentLimitAmps(80)
            .withStatorCurrentLimitEnabled(false)
            .withRampRate(0.25)
            // .withVoltageLimits(6, -6) //TODO: VERIFY MAX VOLTAGE
            
            .withPIDConstants(Gains.Superstructure.Turret.slot0.kP, Gains.Superstructure.Turret.slot0.kI, Gains.Superstructure.Turret.slot0.kD, 0)
            .withFFConstants(Gains.Superstructure.Turret.slot0.kS, Gains.Superstructure.Turret.slot0.kV, Gains.Superstructure.Turret.slot0.kA, 0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign, 0)
            
            .withPIDConstants(Gains.Superstructure.Turret.slot1.kP.get(), Gains.Superstructure.Turret.slot1.kI.get(), Gains.Superstructure.Turret.slot1.kD.get(), 1)
            .withFFConstants(Gains.Superstructure.Turret.slot1.kS.get(), Gains.Superstructure.Turret.slot1.kV.get(), Gains.Superstructure.Turret.slot1.kA.get(), 1)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign, 1)
            
            .withSensorToMechanismRatio(Settings.Superstructure.Turret.GEAR_RATIO_MOTOR_TO_MECH)

            .withSoftLimits(
                false, false,
                Settings.Superstructure.Turret.SoftwareLimit.FORWARD_MAX_ROTATIONS,
                Settings.Superstructure.Turret.SoftwareLimit.BACKWARDS_MAX_ROTATIONS);

        encoder17tConfig = new Motors.CANCoderConfig()
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(Settings.Superstructure.Turret.Encoder17t.OFFSET.getRotations())
            .withAbsoluteSensorDiscontinuityPoint(1.0);

        encoder18tConfig = new Motors.CANCoderConfig()
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(Settings.Superstructure.Turret.Encoder18t.OFFSET.getRotations())
            .withAbsoluteSensorDiscontinuityPoint(1.0);

        turretMotor = new TalonFX(Ports.Superstructure.Turret.MOTOR, Ports.RIO);
        encoder17t = new CANcoder(Ports.Superstructure.Turret.ENCODER17T, Ports.RIO);
        encoder18t = new CANcoder(Ports.Superstructure.Turret.ENCODER18T, Ports.RIO);

        turretConfig.configure(turretMotor);
        encoder17tConfig.configure(encoder17t);
        encoder18tConfig.configure(encoder18t);

        hasUsedAbsoluteEncoder = false;
        voltageOverride = Optional.empty();
        controller = new PositionVoltage(getTargetAngle().getRotations()).withEnableFOC(true);
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
        // motor.setPosition(0); //TODO: SEED USING CRT INSTEAD OF TS, TS IS TEMP
        turretMotor.setPosition(getVectorSpaceAngle().getRotations());
    }

    public boolean isWrapping() {
        return isWrapping;
    }
    
    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turretMotor.getPosition().getValueAsDouble());
    }
    
    @Override
    public boolean atTolerance() {
        double error = getAngle().minus(getTargetAngle()).getRotations();
        return Math.abs(error) < Settings.Superstructure.Turret.TOLERANCE.getRotations();
    }
    
    private double getDelta(double target, double current) {
        double delta = (target - current) % 360;
        
        if (delta > 180.0) delta -= 360;
        else if (delta < -180) delta += 360;

        // if (Math.abs(current + delta) < Settings.Superstructure.Turret.RANGE) return delta;
        if (current + delta < Settings.Superstructure.Turret.RANGE_LEFT) return delta + 360;
        if (current + delta > Settings.Superstructure.Turret.RANGE_RIGHT) return delta - 360;

        // return delta < 0 ? delta + 360 : delta - 360;
        return delta;
    }

    @Override
    public void periodic() {
        super.periodic();

        turretConfig.updateGainsConfig(turretMotor, 1, 
        Gains.Superstructure.Turret.slot1.kP, 
        Gains.Superstructure.Turret.slot1.kI,
        Gains.Superstructure.Turret.slot1.kD,
        Gains.Superstructure.Turret.slot1.kS, 
        Gains.Superstructure.Turret.slot1.kV, 
        Gains.Superstructure.Turret.slot1.kA);

        if (!hasUsedAbsoluteEncoder) {
            seedTurret();
            hasUsedAbsoluteEncoder = true;
            System.out.println("Absolute Encoder Reset triggered");
        }

        double currentAngle = getAngle().getDegrees();
        double actualTargetDeg = currentAngle + getDelta(getTargetAngle().getDegrees(), currentAngle);

        isWrapping =    Math.abs(actualTargetDeg - currentAngle) > 
                        Settings.Superstructure.Turret.GAIN_SWITCHING_THRESHOLD.getDegrees();
        int slot = 0;

        if(isWrapping) {
            slot = 1;
        }

        if (EnabledSubsystems.TURRET.get()) {
            if (voltageOverride.isPresent()) {
                turretMotor.setVoltage(voltageOverride.get());
            } else {
                turretMotor.setControl(controller.withPosition(actualTargetDeg / 360.0).withSlot(slot));
            }
        } else {
            turretMotor.stopMotor();
        }

        if (Settings.DEBUG_MODE) {

            SmartDashboard.putNumber("Superstructure/Turret/Relative Encoder Position (Rot)", turretMotor.getPosition().getValueAsDouble() * 360.0);
            SmartDashboard.putNumber("Superstructure/Turret/Closed Loop Error (deg)", turretMotor.getClosedLoopError().getValueAsDouble() * 360.0);

            SmartDashboard.putNumber("Superstructure/Turret/Encoder18t Abs Position (Rot)", encoder18t.getAbsolutePosition().getValueAsDouble());
            SmartDashboard.putNumber("Superstructure/Turret/Encoder17t Abs Position (Rot)", encoder17t.getAbsolutePosition().getValueAsDouble());
            // SmartDashboard.putNumber("Superstructure/Turret/Vector Space Position (Deg)", getVectorSpaceAngle().getDegrees());

            SmartDashboard.putNumber("Superstructure/Turret/Voltage", turretMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Superstructure/Turret/Stator Current", turretMotor.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Superstructure/Turret/Supply Current", turretMotor.getSupplyCurrent().getValueAsDouble());
            
            SmartDashboard.putNumber("Superstructure/Turret/Wrapped Target Angle (deg)", actualTargetDeg);

            SmartDashboard.putNumber("Current Draws/Turret (amps)", turretMotor.getSupplyCurrent().getValueAsDouble());
        }
    }
    
    private void setVoltageOverride(Optional<Double> volts) {
        this.voltageOverride = volts;
    }

    @Override
    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
                2,
                6,
                "Turret",
                voltage -> setVoltageOverride(Optional.of(voltage)),
                () -> this.turretMotor.getPosition().getValueAsDouble(),
                () -> this.turretMotor.getVelocity().getValueAsDouble(),
                () -> this.turretMotor.getMotorVoltage().getValueAsDouble(),
                getInstance());
    }
}