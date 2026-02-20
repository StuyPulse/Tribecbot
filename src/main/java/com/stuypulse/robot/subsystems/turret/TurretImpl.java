/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.turret;


import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Turret.Constants;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.Optional;

public class TurretImpl extends Turret {
    private final TalonFX motor;
    private final CANcoder encoder17t;
    private final CANcoder encoder18t;

    private boolean hasUsedAbsoluteEncoder;
    private Optional<Double> voltageOverride;
    private final PositionVoltage controller;
    

    public TurretImpl() {
        motor = new TalonFX(Ports.Turret.MOTOR, Ports.bus);
        encoder17t = new CANcoder(Ports.Turret.ENCODER17T, Ports.bus);
        encoder18t = new CANcoder(Ports.Turret.ENCODER18T, Ports.bus);

        Motors.Turret.TURRET.configure(motor);
        encoder17t.getConfigurator().apply(Motors.Turret.ENCODER_17T);
        encoder18t.getConfigurator().apply(Motors.Turret.ENCODER_18T);

        // motor.getConfigurator().apply(Motors.Turret.turretSoftwareLimitSwitchConfigs);

        seedTurret();

        hasUsedAbsoluteEncoder = false;
        voltageOverride = Optional.empty();
        controller = new PositionVoltage(getTargetAngle().getRotations());
    }

    private Rotation2d getEncoderPos17t() {
        return Rotation2d.fromRotations(this.encoder17t.getAbsolutePosition().getValueAsDouble());
    }

    private Rotation2d getEncoderPos18t() {
        return Rotation2d.fromRotations(this.encoder18t.getAbsolutePosition().getValueAsDouble());
    }

    public Rotation2d getAbsoluteTurretAngle() {
        final int inverseMod17t = 1;
        final int inverseMod18t = -1;

        final Rotation2d encoder17tPosition = getEncoderPos17t();
        final double numberOfGearTeethRotated17 = (encoder17tPosition.getRotations()
                * (double) Constants.Encoder17t.TEETH);

        final Rotation2d encoder18tPosition = getEncoderPos18t();
        final double numberOfGearTeethRotated18 = (encoder18tPosition.getRotations()
                * (double) Constants.Encoder18t.TEETH);

        final double crt_Partial17 = numberOfGearTeethRotated17 * inverseMod17t * Constants.Encoder17t.TEETH;
        final double crt_Partial18 = numberOfGearTeethRotated18 * inverseMod18t * Constants.Encoder18t.TEETH;

        double crt_pos = (crt_Partial17 + crt_Partial18)
                % (Constants.Encoder17t.TEETH * Constants.Encoder18t.TEETH);

        // Java's % operator is not actually the same as the modulo operator, the lines below account for that 
        crt_pos = (crt_pos < 0) ? (crt_pos + Constants.Encoder17t.TEETH * Constants.Encoder18t.TEETH)
                : crt_pos;

        final double turretAngle = (crt_pos / (double) Constants.BigGear.TEETH);

        return Rotation2d.fromRotations(turretAngle);
    }
    
    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(motor.getPosition().getValueAsDouble());
    }

    @Override
    public boolean atTargetAngle() {
        return Math.abs(getAngle().minus(getTargetAngle()).getDegrees() + 180.0) < Settings.Turret.TOLERANCE_DEG;
    }

    private double getDelta(double target, double current) {
        double delta = (target - current) % 360;

        if (delta > 180.0) delta -= 360;
        else if (delta < -180) delta += 360;

        if (Math.abs(current + delta) < Constants.RANGE) return delta;
        
        return delta < 0 ? delta + 360 : delta - 360;
    }

    public void seedTurret() {
        motor.setPosition(0.0);
    }
    
    @Override
    public void periodic() {
        super.periodic();

        if (!hasUsedAbsoluteEncoder || getAbsoluteTurretAngle().getRotations() > 1.0 || getAngle().getRotations() < 0.0) {
            motor.setPosition((getAbsoluteTurretAngle().getDegrees() % 360.0) / 360.0);
            hasUsedAbsoluteEncoder = true;
            System.out.println("Absolute Encoder Reset triggered");
        }

        double currentAngle = getAngle().getDegrees();
        double actualTargetDeg = currentAngle + getDelta(getTargetAngle().getDegrees(), currentAngle);

        SmartDashboard.putNumber("Turret/Delta (deg)", getDelta(getTargetAngle().getDegrees(), getAngle().getDegrees()));
        SmartDashboard.putNumber("Turret/Actual Target (deg)", actualTargetDeg);

        if (EnabledSubsystems.TURRET.get() && getState() != TurretState.IDLE) {
            if (voltageOverride.isPresent()) {
                motor.setVoltage(voltageOverride.get());
            } else {
                motor.setControl(controller.withPosition(actualTargetDeg / 360.0));
            }
        } else {
            motor.stopMotor();
        }

        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("Turret/Encoder18t Abs Position (Rot)", encoder18t.getAbsolutePosition().getValueAsDouble());
            SmartDashboard.putNumber("Turret/Encoder17t Abs Position (Rot)", encoder17t.getAbsolutePosition().getValueAsDouble());
            SmartDashboard.putNumber("Turret/CRT Position (Rot)", getAbsoluteTurretAngle().getRotations());
            SmartDashboard.putNumber("Turret/Relative Encoder Position (Rot)", motor.getPosition().getValueAsDouble() * 360.0);
            SmartDashboard.putNumber("Turret/Voltage", motor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Turret/Error", motor.getClosedLoopError().getValueAsDouble() * 360.0);
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