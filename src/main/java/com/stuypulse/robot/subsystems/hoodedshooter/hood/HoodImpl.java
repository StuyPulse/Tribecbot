/************************ PROJECT ALPHA *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.hoodedshooter.hood;

import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.Optional;

public class HoodImpl extends Hood {
    private final TalonFX hoodMotor;
    private final CANcoder hoodEncoder;

    private final PositionVoltage controller;
    
    private Optional<Double> voltageOverride;
    
    private boolean hasUsedAbsoluteEncoder;

    public HoodImpl() {
        hoodMotor = new TalonFX(Ports.HoodedShooter.Hood.MOTOR);
        hoodEncoder = new CANcoder(Ports.HoodedShooter.Hood.THROUGHBORE_ENCODER);

        Motors.HoodedShooter.Hood.HOOD.configure(hoodMotor);

        hoodMotor.getConfigurator().apply(Motors.HoodedShooter.Hood.SLOT_0);

        hoodMotor.getConfigurator().apply(Motors.HoodedShooter.Hood.SOFT_LIMITS);
        hoodEncoder.getConfigurator().apply(Motors.HoodedShooter.Hood.HOOD_ENCODER);

        controller = new PositionVoltage(getTargetAngle().getRotations())
            .withEnableFOC(true);

        voltageOverride = Optional.empty();
    }

    @Override
    public Rotation2d getHoodAngle() {
        return Rotation2d.fromRotations(hoodMotor.getPosition().getValueAsDouble());
    }

    @Override 
    public void periodic() {
        super.periodic();

        if (!hasUsedAbsoluteEncoder) {
            hoodMotor.setPosition(hoodEncoder.getAbsolutePosition().getValueAsDouble() / Settings.HoodedShooter.Hood.SENSOR_TO_HOOD_RATIO);
            hasUsedAbsoluteEncoder = true;
        }

        if (EnabledSubsystems.HOOD.get()) {
            if (voltageOverride.isPresent()) {
                hoodMotor.setVoltage(voltageOverride.get());
            } else {
                hoodMotor.setControl(controller.withPosition(getTargetAngle().getRotations()));
            }
        } else {
            hoodMotor.stopMotor();
        }

        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("HoodedShooter/Hood/Hood Absolute Angle (deg)", hoodEncoder.getPosition().getValueAsDouble() * 360.0 / Settings.HoodedShooter.Hood.SENSOR_TO_HOOD_RATIO);

            SmartDashboard.putNumber("HoodedShooter/Hood/Applied Voltage", hoodMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("HoodedShooter/Hood/Supply Current", hoodMotor.getSupplyCurrent().getValueAsDouble());

            SmartDashboard.putNumber("HoodedShooter/Hood/Closed Loop Error (deg)", hoodMotor.getClosedLoopError().getValueAsDouble() * 360.0);
            SmartDashboard.putBoolean("HoodedShooter/Hood/Has Used Absolute Encoder", hasUsedAbsoluteEncoder);

            SmartDashboard.putNumber("InterpolationTesting/Hood Applied Voltage", hoodMotor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("InterpolationTesting/Hood Supply Current", hoodMotor.getSupplyCurrent().getValueAsDouble());
        }
    }

    public void setVoltageOverride(Optional<Double> voltageOverride) {
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