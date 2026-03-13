/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.hood;

import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.robot.util.superstructure.VisualizerHood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Optional;

public class HoodSim extends Hood {

    // Simulates hood motion as a linear elevator, then converts to angle
    private final ElevatorSim sim;

    private Optional<Double> voltageOverride;

    // Arc length constants — tune to match your hood geometry
    private static final double HOOD_ARM_LENGTH_METERS = 0.3;

    private static final double MIN_HEIGHT = HOOD_ARM_LENGTH_METERS *
        Math.sin(Settings.Superstructure.Hood.Angles.MIN.getRadians());
    private static final double MAX_HEIGHT = HOOD_ARM_LENGTH_METERS *
        Math.sin(Settings.Superstructure.Hood.Angles.MAX.getRadians());

    public HoodSim() {
        sim = new ElevatorSim(
            DCMotor.getKrakenX60(1),
            1.0,        // gear ratio
            1.0,        // carriage mass kg
            0.01,       // drum radius m
            MIN_HEIGHT,
            MAX_HEIGHT,
            true,
            MIN_HEIGHT,
            0.001, 0.001     // measurementStdDevs as plain double vararg
        );

        voltageOverride = Optional.empty();
    }

    // Convert linear elevator height back to hood angle
    @Override
    public Rotation2d getAngle() {
        double height = sim.getPositionMeters();
        double clampedHeight = Math.max(MIN_HEIGHT, Math.min(MAX_HEIGHT, height));
        double angleRad = Math.asin(clampedHeight / HOOD_ARM_LENGTH_METERS);
        return Rotation2d.fromRadians(angleRad);
    }

    @Override
    public boolean isStalling() {
        return false;
    }

    @Override
    public void zeroHoodEncoderAtUpperHardstop() {
        // No-op in sim
    }

    @Override
    public void seedHood() {
        // No-op in sim — ElevatorSim starts at MIN_HEIGHT
    }

    private void setVoltageOverride(Optional<Double> volts) {
        this.voltageOverride = volts;
    }

    @Override
    public SysIdRoutine getHoodSysIdRoutine() {
        return SysId.getRoutine(
            0.45,
            2,
            "Hood",
            voltage -> setVoltageOverride(Optional.of(voltage)),
            () -> sim.getPositionMeters(),
            () -> sim.getVelocityMetersPerSecond(),
            () -> voltageOverride.orElse(0.0),
            getInstance()
        );
    }

    @Override
    public void periodic() {
        super.periodic();

        // Drive sim toward target angle via a simple proportional voltage
        double targetHeight = HOOD_ARM_LENGTH_METERS *
            Math.sin(getTargetAngle().getRadians());
        double error = targetHeight - sim.getPositionMeters();
        double voltage = Math.max(-12.0, Math.min(12.0, error * 50.0));

        if (EnabledSubsystems.HOOD.get()) {
            sim.setInputVoltage(voltageOverride.orElse(voltage));
        } else {
            sim.setInputVoltage(0.0);
        }

        sim.update(Settings.DT);

        VisualizerHood.getInstance().update(getAngle(), atTolerance());

        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("Superstructure/Hood/Sim Height (m)", sim.getPositionMeters());
        }
    }

    @Override
    public void seedHoodAtUpperHardStop() {
        return;
    }

    @Override
    public void zeroHoodEncodersAfterSeed() {
        return;
    }

    @Override
    public double getCurrentDraw() {
        return 0;
    }
}