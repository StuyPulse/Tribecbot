/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.climber;

import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class ClimberSim extends Climber {

    private final ElevatorSim sim;
    private final ClimberVisualizer visualizer;
    private double voltage;
    private final double positionConversionFactor;

    private Optional<Double> voltageOverride;

    public ClimberSim() {
        visualizer = ClimberVisualizer.getInstance();
        positionConversionFactor = (Settings.ClimberHopper.MAX_ROTATIONS - Settings.ClimberHopper.MIN_ROTATIONS)
                * (Settings.ClimberHopper.DRUM_RADIUS_METERS * Math.PI * 2);
        sim = new ElevatorSim(
                DCMotor.getKrakenX60(1),
                Settings.ClimberHopper.GEAR_RATIO,
                Settings.ClimberHopper.MASS_KG,
                Settings.ClimberHopper.DRUM_RADIUS_METERS,
                Settings.ClimberHopper.MIN_ROTATIONS,
                Settings.ClimberHopper.MAX_HEIGHT_METERS,
                false,
                Settings.ClimberHopper.MIN_ROTATIONS);

        voltageOverride = Optional.empty();
    }

    public boolean getStalling() {
        return sim.getCurrentDrawAmps() > Settings.ClimberHopper.STALL;
    }

    public double getCurrentHeight() {
        return sim.getPositionMeters();
    }

    @Override
    public double getCurrentRotations() {
        return sim.getPositionMeters() / positionConversionFactor;
    }

    private boolean isWithinTolerance(double toleranceMeters) {
        return Math.abs(getState().getTargetHeight() - getCurrentHeight()) < toleranceMeters;
    }

    @Override
    public boolean atTargetHeight() {
        return isWithinTolerance(Settings.ClimberHopper.TOLERANCE_ROTATIONS * positionConversionFactor);
    }

    @Override
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
    }

    @Override
    public void resetPostionUpper() {
        // No encoder reset for sim
    }

    @Override
    public void periodic() {
        super.periodic();
        ClimberState state = getState();
        double currentHeight = getCurrentHeight();
        double TargetHeight = state.getTargetHeight() * positionConversionFactor;

        if (voltageOverride.isPresent()) {
            voltage = voltageOverride.get();
        } else if (!atTargetHeight()) {
            if (state == ClimberState.CLIMBER_DOWN && currentHeight > TargetHeight)
                voltage = -Settings.ClimberHopper.MOTOR_VOLTAGE;
            else if (state == ClimberState.CLIMBER_UP && currentHeight < TargetHeight)
                voltage = Settings.ClimberHopper.MOTOR_VOLTAGE;
            else
                voltage = 0;
        } else {
            voltage = 0;

        }
        sim.setInputVoltage(voltage);

        SmartDashboard.putNumber("Climber/Voltage", voltage);
        SmartDashboard.putNumber("Climber/Current", sim.getCurrentDrawAmps());
        SmartDashboard.putBoolean("Climber/Stalling", getStalling());
        SmartDashboard.putBoolean("Climber/At Target Height", atTargetHeight());
        SmartDashboard.putNumber("Climber/Height", getCurrentHeight());
        SmartDashboard.putNumber("Climber/Position Conversion Factor to reach top", positionConversionFactor);
        visualizer.update(getCurrentHeight());

        sim.update(0.02);
    }
}