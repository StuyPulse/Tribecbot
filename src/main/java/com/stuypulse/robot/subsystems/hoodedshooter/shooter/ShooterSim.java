/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.hoodedshooter.shooter;

import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Optional;

public class ShooterSim extends Shooter {

    private LinearSystemSim<N1, N1, N1> sim;
    private final LinearSystemLoop<N1, N1, N1> controller;

    private Optional<Double> voltageOverride;

    public ShooterSim() {
        LinearSystem<N1, N1, N1> flywheel = LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX44(2), 0.20, 1.0);

        sim = new LinearSystemSim<>(flywheel);
        
        LinearQuadraticRegulator<N1, N1, N1> lqr = new LinearQuadraticRegulator<N1, N1, N1>(
            flywheel, 
            VecBuilder.fill(8.0), 
            VecBuilder.fill(12.0),
            Settings.DT);

        KalmanFilter<N1, N1, N1> kalmanFilter = new KalmanFilter<>(
            Nat.N1(), 
            Nat.N1(), 
            flywheel, 
            VecBuilder.fill(3.0), 
            VecBuilder.fill(0.01), 
            Settings.DT);

        controller = new LinearSystemLoop<>(flywheel, lqr, kalmanFilter, 12.0, Settings.DT);

        voltageOverride = Optional.empty();
    }

    @Override
    public double getShooterRPM() {
        return sim.getOutput(0) * 60.0 / (2.0 * Math.PI); // convert to RPM
    }

    private void setVoltageOverride(Optional<Double> volts) {
        voltageOverride = volts;
    }

    @Override
    public void periodic() {
        super.periodic();

        controller.setNextR(VecBuilder.fill(getTargetRPM() * 2.0 * Math.PI / 60.0));
        controller.correct(VecBuilder.fill(sim.getOutput(0)));
        controller.predict(Settings.DT);

        if (EnabledSubsystems.SHOOTER.get()) {
            if (voltageOverride.isPresent()) {
                sim.setInput(voltageOverride.get());
                SmartDashboard.putNumber("HoodedShooter/Shooter/Input Voltage", voltageOverride.get());
            } else {
                SmartDashboard.putNumber("HoodedShooter/Shooter/Input Voltage", controller.getU(0));
                sim.setInput(controller.getU(0));
            }
        } else {
            sim.setInput(0);
            SmartDashboard.putNumber("HoodedShooter/Shooter/Input Voltage", 0.0);
        }

        sim.update(Settings.DT);
    }

    @Override
    public SysIdRoutine getShooterSysIdRoutine() {
        return SysId.getRoutine(
                2,
                6,
                "Shooter",
                voltage -> setVoltageOverride(Optional.of(voltage)),
                () -> 0.0,
                () -> 0.0,
                () -> sim.getInput(0),
                getInstance());
    }
}