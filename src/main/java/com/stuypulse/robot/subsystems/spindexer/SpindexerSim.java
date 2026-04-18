/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.spindexer;

import java.util.Optional;

import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.util.SysId;

import dev.doglog.DogLog;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SpindexerSim extends Spindexer {

    private LinearSystemSim<N1, N1, N1> sim;
    private final LinearSystemLoop<N1, N1, N1> controller;

    private Optional<Double> voltageOverride;
    private boolean hasStartedStallTimer;
    private final Timer unjamTimer;

    public SpindexerSim() {
        LinearSystem<N1, N1, N1> flywheel = LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX44(1), 0.01, 1.0);

        sim = new LinearSystemSim<>(flywheel);

        LinearQuadraticRegulator<N1, N1, N1> lqr = new LinearQuadraticRegulator<>(
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
        hasStartedStallTimer = false;
        unjamTimer = new Timer();
    }

    public double getCurrentRPM() {
        return sim.getOutput(0) * 60.0 / (2.0 * Math.PI);
    }

    private boolean spindexerUnjam() {
        if (!hasStartedStallTimer && Handoff.getInstance().isHandoffStalling()) {
            unjamTimer.start();
            hasStartedStallTimer = true;
            setState(SpindexerState.REVERSE);
            return true;
        } else if (unjamTimer.get() < Settings.Spindexer.REVERSE_TIME && hasStartedStallTimer) {
            setState(SpindexerState.REVERSE);
            return true;
        } else {
            hasStartedStallTimer = false;
            return false;
        }
    }

    @Override
    public void periodic() {
        super.periodic();

        controller.setNextR(VecBuilder.fill(getTargetDutyCycle() * 2.0 * Math.PI / 60.0));
        controller.correct(VecBuilder.fill(sim.getOutput(0)));
        controller.predict(Settings.DT);

        // removed shouldNotShootIntoHub logic (no longer used)

        boolean isUnjamming = spindexerUnjam();

        if (EnabledSubsystems.SHOOTER.get()) {
            if (voltageOverride.isPresent()) {
                sim.setInput(voltageOverride.get());
                DogLog.log("Spindexer/Input Voltage", voltageOverride.get());
            } else if (Superstructure.getInstance().shouldStop() && !isUnjamming) {
                sim.setInput(0);
            } else {
                DogLog.log("Spindexer/Input Voltage", controller.getU(0));
                sim.setInput(controller.getU(0));
            }
        } else {
            sim.setInput(0);
            DogLog.log("Spindexer/Input Voltage", 0.0);
        }

        DogLog.log("Spindexer/Current RPM", getCurrentRPM());
        DogLog.log("Spindexer/Unjamming", isUnjamming);
        
        sim.update(Settings.DT);
    }

    @Override
    public void setVoltageOverride(Optional<Double> volts) {
        voltageOverride = volts;
    }

    @Override
    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
                2,
                6,
                "Spindexer",
                voltage -> setVoltageOverride(Optional.of(voltage)),
                () -> 0.0,
                () -> 0.0,
                () -> sim.getInput(0),
                getInstance());
    }

    @Override
    public double getCurrentDraw() {
        return 0;
    }
}