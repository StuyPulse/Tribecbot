/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.handoff;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
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

public class HandoffSim extends Handoff {

    private LinearSystemSim<N1, N1, N1> sim;
    private final LinearSystemLoop<N1, N1, N1> controller;

    private Optional<Double> voltageOverride;

    public HandoffSim() {
        LinearSystem<N1, N1, N1> flywheel = LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX44(1), 0.01, 1.0);

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


    public boolean shouldStop() {
        Superstructure superstructure = Superstructure.getInstance();
        SuperstructureState superstructureState = superstructure.getState();

        boolean isStopState = getState() == HandoffState.STOP;
        boolean isTurretWrapping = superstructure.isTurretWrapping();
        boolean isBehindHubWhileFerrying = superstructureState == SuperstructureState.FOTM
                && CommandSwerveDrivetrain.getInstance().isBehindHub();
        boolean isOutsideAllianceZone = 
            CommandSwerveDrivetrain.getInstance().isOutsideAllianceZone() && 
            superstructureState != superstructureState.FOTM;
        boolean isUnderTrench = CommandSwerveDrivetrain.getInstance().isUnderTrench() 
            && superstructureState != SuperstructureState.FOTM;
        boolean inManualState =       
            superstructureState == superstructureState.LEFT_CORNER &&
            superstructureState == superstructureState.RIGHT_CORNER &&
            superstructureState == superstructureState.KB;

        boolean turretLaggingSOTM = !superstructure.isTurretAtTolerance() && superstructureState == SuperstructureState.SOTM;

        return isStopState || 
        isTurretWrapping || 
        (isBehindHubWhileFerrying && !inManualState) || 
        turretLaggingSOTM || 
        (isOutsideAllianceZone  && !inManualState) || 
        (isUnderTrench && !inManualState);
    }

    @Override
    public double getLeaderRPM() {
        return sim.getOutput(0) * 60.0 / (2.0 * Math.PI); // convert to RPM
    }

    @Override
    public void periodic() {
        super.periodic();

        controller.setNextR(VecBuilder.fill(getTargetDutyCycle() * 2.0 * Math.PI / 60.0));
        controller.correct(VecBuilder.fill(sim.getOutput(0)));
        controller.predict(Settings.DT);

        boolean shouldNotShootIntoHub = (Superstructure.getInstance().superstructureInShootIntoHubMode()) ? 
            !CommandSwerveDrivetrain.getInstance().canShootIntoHub() 
            : false;

        if (EnabledSubsystems.HANDOFF.get()) {
            if (voltageOverride.isPresent()) {
                sim.setInput(voltageOverride.get());
                SmartDashboard.putNumber("Handoff/Input Voltage", voltageOverride.get());
            } else if (shouldStop() || shouldNotShootIntoHub) {
                sim.setInput(0.0);
                SmartDashboard.putNumber("Handoff/Input Voltage", 0.0);
            } else {
                SmartDashboard.putNumber("Handoff/Input Voltage", controller.getU(0));
                sim.setInput(getTargetDutyCycle() * 12);
            }
        } else {
            sim.setInput(0);
            SmartDashboard.putNumber("Handoff/Input Voltage", 0.0);
        }

        sim.update(Settings.DT);
        
        SmartDashboard.putBoolean("Handoff/Should Stop", shouldStop());
        SmartDashboard.putBoolean("Handoff/Should Not Shoot Into Hub", shouldNotShootIntoHub);
        SmartDashboard.putNumber("Handoff/Target Duty Cycle", getTargetDutyCycle());
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
                "Handoff",
                voltage -> setVoltageOverride(Optional.of(voltage)),
                () -> 0.0,
                () -> 0.0,
                () -> sim.getInput(0),
                getInstance());
    }

    @Override
    public boolean isHandoffStalling() {
        return false;
    }

    @Override
    public double getCurrentDraw() {
        return 0;
    }
}