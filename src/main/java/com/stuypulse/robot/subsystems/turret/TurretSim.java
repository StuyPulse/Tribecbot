/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.turret;

import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Optional;

public class TurretSim extends Turret {

    private LinearSystemSim<N2, N1, N2> sim;
    private final LinearSystemLoop<N2, N1, N2> controller;

    private TrapezoidProfile profile;
    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile.State goal;

    private double maxAngularVelRadiansPerSecond;
    private double maxAngularAccelRadiansPerSecondSquared;

    private Optional<Double> voltageOverride;

    public TurretSim() {
        LinearSystem<N2, N1, N2> linearSystem = LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1.0, 2.80);

        sim = new LinearSystemSim<>(linearSystem);
        
        LinearQuadraticRegulator<N2, N1, N2> lqr = new LinearQuadraticRegulator<N2, N1, N2>(
            linearSystem, 
            VecBuilder.fill(0.00001, 100), 
            VecBuilder.fill(12),
            Settings.DT);

        KalmanFilter<N2, N1, N2> kalmanFilter = new KalmanFilter<>(
            Nat.N2(), 
            Nat.N2(), 
            linearSystem, 
            VecBuilder.fill(3.0, 3.0), 
            VecBuilder.fill(0.01, 0.01), 
            Settings.DT);

        controller = new LinearSystemLoop<>(linearSystem, lqr, kalmanFilter, 12.0, Settings.DT);

        maxAngularVelRadiansPerSecond = Units.degreesToRadians(200.0);
        maxAngularAccelRadiansPerSecondSquared = Units.degreesToRadians(400.0);

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxAngularVelRadiansPerSecond, maxAngularAccelRadiansPerSecondSquared);
        profile = new TrapezoidProfile(constraints);
        goal = new TrapezoidProfile.State();
        setpoint = new TrapezoidProfile.State();

        voltageOverride = Optional.empty();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(sim.getOutput(0));
    }

    @Override
    public void seedTurret() {
        return;
    }

    @Override
    public void zeroEncoders() {
        return;
    }

    private double getAngularVelocityRadPerSec() {
        return sim.getOutput(1);
    }

    private void setVoltageOverride(Optional<Double> volts) {
        voltageOverride = volts;
    }

    @Override
    public void periodic() {
        super.periodic();
        
        goal = new TrapezoidProfile.State(getTargetAngle().getRadians(), 0.0);
        setpoint = profile.calculate(Settings.DT, setpoint, goal);

        SmartDashboard.putNumber("Turret/Constraints/Max Vel (deg per s)", Units.radiansToDegrees(maxAngularVelRadiansPerSecond));
        SmartDashboard.putNumber("Turret/Constraints/Max Accel (deg per s per s)", Units.radiansToDegrees(maxAngularAccelRadiansPerSecondSquared));

        SmartDashboard.putNumber("Turret/Setpoint (deg)", Units.radiansToDegrees(setpoint.position));
        SmartDashboard.putNumber("Turret/Target (deg)", Units.radiansToDegrees(getTargetAngle().getRadians()));
        SmartDashboard.putBoolean("Turret/At Target", atTargetAngle());
        SmartDashboard.putNumber("Turret/Error: abs(turret - target) (deg)", Math.abs(getAngle().minus(getTargetAngle()).getDegrees()));

        SmartDashboard.putNumber("Turret/Current Angle (deg)", sim.getOutput(0));

    controller.setNextR(VecBuilder.fill(setpoint.position, 0.0));
        controller.correct(VecBuilder.fill(sim.getOutput(0), sim.getOutput(1)));
        controller.predict(Settings.DT);

        if (EnabledSubsystems.TURRET.get()) {
            if (voltageOverride.isPresent()) {
                sim.setInput(voltageOverride.get());
            } else {
                sim.setInput(controller.getU(0));
            }
        } else {
            sim.setInput(0);
        }

        sim.update(Settings.DT);
    }

    /* USELESS, DON'T DELETE */
    @Override
    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
                2,
                6,
                "Turret",
                voltage -> setVoltageOverride(Optional.of(voltage)),
                () -> getAngle().getRotations(),
                () -> getAngularVelocityRadPerSec(),
                () -> sim.getInput(0),
                getInstance());
    }
}