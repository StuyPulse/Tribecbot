/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.superstructure.turret;

import java.util.Optional;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.DriverConstants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;

import dev.doglog.DogLog;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class TurretSim extends Turret {

    private LinearSystemSim<N2, N1, N2> sim;
    private final LinearSystemLoop<N2, N1, N2> controller;

    private TrapezoidProfile profile;
    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile.State goal;

    private double maxAngularVelRadiansPerSecond;
    private double maxAngularAccelRadiansPerSecondSquared;

    private boolean isWrapping;
    private boolean hasInitializedFilter;
    private double prevActualTargetAngle;

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
        isWrapping = false;
        prevActualTargetAngle = getTargetAngle().getDegrees();
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

    @Override
    public boolean isWrapping() {
        return isWrapping;
    }

    @Override
    public boolean atTolerance() {
        double error = getAngle().minus(getTargetAngle()).getRotations();
        return Math.abs(error) < Settings.Superstructure.Turret.TOLERANCE.getRotations();
    }

    private double getAngularVelocityRadPerSec() {
        return sim.getOutput(1);
    }

    private void setVoltageOverride(Optional<Double> volts) {
        voltageOverride = volts;
    }

    private double getDelta(double target, double current) {
        double delta = (target - current) % 360;

        if (delta > 180.0) {
            delta -= 360;
        } else if (delta < -180) {
            delta += 360;
        }

        if (current + delta > Settings.Superstructure.Turret.RANGE_CW) {
            return delta - 360;
        }
        if (current + delta < Settings.Superstructure.Turret.RANGE_CCW) {
            return delta + 360;
        }

        return delta;
    }
    @Override
    public void periodic() {
        super.periodic();

        double currentAngle = getAngle().getDegrees();
        double actualTargetAngle = currentAngle + getDelta(getTargetAngle().getDegrees(), currentAngle);

        if (!hasInitializedFilter) {
            prevActualTargetAngle = actualTargetAngle;
            hasInitializedFilter = true;
        }
        
        boolean deltaIsSignificant = Math.abs(actualTargetAngle - prevActualTargetAngle) >= Settings.Superstructure.Turret.SETPOINT_FILTER_THRESHOLD_DEG;

        boolean driverIsMoving = Math.abs(RobotContainer.driver.getLeftX()) > DriverConstants.Driver.Drive.DEADBAND || 
            Math.abs(RobotContainer.driver.getLeftY()) > DriverConstants.Driver.Drive.DEADBAND || 
            Math.abs(RobotContainer.driver.getRightX()) > DriverConstants.Driver.Drive.DEADBAND;

        if (deltaIsSignificant || driverIsMoving) {
            prevActualTargetAngle = actualTargetAngle;
        }
        isWrapping = Math.abs(actualTargetAngle - currentAngle) > 
                     Settings.Superstructure.Turret.GAIN_SWITCHING_THRESHOLD.getDegrees();
        
        goal = new TrapezoidProfile.State(Units.degreesToRadians(actualTargetAngle), 0.0);
        setpoint = profile.calculate(Settings.DT, setpoint, goal);

        DogLog.log("Superstructure/Turret/Constraints/Max Vel (deg per s)", Units.radiansToDegrees(maxAngularVelRadiansPerSecond));
        DogLog.log("Superstructure/Turret/Constraints/Max Accel (deg per s per s)", Units.radiansToDegrees(maxAngularAccelRadiansPerSecondSquared));

        DogLog.log("Superstructure/Turret/Motion Profile Setpoint (deg)", Units.radiansToDegrees(setpoint.position));
        DogLog.log("Superstructure/Turret/Error: abs(turret - target) (deg)", Math.abs(getAngle().minus(getTargetAngle()).getDegrees()));
        DogLog.log("Superstructure/Turret/Current Angle (deg)", Units.radiansToDegrees(sim.getOutput(0)));
        DogLog.log("Superstructure/Turret/Wrapped Target Angle (deg)", actualTargetAngle);
        DogLog.log("Superstructure/Turret/Setpoint Filtered Angle (deg)", prevActualTargetAngle);
        DogLog.log("Superstructure/Turret/Is Wrapping", isWrapping);

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

    @Override
    public double getCurrentDraw() {
        return 0;
    }
}