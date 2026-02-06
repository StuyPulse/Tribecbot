package com.stuypulse.robot.subsystems.spindexer;

import com.stuypulse.robot.constants.Gains;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SpindexerSim extends Spindexer {
    private final DCMotorSim leadMotor;
    private final DCMotorSim followerMotor;

    public SpindexerSim() {
        super();
        leadMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(Gains.Spindexer.kV, Gains.Spindexer.kA),
            DCMotor.getKrakenX60(1));
        followerMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(Gains.Spindexer.kV, Gains.Spindexer.kA),
            DCMotor.getKrakenX60(1));
        
    }

    @Override
    public double getRPMBasedOnDistance() {
        return 0;
    }

    @Override
    public double getTargetRPM() {

    }

    @Override
    public void periodic() {
        super.periodic();
    }
}