package com.stuypulse.robot.subsystems.spindexer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.SpindexerInterpolation;

import edu.wpi.first.math.geometry.Translation2d;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;

public class SpindexerImpl extends Spindexer {
    private TalonFX leadMotor;
    private TalonFX follower;

    protected SpindexerImpl() {
        super();
        leadMotor = new TalonFX(Ports.Spindexer.SPINDEXER_1);
        follower = new TalonFX(Ports.Spindexer.SPINDEXER_2);

        Motors.Spindexer.MOTOR_CONFIG.configure(leadMotor);
        Motors.Spindexer.MOTOR_CONFIG.configure(follower);
    }

    public double getVoltageBasedOnDistance() {
        Translation2d hub = Field.getHubCenterPose().getTranslation();
        Translation2d rob = CommandSwerveDrivetrain.getInstance().getPose().getTranslation();
        double distance = hub.getDistance(rob);
        return SpindexerInterpolation.getVoltage(distance); 
    }

    @Override
    public void periodic(){
        super.periodic();

        if (getTargetVoltage() == 0) {
            leadMotor.setVoltage(0);
            follower.setVoltage(0);
        } else {
            leadMotor.setControl(new VoltageOut(getTargetVoltage()));
            follower.setControl(new Follower(Ports.Spindexer.SPINDEXER_1, MotorAlignmentValue.Aligned));
        }
    }
}
