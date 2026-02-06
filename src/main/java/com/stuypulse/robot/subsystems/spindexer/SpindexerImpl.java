package com.stuypulse.robot.subsystems.spindexer;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.SpindexerInterpolation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SpindexerImpl extends Spindexer {
    private final TalonFX leadMotor;
    private final TalonFX follower;

    protected SpindexerImpl() {
        super();
        leadMotor = new TalonFX(Ports.Spindexer.SPINDEXER_1);
        follower = new TalonFX(Ports.Spindexer.SPINDEXER_2);

        Motors.Spindexer.MOTOR_CONFIG.configure(leadMotor);
        Motors.Spindexer.MOTOR_CONFIG.configure(follower);
    }

    /**
     * @return RPM of the lead motor for the spindexer
     */
    public double getCurrentLeadMotorRPM() {
        return leadMotor.getVelocity().getValueAsDouble() * 60;
    }

    /**
     * @return RPM of the follower motor for the spindexer
     */
    public double getCurrentFollowerMotorRPM() {
        return follower.getVelocity().getValueAsDouble() * 60;
    }

    @Override
    public double getRPMBasedOnDistance() {
        Translation2d hub = Field.getHubCenterPose().getTranslation();
        Translation2d rob = CommandSwerveDrivetrain.getInstance().getPose().getTranslation();
        double distance = hub.getDistance(rob);
        return SpindexerInterpolation.getRPM(distance);
    }

    @Override
    public void periodic(){
        super.periodic();

        if (getTargetRPM() == 0) {
            // TODO: adjust the following lines of code below
            leadMotor.setVoltage(0);
            follower.setVoltage(0);
        } else {
            leadMotor.setControl(new VoltageOut(getTargetRPM()));
            follower.setControl(new Follower(Ports.Spindexer.SPINDEXER_1, MotorAlignmentValue.Aligned));
        }

        SmartDashboard.putNumber("Spindexer/Lead Motor Speed", getCurrentLeadMotorRPM());
        SmartDashboard.putNumber("Spindexer/Follower Motor RPM", getCurrentFollowerMotorRPM());
    }
}
