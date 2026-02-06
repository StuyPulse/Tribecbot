package com.stuypulse.robot.subsystems.spindexer;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
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
        return leadMotor.getVelocity().getValueAsDouble() * Settings.Spindexer.SECONDS_IN_A_MINUTE; 
    }

    /**
     * @return RPM of the follower motor for the spindexer
     */
    public double getCurrentFollowerMotorRPM() {
        return follower.getVelocity().getValueAsDouble() * Settings.Spindexer.SECONDS_IN_A_MINUTE; 
    }

    /**
     * @return RPM based on distance; interpolated from data points
     */
    @Override
    public double getRPMBasedOnDistance() {
        Translation2d hubPos = Field.getHubCenterPose().getTranslation();
        Translation2d robotPos = CommandSwerveDrivetrain.getInstance().getPose().getTranslation();
        double distance = hubPos.getDistance(robotPos);
        return SpindexerInterpolation.getRPM(distance);
    }

    /**
     * @return a boolean if lead motor is with a 3% tolerance of target RPM
     */
    public boolean atTargetRPM(){
        return Math.abs(getCurrentLeadMotorRPM() - getTargetRPM()) < Settings.Spindexer.SPINDEXER_TOLERANCE * getTargetRPM();
    }

    @Override
    public void periodic(){
        super.periodic();

        if (getTargetRPM() == 0) {
            leadMotor.setVoltage(0);
            follower.setVoltage(0);
        } else {
            leadMotor.setControl(new VelocityVoltage(getTargetRPM()));
            follower.setControl(new Follower(Ports.Spindexer.SPINDEXER_1, MotorAlignmentValue.Aligned));
        }
        
        SmartDashboard.putNumber("Spindexer/Lead Motor Speed", getCurrentLeadMotorRPM());
        SmartDashboard.putNumber("Spindexer/Follower Motor RPM", getCurrentFollowerMotorRPM());
        SmartDashboard.putNumber("Spindexer/Projected RPM Based on Distance", getRPMBasedOnDistance());
        SmartDashboard.putBoolean("Spindexer/At Target RPM", atTargetRPM());
    }
}
