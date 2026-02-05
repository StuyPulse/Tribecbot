package com.stuypulse.robot.commands.climberhopper;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper;
import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper.ClimberHopperState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class ClimberHopperDefaultCommand extends Command {
    private final ClimberHopper climberHopper;
    private final CommandSwerveDrivetrain swerve;
    private final Pigeon2 gyro;
    private Pose2d pose;

    public ClimberHopperDefaultCommand() {
        climberHopper = ClimberHopper.getInstance();
        swerve = CommandSwerveDrivetrain.getInstance();
        gyro = swerve.getPigeon2();
        pose = swerve.getPose();

        
        addRequirements(climberHopper);

        
    }

    @Override
    public void execute() {
        boolean level = Math.abs(gyro.getPitch().getValueAsDouble()) < Settings.ClimberHopper.TOLERANCE;
        // double x = pose.getX() * Math.cos(pose.getRotation().getRadians());
        // double y = pose.getY() * Math.sin(pose.getRotation().getRadians());

        boolean under = (Field.RightTrench.farEdge.getY() < pose.getY() && pose.getY() < Field.RightTrench.nearEdge.getY())
            || (Field.LeftTrench.farEdge.getY() > pose.getY() && pose.getY() > Field.LeftTrench.nearEdge.getY())
            && (Math.abs(pose.getY() - Field.RightTrench.farEdge.getY()) < Field.trenchYTolerance
            || Math.abs(pose.getY() - Field.RightTrench.farEdge.getY()) < Field.trenchYTolerance);
        
        // TO-DO FIX SOON
        if (level && under && !climberHopper.getStalling()){ // shouldn't be stalling in hopper_up with 0 voltage
            climberHopper.setState(ClimberHopperState.HOPPER_DOWN);
        }   else { // if stalling after hopper down, set to hopper up
            climberHopper.setState(ClimberHopperState.HOPPER_UP);
        }
    }
}