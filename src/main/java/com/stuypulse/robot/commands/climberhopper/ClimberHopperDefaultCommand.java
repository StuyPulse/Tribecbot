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
    private boolean flag = false; // To prevent repeated stalling under trench

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



        boolean under = ((Field.NearRightTrench.rightEdge.getY() < pose.getY() && Field.NearRightTrench.leftEdge.getY() > pose.getY()) // Right trench Y logic
                      || (Field.NearLeftTrench.rightEdge.getY() < pose.getY() && Field.NearLeftTrench.leftEdge.getY() > pose.getY())) // Left trench Y logic
                      && (Math.abs(pose.getX() - Field.NearRightTrench.rightEdge.getX()) < Field.trenchXTolerance   // Near X tolerance
                      || (Math.abs(pose.getX() - Field.FarRightTrench.rightEdge.getX())) < Field.trenchXTolerance); // Far X tolerance
        
        boolean isUp = (climberHopper.getState() == ClimberHopperState.CLIMBER_UP || climberHopper.getState() == ClimberHopperState.HOPPER_UP);
        boolean isDown = (climberHopper.getState() == ClimberHopperState.CLIMBER_DOWN || climberHopper.getState() == ClimberHopperState.HOPPER_DOWN);
        
        // If is stalling 67
        if (under && !climberHopper.getStalling() && !flag) { // shouldn't be stalling in up state with 0 voltage
            if (climberHopper.getState() != ClimberHopperState.HOLDING_DOWN) { // only necessary if holding down doesn't stall the motor
                climberHopper.setState(ClimberHopperState.HOPPER_DOWN);
            }
        } else { // if stalling after hopper down, set to hopper up
            if (climberHopper.getState() != ClimberHopperState.HOLDING_UP) {
                climberHopper.setState(ClimberHopperState.HOPPER_UP);
            }
            flag = true; // prevent hopper from going back down while still under trench with too many balls
        }

        if (!under) {
            // TODO: If we have LEDs, have them show the flag status.
            flag = false;
        }
    }
}