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
    private Pose2d pose;
    private boolean flag = false; // To prevent repeated stalling under trench

    public ClimberHopperDefaultCommand() {
        climberHopper = ClimberHopper.getInstance();
        swerve = CommandSwerveDrivetrain.getInstance();
        pose = swerve.getPose();

        
        addRequirements(climberHopper);
    }

    @Override
    public void execute() {
        boolean isUp = climberHopper.getState() == ClimberHopperState.CLIMBER_UP || climberHopper.getState() == ClimberHopperState.HOPPER_UP;
        boolean isDown = climberHopper.getState() == ClimberHopperState.CLIMBER_DOWN || climberHopper.getState() == ClimberHopperState.HOPPER_DOWN;

        if (isUp && climberHopper.getStalling()) {
            climberHopper.setState(ClimberHopperState.HOLDING_UP);
        }
        else if (isDown && climberHopper.getStalling()) {
            climberHopper.setState(ClimberHopperState.HOLDING_DOWN);
        }

        // Reminder from driver's perspective, positive X to the opposite alliance and positive Y points to the left.
        boolean isBetweenRightTrenchesY = Field.NearRightTrench.rightEdge.getY() < pose.getY() && Field.NearRightTrench.leftEdge.getY() > pose.getY();

        boolean isBetweenLeftTrenchesY = Field.NearLeftTrench.rightEdge.getY() < pose.getY() && Field.NearLeftTrench.leftEdge.getY() > pose.getY();

        boolean isCloseToNearTrenchesX = Math.abs(pose.getX() - Field.NearRightTrench.rightEdge.getX()) < Field.trenchXTolerance;

        boolean isCloseToFarTrenchesX = Math.abs(pose.getX() - Field.FarRightTrench.rightEdge.getX()) < Field.trenchXTolerance;

        boolean isUnderTrench = (isBetweenRightTrenchesY || isBetweenLeftTrenchesY) && (isCloseToNearTrenchesX || isCloseToFarTrenchesX); // Far X tolerance
        
        boolean stalledByBalls = climberHopper.getStalling() && (Math.abs(climberHopper.getPosition()) > Settings.ClimberHopper.HEIGHT_TOLERANCE);
        
        // If is stalling from the hardstop and not stalling from balls
        if (isUnderTrench && !stalledByBalls && !flag) { // shouldn't be stalling in up state with 0 voltage
            if (climberHopper.getState() != ClimberHopperState.HOLDING_DOWN) {
                climberHopper.setState(ClimberHopperState.HOPPER_DOWN);
            }
        } else { // if stalling after hopper down, set to hopper up
            if (climberHopper.getState() != ClimberHopperState.HOLDING_UP) {
                climberHopper.setState(ClimberHopperState.HOPPER_UP);
            }
            flag = true; // prevent hopper from going back down while still under trench with too many balls
        }

        if (!isUnderTrench) {
            // TODO: If we have LEDs, have them show the flag status.
            flag = false;
        }
    }
}