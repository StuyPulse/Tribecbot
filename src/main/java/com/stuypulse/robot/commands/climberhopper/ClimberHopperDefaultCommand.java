package com.stuypulse.robot.commands.climberhopper;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper;
import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper.ClimberHopperState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        // Robot position logic
        // Reminder from driver's perspective, positive X to the opposite alliance and positive Y points to the left.
        pose = swerve.getPose();

        boolean isBetweenRightTrenchesY = Field.NearRightTrench.rightEdge.getY() < pose.getY() && Field.NearRightTrench.leftEdge.getY() > pose.getY();

        boolean isBetweenLeftTrenchesY = Field.NearLeftTrench.rightEdge.getY() < pose.getY() && Field.NearLeftTrench.leftEdge.getY() > pose.getY();

        boolean isCloseToNearTrenchesX = Math.abs(pose.getX() - Field.NearRightTrench.rightEdge.getX()) < Field.trenchXTolerance;

        boolean isCloseToFarTrenchesX = Math.abs(pose.getX() - Field.FarRightTrench.rightEdge.getX()) < Field.trenchXTolerance;

        boolean isUnderTrench = (isBetweenRightTrenchesY || isBetweenLeftTrenchesY) && (isCloseToNearTrenchesX || isCloseToFarTrenchesX); // Far X tolerance
        
        // Climber position and state logic
        boolean isUp = climberHopper.getState() == ClimberHopperState.CLIMBER_UP || climberHopper.getState() == ClimberHopperState.HOPPER_UP;

        boolean isDown = climberHopper.getState() == ClimberHopperState.CLIMBER_DOWN || climberHopper.getState() == ClimberHopperState.HOPPER_DOWN;

        boolean isRetracted = Math.abs(climberHopper.getPosition()) < Settings.ClimberHopper.HEIGHT_TOLERANCE;

        boolean isExtended = Math.abs(Settings.ClimberHopper.MAX_HEIGHT_METERS - climberHopper.getPosition()) < Settings.ClimberHopper.HEIGHT_TOLERANCE;

        boolean stalledByBalls = climberHopper.getStalling() && !isRetracted;
        // boolean stalledByBalls = true;

        if (isUp && climberHopper.getStalling()) {
            climberHopper.setState(ClimberHopperState.HOLDING_UP);
        }
        else if (isDown && climberHopper.getStalling()) {
            climberHopper.setState(ClimberHopperState.HOLDING_DOWN);
        }

        // If is stalling from the hardstop and not stalling from balls
        if (isUnderTrench) { // shouldn't be stalling in up state with 0 voltage
            if (!stalledByBalls && !flag) {
                if (climberHopper.getState() != ClimberHopperState.HOLDING_DOWN && climberHopper.getState() != ClimberHopperState.HOPPER_DOWN) {
                    climberHopper.setState(ClimberHopperState.HOPPER_DOWN);
                }
                if (climberHopper.getStalling()) {
                    climberHopper.setState(ClimberHopperState.HOLDING_DOWN);
                }
            } else {
                if (climberHopper.getState() != ClimberHopperState.HOLDING_UP && climberHopper.getState() != ClimberHopperState.HOPPER_UP) {
                    climberHopper.setState(ClimberHopperState.HOPPER_UP);
                }
                flag = true; // prevent hopper from going back down while still under trench with too many balls
            }
        } else { // If not under trench, set hopper up
            if (climberHopper.getState() == ClimberHopperState.CLIMBER_DOWN || climberHopper.getState() == ClimberHopperState.HOLDING_DOWN) {
                // We're climbing, do nothing
            } else if (!isExtended) {
                climberHopper.setState(ClimberHopperState.HOPPER_UP);
            } else if (climberHopper.getStalling()) {
                // This is assuming HOLDING_UP will not cause stalling
                climberHopper.setState(ClimberHopperState.HOLDING_UP);
            }
        }

        if (!isUnderTrench) {
            // TODO: If we have LEDs, have them show the flag status.
            flag = false;
        }

        SmartDashboard.putBoolean("ClimberHopper/UnderTrench", isUnderTrench);
    }
}