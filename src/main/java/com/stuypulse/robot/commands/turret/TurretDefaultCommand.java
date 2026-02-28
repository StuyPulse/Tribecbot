package com.stuypulse.robot.commands.turret;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.turret.Turret;
import com.stuypulse.robot.subsystems.turret.Turret.TurretState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TurretDefaultCommand extends Command {
    private final Turret turret;
    private final CommandSwerveDrivetrain swerve;

    public TurretDefaultCommand() {
        turret = Turret.getInstance();
        swerve = CommandSwerveDrivetrain.getInstance();
        
        addRequirements(turret);
    }

    @Override
    public void execute() {
        // === Robot Position Logic ===
        // Reminder from driver's perspective, positive X to the opposite alliance and positive Y points to the left.

        boolean isInAllianceZone = swerve.getPose().getX() < Field.getHubPose().getX();

        if (isInAllianceZone) { 
            turret.setState(TurretState.FERRYING);
        }
        else {
            turret.setState(TurretState.SHOOTING);
        }
    }

}