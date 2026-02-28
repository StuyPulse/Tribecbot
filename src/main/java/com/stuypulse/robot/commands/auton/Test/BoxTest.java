package com.stuypulse.robot.commands.auton.Test;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.hoodedshooter.HoodedShooterKB;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class BoxTest extends SequentialCommandGroup {
    
    public BoxTest(PathPlannerPath... paths) {

        addCommands(

            new HoodedShooterKB().until(() -> DriverStation.getMatchTime() < 18).andThen(
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3])

        );

    }

}
