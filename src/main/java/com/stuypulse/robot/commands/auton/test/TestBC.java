package com.stuypulse.robot.commands.auton.test;

import java.util.Set;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.intake.IntakeDeploy;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TestBC extends SequentialCommandGroup {

    public TestBC(PathPlannerPath... paths) {

        addCommands(
            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0].name).alongWith(
                new WaitCommand(0.2).andThen(new IntakeDeploy()))
        );

    }
}