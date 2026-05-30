/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.auton.regular;

import java.util.Set;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.intake.IntakeAutoDigest;
import com.stuypulse.robot.commands.intake.IntakeDeploy;
import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.superstructure.SuperstructureAutoInterpolation;
import com.stuypulse.robot.commands.superstructure.SuperstructureSOTM;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.commands.swerve.SwerveXMode;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ShallowSwipeDot extends SequentialCommandGroup {

    public ShallowSwipeDot(PathPlannerPath... paths) {

        addCommands(

            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),

            Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeOne()), Set.of()),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]).alongWith(
                new WaitCommand(0.2).andThen(new IntakeDeploy())
            ),

            // Trip 1 To Score
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]).alongWith(
                new SuperstructureAutoInterpolation()
            ),
            
            new SuperstructureSOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
                new HandoffRun(),
                new SpindexerRun(),
                new IntakeAutoDigest(),
                new WaitCommand(6.5)
            ),
            new SuperstructureAutoInterpolation().alongWith(new IntakeDeploy()), //ensure SOTM is over

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]),

            new SwerveXMode()
        );

    }

}
