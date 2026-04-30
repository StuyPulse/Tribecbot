/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.auton.regular;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.handoff.HandoffStop;
import com.stuypulse.robot.commands.intake.IntakeAutoDigest;
import com.stuypulse.robot.commands.intake.IntakeDeploy;
import com.stuypulse.robot.commands.intake.IntakeDigest;
import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;
import com.stuypulse.robot.commands.superstructure.SuperstructureAutoInterpolation;
import com.stuypulse.robot.commands.superstructure.SuperstructureSOTM;
import com.stuypulse.robot.commands.swerve.SwerveResetHeading;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.Set;

import com.pathplanner.lib.path.PathPlannerPath;

public class RightTwoCorner extends SequentialCommandGroup {
    
    public RightTwoCorner(PathPlannerPath... paths) {

        addCommands(

            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),

            Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeOne()), Set.of()),

            // NZ Trip 1
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
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]),
                new HandoffRun(),
                new SpindexerRun(),
                new WaitCommand(0.5)
                    .andThen(new IntakeAutoDigest().until(() -> Superstructure.getInstance().isHopperEmpty()).withTimeout(15.0)),
                new WaitCommand(1.0).andThen(
                    new WaitUntilCommand(() -> Superstructure.getInstance().isHopperEmpty()).withTimeout(5.0))
            ),
            new SuperstructureAutoInterpolation().alongWith(new IntakeDeploy()),

            // NZ Trip 2
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
                new HandoffStop(),
                new SpindexerStop()
            ),

            new SuperstructureSOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]),
                new HandoffRun(),
                new SpindexerRun(),
                new WaitCommand(0.5)
                    .andThen(new IntakeAutoDigest().until(() -> Superstructure.getInstance().isHopperEmpty()).withTimeout(15.0)),
                new WaitUntilCommand(() -> Superstructure.getInstance().isHopperEmpty()).withTimeout(15.0)
            ),

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[4]).alongWith(new IntakeDeploy())
        
        );

    }

}
