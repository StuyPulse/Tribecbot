/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.auton.regular;

import java.util.Set;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.handoff.HandoffStop;
import com.stuypulse.robot.commands.intake.IntakeAutoDigest;
import com.stuypulse.robot.commands.intake.IntakeDeploy;
import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;
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

public class RightTwoCornerBCNewTwo extends SequentialCommandGroup {
    
    public RightTwoCornerBCNewTwo(PathPlannerPath... paths) {
        //SKETCHY
        //this one removes pathfinder + optimized timeouts

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
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),//.withTimeout(1.0), uncomment if we have issues
                new HandoffRun(),
                new SpindexerRun(),
                new WaitCommand(0.5)
                    .andThen(new IntakeAutoDigest().until(() -> Superstructure.getInstance().isHopperEmpty()).withTimeout(1.5)), //changed to 4 bcs of delay (from 5)
                new WaitCommand(0.5).andThen(
                    new WaitUntilCommand(() -> Superstructure.getInstance().isHopperEmpty()).withTimeout(1.5))
            ), //update to 3.0 for actual, this just ensures pathfinding occurs
            new SuperstructureAutoInterpolation().alongWith(new IntakeDeploy()), //still have to shoot so we go back to interpolating

            // NZ Trip 2
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]),
                new HandoffStop(),
                new SpindexerStop()
            ),

            new SuperstructureSOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),//.withTimeout(1.0), uncomment if we have issues
                new HandoffRun(),
                new SpindexerRun(),
                new WaitCommand(0.5)
                    .andThen(new IntakeAutoDigest().until(() -> Superstructure.getInstance().isHopperEmpty()).withTimeout(1.5)), //changed to 4 bcs of delay (from 5)
                new WaitCommand(0.5).andThen(
                    new WaitUntilCommand(() -> Superstructure.getInstance().isHopperEmpty()).withTimeout(1.0))
            ), //update to 3.0 for actual, this just ensures pathfinding occurs
            new SuperstructureAutoInterpolation().alongWith(new IntakeDeploy()), //still have to shoot so we go back to interpolating

            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[4]),

            new SwerveXMode()
            //TODO: not accounting for the first loop!! - in terms of paths!!!
        );

    }

}
