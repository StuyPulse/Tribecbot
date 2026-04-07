/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.auton.regular;

import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.intake.IntakeDeploy;
import com.stuypulse.robot.commands.intake.IntakeStow;
import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import com.pathplanner.lib.path.PathPlannerPath;

public class DepotAuton extends SequentialCommandGroup {
    
    public DepotAuton(PathPlannerPath... paths) {

        addCommands(
            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),
            
            // To Depot
            new IntakeDeploy().alongWith(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])
            ),

            new IntakeStow().alongWith(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1])
            ),

            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new SpindexerRun().alongWith(
                new HandoffRun()
            )
            // .until(() -> DriverStation.getMatchTime() < 2).andThen(
            //     new ParallelCommandGroup(
            //         new HandoffStop(),
            //         new SpindexerStop(),
            //         new ClimberDown()
            //     )
            // )

        );

    }

}
