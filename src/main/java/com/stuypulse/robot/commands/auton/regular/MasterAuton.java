/** ********************** PROJECT TRIBECBOT ************************ */
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
 /* Use of this source code is governed by an MIT-style license */
 /* that can be found in the repository LICENSE file.           */
/** ************************************************************ */
package com.stuypulse.robot.commands.auton.regular;

import java.util.Set;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.handoff.HandoffStop;
import com.stuypulse.robot.commands.intake.IntakeAutoDigest;
import com.stuypulse.robot.commands.intake.IntakeDeploy;
import com.stuypulse.robot.commands.leds.LEDApplyState;
import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;
import com.stuypulse.robot.commands.superstructure.SuperstructureAutoInterpolation;
import com.stuypulse.robot.commands.superstructure.SuperstructureSOTM;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.commands.swerve.SwerveXMode;
import com.stuypulse.robot.subsystems.leds.LEDController.LedState;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class MasterAuton extends SequentialCommandGroup {
    //4.5 second shooting time first pass, infinite second pass
    //TODO: update paths to start where the shooting ends after 4.5 seconds
    public MasterAuton(PathPlannerPath... paths) {

        addCommands(
                new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),
                Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeOne()), Set.of()),
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]).deadlineFor(
                        new WaitCommand(0.2).andThen(new IntakeDeploy()),
                        new LEDApplyState(LedState.AUTON_COLOR_ONE)
                ),
                // Trip 1 To Score
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]).deadlineFor(
                        new SuperstructureAutoInterpolation(),
                        new LEDApplyState(LedState.AUTON_COLOR_TWO)
                ),
                new SuperstructureSOTM(),
                new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]).withTimeout(4.5).deadlineFor(
                        new LEDApplyState(LedState.AUTON_COLOR_ONE),
                        new HandoffRun(),
                        new SpindexerRun(),
                        new IntakeAutoDigest()
                ),
                new SuperstructureAutoInterpolation().alongWith(new IntakeDeploy()),
                // NZ Trip 2
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]).deadlineFor(
                        new LEDApplyState(LedState.AUTON_COLOR_TWO),
                        new HandoffStop(),
                        new SpindexerStop()
                ),
                new SuperstructureSOTM(),
                new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]).alongWith( 
                        new LEDApplyState(LedState.AUTON_COLOR_ONE),
                        new HandoffRun(),
                        new SpindexerRun(),
                        new IntakeAutoDigest()
                )
                // new SwerveXMode()
        );

    }

}
