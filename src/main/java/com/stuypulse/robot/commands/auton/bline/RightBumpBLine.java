package com.stuypulse.robot.commands.auton.bline;

import java.util.Set;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.handoff.HandoffStop;
import com.stuypulse.robot.commands.intake.IntakeAutoDigest;
import com.stuypulse.robot.commands.intake.IntakeDeploy;
import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;
import com.stuypulse.robot.commands.superstructure.SuperstructureSOTM;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.BLinePathUtil;

import frc.robot.lib.BLine.Path;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class RightBumpBLine extends SequentialCommandGroup {

    public RightBumpBLine(String... pathNames) {

        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        addCommands(

            Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeOne()), Set.of()),

            new SuperstructureSOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new ParallelCommandGroup(
                new HandoffRun(),
                new SpindexerRun(),
                swerve.getPathBuilder()
                    .withPoseReset(swerve::resetPose)
                    .build(new Path(BLinePathUtil.PATHS_DIR, pathNames[0])),
                new WaitCommand(0.5).andThen(new IntakeDeploy()).andThen(new WaitCommand(1.0))
            ),

            // NZ Trip 1
            new ParallelCommandGroup(
                swerve.getPathBuilder()
                    //.withPoseReset(pose -> {})
                    .build(new Path(BLinePathUtil.PATHS_DIR, pathNames[1])),
                new IntakeDeploy(),
                new HandoffStop(),
                new SpindexerStop()
            ),

            new WaitCommand(0.5),

            // SOTM To Depot
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new HandoffRun().alongWith(new SpindexerRun()),
            new ParallelCommandGroup(
                swerve.getPathBuilder()
                    //.withPoseReset(pose -> {})
                    .build(new Path(BLinePathUtil.PATHS_DIR, pathNames[2])),
                new WaitCommand(6.0).andThen(
                    new HandoffStop().alongWith(new SpindexerStop())
                )
            ),

            new WaitCommand(0.5),

            // Off Depot
            new ParallelCommandGroup(
                new HandoffRun().alongWith(new SpindexerRun()),
                new WaitCommand(3.0).andThen(new IntakeAutoDigest().repeatedly())
            )

        );

    }

}