package com.stuypulse.robot.commands.auton.bline;

import java.util.Set;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.handoff.HandoffStop;
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

public class DepotBLine extends SequentialCommandGroup {

    public DepotBLine(String... pathNames) {

        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

        addCommands(

            Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeOne()), Set.of()),

            new SuperstructureSOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new HandoffRun().alongWith(new SpindexerRun()),
            new ParallelCommandGroup(
                swerve.getPathBuilder()
                    .withPoseReset(swerve::resetPose)
                    .build(new Path(BLinePathUtil.PATHS_DIR, pathNames[0])),
                new WaitCommand(4.5).andThen(new HandoffStop().alongWith(new SpindexerStop())),
                new WaitCommand(0.5).andThen(new IntakeDeploy())
            ),

            new WaitCommand(0.5),

            new HandoffRun().alongWith(new SpindexerRun())

        );

    }

}