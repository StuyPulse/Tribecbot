package com.stuypulse.robot.commands.auton.regular;

import java.util.Set;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.handoff.HandoffStop;
import com.stuypulse.robot.commands.intake.IntakeAutoDigest;
import com.stuypulse.robot.commands.intake.IntakeDeploy;
import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;
import com.stuypulse.robot.commands.superstructure.SuperstructureSOTM;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Bump extends SequentialCommandGroup {
    
    public Bump(PathPlannerPath... paths) {

        addCommands(
            
            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),

            Commands.defer(() -> new WaitCommand(RobotContainer.getWaitTimeOne()), Set.of()),

            new SuperstructureSOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new ParallelCommandGroup(
                new HandoffRun(),
                new SpindexerRun(),
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
                new WaitCommand(0.5).andThen(new IntakeDeploy()).andThen(new WaitCommand(1.0))
            ),

            // NZ Trip 1
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
                new IntakeDeploy(),
                new HandoffStop(),
                new SpindexerStop()
            ),

            new WaitCommand(0.5),

            // SOTM To Depot
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new HandoffRun().alongWith(new SpindexerRun()),
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
                new WaitCommand(5.0).andThen(
                    new HandoffStop().alongWith(new SpindexerStop())
                )
            ),
            
            new WaitCommand(0.5),

            // Off Depot
            new ParallelCommandGroup(
                new HandoffRun().alongWith(new SpindexerRun()),
                new WaitCommand(5.0).andThen(new IntakeAutoDigest().repeatedly())
            )

        );

    }

}
