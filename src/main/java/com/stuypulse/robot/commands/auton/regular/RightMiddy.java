package com.stuypulse.robot.commands.auton.regular;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.handoff.HandoffStop;
import com.stuypulse.robot.commands.intake.IntakeAutoDigest;
import com.stuypulse.robot.commands.intake.IntakeDeploy;
import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;
import com.stuypulse.robot.commands.superstructure.SuperstructureAutoInterpolation;
import com.stuypulse.robot.commands.superstructure.SuperstructureSOTM;
import com.stuypulse.robot.commands.swerve.SwerveResetPose;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class RightMiddy extends SequentialCommandGroup {
    
    public RightMiddy(PathPlannerPath... paths) {

        addCommands(
            
            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),

            new SuperstructureSOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new ParallelCommandGroup(
                new HandoffRun(),
                new SpindexerRun(),
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]),
                new WaitCommand(2.0)
            ),

            // NZ Trip 1
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]).alongWith(
                new ParallelCommandGroup(
                    new IntakeDeploy(),
                    new HandoffStop(),
                    new SpindexerStop()
                )
            ),

            new WaitCommand(0.5),

            // SOTM To Depot
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new HandoffRun().alongWith(new SpindexerRun()),
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
                new WaitCommand(6.0).andThen(
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
