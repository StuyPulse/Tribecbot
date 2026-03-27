package com.stuypulse.robot.commands.auton.regular;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.handoff.HandoffStop;
import com.stuypulse.robot.commands.intake.IntakeDeploy;
import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;
import com.stuypulse.robot.commands.superstructure.SuperstructureAutoInterpolation;
import com.stuypulse.robot.commands.superstructure.SuperstructureAutoInterpolationSOTM;
import com.stuypulse.robot.commands.superstructure.SuperstructureSOTM;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class LeftBumpTwoCycle extends SequentialCommandGroup {
    
    public LeftBumpTwoCycle(PathPlannerPath... paths) {

        addCommands(

            // NZ Trip 1
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0]).alongWith(
                new WaitCommand(0.5).andThen(new IntakeDeploy())
            ),      
            
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]).alongWith(
                new SuperstructureAutoInterpolationSOTM()
            ),
            new WaitCommand(0.75),

            // SOTM 1
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
                new SuperstructureSOTM().alongWith(
                    new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()).andThen(
                        new HandoffRun().andThen(new SpindexerRun())
                    )
                ).withTimeout(4.5)
            ),
            new SuperstructureAutoInterpolationSOTM(),

            // NZ Trip 2
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]),
                new HandoffStop(),
                new SpindexerStop()
            ),
            new WaitCommand(0.75),
            
            // SOTM 2
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[4]),
                new SuperstructureSOTM().alongWith(
                    new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()).andThen(
                        new HandoffRun().andThen(new SpindexerRun())
                    )
                ).withTimeout(4.5)
            ),
            new SuperstructureAutoInterpolationSOTM(),

            // NZ Trip 3
            new ParallelCommandGroup(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[5]),
                new HandoffStop(),
                new SpindexerStop()
            )

        );

    }

}
