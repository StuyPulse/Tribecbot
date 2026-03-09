package com.stuypulse.robot.commands.auton.regular;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.intake.IntakeDeploy;
import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.superstructure.SuperstructureInterpolation;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class RightOneCycle extends SequentialCommandGroup {
    
    public RightOneCycle(PathPlannerPath... paths) {

        addCommands(

            // NZ Trip 1
            new IntakeDeploy().alongWith(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])
            ),
            new SuperstructureInterpolation(),

            // Trip 1 To Score
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),
            new ParallelCommandGroup(
                new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance())
                // new SwerveClimbAlign()
            ),
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
