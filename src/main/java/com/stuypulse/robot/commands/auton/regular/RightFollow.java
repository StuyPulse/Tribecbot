package com.stuypulse.robot.commands.auton.regular;

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
import com.stuypulse.robot.constants.Gains.Spindexer;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class RightFollow extends SequentialCommandGroup {
    
    public RightFollow(PathPlannerPath... paths) {

        addCommands(
            
            new SwerveResetPose(paths[0].getStartingHolonomicPose().get()),

            // Preloads
            new SuperstructureSOTM(),
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new ParallelCommandGroup(
                new HandoffRun(),
                new SpindexerRun(),
                new WaitCommand(RobotContainer.getWaitTimeOne() + 1.0),
                new WaitCommand(1.0).andThen(new IntakeDeploy())
            ),

            // To NZ
            new ParallelCommandGroup(
                new HandoffStop(),
                new SpindexerStop(),
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])
            ),

            new WaitCommand(RobotContainer.getWaitTimeTwo()),

            // Back
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]),

            // SOTM To Corner
            new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()),
            new ParallelCommandGroup(
                new HandoffRun(),
                new SpindexerRun(),
                new IntakeAutoDigest().repeatedly()
            )

        );

    }

}
