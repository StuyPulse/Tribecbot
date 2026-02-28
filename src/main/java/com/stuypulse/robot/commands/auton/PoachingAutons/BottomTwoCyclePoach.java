package com.stuypulse.robot.commands.auton.PoachingAutons;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.climberhopper.ClimberDown;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.handoff.HandoffStop;
import com.stuypulse.robot.commands.hoodedshooter.HoodedShooterInterpolation;
import com.stuypulse.robot.commands.intake.IntakeDeploy;
import com.stuypulse.robot.commands.intake.IntakeStow;
import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;
import com.stuypulse.robot.commands.swerve.SwerveClimbAlign;
import com.stuypulse.robot.subsystems.hoodedshooter.HoodedShooter;
import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class BottomTwoCyclePoach extends SequentialCommandGroup {
    
    public BottomTwoCyclePoach(PathPlannerPath... paths) {

        addCommands(

            // NZ Trip 1
            new IntakeDeploy().alongWith(
                CommandSwerveDrivetrain.getInstance().followPathCommand(paths[0])
            ),

            // Trip 1 To Score
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[1]).alongWith(
                new IntakeStow()
            ),
            new ParallelCommandGroup(
                new WaitUntilCommand(() -> HoodedShooter.getInstance().isShooterAtTolerance()),
                new WaitUntilCommand(() -> HoodedShooter.getInstance().isHoodAtTolerance())
            ),
            new SpindexerRun().alongWith(
                new HandoffRun()
            ).withTimeout(5.0),

            // NZ Trip 2
            new IntakeDeploy().alongWith(
                new ParallelCommandGroup(
                    CommandSwerveDrivetrain.getInstance().followPathCommand(paths[2]),
                    new HandoffStop(),
                    new SpindexerStop()
                )
            ),

            // Trip 2 To Score
            CommandSwerveDrivetrain.getInstance().followPathCommand(paths[3]).alongWith(
                new IntakeStow()
            ),
            new ParallelCommandGroup(
                new WaitUntilCommand(() -> HoodedShooter.getInstance().isShooterAtTolerance()),
                new WaitUntilCommand(() -> HoodedShooter.getInstance().isHoodAtTolerance()),
                new SwerveClimbAlign()
            ),
            new SpindexerRun().alongWith(
                new HandoffRun()
            ).until(() -> DriverStation.getMatchTime() < 2).andThen(
                new ParallelCommandGroup(
                    new HandoffStop(),
                    new SpindexerStop(),
                    new ClimberDown()
                )
            )
            

        );

    }

}
