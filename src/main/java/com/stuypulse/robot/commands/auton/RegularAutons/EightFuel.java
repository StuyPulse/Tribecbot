package com.stuypulse.robot.commands.auton.RegularAutons;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.hoodedshooter.HoodedShooterKB;
import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.subsystems.hoodedshooter.HoodedShooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class EightFuel extends SequentialCommandGroup {
    
    public EightFuel(PathPlannerPath... paths) {

        addCommands(

            new HoodedShooterKB().alongWith(
                new ParallelCommandGroup(

                    new WaitUntilCommand(() -> HoodedShooter.getInstance().isHoodAtTolerance()),
                    new WaitUntilCommand(() -> HoodedShooter.getInstance().isShooterAtTolerance())

                )
            ),

            new SpindexerRun().alongWith(
                new HandoffRun()
            )

        ); 

    }

}
