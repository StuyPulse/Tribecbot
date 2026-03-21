/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.auton.regular;

import com.stuypulse.robot.commands.handoff.HandoffRun;
import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.superstructure.SuperstructureKB;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import com.pathplanner.lib.path.PathPlannerPath;

public class EightFuel extends SequentialCommandGroup {
    
    public EightFuel(PathPlannerPath... paths) {

        addCommands(

            new SuperstructureKB().alongWith(
                new WaitUntilCommand(() -> Superstructure.getInstance().atTolerance()).andThen(
                    new SpindexerRun().alongWith(new HandoffRun())
                )
            )

        ); 

    }

}
