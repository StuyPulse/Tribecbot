/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.    */
/* Use of this source code is governed by an MIT-style license     */
/* that can be found in the repository LICENSE file.               */
/*******************************************************************/

package com.stuypulse.robot.commands.spindexer;

import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class SpindexerDefaultCommand extends Command {
    private final Spindexer spindexer;
    private final Handoff handoff;
    private final Superstructure superstructure;
    private final CommandSwerveDrivetrain swerve;

    public SpindexerDefaultCommand() {
        this.spindexer = Spindexer.getInstance();
        this.handoff = Handoff.getInstance();
        this.superstructure = Superstructure.getInstance();
        this.swerve = CommandSwerveDrivetrain.getInstance();
        
        addRequirements(spindexer);
    }

    @Override
    public void execute() {
        boolean shouldStop = swerve.isBehindTower() 
                          || swerve.isBehindHub() 
                          || swerve.isUnderTrench()
                          || superstructure.isTurretWrapping();
        
        boolean prerequisitesMet = handoff.atTolerance() && superstructure.atTolerance();

        boolean shouldRun = !shouldStop && prerequisitesMet;
        
        if (!shouldRun) {
            spindexer.setState(SpindexerState.STOP);
        } else if (shouldRun && (superstructure.getState() == SuperstructureState.SOTM || superstructure.getState() == SuperstructureState.FOTM)) {
            spindexer.setState(SpindexerState.FORWARD);
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}