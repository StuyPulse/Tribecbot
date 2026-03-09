/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.    */
/* Use of this source code is governed by an MIT-style license     */
/* that can be found in the repository LICENSE file.               */
/*******************************************************************/

package com.stuypulse.robot.commands.handoff;

import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;
import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class HandoffDefaultCommand extends Command {
    private final Handoff handoff;
    private final Superstructure superstructure;
    private final CommandSwerveDrivetrain swerve;

    public HandoffDefaultCommand() {
        this.handoff = Handoff.getInstance();
        this.superstructure = Superstructure.getInstance();
        this.swerve = CommandSwerveDrivetrain.getInstance();
        
        addRequirements(handoff);
    }

    @Override
    public void execute() {
        boolean shouldStop = swerve.isBehindTower() 
                          || swerve.isBehindHub() 
                          || swerve.isUnderTrench()
                          || superstructure.isTurretWrapping();
        
        boolean prerequisiteMet = superstructure.atTolerance();

        boolean shouldRun = !shouldStop && prerequisiteMet;
        
        if (!shouldRun) {
            handoff.setState(HandoffState.STOP);
        } else if (shouldRun && (superstructure.getState() == SuperstructureState.SOTM || superstructure.getState() == SuperstructureState.FOTM)) {
            handoff.setState(HandoffState.FORWARD);
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}