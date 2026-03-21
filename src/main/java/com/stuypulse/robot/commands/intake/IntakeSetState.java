/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.PivotState;
import com.stuypulse.robot.subsystems.intake.Intake.RollerState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.Optional;

public class IntakeSetState extends InstantCommand {

    private final Intake intake;
    private final Optional<PivotState> pivotState;
    private final Optional<RollerState> rollerState;

    public IntakeSetState(RollerState rollerState) {
        intake = Intake.getInstance();
        this.pivotState = Optional.empty();
        this.rollerState = Optional.of(rollerState);
        addRequirements(intake);
    }

    public IntakeSetState(PivotState pivotState) {
        intake = Intake.getInstance();
        this.pivotState = Optional.of(pivotState);
        this.rollerState = Optional.empty();
        addRequirements(intake);
    }

    public IntakeSetState(PivotState pivotState, RollerState rollerState) {
        intake = Intake.getInstance();
        this.pivotState = Optional.of(pivotState);
        this.rollerState = Optional.of(rollerState);
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (pivotState.isPresent()) {
            intake.setPivotState(pivotState.get());
        }

        if (rollerState.isPresent()) {
            intake.setRollerState(rollerState.get());
        }
    }
}