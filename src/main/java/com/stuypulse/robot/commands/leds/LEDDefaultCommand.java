
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.leds;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SolidColor;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.RobotMode;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.handoff.Handoff;
import com.stuypulse.robot.subsystems.handoff.Handoff.HandoffState;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.PivotState;
import com.stuypulse.robot.subsystems.intake.Intake.RollerState;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.leds.LEDController.LedState;
import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.superstructure.hood.Hood;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.vision.LimelightVision;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDDefaultCommand extends InstantCommand{
    private final LEDController leds;
    private final CommandSwerveDrivetrain swerve;
    private final Handoff handoff;
    private final Intake intake;
    private final Spindexer spindexer;
    private final Hood hood;
    private final Shooter shooter;
    private final Turret turret;
    private final Superstructure superstructure;
    private final LimelightVision vision;

    public LEDDefaultCommand() {
        this.leds = LEDController.getInstance();
        this.swerve = CommandSwerveDrivetrain.getInstance();
        this.handoff = Handoff.getInstance();
        this.intake = Intake.getInstance();
        this.spindexer = Spindexer.getInstance();
        this.hood = Hood.getInstance();
        this.shooter = Shooter.getInstance();
        this.turret = Turret.getInstance();
        this.superstructure = Superstructure.getInstance();
        this.vision = LimelightVision.getInstance();

        addRequirements(leds);
    }

    @Override
    public void initialize() {
        if (Robot.getMode() == RobotMode.DISABLED) {
            if (LimelightVision.getInstance().getMaxTagCount() >= Settings.LED.DESIRED_TAGS_WHEN_DISABLED) {
                leds.changeState(LedState.DISABLED_ALIGNED);
            }
            else {
                leds.changeState(LedState.DISABLED);
            }
        }
        
        else {
            if (swerve.isUnderTrench()) {
                leds.changeState(LedState.PASSING_TRENCH);
            }
            else if (turret.isWrapping()) {
                leds.changeState(LedState.TURRET_WRAPPING);
            }
            else if (superstructure.getState() == SuperstructureState.LEFT_CORNER) {
                leds.changeState(LedState.LEFT_CORNER);
            }
            else if (superstructure.getState() == SuperstructureState.RIGHT_CORNER) {
                leds.changeState(LedState.RIGHT_CORNER);
            } 
            else if (superstructure.getState() == SuperstructureState.KB) {
                leds.changeState(LedState.KB_DISTANCE);
            }
            else if (superstructure.getState() == SuperstructureState.SOTM) {
                leds.changeState(LedState.SOTM_ON);
            }
            else if (superstructure.getState() == SuperstructureState.FOTM) {
                leds.changeState(LedState.FOTM_ON);
            }
            // else if (superstructure.getState() == SuperstructureState.STOW) {
            //     leds.changeState(LedState.RESET);
            // }
            else if (intake.getPivotState() == PivotState.STOW) {
                leds.changeState(LedState.INTAKE_STOW);
            }
            else if (intake.getPivotState() == PivotState.DEPLOY) {
                leds.changeState(LedState.INTAKE_DEPLOYED);
            }
        }

        // DogLog.log("Leds/State", state);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}