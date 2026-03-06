package com.stuypulse.robot.commands.superstructure;

import com.stuypulse.robot.commands.spindexer.SpindexerRun;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;
import com.stuypulse.robot.subsystems.superstructure.Superstructure;
import com.stuypulse.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class SuperstructureSOTMConditional extends ConditionalCommand {
    public SuperstructureSOTMConditional() {
        super(new SuperstructureStow(), new SuperstructureSOTM(), () -> (Superstructure.getInstance().getState() == SuperstructureState.SOTM));
    }
}
