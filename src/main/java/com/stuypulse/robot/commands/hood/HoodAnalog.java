package com.stuypulse.robot.commands.hood;

import com.stuypulse.robot.subsystems.superstructure.hood.Hood;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.wpilibj2.command.Command;

public class HoodAnalog extends Command{
    private final Hood hood;
    private final Gamepad driver;

    public HoodAnalog(Gamepad driver) {
        this.driver = driver;
        hood = Hood.getInstance();

        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setState(Hood.HoodState.ANALOG);
    }

    @Override
    public void execute() {
        hood.hoodAnalogToInput(driver);
        //SmartDashboard.putNumber("SuperStructure/Hood/Analog", hood.hoodAnalogToOutput().getDegrees());
    }
}
