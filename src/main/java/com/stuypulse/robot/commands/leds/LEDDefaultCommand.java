package com.stuypulse.robot.commands.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.RobotMode;
import com.stuypulse.robot.subsystems.hoodedshooter.HoodedShooter;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.turret.Turret;

public class LEDDefaultCommand extends Command {
    private final LEDController leds;
    private final HoodedShooter hoodedShooter;
    private final Turret turret;

    //declare subsystems
    
    public LEDDefaultCommand() {
        leds = LEDController.getInstance();
        hoodedShooter = HoodedShooter.getInstance();
        turret = Turret.getInstance();

        //instantiate subsystems
        addRequirements(leds);
    }
    
    @Override
    public void execute() {
        //if Robot.isEnabled 
        if(Robot.getMode() == RobotMode.DISABLED) {
            
        }
    }
}
