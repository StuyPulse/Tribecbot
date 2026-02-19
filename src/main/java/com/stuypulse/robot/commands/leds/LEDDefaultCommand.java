package com.stuypulse.robot.commands.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.Robot.RobotMode;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.hoodedshooter.HoodedShooter;
import com.stuypulse.robot.subsystems.hoodedshooter.HoodedShooter.HoodedShooterState;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.turret.Turret;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper;
import com.stuypulse.robot.commands.swerve.SwerveXMode;
import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper.ClimberHopperState;
import com.stuypulse.stuylib.input.Gamepad;

public class LEDDefaultCommand extends Command {
    private final LEDController leds;
    private final HoodedShooter hoodedShooter;
    private final Turret turret;
    private final ClimberHopper climberHopper;
    
    public LEDDefaultCommand() {
        leds = LEDController.getInstance();
        hoodedShooter = HoodedShooter.getInstance();
        turret = Turret.getInstance();
        climberHopper = ClimberHopper.getInstance();

        addRequirements(leds);
    }
    
    @Override
    public void execute() {
        if(Robot.getMode() == RobotMode.DISABLED) {
            if(LimelightVision.getInstance().getMaxTagCount() >= Settings.LEDS.DESIRED_TAGS_WHEN_DISABLED) {
                leds.applyPattern(Settings.LEDS.DISABLED_ALIGNED);
            }
            else {
                leds.applyPattern(LEDPattern.kOff);
            }
        }
        else {
            if (climberHopper.getState() == ClimberHopperState.HOPPER_DOWN && hoodedShooter.getState() == HoodedShooterState.STOW) {
                leds.applyPattern(Settings.LEDS.TRENCH_LOWERING);           
            }
            else if (climberHopper.getState() == ClimberHopperState.HOLDING_DOWN && hoodedShooter.getState() == HoodedShooterState.STOW) {
                leds.applyPattern(Settings.LEDS.TRENCH_PASS);
            }
            else if (climberHopper.getState() == ClimberHopperState.CLIMBER_DOWN) {
                leds.applyPattern(Settings.LEDS.CLIMBING);
            }
        }
        }
        
    }

