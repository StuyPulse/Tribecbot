package com.stuypulse.robot.commands.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.hoodedshooter.HoodedShooter;
import com.stuypulse.robot.subsystems.hoodedshooter.HoodedShooter.HoodedShooterState;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.turret.Turret;
import com.stuypulse.robot.subsystems.turret.Turret.TurretState;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper;
import com.stuypulse.robot.commands.swerve.SwerveXMode;
import com.stuypulse.robot.subsystems.climberhopper.ClimberHopper.ClimberHopperState;
import com.stuypulse.stuylib.input.Gamepad;
import edu.wpi.first.wpilibj.DriverStation;

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
        if(DriverStation.isDisabled()) {
            if(LimelightVision.getInstance().getMaxTagCount() >= Settings.LEDS.DESIRED_TAGS_WHEN_DISABLED) {
                leds.applyState(Settings.LEDS.LEDState.DISABLED_ALIGNED);
            }
            else {
                leds.applyState(Settings.LEDS.LEDState.DEFAULT_SETTING);
            }
            return;
        }
        
        if (climberHopper.isTrenchSafeRetracted() && hoodedShooter.getState() == HoodedShooterState.STOW)
         {leds.applyState(Settings.LEDS.LEDState.TRENCH_PASS);           
        }
        else if (climberHopper.getState() == ClimberHopperState.HOPPER_DOWN) {
            leds.applyState(Settings.LEDS.LEDState.TRENCH_LOWERING);
        }
        else if (climberHopper.getState() == ClimberHopperState.CLIMBER_DOWN) {
            leds.applyState(Settings.LEDS.LEDState.CLIMBING);
        }
        else if (turret.getState() == TurretState.LEFT_CORNER) {
            leds.applyState((Settings.LEDS.LEDState.LEFT_CORNER));
        }
        else if (turret.getState() == TurretState.RIGHT_CORNER) {
            leds.applyState(Settings.LEDS.LEDState.RIGHT_CORNER);
        }
        else if (turret.getState() == TurretState.FERRYING && hoodedShooter.getState() == HoodedShooterState.FERRY) {
            leds.applyState(Settings.LEDS.LEDState.FERRYING_MODE);
        }
        else if (turret.getState() == TurretState.SHOOTING && hoodedShooter.getState() == HoodedShooterState.SHOOT) {
            leds.applyState(Settings.LEDS.LEDState.SHOOTING_MODE);
        }
    }     
}

