/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.leds;

import java.util.Optional;


import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {
    private final static LEDController instance;

    static {
        instance = new LEDController();
    }

    public static LEDController getInstance() {
        return instance;
    }

    private final CANdle leds;
    private CANdleConfiguration candleConfigs;
    private ControlRequest ledPattern = Settings.LED.solidColorRequest.withColor(Settings.LED.DISABLED);
    

    //different portions of the LED should be a different color to indicate whether certain limelights are dead
    //add the flashing aspect based on if we don't see a tag (with debounce) 
    //  one way to go further with the flashing aspect is make it flash faster over DISTANCE (since last tag was seen) rather than time

    public enum LEDSTATE {
        PASSING_TRENCH(Settings.LED.PASSING_TRENCH),
        IS_BEHIND_HUB(Settings.LED.IS_BEHIND_HUB),
        TURRET_WRAPPING(Settings.LED.TURRET_WRAPPING),
        LEFT_WARNING(Settings.LED.LEFT_WARNING),
        RIGHT_WARNING(Settings.LED.RIGHT_WARNING),
        SHOOT_IN_PLACE(Settings.LED.SHOOT_IN_PLACE),
        SOTM_ON(Settings.LED.SOTM_ON),
        FOTM_ON(Settings.LED.DISABLED), //holder bcs rainbow
        LEFT_CORNER(Settings.LED.LEFT_CORNER),
        RIGHT_CORNER(Settings.LED.RIGHT_CORNER),
        KB_DISTANCE(Settings.LED.KB_DISTANCE),
        REVERSE(Settings.LED.REVERSE),
        STOP_ROLLERS(Settings.LED.STOP_ROLLERS),
        RESET_HEADING(Settings.LED.RESET_HEADING),
        X_WHEELS(Settings.LED.X_WHEELS),
        INTAKE_STOW(Settings.LED.INTAKE_STOW),
        INTAKE_DEPLOYED(Settings.LED.INTAKE_DEPLOYED),
        DISABLED_ALIGNED(Settings.LED.DISABLED_ALIGNED),
        DISABLED(Settings.LED.DISABLED);

        private RGBWColor color;

        private LEDSTATE(RGBWColor color) {
            this.color = color;
        }

        public RGBWColor getColor() {
            return this.color;
        }

        public ControlRequest getAnimation() {
            if (this == LEDSTATE.FOTM_ON) {
                return Settings.LED.rainbowRequest;
            }
            else {
                return Settings.LED.solidColorRequest.withColor(getColor());
            }
        }
    }

    private LEDSTATE state = LEDSTATE.DISABLED;
    private LEDSTATE cachedState = LEDSTATE.DISABLED;

        //CHANGE apply pattern command to change state

    public void applyPattern() {
        if (cachedState != state) { 
            if (!(cachedState.getAnimation().getName().equals(state.getAnimation().getName()))) {
                this.ledPattern = state.getAnimation();
            } 
            
            else if (ledPattern instanceof SolidColor){
                SolidColor solidColor = (SolidColor) ledPattern;
                solidColor.withColor(state.getColor());
                // SolidColor.class.cast(ledPattern).withColor(null); //change if neccesary
            }
            cachedState = state;
        }
    }

    public void changeState(LEDSTATE state) {
        this.state = state;
    }

    private LEDController() {
        leds = new CANdle(Ports.LED.CANDLE_PORT, Ports.CANIVORE);

        candleConfigs = new CANdleConfiguration()
                .withLED(
                        new LEDConfigs()
                                .withBrightnessScalar(1.0)
                                .withStripType(StripTypeValue.GRB)
                                .withLossOfSignalBehavior(LossOfSignalBehaviorValue.KeepRunning))
                                
                .withCANdleFeatures(
                        new CANdleFeaturesConfigs().withStatusLedWhenActive(StatusLedWhenActiveValue.Enabled));
                
        leds.getConfigurator().apply(candleConfigs);

        leds.setControl(ledPattern);
    }

    public void periodicAfterScheduler() {
        if (RobotContainer.EnabledSubsystems.LEDS.get()) {
            leds.clearAllAnimations();
            applyPattern();
            leds.setControl(ledPattern);
        } else {
            leds.clearAllAnimations();
        }

        DogLog.log("LED/Applied Pattern Name", ledPattern.getName());

        DogLog.log("LED/State Pattern Name", state.getAnimation().getName());
        DogLog.log("LED/Cached State Pattern Name", state.getAnimation().getName());

        DogLog.log("LED/State", state.toString());
        DogLog.log("LED/Cached State", state.toString());
    
    }
}