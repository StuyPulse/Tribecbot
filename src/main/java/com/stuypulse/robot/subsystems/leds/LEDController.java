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
    private SolidColor solidColorRequest = new SolidColor(0, Settings.LED.LED_LENGTH - 1).withColor(new RGBWColor(Color.kRed));
    private RainbowAnimation rainbowRequest = new RainbowAnimation(0, Settings.LED.LED_LENGTH - 1).withFrameRate(60).withSlot(0);

    private final static LEDController instance;

    static {
        instance = new LEDController();
    }

    public static LEDController getInstance() {
        return instance;
    }

    private final CANdle leds;
    private CANdleConfiguration candleConfigs;
    private ControlRequest ledPattern = Settings.LED.DISABLED;
    private boolean isChanged;

    //different portions of the LED should be a different color to indicate whether certain limelights are dead
    //add the flashing aspect based on if we don't see a tag (with debounce) 
    //  one way to go further with the flashing aspect is make it flash faster over DISTANCE (since last tag was seen) rather than time

    public enum LEDSTATE {
        PASSING_TRENCH,
        IS_BEHIND_HUB,
        TURRET_WRAPPING,
        LEFT_WARNING,
        RIGHT_WARNING,
        SHOOT_IN_PLACE,
        SOTM_ON,
        FOTM_ON,
        LEFT_CORNER,
        RIGHT_CORNER,
        KB_DISTANCE,
        REVERSE,
        STOP_ROLLERS,
        RESET_HEADING,
        X_WHEELS,
        INTAKE_STOW,
        INTAKE_DEPLOYED,
        DISABLED_ALIGNED,
        DISABLED
    }

    private LEDSTATE state = LEDSTATE.DISABLED;
    private LEDSTATE cachedState = LEDSTATE.DISABLED;

    
    public ControlRequest stateToPattern(LEDSTATE state) {
        return switch (state) {
            case PASSING_TRENCH -> solidColorRequest.withColor(new RGBWColor(Color.kRed));
            case IS_BEHIND_HUB -> solidColorRequest.withColor(new RGBWColor(Color.kRed));
            case TURRET_WRAPPING -> solidColorRequest.withColor(new RGBWColor(Color.kRed));
            case LEFT_WARNING -> solidColorRequest.withColor(new RGBWColor(Color.kBlack));
            case RIGHT_WARNING -> solidColorRequest.withColor(new RGBWColor(Color.kBlack));
            case SHOOT_IN_PLACE -> solidColorRequest.withColor(new RGBWColor(Color.kPurple));
            case SOTM_ON -> solidColorRequest.withColor(new RGBWColor(Color.kCyan));

            case FOTM_ON -> rainbowRequest; //rainbow animation -> need to add cached states and a change pattern type method -> maybe apply pattern should take in the pattern and the color/frequency -> then i would need a system like the status signal where we just call them once and mutate them after

            case LEFT_CORNER -> solidColorRequest.withColor(new RGBWColor(Color.kPurple));
            case RIGHT_CORNER -> solidColorRequest.withColor(new RGBWColor(Color.kBlue));
            case KB_DISTANCE -> solidColorRequest.withColor(new RGBWColor(Color.kPink));
            case REVERSE -> solidColorRequest.withColor(new RGBWColor(Color.kWhite));
            case STOP_ROLLERS -> solidColorRequest.withColor(new RGBWColor(Color.kYellow));
            case RESET_HEADING -> solidColorRequest.withColor(new RGBWColor(Color.kYellow));
            case X_WHEELS -> solidColorRequest.withColor(new RGBWColor(Color.kRed));
            case INTAKE_STOW -> solidColorRequest.withColor(new RGBWColor(Color.kBrown));
            case INTAKE_DEPLOYED -> solidColorRequest.withColor(new RGBWColor(Color.kOrange));
            case DISABLED_ALIGNED -> solidColorRequest.withColor(new RGBWColor(Color.kGreen));
            case DISABLED -> solidColorRequest.withColor(new RGBWColor(Color.kRed));
        };

        //CHANGE apply pattern command to change state
    }

    public void applyPattern(ControlRequest ledPattern) {
        if (cachedState != state) { 
            if (stateToPattern(cachedState) != stateToPattern(state)) {
                this.ledPattern = stateToPattern(state);
            } 
            
            else if (stateToPattern(state) instanceof SolidColor){
                SolidColor.class.cast(ledPattern).withColor(null); //UPDATTTEEE
            }
            cachedState = state;
        }
    }

    public void changeState(LEDSTATE state) {
        this.state = state;
    }

    private LEDController() {
        leds = new CANdle(Ports.LED.CANDLE_PORT, Ports.CANIVORE); // TODO: update ports value

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
        isChanged = true;
    }

    public void periodicAfterScheduler() {
        if (RobotContainer.EnabledSubsystems.LEDS.get()) {
            if(isChanged) {
                leds.clearAllAnimations();
                leds.setControl(ledPattern);
                isChanged = false;
            }
        } else {
            leds.clearAllAnimations();
        }

        DogLog.log("LED/Pattern Name", ledPattern.getName());
        DogLog.log("LED/State", state.toString());
    
    }
}