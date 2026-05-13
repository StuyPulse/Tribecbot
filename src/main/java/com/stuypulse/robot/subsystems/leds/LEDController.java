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


    private int startingIndex = 0;
    private int endIndex = null; //TODO: calculate the total


    private final CANdle leds;
    private CANdleConfiguration candleConfigs;
    private ControlRequest ledPattern = new SolidColor(startingIndex, endIndex).withColor(RGBWColor.fromHSV(0, 0, 100));

    public void applyPattern(ControlRequest ledPattern) {
       this.ledPattern = ledPattern;
    }

    private LEDController() {
        leds = new CANdle(Ports.LED.CANDLE_PORT, Ports.RIO); // TODO: update ports value

        candleConfigs = new CANdleConfiguration()
                .withLED(
                        new LEDConfigs()
                                .withBrightnessScalar(0.7)
                                .withStripType(StripTypeValue.RGB)
                                .withLossOfSignalBehavior(LossOfSignalBehaviorValue.DisableLEDs))
                                
                .withCANdleFeatures(
                        new CANdleFeaturesConfigs().withStatusLedWhenActive(StatusLedWhenActiveValue.Enabled));
                
        leds.getConfigurator().apply(candleConfigs);

        leds.setControl(ledPattern);
        
    }

    public void periodicAfterScheduler() {
        if (RobotContainer.EnabledSubsystems.LEDS.get()) {
            leds.setControl(ledPattern);

        } else {
            leds.clearAllAnimations();
        }
    }
}