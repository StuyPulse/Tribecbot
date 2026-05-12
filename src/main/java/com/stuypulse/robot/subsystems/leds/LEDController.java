
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {

    private final static LEDController instance;

    static {
        instance = new LEDController(Ports.LED.LED_PORT, Settings.LED.LED_LENGTH);
    }

    public static LEDController getInstance() {
        return instance;
    }

    private AddressableLED leds;
    private AddressableLEDBuffer ledsBuffer;

    private final CANdle candle;
    private CANdleConfiguration candleConfigs;
    private RGBWColor candleColor;

    private int startingIndex = 0;
    private int endIndex = 7;
    private boolean indicesChanged = false;

    private final LEDPattern defaultPattern = LEDPattern.kOff;

    protected LEDController(int ledPort, int ledLength) { // TODO: add brightness of CANdle to the constructor
        leds = new AddressableLED(ledPort);
        ledsBuffer = new AddressableLEDBuffer(ledLength);

        leds.setLength(ledLength);
        leds.setData(ledsBuffer);
        leds.start();

        applyPattern(defaultPattern);

        SmartDashboard.putData(instance);

        candle = new CANdle(null, Ports.CANIVORE); // TODO: update ports value

        candleConfigs = new CANdleConfiguration()
                .withLED(
                        new LEDConfigs()
                                .withBrightnessScalar(0.7)
                                .withStripType(StripTypeValue.RGB)
                                .withLossOfSignalBehavior(LossOfSignalBehaviorValue.DisableLEDs))
                                
                .withCANdleFeatures(
                        new CANdleFeaturesConfigs().withStatusLedWhenActive(StatusLedWhenActiveValue.Disabled));
                
        candle.getConfigurator().apply(candleConfigs);
    }

    public void applyPattern(LEDPattern pattern) {
        pattern.applyTo(ledsBuffer);
    }

    public void changeCandleIndices(int start, int end) {
        this.startingIndex = start;         
        this.endIndex = end;
        
        indicesChanged = true;
    }

    public void periodicAfterScheduler() {
        if (RobotContainer.EnabledSubsystems.LEDS.get()) {
            // leds.start();
            leds.setData(ledsBuffer);

            candleColor = new RGBWColor(
                    new Color8Bit(
                            ledsBuffer.getLED(1).toHexString()));

            if (indicesChanged) {
                //TODO: add logic that turns the candle on and off. Use output current amperage (if it is 0.01 A or lower) to determine if it fully turned off.
            }
            
            candle.setControl(new SolidColor(startingIndex, endIndex).withColor(candleColor));

        } else {
            LEDPattern.kOff.applyTo(ledsBuffer);
            leds.setData(ledsBuffer);

            //candle.setControl(new EmptyAnimation(0));
        }
        // SmartDashboard.putString("Leds/Color", ledsBuffer.getLED(1).toString());
    }
}