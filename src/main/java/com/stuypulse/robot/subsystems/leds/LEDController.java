package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.LEDS.LEDState;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase{

    private final static LEDController instance;

    static {
        instance = new LEDController(Ports.LED.LED_PORT, Constants.LED.LED_LENGTH);
    }

    public static LEDController getInstance() {
        return instance;
    }

    private AddressableLED leds;
    private AddressableLEDBuffer ledsBuffer;

    protected LEDController(int port, int length) {
        leds = new AddressableLED(port);
        ledsBuffer = new AddressableLEDBuffer(length);

        leds.setLength(length);
        leds.setData(ledsBuffer);
        leds.start();

        applyState(Settings.LEDS.LEDState.DEFAULT_SETTING); //default is off 

        SmartDashboard.putData(instance);
    }

    public void applyPattern(LEDPattern pattern) {
        pattern.applyTo(ledsBuffer);
        leds.setData(ledsBuffer);
        SmartDashboard.putString("LED Pattern", pattern.toString());
    }

    public void applyState(LEDState state) {
        state.pattern.applyTo(ledsBuffer);
        leds.setData(ledsBuffer);
        SmartDashboard.putString("LED State", state.toString());
    }

    @Override
    public void periodic() {
        if (RobotContainer.EnabledSubsystems.LEDS.get()) {
            leds.start();
            leds.setData(ledsBuffer);
        } else {
            LEDPattern.kOff.applyTo(ledsBuffer);
            leds.setData(ledsBuffer);
        }
    }
    
}
