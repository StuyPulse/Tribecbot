package com.stuypulse.robot.subsystems.leds;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Ports;

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

        applyPattern(LEDPattern.kOff); //default is off 

        SmartDashboard.putData(instance);
    }

    public void applyPattern(LEDPattern pattern) {
        pattern.applyTo(ledsBuffer);
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
