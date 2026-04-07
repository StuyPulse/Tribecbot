package com.stuypulse.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.stuypulse.robot.util.MotorLogger.SubsystemName;
import com.stuypulse.robot.util.MotorLogger.ValueKey;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MasterLogger {
    public static Map<SubsystemName, Map<String, StatusSignal>> fullSignalMap = new HashMap<SubsystemName, Map<String, StatusSignal>>();
    static Map<String, StatusSignal> subsystemSignalMap = new HashMap<String, StatusSignal>();

    private final MotorLogger[] loggers;
    public Map<String, StatusSignal> encoderValues = new HashMap<String, StatusSignal>();

    private List<String> encoderNames = new ArrayList<String>();
    private List<CANcoder> encodersLogged = new ArrayList<CANcoder>();

    private CANcoder[] encoders;
    private String[] names;

    private boolean hasPopulated = false;

    public MasterLogger(MotorLogger... loggers) {
        this.loggers = loggers;
    }

    public Map<String, StatusSignal> getMotorSignalMap(MotorLogger logger) {
        return logger.motorSignals;
    }

    public void logEverything() {
        for (MotorLogger logger : loggers) {
            logger.logEverything();
        }
        if (!encodersLogged.isEmpty()) {
            if (!hasPopulated) {
                populate();
            }

            logEncoders();
            
        }
    }

    public void logEnergy() {
        for (MotorLogger logger : loggers) {
            logger.logEnergy();
        }
    }

    public void logTemp() {
        for (MotorLogger logger : loggers) {
            logger.logTemp();
        }
    }

    public void logConnection() {
        for (MotorLogger logger : loggers) {
            logger.logConnection();
        }
    }

    public void logMovement() {
        for (MotorLogger logger : loggers) {
            logger.logMovement();
        }
    }

    public void addEncoder(CANcoder encoder, String name) {
        encoderValues.put(name, encoder.getAbsolutePosition());
        PhoenixUtil.publishUsingPhoenixUtil(loggers[0].destination, new BaseStatusSignal[] { encoder.getAbsolutePosition() });

        encoderNames.add(name);
        encodersLogged.add(encoder);
    }

    public void populate() {
        CANcoder[] encoders = new CANcoder[encodersLogged.size()]; 
        String[] names = new String[encoderNames.size()];

        //using toArray was giving me errors so I had to manually loop through it
        for (int i = 0; i < encodersLogged.size(); i ++) {
            encoders[i] = encodersLogged.get(i);
            names[i] = encoderNames.get(i);
            subsystemSignalMap.put(names[i] + "/" + "Absolute Encoder Position", encoders[i].getAbsolutePosition());
        } 
        
        hasPopulated = true;

        fullSignalMap.put(loggers[0].subsystem, subsystemSignalMap);

        this.encoders = encoders;
        this.names = names;
        //should override only motors being added in the prescence of encoders.
    }


    public void logEncoders() {
        for (int i = 0; i < encoders.length; i ++) {
            DogLog.log(loggers[0].subsystem.toString() + "/" + names[i] + "/" + "Absolute Encoder Position", encoders[i].getAbsolutePosition().getValueAsDouble());
        }
    }

    //last thing, static TOTAL map
}
