package com.stuypulse.robot.util;

import java.util.HashMap;
import java.util.Map;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class to get and use energy usage statistics
 * <p>
 * inspired by Mechanical Advantage
 * https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/699?u=dandon
 * </p>
 */
public class EnergyUtil {
    private double totalCurrent = 0.0;
    private double totalPower = 0.0;
    private double totalEnergy = 0.0;
    private double batteryVoltage = 12.6;
    private double rioCurrent = 0.0;

    private Map<String, Double> subsytemCurrents = new HashMap<>();
    private Map<String, Double> subsytemPowers = new HashMap<>();
    private Map<String, Double> subsytemEnergies = new HashMap<>();

    public void logEnergyUsage(String subsystem, double amps) {
        double power = amps * batteryVoltage; // Supply current draw of the subsystem
        double energy = power * Settings.DT;

        totalCurrent += amps;
        totalPower += power;
        totalEnergy += energy;

        subsytemCurrents.put(subsystem, amps);
        subsytemPowers.put(subsystem, power);
        subsytemEnergies.merge(subsystem, energy, (a, b) -> {
            return a + b;
        });

        String[] keys = subsystem.split("/|-");
        if (keys.length < 2) {
            return;
        }

        String subkey = "";
        for (int i = 0; i < keys.length - 1; i++) {
            subkey += keys[i];
            if (i < keys.length - 2) {
                subkey += "/";
            }
            subsytemCurrents.merge(subkey, amps, Double::sum);
            subsytemPowers.merge(subkey, power, Double::sum);
            subsytemEnergies.merge(subkey, energy, Double::sum);
        }
    }

    public void periodic() {

        SmartDashboard.putNumber("EnergyUtil/Total Supply Current Amps", totalCurrent);
        SmartDashboard.putNumber("EnergyUtil/Total Supply Power Watts", totalPower);
        SmartDashboard.putNumber("EnergyUtil/Total Used Energy Watt Hours", totalEnergy);

        for (var entry : subsytemCurrents.entrySet()) {
            SmartDashboard.putNumber("EnergyLogger/Supply Current Amps/" + entry.getKey(), entry.getValue());
            subsytemCurrents.put(entry.getKey(), 0.0);
        }
        for (var entry : subsytemPowers.entrySet()) {
            SmartDashboard.putNumber("EnergyLogger/Power Watts/" + entry.getKey(), entry.getValue());
            subsytemPowers.put(entry.getKey(), 0.0);
        }
        for (var entry : subsytemEnergies.entrySet()) {
            SmartDashboard.putNumber(
                    "EnergyLogger/Energy watt hours/" + entry.getKey(),
                    joulesToWattHours(entry.getValue()));
        }
    }

    public double getTotalCurrent() {
        return totalCurrent;
    }

    private double joulesToWattHours(double joules) {
        return joules / 3600.0;
    }

    public void setBatteryVoltage(double voltage) {
        this.batteryVoltage = voltage;
    }

    public void setRioCurrent(double current) {
        this.rioCurrent = current;
    }

}
