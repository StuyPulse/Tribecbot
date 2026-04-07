package com.stuypulse.robot.util;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.PhoenixUtil.PublishingDestination;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorLogger {

    private final StatusSignal<Current> motorSupplyCurrent;
    private final StatusSignal<Current> motorStatorCurrent;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Current> motorStallCurrent;
    private final StatusSignal<Voltage> motorMotorVoltage;
    private final StatusSignal<Voltage> motorSupplyVoltage;

    private final StatusSignal<Temperature> motorTemperature;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<AngularAcceleration> motorAcceleration;
    private final StatusSignal<Double> motorClosedLoopError;

    private final StatusSignal<ConnectedMotorValue> motorConnected;

    public final SubsystemName subsystem;
    public final String motorName;
    public final PublishingDestination destination;
    public final int port;

    public Map<String, StatusSignal> motorSignals = new HashMap<String, StatusSignal>();
    private BaseStatusSignal[] publishingSignals;
    //to avoid confusion decided to make this private

    public MotorLogger(SubsystemName subsystem, String motorName, TalonFX motor, int port, PublishingDestination destination) {
        motorSupplyCurrent = motor.getSupplyCurrent();
        motorStatorCurrent = motor.getStatorCurrent();
        motorTorqueCurrent = motor.getTorqueCurrent();
        motorStallCurrent = motor.getMotorStallCurrent();
        motorMotorVoltage = motor.getMotorVoltage();
        motorSupplyVoltage = motor.getSupplyVoltage();

        motorTemperature = motor.getDeviceTemp();

        motorPosition = motor.getPosition();
        motorVelocity = motor.getVelocity(); //convert
        motorAcceleration = motor.getAcceleration(); //convert
        motorClosedLoopError = motor.getClosedLoopError();

        motorConnected = motor.getConnectedMotor(); //double check

        this.subsystem = subsystem;
        this.motorName = motorName;

        this.port = port;
        this.destination = destination;

        populateMaps();
        populateSignalArray();

        //now i must implement this
        PhoenixUtil.publishUsingPhoenixUtil(destination, publishingSignals);
    }

    private void populateMaps() {
        motorSignals.put(ValueKey.Supply_Current.toString(), motorSupplyCurrent);
        motorSignals.put(ValueKey.Stator_Current.toString(), motorStatorCurrent);
        motorSignals.put(ValueKey.Torque_Current.toString(), motorTorqueCurrent);
        motorSignals.put(ValueKey.Stall_Current.toString(), motorStallCurrent);
        motorSignals.put(ValueKey.Motor_Voltage.toString(), motorMotorVoltage);
        motorSignals.put(ValueKey.Supply_Voltage.toString(), motorSupplyVoltage);
        motorSignals.put(ValueKey.Device_Temperature.toString(), motorTemperature);
        motorSignals.put(ValueKey.Position.toString(), motorPosition);
        motorSignals.put(ValueKey.VelocityRPS.toString(), motorVelocity);
        motorSignals.put(ValueKey.Acceleration.toString(), motorAcceleration);
        motorSignals.put(ValueKey.Closed_Loop_Error.toString(), motorClosedLoopError);
        motorSignals.put(ValueKey.Is_Connected.toString(), motorConnected);

        MasterLogger.subsystemSignalMap.putAll(motorSignals);

        MasterLogger.fullSignalMap.put(this.subsystem, motorSignals);
    }

    private void populateSignalArray() {
        publishingSignals = new BaseStatusSignal[]{
            motorSupplyCurrent,
            motorStatorCurrent,
            motorTorqueCurrent,
            motorStallCurrent,
            motorMotorVoltage,
            motorSupplyVoltage,
            motorTemperature,
            motorPosition,
            motorVelocity,
            motorAcceleration,
            motorClosedLoopError,
            motorConnected
        };
    }

    public enum ValueKey {
        Supply_Current,
        Stator_Current,
        Torque_Current,
        Stall_Current,
        Motor_Voltage,
        Supply_Voltage,
        Device_Temperature,
        Position,
        VelocityRPS,
        Acceleration,
        Closed_Loop_Error,
        Is_Connected
    }

    public enum SubsystemName {
        Handoff,
        Spindexer,
        Hood,
        Shooter,
        Turret,
        Intake
    }

    public void logEverything() {
        logEnergy();
        logTemp();
        logConnection();
        logMovement();
    }

    public void logEnergy() {
        DogLog.log(subsystem.toString() + "/" + motorName + "/" + "Supply Current", motorSupplyCurrent.getValueAsDouble());
        DogLog.log(subsystem.toString() + "/" + motorName + "/" + "Stator Current", motorStatorCurrent.getValueAsDouble());
        DogLog.log(subsystem.toString() + "/" + motorName + "/" + "Torque Current", motorTorqueCurrent.getValueAsDouble());
        DogLog.log(subsystem.toString() + "/" + motorName + "/" + "Stall Current", motorStallCurrent.getValueAsDouble());
        DogLog.log(subsystem.toString() + "/" + motorName + "/" + "Motor Voltage", motorMotorVoltage.getValueAsDouble());
        DogLog.log(subsystem.toString() + "/" + motorName + "/" + "Supply Voltage", motorSupplyVoltage.getValueAsDouble());
    }

    public void logTemp() {
        DogLog.log(subsystem.toString() + "/" + motorName + "/" + "Temperature", motorTemperature.getValueAsDouble());
    }

    public void logConnection() {
        DogLog.log("Robot/CAN/" + subsystem.toString() + "/" + motorName + "/" + "Is Connected? (ID " + String.valueOf(port) + ")", motorConnected.getValue() == ConnectedMotorValue.Unknown ? false : true);
    }

    public void logMovement() { //please help me find a better name for this ;-;
        DogLog.log(subsystem.toString() + "/" + motorName + "/" + "Velocity (RPS)", motorVelocity.getValueAsDouble());
        DogLog.log(subsystem.toString() + "/" + motorName + "/" + "Velocity (RPM)", motorVelocity.getValueAsDouble() * 60);
        DogLog.log(subsystem.toString() + "/" + motorName + "/" + "Position", motorPosition.getValueAsDouble());
        DogLog.log(subsystem.toString() + "/" + motorName + "/" + "Acceleration", motorAcceleration.getValueAsDouble());
        DogLog.log(subsystem.toString() + "/" + motorName + "/" + "Closed Loop Error", motorClosedLoopError.getValueAsDouble());
    }
}
