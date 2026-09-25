package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.HardwareDevices.ComplexMotor;
import org.firstinspires.ftc.teamcode.Base.HardwareDevices.ComplexServo;
import org.firstinspires.ftc.teamcode.Base.HardwareDevices.ComplexVoltageSensor;
import org.firstinspires.ftc.teamcode.Base.HardwareDevices.HardwareDevice;

import java.util.ArrayList;

public class HardwareTable {
    private ArrayList<String> registeredHardwareNames = new ArrayList<>();
    private ArrayList<HardwareDevice> registeredHardware = new ArrayList<>();
    private HardwareMap hardwareMap;
    private ComplexVoltageSensor voltageSensor;

    public HardwareTable(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        voltageSensor = new ComplexVoltageSensor(hardwareMap);

        registerDevice(voltageSensor);
    }

    public void registerDevice(HardwareDevice device) {
        if (!registeredHardware.contains(device)) {
            registeredHardware.add(device);
            registeredHardwareNames.add(device.getDeviceName());
        }
    }

    public boolean hasDevice(String device) {
        return registeredHardwareNames.contains(device);
    }

    public boolean hasDevice(HardwareDevice device) {
        return registeredHardware.contains(device);
    }

    public ArrayList<HardwareDevice> getDevices() {
        return registeredHardware;
    }

    public ComplexMotor createMotor(String deviceName) {
        ComplexMotor motor = new ComplexMotor(deviceName, hardwareMap, voltageSensor);

        registerDevice(motor);

        return motor;
    }

    public ComplexServo createServo(String deviceName, double minAngle, double maxAngle, AngleUnit angleUnit) {
        ComplexServo servo = new ComplexServo(deviceName, hardwareMap, minAngle, maxAngle, angleUnit);

        registerDevice(servo);

        return servo;
    }

    public ComplexVoltageSensor getVoltageSensor() {
        return voltageSensor;
    }
}