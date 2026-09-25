package org.firstinspires.ftc.teamcode.Base.HardwareDevices;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class ComplexVoltageSensor extends HardwareDevice {
    private VoltageSensor sensor;
    private HardwareMap hardwareMap;

    public ComplexVoltageSensor(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        sensor = this.hardwareMap.voltageSensor.iterator().next();
    }

    public double getVoltage() {
        return sensor.getVoltage();
    }

    @Override
    public String getDeviceName() {
        return "Voltage Sensor";
    }
}
