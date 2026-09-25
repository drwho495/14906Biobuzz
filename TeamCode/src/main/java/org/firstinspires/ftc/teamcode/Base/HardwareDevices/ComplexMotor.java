package org.firstinspires.ftc.teamcode.Base.HardwareDevices;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Base.Misc.PIDFController;
import org.firstinspires.ftc.teamcode.Base.Parameters;

import java.util.concurrent.TimeUnit;

enum ComplexMotorDriveMode {
    RAW_POWER,
    VELOCITY,
    POSITION
}

public class ComplexMotor extends HardwareDevice {
    private HardwareMap hardwareMap;
    private DcMotorEx motorInterface = null;
    private ComplexMotorDriveMode currentDriveMode = ComplexMotorDriveMode.RAW_POWER;
    private ComplexMotor encoderMotor = null;
    private double motorPower = 0;
    private double motorPowerTarget = 0; // updated with setPower
    private double lastMotorPower = 0;
    private double lastMotorVelocity = 0;
    private int lastMotorTickPosition = 0;
    private double motorVelocity = 0;
    private double currentVelocity;
    private double voltageTarget = 12.0;
    private final PIDFController velocityController = new PIDFController(0, 0, 0, 0);
    private ComplexVoltageSensor vSensor = null;
    private ComplexMotor childMotor = null;
    private double currentLimit = 0;
    private CurrentUnit currentLimitUnit = CurrentUnit.AMPS;
    private String deviceName = null;
    private boolean motorReversed = false;
    private ElapsedTime loopTimer = new ElapsedTime();

    public ComplexMotor(String hwName, HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.motorInterface = this.hardwareMap.get(DcMotorEx.class, hwName);
        this.deviceName = hwName;
        this.encoderMotor = this;
        this.motorInterface.setMotorEnable();
    }

    public ComplexMotor(String hwName, HardwareMap hardwareMap, ComplexVoltageSensor vSensor) {
        this.hardwareMap = hardwareMap;
        this.vSensor = vSensor;
        this.motorInterface = this.hardwareMap.get(DcMotorEx.class, hwName);
        this.deviceName = hwName;
        this.encoderMotor = this;
        this.motorInterface.setMotorEnable();
    }

    @Override
    public String getDeviceName() {
        return deviceName;
    }

    public void enableBrake() {
        this.motorInterface.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void enableFloat() {
        this.motorInterface.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setLinkedMotor(ComplexMotor linkedMotor) {
        this.childMotor = linkedMotor;
    }

    public void setEncoderDevice(ComplexMotor motor) {
        encoderMotor = motor;
    }

    public double getVelocity() {
        return currentVelocity;
    }

    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        velocityController.setPIDF(p, i, d, f);
    }

    public double[] getVelocityPIDFCoefficients() {
        return velocityController.getCoefficients();
    }

    public void setMotorRunMode(DcMotor.RunMode runMode) {
        motorInterface.setMode(runMode);
    }

    public void setVelocity(double newVelo) {
        currentDriveMode = ComplexMotorDriveMode.VELOCITY;
        motorVelocity = newVelo;
    }

    public void resetEncoder() {
        DcMotor.RunMode oldState = this.encoderMotor.motorInterface.getMode();

        this.encoderMotor.motorInterface.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.encoderMotor.motorInterface.setMode(oldState);
    }

    public void setReversed(boolean reversed) {
        motorReversed = reversed;
        motorInterface.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public void update() {
        int currentMotorTickPosition = 0;

        if (encoderMotor != null) {
            // get velocity in ticks per second
            currentMotorTickPosition = encoderMotor.motorInterface.getCurrentPosition();
            currentVelocity = (currentMotorTickPosition - lastMotorTickPosition) * (1000d / loopTimer.time(TimeUnit.MILLISECONDS));
        } else {
            currentVelocity = 0;
        }

        if (currentDriveMode == ComplexMotorDriveMode.VELOCITY) {
            if (motorVelocity != 0) {
                velocityController.setSetPoint(motorVelocity);
                motorPower = velocityController.calculate(currentVelocity);

                if (vSensor != null) {
                    motorPower *= (Parameters.MOTOR_VOLTAGE_TARGET / vSensor.getVoltage());
                }
            } else {
                motorPower = 0;
            }
        } else if (currentDriveMode == ComplexMotorDriveMode.RAW_POWER) {
            motorPower = motorPowerTarget;
        }

        if (lastMotorPower != motorPower) {
            motorInterface.setPower(motorPower);

            if (childMotor != null) {
                childMotor.setPower(motorPower);
            }
        }

        lastMotorPower = motorPower;
        lastMotorVelocity = motorVelocity;
        lastMotorTickPosition = currentMotorTickPosition;

        loopTimer.reset();
    }

    public double getCurrent() {
        return motorInterface.getCurrent(CurrentUnit.AMPS);
    }

    public void setPower(double power) {
        currentDriveMode = ComplexMotorDriveMode.RAW_POWER;
        motorPowerTarget = power;
    }

    public boolean atVelocity(double error) {
        if (currentDriveMode == ComplexMotorDriveMode.VELOCITY) {
            return abs(currentVelocity - motorVelocity) <= error;
        }
        return true;
    }

    public void setCurrentLimit(CurrentUnit unit, double value) {
        if (unit != currentLimitUnit || value != currentLimit) {
            motorInterface.setCurrentAlert(value, unit);
        }
    }

    public boolean isOverCurrent(CurrentUnit unit, double value) {
        setCurrentLimit(unit, value);

        return isOverCurrent();
    }

    public boolean isOverCurrent() {
        return motorInterface.isOverCurrent();
    }

    public boolean isReversed() {
        return motorReversed;
    }

    public void enableEncoderDrive() {
        motorInterface.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void disableEncoderDrive() {
        motorInterface.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}