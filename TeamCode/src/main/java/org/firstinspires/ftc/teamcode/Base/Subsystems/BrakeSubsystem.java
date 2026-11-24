package org.firstinspires.ftc.teamcode.Base.Subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.HardwareDevices.ComplexServo;
import org.firstinspires.ftc.teamcode.Base.HardwareTable;
import org.firstinspires.ftc.teamcode.Base.Parameters;

public class BrakeSubsystem {
    private HardwareTable hardwareTable;
    private ComplexServo leftBrakeServo;
    private ComplexServo rightBrakeServo;
    private double leftBrakePosition = 0;
    private double rightBrakePosition = 0;
    private boolean brakeLowered = false;

    public void initialize(HardwareTable hardwareTable) {
        this.hardwareTable = hardwareTable;

        leftBrakeServo = new ComplexServo(
                "leftBrakeServo",
                hardwareTable.getHardwareMap(),
                0,
                355,
                AngleUnit.DEGREES
        );
        leftBrakeServo.setInverted(true);

        rightBrakeServo = new ComplexServo(
                "rightBrakeServo",
                hardwareTable.getHardwareMap(),
                0,
                355,
                AngleUnit.DEGREES
        );
        rightBrakeServo.setInverted(!leftBrakeServo.getInverted());

        leftBrakeServo.setAngleOffset(Parameters.LEFT_BRAKE_SERVO_OFFSET, AngleUnit.DEGREES);
        rightBrakeServo.setAngleOffset(Parameters.RIGHT_BRAKE_SERVO_OFFSET, AngleUnit.DEGREES);
    }

    public void update() {
        leftBrakeServo.setAngleOffset(Parameters.LEFT_BRAKE_SERVO_OFFSET, AngleUnit.DEGREES);
        rightBrakeServo.setAngleOffset(Parameters.RIGHT_BRAKE_SERVO_OFFSET, AngleUnit.DEGREES);

        leftBrakeServo.turnToAngle(leftBrakePosition);
        rightBrakeServo.turnToAngle(rightBrakePosition);
    }

    private void updateBrakePosition(boolean lower) {
        if (lower) {
            leftBrakePosition = Parameters.BRAKE_DOWN_POSITION;
            rightBrakePosition = Parameters.BRAKE_DOWN_POSITION;
        } else {
            leftBrakePosition = Parameters.BRAKE_UP_POSITION;
            rightBrakePosition = Parameters.BRAKE_UP_POSITION;
        }

        brakeLowered = lower;
    }

    public void lower() {
        updateBrakePosition(true);
    }

    public void raise() {
        updateBrakePosition(false);
    }

    public void toggle() {
        updateBrakePosition(!brakeLowered);
    }

    public boolean isLowered() {
        return brakeLowered;
    }
}
