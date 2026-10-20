package org.firstinspires.ftc.teamcode.Base.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Base.HardwareDevices.ComplexMotor;
import org.firstinspires.ftc.teamcode.Base.HardwareDevices.ComplexServo;
import org.firstinspires.ftc.teamcode.Base.HardwareTable;
import org.firstinspires.ftc.teamcode.Base.Parameters;

public class IntakeSubsystem {
    private ComplexMotor intakeMotor1;
    private ComplexMotor intakeMotor2;
    private ComplexServo flowerMechServo;
    private double frontRollersPower = 0;
    private double transferRollersPower = 0;
    private boolean transferOvercurrent = false;
    private boolean transferOvercurrentSet = false;
    private boolean flowerMechanismDown = false;

    public void initialize(HardwareMap hardwareMap, HardwareTable hardwareTable) {
        intakeMotor1 = hardwareTable.createMotor("intakeMotor1");
        intakeMotor1.enableBrake();
        intakeMotor1.setReversed(false);

        intakeMotor2 = hardwareTable.createMotor("intakeMotor2");
        intakeMotor2.enableFloat();
//        intakeMotor2.enableEncoderDrive();
        intakeMotor2.setReversed(intakeMotor1.isReversed());

        flowerMechServo = new ComplexServo(
                "flowerMechServo",
                hardwareMap,
                0,
                180,
                 AngleUnit.DEGREES
        );
        flowerMechServo.setInverted(true);
        flowerMechServo.setPositionOffset(Parameters.FLOWER_MECHANISM_SERVO_OFFSET);
    }

    public void update() {
        intakeMotor1.setPower(frontRollersPower);
        intakeMotor2.setPower(transferRollersPower);

        intakeMotor2.setCurrentLimit(CurrentUnit.AMPS, Parameters.TRANSFER_OVERCURRENT_THRESHOLD);

        intakeMotor1.update();
        intakeMotor2.update();

        if (flowerMechanismDown) {
            flowerMechServo.turnToAngle(Parameters.FLOWER_MECHANISM_SERVO_DOWN);
        } else {
            flowerMechServo.turnToAngle(Parameters.FLOWER_MECHANISM_SERVO_UP);
        }

        transferOvercurrentSet = false;
    }

    public void setFrontRollersPower(double power) {
        frontRollersPower = power;
    }

    public void setTransferRollersPower(double power) {
        transferRollersPower = power;
    }

    public boolean isFlowerMechanismDown() {
        return flowerMechanismDown;
    }

    public void lowerFlowerMechanism() {
        flowerMechanismDown = true;
    }

    public void raiseFlowerMechanism() {
        flowerMechanismDown = true;
    }

    public void toggleFlowerMechanismPosition() {
        flowerMechanismDown = !flowerMechanismDown;
    }

    public boolean isTransferOvercurrent() {
        if (!transferOvercurrentSet) {
            transferOvercurrentSet = true;
            transferOvercurrent = intakeMotor2.isOverCurrent();
        }

        return transferOvercurrent;
    }
}
