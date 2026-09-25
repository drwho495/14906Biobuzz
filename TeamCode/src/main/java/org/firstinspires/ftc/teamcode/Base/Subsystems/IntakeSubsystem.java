package org.firstinspires.ftc.teamcode.Base.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Base.HardwareDevices.ComplexMotor;
import org.firstinspires.ftc.teamcode.Base.HardwareTable;

public class IntakeSubsystem {
    private ComplexMotor intakeMotor1;
    private ComplexMotor intakeMotor2;
    private double frontRollersPower = 0;
    private double transferRollersPower = 0;

    public void initialize(HardwareMap hardwareMap, HardwareTable hardwareTable) {
        intakeMotor1 = hardwareTable.createMotor("intakeMotor1");
        intakeMotor1.enableBrake();
        intakeMotor1.setReversed(false);

        intakeMotor2 = hardwareTable.createMotor("intakeMotor2");
        intakeMotor2.enableFloat();
        intakeMotor2.setReversed(intakeMotor1.isReversed());
    }

    public void update() {
        intakeMotor1.setPower(frontRollersPower);
        intakeMotor2.setPower(transferRollersPower);

        intakeMotor1.update();
        intakeMotor2.update();
    }

    public void setFrontRollersPower(double power) {
        frontRollersPower = power;
    }

    public void setTransferRollersPower(double power) {
        transferRollersPower = power;
    }
}
