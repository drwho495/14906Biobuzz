package org.firstinspires.ftc.teamcode.Base.Subsystems;

//import com.bylazar.configurables.annotations.Configurable;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.HardwareDevices.ComplexMotor;
import org.firstinspires.ftc.teamcode.Base.HardwareDevices.ComplexServo;
import org.firstinspires.ftc.teamcode.Base.HardwareTable;
import org.firstinspires.ftc.teamcode.Base.Parameters;

//@Configurable
@Config
public class ShooterSubsystem {
    private ComplexMotor shooterMotor1;
    private ComplexMotor shooterMotor2;
    private ComplexServo gateServo;
    private double motorVelocity = 0;
    private boolean gateOpen = false;
    private boolean poweredOn = false;

    public static double transferP = 0;
    public static double transferS = 0.05;
    public static double transferV = 0.00035;
    public static double transferA = 0.2;

    public void initialize(HardwareTable hardwareTable) {
        shooterMotor1 = hardwareTable.createMotor("shooterMotor1");
        shooterMotor1.setReversed(false);
        shooterMotor1.disableEncoderDrive();
        shooterMotor1.enableFloat();

        shooterMotor2 = hardwareTable.createMotor("shooterMotor2");
        shooterMotor2.setReversed(!shooterMotor1.isReversed());
        shooterMotor2.disableEncoderDrive();
        shooterMotor2.enableFloat();

        shooterMotor1.useCustomController();
        shooterMotor2.useCustomController();
        shooterMotor1.resetEncoder();
        shooterMotor2.resetEncoder();

        gateServo = hardwareTable.createServo(
                "gateServo",
                0,
                270,
                AngleUnit.DEGREES
        );
        gateServo.setInverted(false);
    }

    public double getMotor1Current() {
        return shooterMotor1.getCurrent();
    }

    public double getMotor2Current() {
        return shooterMotor2.getCurrent();
    }

    public void setTargetVelocity(double velocity) {
        motorVelocity = velocity;
    }

    public double getTargetVelocity() {
        return motorVelocity;
    }

    public double getMotorVelocity() {
        return shooterMotor1.getVelocity();
    }

    public void update() {
        if (gateOpen) {
            gateServo.turnToAngle(Parameters.GATE_OPEN_POSITION);
        } else {
            gateServo.turnToAngle(Parameters.GATE_CLOSED_POSITION);
        }

        shooterMotor1.setVelocityCoeffs(
                transferP,
                transferS,
                transferV,
                transferA
        );

        shooterMotor2.setVelocityCoeffs(
                transferP,
                transferS,
                transferV,
                transferA
        );

        if (poweredOn) {
            shooterMotor1.setVelocity(Parameters.SHOOTER_VELOCITY);
            shooterMotor2.setVelocity(Parameters.SHOOTER_VELOCITY);
        } else {
            shooterMotor1.setPower(0);
            shooterMotor2.setPower(0);
        }

        shooterMotor1.update();
        shooterMotor2.update();
    }

    public boolean ready() {
        return shooterMotor1.atVelocity(Parameters.SHOOTER_READY_TOLERANCE);
    }

    public void openGate() {
        gateOpen = true;
    }

    public void closeGate() {
        gateOpen = false;
    }

    public void powerOff() {
        poweredOn = false;
    }

    public void powerOn() {
        poweredOn = true;
    }

    public void togglePowered() {
        poweredOn = !poweredOn;
    }

    public boolean isPoweredOn() {
        return poweredOn && motorVelocity > 0;
    }
}
