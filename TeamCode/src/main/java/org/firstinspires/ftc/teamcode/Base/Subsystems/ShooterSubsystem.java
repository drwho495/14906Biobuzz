package org.firstinspires.ftc.teamcode.Base.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.HardwareDevices.ComplexMotor;
import org.firstinspires.ftc.teamcode.Base.HardwareDevices.ComplexServo;
import org.firstinspires.ftc.teamcode.Base.HardwareTable;
import org.firstinspires.ftc.teamcode.Base.Parameters;

@Configurable
public class ShooterSubsystem {
    private ComplexMotor shooterMotor1;
    private ComplexMotor shooterMotor2;
    private ComplexServo gateServo;
    private double motorVelocity = 0;
    private boolean gateOpen = false;

    public static double transferP = 0.003;
    public static double transferI = 0;
    public static double transferD = 0;
    public static double transferF = 0.0005;

    public void initialize(HardwareMap hardwareMap, HardwareTable hardwareTable) {
        shooterMotor1 = hardwareTable.createMotor("shooterMotor1");
        shooterMotor1.setReversed(true);
        shooterMotor1.enableBrake();

        shooterMotor2 = hardwareTable.createMotor("shooterMotor2");
        shooterMotor2.setReversed(!shooterMotor1.isReversed());
        shooterMotor2.enableBrake();

        shooterMotor1.setLinkedMotor(shooterMotor2);
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
            gateServo.setPosition(Parameters.GATE_OPEN_POSITION);
        } else {
            gateServo.setPosition(Parameters.GATE_CLOSED_POSITION);
        }

        shooterMotor1.setVelocityPIDFCoefficients(
                transferP,
                transferI,
                transferD,
                transferF
        );

        shooterMotor1.setPower(Parameters.SHOOTER_VELOCITY);

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
}
