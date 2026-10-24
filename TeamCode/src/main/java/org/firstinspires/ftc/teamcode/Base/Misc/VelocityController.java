package org.firstinspires.ftc.teamcode.Base.Misc;

// re-implementation of https://github.wpilib.org/allwpilib/docs/release/cpp/_simple_motor_feedforward_8h_source.html with some modifications.
public class VelocityController {
    private double kP = 0; // currently unused
    private double kS = 0;
    private double kV = 0;
    private double kA = 0;

    public VelocityController(double kP, double kS, double kV, double kA) {
        this.kP = kP;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    public VelocityController() {}

    public void set(double kP, double kS, double kV, double kA){
        this.kP = kP;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    public double calculate(double currentVelocity, double targetVelocity, long deltaTime) {
        if (kA < 1e-9) {
            return kS * Math.signum(currentVelocity) + kV * targetVelocity;
        } else {
            double A = -kV / kA;
            double B = 1.0 / kA;
            double A_d = Math.exp(A * deltaTime);
            double B_d = A > -1e-9 ? B * deltaTime : 1.0 / A * (A_d - 1.0) * B;
            return kS * Math.signum(currentVelocity) + 1.0 / B_d * (targetVelocity - A_d * currentVelocity);
        }
    }
}
