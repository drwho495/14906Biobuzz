package org.firstinspires.ftc.teamcode.Base.Misc;

public class BangBangController {
    private double controlEffort = 1;

    public BangBangController(double controlEffort) {
        this.controlEffort = controlEffort;
    }

    public void set(double controlEffort) {
        this.controlEffort = controlEffort;
    }

    public double calculate(double current, double target) {
        if (current < target) {
            return controlEffort;
        } else {
            return 0;
        }
    }
}
