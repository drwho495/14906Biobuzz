package org.firstinspires.ftc.teamcode.Base.Misc;

import com.pedropathing.math.Pose;

public class Flower {
    public Pose robotIntakePosition;
    public Pose robotBackAwayPosition;
    public double robotHeading;

    public Flower(Pose intakePosition, Pose backAwayPosition, double heading) {
        robotIntakePosition = intakePosition;
        robotBackAwayPosition = backAwayPosition;
        robotHeading = heading;
    }
}
