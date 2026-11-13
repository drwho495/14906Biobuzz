package org.firstinspires.ftc.teamcode.Base.Field;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

@Config
public class Hive {
    public double centerX;
    public double centerY;
    public double scoreCircleRadius = 40.0;
    public int direction = 1; // y-axis direction

    public Hive(double centerX, double centerY, int direction) {
        this.centerX = centerX;
        this.centerY = centerY;
        this.direction = direction;
    }

    public Pose getScorePoint(double fromX) {
        double raisedXandR = Math.pow(scoreCircleRadius, 2) - Math.pow(fromX - centerX, 2);

        if (raisedXandR < 0) {
            return Pose.zero();
        }

        double y = Math.sqrt(raisedXandR);

        return new Pose(fromX, (y * Math.signum(direction)) + centerY);
    }

    public Pose getScorePoint() {
        return getScorePoint(centerX);
    }

    public double getScoreHeading(Pose fromScorePoint) {
        return Math.atan2(centerY - fromScorePoint.y(), centerX - fromScorePoint.x()) + Math.PI;
    }

    public double getScoreHeading(double fromX) {
        return getScoreHeading(getScorePoint(fromX));
    }

    public double getScoreHeading() {
        return getScoreHeading(getScorePoint());
    }

    public static Hive UPPER_LEFT = new Hive(
            60,
            92,
            1
    );

    public static Hive UPPER_RIGHT = new Hive(
            83.1,
            92,
            1
    );

    public static Hive LOWER_LEFT = new Hive(
            60,
            53.3,
            -1
    );

    public static Hive LOWER_RIGHT = new Hive(
            83.1,
            53.3,
            -1
    );

    public Pose getCenterPose() {
        return new Pose(centerX, centerY);
    }
}
