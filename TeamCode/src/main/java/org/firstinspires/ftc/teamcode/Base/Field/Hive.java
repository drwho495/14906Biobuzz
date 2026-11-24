package org.firstinspires.ftc.teamcode.Base.Field;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Base.Parameters;

@Config
public class Hive {
    public double centerX;
    public double centerY;
    public double scoreCircleRadius = Parameters.HIVE_SCORE_CIRCLE_RADIUS;
    public int direction; // y-axis direction

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

    public double getFarScoreHeading(Pose fromScorePoint) {
        Pose farAimPoint = getFarAimPoint();

        return Math.atan2(farAimPoint.y() - fromScorePoint.y(), farAimPoint.x() - fromScorePoint.x()) + Math.PI;
    }

    public Pose getFarAimPoint() {
        return new Pose(centerX, centerY - (Parameters.HIVE_FAR_AIM_OFFSET * direction));
    }

    public double getScoreHeading(double fromX) {
        return getScoreHeading(getScorePoint(fromX));
    }

    public double getScoreHeading() {
        return getScoreHeading(getScorePoint());
    }

    public static Hive UPPER_LEFT = new Hive(
            60,
            101,
            1
    );

    public static Hive UPPER_RIGHT = new Hive(
            81,
            99,
            1
    );

    public static Hive LOWER_LEFT = new Hive(
            60,
            44.5,
            -1
    );

    public static Hive LOWER_RIGHT = new Hive(
            81,
            44.5,
            -1
    );

    public Pose getCenterPose() {
        return new Pose(centerX, centerY);
    }
}
