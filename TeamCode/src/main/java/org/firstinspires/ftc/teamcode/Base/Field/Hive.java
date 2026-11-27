package org.firstinspires.ftc.teamcode.Base.Field;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.Utilities.MathUtils;

@Config
public class Hive {
    public double centerX;
    public double centerY;
    public double regressionX;
    public double regressionY;
    public double scoreCircleRadius = Parameters.HIVE_SCORE_CIRCLE_RADIUS;
    public int direction; // y-axis direction

    public Hive(double centerX, double centerY, double regressionX, double regressionY, int direction) {
        this.centerX = centerX;
        this.centerY = centerY;
        this.regressionX = regressionX;
        this.regressionY = regressionY;
        this.direction = direction;
    }

    public Hive(Pose center, Pose regressionPose, int direction) {
        this.centerX = center.x();
        this.centerY = center.y();
        this.regressionX = regressionPose.x();
        this.regressionY = regressionPose.y();
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

    public Pose getFarAimPoint() {
        return new Pose(centerX, centerY - (Parameters.HIVE_FAR_AIM_OFFSET * direction));
    }

    public double getScoreHeading(double fromX) {
        return getScoreHeading(getScorePoint(fromX));
    }

    public double getScoreHeading() {
        return getScoreHeading(getScorePoint());
    }

    public Pose getCenterPose() {
        return new Pose(centerX, centerY);
    }

    public Pose getRegressionPose() {
        return new Pose(regressionX, regressionY);
    }

    public static Hive UPPER_LEFT = new Hive(
            62,
            83,
            65,
            81,
            1
    );

    public static Hive UPPER_RIGHT = new Hive(
            2*(Parameters.FIELD_CENTER.x()) - UPPER_LEFT.centerX,
            UPPER_LEFT.centerY,
            2*(Parameters.FIELD_CENTER.x()) - UPPER_LEFT.regressionX,
            UPPER_LEFT.regressionY,
            1
    );

    public static Hive LOWER_LEFT = new Hive(
            MathUtils.rotateAboutFieldCenter(UPPER_RIGHT.getCenterPose(), Math.PI),
            MathUtils.rotateAboutFieldCenter(UPPER_RIGHT.getRegressionPose(), Math.PI),
            -1
    );

    public static Hive LOWER_RIGHT = new Hive(
            MathUtils.rotateAboutFieldCenter(UPPER_LEFT.getCenterPose(), Math.PI),
            MathUtils.rotateAboutFieldCenter(UPPER_LEFT.getRegressionPose(), Math.PI),
            -1
    );
}
