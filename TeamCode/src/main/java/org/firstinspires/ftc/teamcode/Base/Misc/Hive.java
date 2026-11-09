package org.firstinspires.ftc.teamcode.Base.Misc;

import com.pedropathing.math.Pose;

public class Hive {
    public Pose centerPosition;
    public double scoreCircleRadius = 35.0;
    public int direction = 1; // y-axis direction

    public Hive(Pose center, int direction) {
        this.centerPosition = center;
        this.direction = direction;
    }

    public Pose getScorePoint(double fromX) {
        double raisedXandR = Math.pow(scoreCircleRadius, 2) - Math.pow(fromX - centerPosition.x(), 2);

        if (raisedXandR < 0) {
            return Pose.zero();
        }

        double y = Math.sqrt(raisedXandR);

        // two solutions, find the correct one with direction.
        return centerPosition.withY((y * Math.signum(direction)) + centerPosition.y());
    }

    public Pose getScorePoint() {
        return getScorePoint(centerPosition.x());
    }

    public static Hive UPPER_LEFT = new Hive(
            new Pose(60.8, 52.6),
            1
    );

    public static Hive UPPER_RIGHT = new Hive(
            new Pose(0, 0),
            1
    );

    public static Hive LOWER_LEFT = new Hive(
            new Pose(60.8, 89.5),
            -1
    );

    public static Hive LOWER_RIGHT = new Hive(
            new Pose(0, 0),
            -1
    );
}
