package org.firstinspires.ftc.teamcode.Base.Utilities;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

import org.firstinspires.ftc.teamcode.Base.Parameters;

public class MathUtils {
    public static double ticksPerSecondToRPM(double tps) {
        return (tps * 60) / 28;
    }

    public static Pose rotatePoseBy(Pose pose, double radians) {
        // rotate by field center
        return pose.toVector2D()
                .minus(Parameters.FIELD_CENTER)
                .rotate(radians)
                .plus(Parameters.FIELD_CENTER)
                .toPose()
                .withHeading(pose.heading() + radians);
    }
}
