package org.firstinspires.ftc.teamcode.Base.Utilities;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

import org.firstinspires.ftc.teamcode.Base.Parameters;

public class MathUtils {
    public static double convertToRPM(double fromTicksPerSecond) {
        return (fromTicksPerSecond * 60) / 28;
    }

    public static Pose rotateAboutFieldCenter(Pose fromPose, double byRadians) {
        // rotate by field center
        return fromPose.toVector2D()
                .minus(Parameters.FIELD_CENTER)
                .rotate(byRadians)
                .plus(Parameters.FIELD_CENTER)
                .toPose()
                .withHeading(fromPose.heading() + byRadians);
    }

    public static Pose distanceAlongDirection(Vector2D startVector, double distance, Vector2D direction) {
        return startVector.plus(direction.normalized().times(distance)).toPose();
    }

    public static Pose distanceAlongDirection(Pose startPose, double distance, Vector2D direction) {
        return distanceAlongDirection(startPose.toVector2D(), distance, direction);
    }
}
