package org.firstinspires.ftc.teamcode.Base.Field;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

public class Flower {
    public Vector2D flowerCenter;
    public Vector2D direction; // direction facing AWAY from the field.

    public Flower(Vector2D center, Vector2D direction) {
        this.flowerCenter = center;
        this.direction = direction;
    }

    // along direction
    public Pose getPoseFromCenter(double distance) {
        return flowerCenter.plus(direction.times(distance)).toPose();
    }

    public double getIntakeHeading() {
        return direction.theta() + Math.PI;
    }

    public static Flower LEFT = new Flower(
            Vector2D.cartesian(2.5, 49.5),
            Vector2D.unit(0)
    );

    public static Flower RIGHT = new Flower(
            Vector2D.cartesian(139, 94.5),
            Vector2D.unit(Math.PI)
    );

    public static Flower UPPER = new Flower(
            Vector2D.cartesian(49, 143),
            Vector2D.unit((3*Math.PI)/2)
    );

    public static Flower LOWER = new Flower(
            Vector2D.cartesian(94, 2.5),
            Vector2D.unit(Math.PI / 2)
    );
}
