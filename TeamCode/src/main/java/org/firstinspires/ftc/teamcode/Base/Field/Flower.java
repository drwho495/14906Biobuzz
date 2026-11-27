package org.firstinspires.ftc.teamcode.Base.Field;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

import org.firstinspires.ftc.teamcode.Base.Utilities.MathUtils;

public class Flower {
    public Vector2D flowerCenter;
    public Vector2D direction; // direction facing AWAY from the field.

    public Flower(Vector2D center, Vector2D direction) {
        this.flowerCenter = center;
        this.direction = direction;
    }

    public Pose getPoseFromCenter(double distance, double directionOffset) {
        return MathUtils.distanceAlongDirection(flowerCenter, distance, direction.rotate(directionOffset));
    }

    public Pose getPoseFromCenter(double distance) {
        return getPoseFromCenter(distance, 0);
    }

    public double getIntakeHeading() {
        return direction.theta() + Math.PI;
    }

    public static final Flower LEFT = new Flower(
            Vector2D.cartesian(2, 46.5),
            Vector2D.unit(0)
    );

    public static final Flower RIGHT = new Flower(
            MathUtils.rotateAboutFieldCenter(Flower.LEFT.flowerCenter.toPose(), Math.PI).toVector2D(),
            Vector2D.unit(Math.PI)
    );

    public static final Flower UPPER = new Flower(
            Vector2D.cartesian(49.5, 144),
            Vector2D.unit((3*Math.PI)/2)
    );

    public static final Flower LOWER = new Flower(
            MathUtils.rotateAboutFieldCenter(Flower.UPPER.flowerCenter.toPose(), Math.PI).toVector2D(),
            Vector2D.unit(Math.PI / 2)
    );
}
