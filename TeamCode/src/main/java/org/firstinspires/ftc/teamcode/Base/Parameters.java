package org.firstinspires.ftc.teamcode.Base;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

import org.firstinspires.ftc.teamcode.Base.Misc.Flower;

@Config
public class Parameters {
    public static double ACCEPTABLE_HEADING_LOCK_ERROR = Math.toRadians(5);
    public static double SCORE_TRANSFER_VELOCITY = 2800;
    public static double MOTOR_VOLTAGE_TARGET = 12.0;
    public static double SHOOTER_VELOCITY = 1430;
    public static double HEADING_LOCK_STRENGTH = 1.0;
    public static double SHOOTER_READY_TOLERANCE = 100.0;
    public static double GATE_OPEN_POSITION = 15.0;
    public static double GATE_CLOSED_POSITION = 120.0;
    public static long SHOOTER_TRANSFER_TIME = 1000;
    public static long TRANSFER_START_DELAY_TIME = 250;
    public static double TRANSFER_OVERCURRENT_THRESHOLD = 4.5;
    public static double FLOWER_MECHANISM_SERVO_OFFSET = 18;
    public static double FLOWER_MECHANISM_SERVO_DOWN = 0;
    public static double FLOWER_MECHANISM_SERVO_UP = 80;

    public static double FLOWER_INTAKE_DISTANCE = 17;
    public static double FLOWER_BACK_AWAY_DISTANCE = 30;

    public static double LEFT_BRAKE_SERVO_OFFSET = 148;
    public static double RIGHT_BRAKE_SERVO_OFFSET = 180;
    public static double BRAKE_DOWN_POSITION = 60.0;
    public static double BRAKE_UP_POSITION = 0;

    public static Vector2D FIELD_CENTER = Vector2D.cartesian(70.75, 70.75);
    public static Pose ORIGIN = new Pose(7.875, 8, Math.PI/2);
    public static Pose GOAL_RED_AUTO_START = new Pose(55.3, 12, Math.toRadians(270));
    public static Pose FLOWER_RED_AUTO_START = new Pose(36, 133, Math.toRadians(90));
}
