package org.firstinspires.ftc.teamcode.Base;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Base.Misc.Flower;

@Config
public class Parameters {
    public static double ACCEPTABLE_HEADING_LOCK_ERROR = Math.toRadians(5);
    public static double SCORE_TRANSFER_VELOCITY = 1200;
    public static double MOTOR_VOLTAGE_TARGET = 12.0;
    public static double SHOOTER_VELOCITY = 955;
    public static double HEADING_LOCK_STRENGTH = 1.0;
    public static double SHOOTER_READY_TOLERANCE = 100.0;
    public static double GATE_OPEN_POSITION = 30.0;
    public static double GATE_CLOSED_POSITION = 110.0;
    public static long SHOOTER_TRANSFER_TIME = 1000;
    public static double TRANSFER_OVERCURRENT_THRESHOLD = 4.5;
    public static double FLOWER_MECHANISM_SERVO_OFFSET = 20;
    public static double FLOWER_MECHANISM_SERVO_DOWN = 0;
    public static double FLOWER_MECHANISM_SERVO_UP = 83;

    public static Pose RED_AIM_GOAL_1 = new Pose(40, 40);
    public static Pose RED_AIM_GOAL_2 = new Pose(40, 90);
    public static Pose BLUE_AIM_GOAL_1 = new Pose(90, 40);
    public static Pose BLUE_AIM_GOAL_2 = new Pose(90, 90);

    public static Pose ORIGIN = new Pose(7.875, 8, Math.PI/2);
    public static Pose GOAL_RED_AUTO_START = new Pose(55.3, 12, Math.toRadians(270));
    public static Pose FLOWER_RED_AUTO_START = new Pose(36, 133, Math.toRadians(90));

    public static Flower LEFT_FLOWER = new Flower(new Pose(18, 49), new Pose(25, 49), Math.PI);
}
