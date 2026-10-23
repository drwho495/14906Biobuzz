package org.firstinspires.ftc.teamcode.Base;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

@Config
public class Parameters {
    public static double ACCEPTABLE_HEADING_LOCK_ERROR = Math.toRadians(5);
    public static double SCORE_TRANSFER_SPEED = 0.8;
    public static double MOTOR_VOLTAGE_TARGET = 12.0;
    public static double SHOOTER_VELOCITY = 880;
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
}
