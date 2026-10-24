package org.firstinspires.ftc.teamcode.Base.Misc;

import com.pedropathing.math.Pose;

public class AutoUtils {
    public static Pose getMirroredPose(Pose pose) {
        double poseX = pose.x();

        return new Pose(poseX - (poseX - 70.75), pose.y(), Math.PI - pose.heading());
    }
}
