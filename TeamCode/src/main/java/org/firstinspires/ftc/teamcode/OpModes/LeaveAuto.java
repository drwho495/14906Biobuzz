package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import static com.pedropathing.api.Paths.*;
import org.firstinspires.ftc.teamcode.Base.RobotManager;

@Autonomous(name = "Leave Auto", preselectTeleOp = "0: Main Teleop")
public class LeaveAuto extends LinearOpMode {
    private RobotManager robot = new RobotManager();
    private Follower follower = null;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(this);
        follower = robot.getFollower();

        follower.setPose(new Pose(0, 0, 0));
        follower.holdEnd.set(true);

        waitForStart();

        follower.follow(
                line(robot.getPose(), new Pose(0, -20)).constant(Math.toRadians(0))
        );

        ElapsedTime timer = new ElapsedTime();

        while (follower.isBusy() && opModeIsActive()) {
            robot.update();
        }
    }
}
