package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import static com.pedropathing.api.Paths.*;

@Autonomous(name = "Flower Side Auto (Park)", group = "0")
public class FlowerSideAuto extends LinearOpMode {
    private RobotManager robot = new RobotManager();

    @Override
    public void runOpMode() {
        robot.initialize(this);
        robot.setPose(Parameters.FLOWER_RED_AUTO_START);

        while (opModeInInit()) {
            robot.update();
        }

        waitForStart();

        Pose robotPose = robot.getPose();
        robot.runPathBlocking(
                curve(
                        robotPose,
                        new Pose(9.851, 134.8353),
                        new Pose(8.5472, 114.0236)
                ).constant(Math.toRadians(90))
        );

        robot.sleep(5000);
    }
}
