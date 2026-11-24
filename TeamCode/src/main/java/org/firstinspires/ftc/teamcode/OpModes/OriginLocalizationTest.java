package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;

@Utility(name = "Utility: Origin Localization Test", description = "Robot Localization Test, the robot should start on the bottom-left of the field (the origin).")
public class OriginLocalizationTest extends LinearOpMode {
    private RobotManager robot = new RobotManager();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(this);
        robot.calibrateIMU();

        waitForStart();

        robot.setPose(Parameters.ORIGIN_RED);

        while (opModeIsActive()) {
            Pose robotPose = robot.getPose();

            telemetry.addData("X: ", robotPose.x());
            telemetry.addData("Y: ", robotPose.y());
            telemetry.addData("Heading: ", Math.toDegrees(robotPose.heading()));
            telemetry.update();

            robot.update();
        }
    }
}
