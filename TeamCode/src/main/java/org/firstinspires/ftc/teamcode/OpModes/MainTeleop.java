package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Base.RobotManager;

@TeleOp(name = "0: Main Teleop", group = "0")
public class MainTeleop extends LinearOpMode {
    RobotManager robot = new RobotManager();
    
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap);
        
        waitForStart();

        while (opModeIsActive()) {
            Pose robotPose = robot.getPose();

            robot.setDrivePowers(
                    -gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x
            );

            if (gamepad1.aWasPressed()) {
                robot.startScoringElements();
            } else if (gamepad1.aWasReleased() && robot.isScoringElements()) {
                robot.stopScoringElements();
            }

            if (gamepad1.xWasPressed()) {
                robot.enableHeadingLock();
            } else if (gamepad1.xWasReleased() && robot.isHeadingLockEnabled()) {
                robot.disableHeadingLock();
            }

            if (gamepad1.dpadDownWasPressed()) {
                robot.selectOtherAlliance();
            }

            if (gamepad1.optionsWasPressed()) {
                robot.setDriverAngle(robotPose.heading());
            }

            if (gamepad1.shareWasPressed()) {
                robot.calibrateIMU();
            }

            telemetry.addData("X: ", robotPose.x());
            telemetry.addData("Y: ", robotPose.y());
            telemetry.addData("Heading: ", robotPose.heading());
            telemetry.addData("Shooter Velocity: ", robot.getShooterVelocity());
            telemetry.addData("Selected Alliance: ", robot.getAlliance());
            telemetry.update();

            robot.setIntakePower(gamepad1.right_trigger);
            robot.update();
        }
    }
}
