package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "0: Main Teleop", group = "0")
public class MainTeleop extends LinearOpMode {
    private RobotManager robot = new RobotManager();
    private ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(this);
        robot.enableTransferEcoMode();
        robot.enableAutoTransferPollenDuringScoreCycle();
        
        waitForStart();

        while (opModeIsActive()) {
            Pose robotPose = robot.getPose();

            if (!robot.isBrakeLowered()) {
                robot.setDrivePowers(
                        -gamepad1.left_stick_x,
                        -gamepad1.left_stick_y,
                        -gamepad1.right_stick_x
                );
            }

            if (gamepad1.rightBumperWasPressed()) {
                robot.startScoringCycle();
            } else if (gamepad1.rightBumperWasReleased() && robot.isScoringCycleEnabled()) {
                robot.stopScoringCycle();
            }

            if (gamepad1.leftBumperWasPressed()) {
                robot.enableHeadingLock();
            } else if (gamepad1.leftBumperWasReleased() && robot.isHeadingLockEnabled()) {
                robot.disableHeadingLock();
            }

            if (gamepad1.psWasPressed()) {
                robot.setPose(Parameters.ORIGIN);
            }

            if (gamepad1.dpadDownWasPressed()) {
                robot.selectOtherAlliance();
            }

            if (gamepad1.optionsWasPressed()) {
                robot.setDriverAngle(robotPose.heading(), AngleUnit.RADIANS);
            }

            if (gamepad1.yWasPressed()) {
                robot.toggleFlowerMechanismPosition();
            }

            if (gamepad1.shareWasPressed()) {
                robot.calibrateIMU();
            }

            if (gamepad1.dpadUpWasPressed()) {
                robot.toggleShooter();
            }

            if (gamepad1.x && !robot.isBrakeLowered() && robot.headingAtTarget()) {
                robot.lowerBrake();
                robot.stopDriving();
            } else if (gamepad1.xWasReleased()) {
                robot.raiseBrake();
            }

            telemetry.addData("X: ", robotPose.x());
            telemetry.addData("Y: ", robotPose.y());
            telemetry.addData("Heading: ", Math.toDegrees(robotPose.heading()));
            telemetry.addData("Heading Target: ", Math.toDegrees(robot.getHeadingLockAngle()));
            telemetry.addData("Shooter Velocity: ", robot.getShooterVelocity());
            telemetry.addData("Selected Alliance: ", robot.getAlliance());
            telemetry.addData("Loop Time: ", loopTimer.time(TimeUnit.MILLISECONDS));
            telemetry.addData("Flower Mechanism Down: ", robot.isFlowerMechanismDown());
            telemetry.addData("Is Brake Engaged: ", robot.isBrakeLowered());
            telemetry.update();

            if (gamepad1.a) {
                robot.setIntakePower(1);
            } else if (gamepad1.right_trigger > .1) {
                robot.setIntakePower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > .1) {
                robot.setIntakePower(-gamepad1.left_trigger);
            } else {
                robot.setIntakePower(0);
            }

            loopTimer.reset();

            robot.update();
        }
    }
}
