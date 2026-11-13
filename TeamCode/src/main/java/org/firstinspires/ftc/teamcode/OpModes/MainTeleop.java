package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.Base.Utilities.MathUtils;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "0: Main Teleop", group = "0")
public class MainTeleop extends LinearOpMode {
    private RobotManager robot = new RobotManager();
    private ElapsedTime loopTimer = new ElapsedTime();
    private boolean printDebugInfo = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(this);
        robot.enableTransferEcoMode();
        robot.enableAutoTransferPollenDuringScoreCycle();
        robot.disableSubsystems();
        robot.resetDriverAngle();

        while (opModeInInit()) {
            robot.update();
        }
        
        waitForStart();

        // do not remove!
        robot.enableSubsystems();

        while (opModeIsActive()) {
            Pose robotPose = robot.getPose();

            // do not remove!
            if (!robot.areSubsystemsEnabled()) {
                if (gamepad1.a || gamepad2.a) {
                    robot.enableSubsystems();
                }

                telemetry.addLine("!!! SUBSYSTEMS ARE DISABLED !!!");
                telemetry.addLine("!!! PRESS 'A' ON ANY GAMEPAD !!!");
                telemetry.update();

                continue;
            }

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
                robot.resetDriverAngle();
            }

            if (gamepad1.dpadDownWasPressed()) {
                robot.switchAlliance();
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

            if (gamepad2.optionsWasPressed()) {
                printDebugInfo = !printDebugInfo;
            }

            telemetry.addData("Selected Alliance: ", robot.getAlliance());
            telemetry.addData("Shooter Velocity (RPM): ", MathUtils.convertToRPM(robot.getShooterVelocity()));
            telemetry.addLine();
            telemetry.addData("X: ", robotPose.x());
            telemetry.addData("Y: ", robotPose.y());
            telemetry.addData("Heading: ", Math.toDegrees(robotPose.heading()));

            if (printDebugInfo) {
                telemetry.addLine();
                telemetry.addLine("Debug Information:");
                telemetry.addData("Heading Target: ", Math.toDegrees(robot.getHeadingLockAngle()));
                telemetry.addData("Heading at Target: ", robot.headingAtTarget());
                telemetry.addData("Heading Error: ", Math.toDegrees(robot.getHeadingError()));
                telemetry.addData("Loop Time: ", loopTimer.time(TimeUnit.MILLISECONDS));
            }

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
