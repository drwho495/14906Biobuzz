package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;

@TeleOp(name = "Utility: Shooter Test"/*, description = "Provides the user with tools and data to reliably test the shooter."*/)
public class ShooterTest extends LinearOpMode {
    RobotManager robot = new RobotManager();
    FtcDashboard dashboard;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap);

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        TelemetryPacket packet = new TelemetryPacket();

        double shooterVelocity = 0;

        boolean shooterCycleRunning = false;
        boolean shooterCycleComplete = false;
        double scoringCycleStartShooterVelocity = 0;
        double scoringCycleEndShooterVelocity = 0;

        while (opModeIsActive()) {
            packet.clearLines();

            if (gamepad1.dpadUpWasPressed()) {
                robot.toggleShooter();
            }

            if (robot.isShooterPoweredOn()) {
                shooterVelocity = robot.getShooterVelocity();

                if (gamepad1.rightBumperWasPressed() && !shooterCycleRunning) {
                    robot.startScoringCycle();

                    scoringCycleStartShooterVelocity = shooterVelocity;

                    shooterCycleRunning = true;
                    shooterCycleComplete = false;
                }

                if (!robot.isAutoTransferPollenDuringScoreCycleEnabled() || !robot.isTransferCycleTimerEnabled() && shooterCycleRunning && gamepad1.rightBumperWasReleased()) {
                    robot.stopScoringCycle();
                }

                if (shooterCycleRunning && !robot.isScoringCycleEnabled()) {
                    scoringCycleEndShooterVelocity = shooterVelocity;

                    shooterCycleRunning = false;
                    shooterCycleComplete = true;
                }

                telemetry.addData("Shooter Velocity: ", shooterVelocity);
            } else {
                telemetry.addLine("Press DPAD_UP to start the shooter.");
            }

            robot.setDrivePowers(
                    -gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x
            );

            if (gamepad1.a) {
                robot.setIntakePower(1);
            } else if (gamepad1.left_trigger > .1) {
                robot.setIntakePower(-gamepad1.left_trigger);
            } else {
                robot.setIntakePower(0);
            }

            if (gamepad1.optionsWasPressed()) {
                robot.setDriverAngle(robot.getPose().heading(), AngleUnit.RADIANS);
            }

            packet.put("Shooter Velocity", shooterVelocity);
            packet.put("Target Shooter Velocity", Parameters.SHOOTER_VELOCITY);

            if (shooterCycleComplete) {
                telemetry.addData("Scoring Cycle - Start Shooter Velocity: ", scoringCycleStartShooterVelocity);
                telemetry.addData("Scoring Cycle - End Shooter Velocity: ", scoringCycleEndShooterVelocity);
            }

            robot.update();
            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
