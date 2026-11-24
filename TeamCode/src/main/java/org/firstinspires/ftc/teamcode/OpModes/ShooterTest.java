package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.Utilities.MathUtils;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;

@Utility(name = "Shooter Test", description = "Provides the user with tools and data to reliably test the shooter.")
public class ShooterTest extends LinearOpMode {
    RobotManager robot = new RobotManager();
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(this);

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        TelemetryPacket packet = new TelemetryPacket();

        boolean shooterCycleRunning = false;
        boolean shooterCycleComplete = false;
        double scoringCycleStartShooterVelocity = 0;
        double scoringCycleLowShooterVelocity = 0;
        double shooterVelocity;

        while (opModeIsActive()) {
            shooterVelocity = robot.getShooterVelocity();

            packet.clearLines();

            if (gamepad1.dpadUpWasPressed()) {
                robot.toggleShooter();
            }

            if (robot.isShooterPoweredOn()) {
                if (gamepad1.rightBumperWasPressed() && !shooterCycleRunning) {
                    robot.startScoringCycle();

                    scoringCycleLowShooterVelocity = shooterVelocity;
                    scoringCycleStartShooterVelocity = shooterVelocity;
                    shooterCycleRunning = true;
                    shooterCycleComplete = false;
                }

                if (!robot.isAutoTransferPollenDuringScoreCycleEnabled() || !robot.isTransferCycleTimerEnabled() && shooterCycleRunning && gamepad1.rightBumperWasReleased()) {
                    robot.stopScoringCycle();
                }

                if (shooterCycleRunning) {
                    if (shooterVelocity < scoringCycleLowShooterVelocity) {
                        scoringCycleLowShooterVelocity = shooterVelocity;
                    }

                    if (!robot.isScoringCycleEnabled()) {
                        shooterCycleRunning = false;
                        shooterCycleComplete = true;
                    }
                }

                telemetry.addData("Shooter Velocity: ", MathUtils.convertToRPM(shooterVelocity));
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

            packet.put("Shooter Velocity", MathUtils.convertToRPM(shooterVelocity));
            packet.put("Target Shooter Velocity", MathUtils.convertToRPM(Parameters.SHOOTER_VELOCITY));

            if (shooterCycleComplete) {
                telemetry.addData("Scoring Cycle - Shooter Velocity Drop (RPM): ", MathUtils.convertToRPM(scoringCycleStartShooterVelocity) - MathUtils.convertToRPM(scoringCycleLowShooterVelocity));
            }

            robot.update();
            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
