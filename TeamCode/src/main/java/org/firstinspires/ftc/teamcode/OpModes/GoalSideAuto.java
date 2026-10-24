package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Base.Misc.Flower;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;

import static com.pedropathing.api.Paths.*;

@Autonomous(name = "Goal Side Auto V1", group = "0", preselectTeleOp = "0: Main Teleop")
public class GoalSideAuto extends LinearOpMode {
    private RobotManager robot = new RobotManager();
    private Pose robotPose;

    private void score() {
        robot.setAutoSpeed(1);
        robot.setIntakePower(0);

        Pose robotPose = robot.getPose();
        robot.runPathBlocking(
                line(
                        robotPose,
                        new Pose(59, 12.5)
                ).linear(robotPose.heading(), Math.toRadians(270))
        );

        robot.waitForShooter(2000);
        robot.startScoringCycle(false, false);

        robot.sleep(Parameters.SHOOTER_TRANSFER_TIME + 200);
        robot.stopScoringCycle();
    }

    private void park() {
        robot.setAutoSpeed(1);
        robot.powerOffShooter();

        robotPose = robot.getPose();

        if (robotPose.x() < 50) {
            robot.runPathBlocking(
                    curve(
                            robotPose,
                            new Pose(robotPose.x() + 15, 80),
                            new Pose(15, 100)
                    ).constant(robotPose.heading())
            );
        }
    }

    private void flowerIntake(Flower flower) {
        robot.setAutoSpeed(1);
        robotPose = robot.getPose();

        if (robotPose.x() > 30 && robotPose.y() < 70) {
            robot.runPathBlocking(
                    curve(
                            robotPose,
                            new Pose(flower.robotBackAwayPosition.x() + 15, flower.robotBackAwayPosition.y() - 30),
                            flower.robotBackAwayPosition
                    ).linear(robotPose.heading(), flower.robotHeading)
            );
        }

        if (!robot.isFlowerMechanismDown()) {
            robot.lowerFlowerMechanism();
            robot.sleep(350);
        }

        robotPose = robot.getPose();
        robot.setIntakePower(1);
        robot.runPathBlocking(
                line(
                        robotPose,
                        flower.robotIntakePosition
                ).constant(flower.robotHeading)
        );

        robot.sleep(1000);

        robotPose = robot.getPose();
        robot.runPathBlocking(
                line(
                        robotPose,
                        flower.robotBackAwayPosition
                ).constant(flower.robotHeading)
        );
        robot.raiseFlowerMechanism();
        robot.sleep(350);

        robotPose = robot.getPose();
        robot.runPathBlocking(
                line(
                        robotPose,
                        flower.robotIntakePosition
                ).constant(flower.robotHeading)
        );
        robot.sleep(500);
        robot.setIntakePower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(this);
        robot.enableAutoTransferPollenDuringScoreCycle();
        robot.enableTransferEcoMode();

        robot.setPose(Parameters.GOAL_RED_AUTO_START);
        robot.update();

        ElapsedTime autoTimer = new ElapsedTime();

        while (opModeInInit()) {
            Pose robotPose = robot.getPose();

            telemetry.addData("Alliance: ", robot.getAlliance());
            telemetry.addData("Robot X: ", robotPose.x());
            telemetry.addData("Robot Y: ", robotPose.y());
            telemetry.addData("Robot Heading: ", Math.toDegrees(robotPose.heading()));
            telemetry.update();

            robot.update();
        }

        waitForStart();
        robot.powerOnShooter();

        score();
        robot.setIntakePower(1);

        double wallX = 8;
        double wallY = 13;
        double wallRetractY = 20;

        robotPose = robot.getPose();
        robot.runPathBlocking(
                curve(
                        robotPose,
                        new Pose(robotPose.x() - 15, 17),
                        new Pose(wallX, 21)
                ).constant(Math.toRadians(270))
        );

        robot.setAutoSpeed(.6);

        for (int i = 0; i < 3; i++) {
            robotPose = robot.getPose();
            robot.addPathTimeout(1000);
            robot.runPathBlocking(
                    line(
                            robotPose,
                            new Pose(wallX, wallY)
                    ).constant(Math.toRadians(270))
            );

            robotPose = robot.getPose();
            robot.runPathBlocking(
                    line(
                            robotPose,
                            new Pose(wallX, wallRetractY)
                    ).constant(Math.toRadians(270))
            );
        }

        score();
        flowerIntake(Parameters.LEFT_FLOWER);
        park();

        robot.setAutoSpeed(1);

        robot.sleep(autoTimer, 29_000);
        robot.powerOffShooter();
        robot.stopFollowing();
        robot.update();
    }
}
