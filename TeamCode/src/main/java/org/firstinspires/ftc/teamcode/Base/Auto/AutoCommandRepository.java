package org.firstinspires.ftc.teamcode.Base.Auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Base.Field.Flower;
import org.firstinspires.ftc.teamcode.Base.Field.Hive;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;

public class AutoCommandRepository {
    public static class ScoreCommand extends AutoCommand {
        private Hive hive;

        public ScoreCommand(Hive hive) {
            this.hive = hive;
        }

        @Override
        public AutoCommandType getType() {
            return AutoCommandType.SCORE;
        }

        @Override
        public String getLabel() {
            return "Score Elements";
        }

        @Override
        public void execute(RobotManager robot, Pose startPose, AutoRunSide runSide, AutoCommand lastCommand, AutoCommand nextCommand) {
            if (!robot.isShooterPoweredOn()) {
                robot.powerOnShooter();
            }

            robot.setAutoSpeed(1);
            robot.setIntakePower(0);

            Pose robotPose = robot.getPose();
            Pose scorePoint = hive.getScorePoint();
            double scoreHeading = hive.getScoreHeading(scorePoint);

            if (
                    hive == robot.getUpperHive()
                            && robotPose.y() <= Parameters.FIELD_CENTER.y()
            ) {
                robot.runPathBlocking(
                        curve(
                                robotPose,
                                robot.getFixedPose(35, 91.495),
                                robot.getFixedPose(35, 113.545, 0),
                                scorePoint.withY(robot.getFixedX(scorePoint.y() - 10))
                                ).linear(robotPose.heading(), scoreHeading)
                );

                robotPose = robot.getPose();

                robot.runPathBlocking(
                        line(
                                robotPose,
                                scorePoint
                        ).constant(scoreHeading)
                );
            } else if (
                    hive == robot.getUpperHive()
                            && lastCommand != null
                            && lastCommand.getType() == AutoCommandType.FLOWER_INTAKE
            ) {
                robot.runPathBlocking(
                        curve(
                                robotPose,
                                scorePoint.withY(robot.getFixedX(scorePoint.y() - 20)),
                                scorePoint
                        ).linear(robotPose.heading(), scoreHeading)
                );

                robotPose = robot.getPose();

                robot.runPathBlocking(
                        line(
                                robotPose,
                                scorePoint
                        ).constant(scoreHeading)
                );
            } else {
                robot.runPathBlocking(
                        line(
                                robotPose,
                                scorePoint
                        ).linear(robotPose.heading(), scoreHeading)
                );
            }

            robot.waitForPosition(2000, .5);
            robot.enableAimToHive(hive);

            robot.waitForShooter(2000);
            robot.startScoringCycle(false, true);

            robot.sleep(Parameters.SHOOTER_TRANSFER_TIME + 200);
            robot.stopScoringCycle();
        }
    }

    public static class WallIntakeCommand extends AutoCommand {
        @Override
        public AutoCommandType getType() {
            return AutoCommandType.WALL_INTAKE;
        }

        @Override
        public String getLabel() {
            return "Intake from Wall";
        }

        @Override
        public void execute(RobotManager robot, Pose startPose, AutoRunSide runSide, AutoCommand lastCommand, AutoCommand nextCommand) {
            double wallX = 8.5;
            double wallY = 14;
            double wallRetractY = 18.5;

            robot.setIntakePower(1);

            Pose robotPose = robot.getPose();
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
        }
    }

    public static class FlowerIntakeCommand extends AutoCommand {
        private Flower flower = null;

        public FlowerIntakeCommand(Flower flower) {
            this.flower = flower;
        }

        @Override
        public AutoCommandType getType() {
            return AutoCommandType.FLOWER_INTAKE;
        }

        @Override
        public String getLabel() {
            return "Intake from Flower";
        }

        @Override
        public void execute(RobotManager robot, Pose startPose, AutoRunSide runSide, AutoCommand lastCommand, AutoCommand nextCommand) {
            robot.setAutoSpeed(1);
            Pose robotPose = robot.getPose();
            Pose backAwayPose = flower.getPoseFromCenter(Parameters.FLOWER_BACK_AWAY_DISTANCE);
            Pose intakePose = flower.getPoseFromCenter(Parameters.FLOWER_INTAKE_DISTANCE);
            double headingPose = flower.getIntakeHeading();

//            robot.runPathBlocking(
//                    curve(
//                            robotPose,
//                            robot.getFixedPose(backAwayPose.x() + 15, backAwayPose.y() - 30),
//                            backAwayPose
//                    ).linear(robotPose.heading(), headingPose)
//            );

            robot.runPathBlocking(
                    curve(
                            robotPose,
                            robot.getFixedPose(backAwayPose.x(), backAwayPose.y() - 10),
                            backAwayPose
                    ).linear(robotPose.heading(), headingPose)
            );

            if (!robot.isFlowerMechanismDown()) {
                robot.lowerFlowerMechanism();
                robot.sleep(350);
            }

            robotPose = robot.getPose();
            robot.setIntakePower(1);
            robot.addPathTimeout(1000);
            robot.setAutoSpeed(.5);
            robot.runPathBlocking(
                    line(
                            robotPose,
                            intakePose
                    ).constant(headingPose)
            );

            robot.sleep(1000);

            robotPose = robot.getPose();
            robot.setAutoSpeed(1);
            robot.runPathBlocking(
                    line(
                            robotPose,
                            backAwayPose
                    ).constant(headingPose)
            );
            robot.raiseFlowerMechanism();
            robot.sleep(350);

            robotPose = robot.getPose();
            robot.addPathTimeout(1000);
            robot.setAutoSpeed(.5);
            robot.runPathBlocking(
                    line(
                            robotPose,
                            intakePose
                    ).constant(headingPose)
            );
            robot.sleep(500);
            robot.setIntakePower(0);
            robot.setAutoSpeed(1);
        }
    }

    public static class ParkCommand extends AutoCommand {
        @Override
        public AutoCommandType getType() {
            return AutoCommandType.PARK;
        }

        @Override
        public String getLabel() {
            return "Park";
        }

        @Override
        public void execute(RobotManager robot, Pose startPose, AutoRunSide runSide, AutoCommand lastCommand, AutoCommand nextCommand) {
            robot.setAutoSpeed(1);
            robot.powerOffShooter();

            Pose robotPose = robot.getPose();

            double parkX = 15;

            if (robotPose.y() <= Parameters.FIELD_CENTER.y()) {
                robot.runPathBlocking(
                        curve(
                                robotPose,
                                robot.getFixedPose(robotPose.x() + 15, 80),
                                robot.getFixedPose(parkX, 90)
                        ).linear(robotPose.heading(), robot.getFixedHeading(90))
                );
            } else {
                robot.runPathBlocking(
                        curve(
                                robotPose,
                                robot.getFixedPose(robotPose.x() - 15, 110),
                                robot.getFixedPose(parkX, 120)
                        ).linear(robotPose.heading(), robot.getFixedHeading(90))
                );
            }

            robot.sleep(5000);
        }
    }
}
