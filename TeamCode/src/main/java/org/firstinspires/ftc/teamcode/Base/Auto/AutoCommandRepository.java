package org.firstinspires.ftc.teamcode.Base.Auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Base.Field.Flower;
import org.firstinspires.ftc.teamcode.Base.Field.Hive;
import org.firstinspires.ftc.teamcode.Base.Field.LocalFlowerLocation;
import org.firstinspires.ftc.teamcode.Base.Field.LocalHiveLocation;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class AutoCommandRepository {
    public static class ScoreCommand extends AutoCommand {
        private LocalHiveLocation localHive;

        public ScoreCommand(LocalHiveLocation localHiveLocation) {
            this.localHive = localHiveLocation;
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
            Hive hive = robot.getHive(localHive);

            if (!robot.isShooterPoweredOn()) {
                robot.powerOnShooter();
            }

            robot.setIntakePower(0);

            Pose robotPose = robot.getPose();
            Pose scorePoint = hive.getScorePoint();
            double scoreHeading = hive.getScoreHeading(scorePoint);

            robot.updateShooterVelocityWithRegression(scorePoint, hive);

            if (
                    localHive == LocalHiveLocation.UPPER
                            && robot.getFixedPose(robotPose).y() <= Parameters.FIELD_CENTER.y()
            ) {
//                robot.runPathBlocking(
//                        curve(
//                                robotPose,
//                                robot.getFixedPose(35, 91.495),
//                                robot.getFixedPose(35, 113.545, 0),
//                                scorePoint.withY(scorePoint.y() - robot.fixParameterDirection(10))
//                        ).linear(robotPose.heading(), scoreHeading)
//                );

                robot.runPathBlocking(
                        line(
                                robotPose,
                                robot.getFixedPose(33, 20)
                        ).linear(robotPose.heading(), scoreHeading)
                );

                robotPose = robot.getPose();
                robot.addPathTimeout(4000);
                robot.runPathBlocking(
                        curve(
                                robotPose,
                                robot.getFixedPose(35, 91.495),
                                robot.getFixedPose(35, 106),
                                scorePoint.withY(scorePoint.y() - robot.fixParameterDirection(20)).withX(scorePoint.x() - robot.fixParameterDirection(3))
                        ).constant(scoreHeading)
                );

                robotPose = robot.getPose();

                robot.addPathTimeout(2000);
                robot.runPathBlocking(
                        line(
                                robotPose,
                                scorePoint
                        )
                                .constant(scoreHeading)
                                .with(Constants.foresightConfig.maxPathSpeed.at(0.6))
                );
            } else if (
                    localHive == LocalHiveLocation.UPPER
                            && lastCommand != null
                            && lastCommand.getType() == AutoCommandType.FLOWER_INTAKE
            ) {
                robot.runPathBlocking(
                        curve(
                                robotPose,
                                scorePoint.withY(scorePoint.y() - robot.fixParameterDirection(20)).withX(scorePoint.x() + robot.fixParameterDirection(1)),
                                scorePoint
                        ).linear(robotPose.heading(), scoreHeading)
                );

                robotPose = robot.getPose();

                robot.runPathBlocking(
                        line(
                                robotPose,
                                scorePoint
                        )
                                .constant(scoreHeading)
                                .with(Constants.foresightConfig.maxPathSpeed.at(0.6))
                );
            } else {
                robot.runPathBlocking(
                        line(
                                robotPose,
                                scorePoint
                        )
                                .linear(robotPose.heading(), scoreHeading)
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
                            robotPose.withX(robotPose.x() - robot.fixParameterDirection(15)).withY(robot.getFixedParameter(17)),
                            robot.getFixedPose(wallX, 21)
                    )
                            .constant(robot.getFixedHeading(270))
            );

            for (int i = 0; i < 3; i++) {
                robotPose = robot.getPose();
                robot.addPathTimeout(1000);
                robot.runPathBlocking(
                        line(
                                robotPose,
                                robot.getFixedPose(wallX, wallY)
                        )
                                .constant(robot.getFixedHeading(270))
                                .with(Constants.foresightConfig.maxPathSpeed.at(.6))
                );

                robotPose = robot.getPose();
                robot.runPathBlocking(
                        line(
                                robotPose,
                                robot.getFixedPose(wallX, wallRetractY)
                        )
                                .constant(robot.getFixedHeading(270))
                                .with(Constants.foresightConfig.maxPathSpeed.at(.6))
                );
            }
        }
    }

    public static class FlowerIntakeCommand extends AutoCommand {
        private LocalFlowerLocation localFlower = null;

        public FlowerIntakeCommand(LocalFlowerLocation localFlower) {
            this.localFlower = localFlower;
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
            Flower flower = robot.getFlower(localFlower);

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
                            backAwayPose.withY(backAwayPose.y() - robot.fixParameterDirection(10)),
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
            robot.runPathBlocking(
                    line(
                            robotPose,
                            intakePose
                    )
                            .constant(headingPose)
                            .with(Constants.foresightConfig.maxPathSpeed.at(.5))
                    , false
            );

            robot.sleep(1000);

            robotPose = robot.getPose();
            robot.runPathBlocking(
                    line(
                            robotPose,
                            backAwayPose
                    )
                            .constant(headingPose)
            );
            robot.raiseFlowerMechanism();
            robot.sleep(350);

            robotPose = robot.getPose();
            robot.addPathTimeout(1000);
            robot.runPathBlocking(
                    line(
                            robotPose,
                            intakePose
                    )
                            .constant(headingPose)
                            .with(Constants.foresightConfig.maxPathSpeed.at(.5))
                    , false
            );
            robot.sleep(500);
            robot.setIntakePower(0);
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
            robot.powerOffShooter();

            Pose robotPose = robot.getPose();

            double parkX = 13;
            double decelerationScale = .0001;

            if (robot.getFixedPose(robotPose).y() <= Parameters.FIELD_CENTER.y()) {
                robot.runPathBlocking(
                        curve(
                                robotPose,
                                robot.getFixedPose(robotPose.x() + 15, 80),
                                robot.getFixedPose(parkX, 90)
                        )
                                .linear(robotPose.heading(), robot.getFixedHeading(90))
                                .with(Constants.foresightConfig.maxDecelerationScale.at(decelerationScale))
                );
            } else {
                robot.runPathBlocking(
                        curve(
                                robotPose,
                                robot.getFixedPose(robotPose.x() - 15, 110),
                                robot.getFixedPose(parkX, 120)
                        )
                                .linear(robotPose.heading(), robot.getFixedHeading(90))
                                .with(Constants.foresightConfig.maxDecelerationScale.at(decelerationScale))
                );
            }

            robot.sleep(5000);
        }
    }
}
