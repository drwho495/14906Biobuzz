package org.firstinspires.ftc.teamcode.Base.Auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Base.Field.Flower;
import org.firstinspires.ftc.teamcode.Base.Field.Hive;
import org.firstinspires.ftc.teamcode.Base.Field.LocalFlowerLocation;
import org.firstinspires.ftc.teamcode.Base.Field.LocalHiveLocation;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.Policy.ShooterControlPolicy;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.Base.Utilities.MathUtils;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class AutoCommandRepository {
    public static class ScoreCommand extends AutoCommand {
        private LocalHiveLocation localHive;
        private long scoreDelay = 0;
        private boolean quickScore = false;

        public ScoreCommand(LocalHiveLocation localHiveLocation, long scoreDelay, boolean quickScore) {
            this.localHive = localHiveLocation;
            this.scoreDelay = scoreDelay;
            this.quickScore = quickScore;
        }

        public ScoreCommand(LocalHiveLocation localHiveLocation, long scoreDelay) {
            this.localHive = localHiveLocation;
            this.scoreDelay = scoreDelay;
        }

        public ScoreCommand(LocalHiveLocation localHiveLocation, boolean quickScore) {
            this.localHive = localHiveLocation;
            this.quickScore = quickScore;
        }

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
            Pose scorePose;

            if (quickScore && localHive == LocalHiveLocation.UPPER) {
                scorePose = robot.getFixedPose(20, 100);
            } else {
                scorePose = hive.getScorePoint();
            }

            double scoreHeading = hive.getScoreHeading(scorePose);

            if (robot.getShooterControlPolicy() == ShooterControlPolicy.MANUAL) {
                robot.setShooterVelocity(robot.getShooterVelocityFromRegression(scorePose, hive));
            }

            if (
                    localHive == LocalHiveLocation.UPPER
                            && !quickScore
                            && robot.getFixedPose(robotPose).y() <= Parameters.FIELD_CENTER.y()
            ) {
//                robot.runPathBlocking(
//                        curve(
//                                robotPose,
//                                robot.getFixedPose(35, 91.495),
//                                robot.getFixedPose(35, 113.545, 0),
//                                scorePose.withY(scorePose.y() - robot.fixParameterDirection(10))
//                        ).linear(robotPose.heading(), scoreHeading)
//                );

                robot.runPathBlocking(
                        line(
                                robotPose,
                                robot.getFixedPose(33, 20)
                        )
                                .linear(robotPose.heading(), scoreHeading)
                                .with(Constants.foresightConfig.maxPathSpeed.at(.8))
                );

                robotPose = robot.getPose();
                robot.addPathTimeout(4000);
                robot.runPathBlocking(
                        curve(
                                robotPose,
                                robot.getFixedPose(35, 95),
                                robot.getFixedPose(35, 112),
                                scorePose.withY(scorePose.y() - robot.fixParameterDirection(20)).withX(scorePose.x() - robot.fixParameterDirection(3))
                        ).constant(scoreHeading)
                );

                robotPose = robot.getPose();

                robot.addPathTimeout(2000);
                robot.runPathBlocking(
                        line(
                                robotPose,
                                scorePose
                        )
                                .constant(scoreHeading)
                                .with(Constants.foresightConfig.maxPathSpeed.at(0.6))
                );
            } else if (
                    localHive == LocalHiveLocation.UPPER
                            && !quickScore
                            && lastCommand != null
                            && lastCommand.getType() == AutoCommandType.FLOWER_INTAKE
            ) {
                robot.runPathBlocking(
                        curve(
                                robotPose,
                                scorePose.withY(scorePose.y() - robot.fixParameterDirection(10)).withX(scorePose.x() + robot.fixParameterDirection(1)),
                                scorePose
                        ).linear(robotPose.heading(), scoreHeading)
                );

                robotPose = robot.getPose();

                robot.runPathBlocking(
                        line(
                                robotPose,
                                scorePose
                        )
                                .constant(scoreHeading)
                                .with(Constants.foresightConfig.maxPathSpeed.at(0.6))
                );
            } else {
                robot.runPathBlocking(
                        line(
                                robotPose,
                                scorePose
                        )
                                .linear(robotPose.heading(), scoreHeading)
                );
            }

            robot.waitForPosition(2000, .5);
            robot.enableAimToHive(hive);

            if (scoreDelay != 0) {
                robot.sleep(scoreDelay);
            }

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
            double wallX = 13;
            double wallY = 12.5;
            double wallRetractY = 14.5;
            double intakeHeading = 265;
            long dwellTime = 250;

            robot.setIntakePower(1);

            Pose robotPose = robot.getPose();
            robot.runPathBlocking(
                    curve(
                            robotPose,
                            robotPose.withX(robotPose.x() - robot.fixParameterDirection(15)).withY(robot.getFixedParameter(17)),
                            robot.getFixedPose(wallX, 21)
                    )
                            .constant(robot.getFixedHeading(intakeHeading))
            );

            for (int i = 0; i < 3; i++) {
                robotPose = robot.getPose();
                robot.addPathTimeout(1000);
                robot.runPathBlocking(
                        line(
                                robotPose,
                                robot.getFixedPose(wallX, wallY + .5)
                        )
                                .constant(robot.getFixedHeading(intakeHeading))
                                .with(Constants.foresightConfig.maxPathSpeed.at(.6)),
                        false
                );

                robot.addPathTimeout(250);
                robot.runPathBlocking(
                        line(
                                robotPose,
                                robot.getFixedPose(wallX, wallY)
                        )
                                .linear(robotPose.heading(), robot.getFixedHeading(270))
                                .with(Constants.foresightConfig.maxPathSpeed.at(.6)),
                        false
                );

                robot.sleep(dwellTime);

                robotPose = robot.getPose();
                robot.addPathTimeout(1000);
                robot.runPathBlocking(
                        line(
                                robotPose,
                                robot.getFixedPose(wallX, wallRetractY)
                        )
                                .constant(robot.getFixedHeading(intakeHeading))
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

            double headingOffset = Math.toRadians(-10);
            double headingPose = flower.getIntakeHeading() + headingOffset;

            Pose robotPose = robot.getPose();
            Pose intakePose = flower.getPoseFromCenter(Parameters.FLOWER_INTAKE_DISTANCE, headingOffset);
            Pose backAwayPose = MathUtils.distanceAlongDirection(intakePose, Parameters.FLOWER_BACK_AWAY_DISTANCE, flower.direction);

//            robot.runPathBlocking(
//                    curve(
//                            robotPose,
//                            robot.getFixedPose(backAwayPose.x() + 15, backAwayPose.y() - 30),
//                            backAwayPose
//                    ).linear(robotPose.heading(), headingPose)
//            );

            if (localFlower == LocalFlowerLocation.WALL && lastCommand.getType() == AutoCommandType.SCORE && ((ScoreCommand) lastCommand).localHive == LocalHiveLocation.UPPER) {
                robot.runPathBlocking(
                        curve(
                                robotPose,
                                new Pose(robotPose.x(), robotPose.y() - robot.fixParameterDirection(30)),
                                new Pose(robot.getFixedParameter(10), robotPose.y() - robot.fixParameterDirection(20)),
                                backAwayPose
                        ).linear(robotPose.heading(), headingPose)
                );
            } else {
                robot.runPathBlocking(
                        curve(
                                robotPose,
                                backAwayPose.withY(backAwayPose.y() - robot.fixParameterDirection(10)),
                                backAwayPose
                        ).linear(robotPose.heading(), headingPose)
                );
            }

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
                            .with(Constants.foresightConfig.maxPathSpeed.at(.4))
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

            double parkHeading;

            if (robotPose.heading() > 0 && robotPose.heading() < Math.PI) {
                parkHeading = robot.getFixedHeading(90);
            } else {
                parkHeading = robot.getFixedHeading(270);
                parkX = 17; // TODO: fix localization issue
            }

            if (robot.getFixedPose(robotPose).y() <= Parameters.FIELD_CENTER.y()) {
                robot.runPathBlocking(
                        curve(
                                robotPose,
                                new Pose(robotPose.x() - robot.fixParameterDirection(20), robot.getFixedParameter(30)),
                                robot.getFixedPose(parkX, 96)
                        )
                                .linear(robotPose.heading(), parkHeading)
                                .with(Constants.foresightConfig.maxDecelerationScale.at(decelerationScale))
                );
            } else {
                robot.runPathBlocking(
                        curve(
                                robotPose,
                                new Pose(robotPose.x() - robot.fixParameterDirection(15), robot.getFixedParameter(110)),
                                robot.getFixedPose(parkX, 120)
                        )
                                .linear(robotPose.heading(), parkHeading)
                                .with(Constants.foresightConfig.maxDecelerationScale.at(decelerationScale))
                );
            }
        }
    }
}
