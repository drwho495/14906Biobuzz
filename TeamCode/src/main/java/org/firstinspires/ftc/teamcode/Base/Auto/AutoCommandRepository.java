package org.firstinspires.ftc.teamcode.Base.Auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Base.Misc.Flower;
import org.firstinspires.ftc.teamcode.Base.Misc.Hive;
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

            if (
                    hive == robot.getUpperHive()
                            && robotPose.x() <= robot.getFixedX(45)
                            && robotPose.y() <= Parameters.FIELD_CENTER.y()
            ) {
                robot.runPathBlocking(
                        curve(
                                robotPose,
                                robot.getFixedPose(66, 8),
                                robot.getFixedPose(58, 47),
                                robot.getFixedPose(58, 15),
                                robot.getFixedPose(59, 12.5)
                        ).linear(robotPose.heading(), robot.getFixedHeading(270))
                );
            }
            if (
                    hive == robot.getUpperHive()
                            && robotPose.x() <= robot.getFixedX(45)
                            && robotPose.y() >= Parameters.FIELD_CENTER.y()
            ) {
                robot.runPathBlocking(
                        curve(
                                robotPose,
                                robot.getFixedPose(66, 8),
                                robot.getFixedPose(58, 47),
                                robot.getFixedPose(58, 15),
                                robot.getFixedPose(59, 12.5)
                        ).linear(robotPose.heading(), robot.getFixedHeading(270))
                );
            } else {
                robot.runPathBlocking(
                        line(
                                robotPose,
                                robot.getFixedPose(59, 12.5)
                        ).linear(robotPose.heading(), robot.getFixedHeading(270))
                );
            }

            robot.waitForShooter(2000);
            robot.startScoringCycle(false, false);

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

            robot.runPathBlocking(
                    curve(
                            robotPose,
                            new Pose(backAwayPose.x() + 15, backAwayPose.y() - 30),
                            backAwayPose
                    ).linear(robotPose.heading(), headingPose)
            );

            if (!robot.isFlowerMechanismDown()) {
                robot.lowerFlowerMechanism();
                robot.sleep(350);
            }

            robotPose = robot.getPose();
            robot.setIntakePower(1);
            robot.runPathBlocking(
                    line(
                            robotPose,
                            intakePose
                    ).constant(headingPose)
            );

            robot.sleep(1000);

            robotPose = robot.getPose();
            robot.runPathBlocking(
                    line(
                            robotPose,
                            backAwayPose
                    ).constant(headingPose)
            );
            robot.raiseFlowerMechanism();
            robot.sleep(350);

            robotPose = robot.getPose();
            robot.runPathBlocking(
                    line(
                            robotPose,
                            intakePose
                    ).constant(headingPose)
            );
            robot.sleep(500);
            robot.setIntakePower(0);
        }
    }
}
