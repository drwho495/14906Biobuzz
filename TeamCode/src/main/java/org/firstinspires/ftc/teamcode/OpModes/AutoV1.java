package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Base.Auto.AutoCommand;
import org.firstinspires.ftc.teamcode.Base.Auto.AutoCommandRepository;
import org.firstinspires.ftc.teamcode.Base.Auto.AutoProgram;
import org.firstinspires.ftc.teamcode.Base.Auto.AutoRunSide;
import org.firstinspires.ftc.teamcode.Base.Field.LocalFlowerLocation;
import org.firstinspires.ftc.teamcode.Base.Field.LocalHiveLocation;
import org.firstinspires.ftc.teamcode.Base.Misc.Alliance;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Auto V1", group = "0")
public class AutoV1 extends LinearOpMode {
    private RobotManager robot = new RobotManager();
    private List<AutoProgram> allPrograms = new ArrayList<>();
    private AutoProgram selectedProgram;
    private int selectedProgramIndex;
    private boolean controlsLocked = false;

    @Override
    public void runOpMode() {
        robot.initialize(this);
        robot.disableSubsystems();

        Constants.foresightConfig.maxPathSpeed.set(1.0);
        Constants.foresightConfig.maxDecelerationScale.set(ForesightConfig.Constraint.NONE);

        registerPrograms();
        int numberOfPrograms = allPrograms.size();

        selectedProgramIndex = 0;
        selectedProgram = allPrograms.get(selectedProgramIndex);

        robot.calibrateIMU();
        robot.sleep(500);
        robot.setPose(getStartPose());

        while (opModeInInit()) {
            if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()) {
                controlsLocked = !controlsLocked;
            }

            if (!controlsLocked) {
                if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
                    selectedProgramIndex++;

                    if (selectedProgramIndex == numberOfPrograms) {
                        selectedProgramIndex = 0;
                    }

                    selectedProgram = allPrograms.get(selectedProgramIndex);
                    robot.setPose(getStartPose());
                } else if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
                    selectedProgramIndex--;

                    if (selectedProgramIndex < 0) {
                        selectedProgramIndex = numberOfPrograms - 1;
                    }

                    selectedProgram = allPrograms.get(selectedProgramIndex);
                    robot.setPose(getStartPose());
                }

                if (gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed()) {
                    robot.switchAlliance();
                    robot.setPose(getStartPose());
                }
            }

            if (controlsLocked) {
//                robot.setStatusLight(LEDColor.GREEN);

                telemetry.addLine("Controls are locked.");
                telemetry.addLine("Press 'Right Bumper' on any gamepad to unlock controls.");
            } else {
//                robot.setStatusLight(LEDColor.RED);

                telemetry.addLine("Controls are unlocked.");
                telemetry.addLine("Press 'Right Bumper' on any gamepad to lock the controls.");
            }

            Pose robotPose = robot.getPose();

            telemetry.addLine();
            telemetry.addData("Alliance: ", robot.getAlliance());
            telemetry.addData("X: ", robotPose.x());
            telemetry.addData("Y: ", robotPose.y());
            telemetry.addData("Heading: ", robotPose.heading());
            telemetry.addLine();
            telemetry.addData("Selected Program: ", selectedProgram.getName());
            telemetry.addLine();
            telemetry.addLine("Commands:");

            for (final AutoCommand command : selectedProgram.getCommands()) {
                telemetry.addLine(command.getLabel());
            }

            telemetry.update();

            robot.update();
        }

        waitForStart();

        robot.enableSubsystems();

        selectedProgram.execute(getStartPose());
    }

    private Pose getStartPose() {
        AutoRunSide runSide = selectedProgram.getRunSide();

        if (runSide == AutoRunSide.GOAL) {
            if (robot.getAlliance() == Alliance.RED) {
                return Parameters.GOAL_RED_AUTO_START;
            } else {
                return Parameters.GOAL_BLUE_AUTO_START;
            }
        } else if (runSide == AutoRunSide.FLOWER) {
            if (robot.getAlliance() == Alliance.RED) {
                return Parameters.FLOWER_RED_AUTO_START;
            } else {
                return Parameters.FLOWER_BLUE_AUTO_START;
            }
        }

        return Pose.zero();
    }

    private void registerPrograms() {
        allPrograms.add(
                new AutoProgram(robot, "Goal Side: 2 Tip")
                        .setRunSide(AutoRunSide.GOAL)
                        .addCommand(new AutoCommandRepository.ScoreCommand(LocalHiveLocation.LOWER))
                        .addCommand(new AutoCommandRepository.WallIntakeCommand())
                        .addCommand(new AutoCommandRepository.ScoreCommand(LocalHiveLocation.UPPER))
                        .addCommand(new AutoCommandRepository.FlowerIntakeCommand(LocalFlowerLocation.UPPER))
                        .addCommand(new AutoCommandRepository.ScoreCommand(LocalHiveLocation.UPPER))
                        .addCommand(new AutoCommandRepository.ParkCommand())
        );

        allPrograms.add(
                new AutoProgram(robot, "Goal Side: 1 Tip Safe")
                        .setRunSide(AutoRunSide.GOAL)
                        .addCommand(new AutoCommandRepository.ScoreCommand(LocalHiveLocation.LOWER))
                        .addCommand(new AutoCommandRepository.WallIntakeCommand())
                        .addCommand(new AutoCommandRepository.ScoreCommand(LocalHiveLocation.LOWER))
                        .addCommand(new AutoCommandRepository.FlowerIntakeCommand(LocalFlowerLocation.WALL))
                        .addCommand(new AutoCommandRepository.ParkCommand())
        );
    }
}
