package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Base.Auto.AutoCommandRepository;
import org.firstinspires.ftc.teamcode.Base.Auto.AutoProgram;
import org.firstinspires.ftc.teamcode.Base.RobotManager;

@Autonomous(name = "Autonomous V1", group = "0")
public class AutoV1 extends LinearOpMode {
    private RobotManager robot = new RobotManager();
    private AutoProgram configuredProgram;

    @Override
    public void runOpMode() {
        robot.initialize(this);

        configuredProgram = new AutoProgram(robot);

        while (opModeInInit()) {
            // TODO: Add configuration UI
        }

        configuredProgram.addCommands(
                new AutoCommandRepository.ScoreCommand(robot.getLowerHive()),
                new AutoCommandRepository.WallIntakeCommand(),
                new AutoCommandRepository.ScoreCommand(robot.getUpperHive()),
                new AutoCommandRepository.FlowerIntakeCommand(robot.getWallFlower())
        );

        waitForStart();

        configuredProgram.execute();
    }
}
