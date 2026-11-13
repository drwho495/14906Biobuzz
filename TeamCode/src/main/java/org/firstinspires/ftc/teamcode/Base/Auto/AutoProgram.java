package org.firstinspires.ftc.teamcode.Base.Auto;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Base.Misc.Alliance;
import org.firstinspires.ftc.teamcode.Base.Parameters;
import org.firstinspires.ftc.teamcode.Base.RobotManager;

import java.util.ArrayList;
import java.util.Arrays;

// This class stores a list of commands and executes them when needed.
public class AutoProgram {
    // This is a list of default initialized commands that can be used in an OpMode.
    private final RobotManager robot;
    private ArrayList<AutoCommand> commands = new ArrayList<>();
    private AutoRunSide runSide;
    private String name = "Unnamed Autonomous Program";

    public AutoProgram(RobotManager robot, String autoName) {
        this.robot = robot;
        this.name = autoName;
    }

    public AutoProgram(RobotManager robot) {
        this.robot = robot;
    }

    public void setName(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public ArrayList<AutoCommand> getCommands() {
        return commands;
    }

    public AutoProgram setRunSide(AutoRunSide runSide) {
        this.runSide = runSide;

        return this;
    }

    public Pose getStartPose() {
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

    public void execute(AutoRunSide runSide) {
        setRunSide(runSide);
        execute();
    }

    public void setCommands(ArrayList<AutoCommand> commands) {
        this.commands = commands;
    }

    public AutoProgram addCommands(AutoCommand... commands) {
        this.commands.addAll(Arrays.asList(commands));

        return this;
    }

    public AutoProgram addCommand(AutoCommand command) {
        this.commands.add(command);

        return this;
    }

    public void execute() {
        AutoCommand currentCommand = null;
        AutoCommand lastCommand = null;
        AutoCommand nextCommand = null;

        Pose startPose = getStartPose();

        for (int i = 0; i < commands.size(); i++) {
            currentCommand = commands.get(i);

            if (i > 0) {
                lastCommand = commands.get(i - 1);
            }

            if ((i + 1) < commands.size()) {
                nextCommand = commands.get(i + 1);
            } else {
                nextCommand = null;
            }

            currentCommand.execute(
                    robot,
                    startPose,
                    runSide,
                    lastCommand,
                    nextCommand
            );
        }
    }
}
