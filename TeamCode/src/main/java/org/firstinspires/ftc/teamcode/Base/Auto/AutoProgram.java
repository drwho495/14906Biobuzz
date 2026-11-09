package org.firstinspires.ftc.teamcode.Base.Auto;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Base.RobotManager;

import java.util.ArrayList;
import java.util.Arrays;

// This class stores a list of commands and executes them when needed.
public class AutoProgram {
    // This is a list of default initialized commands that can be used in an OpMode.
    private final RobotManager robot;
    private ArrayList<AutoCommand> commands = new ArrayList<>();
    private AutoRunSide runSide;
    private Pose startPose;
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

    public void setRunSide(AutoRunSide runSide) {
        this.runSide = runSide;
    }

    public void setStartPose(Pose startPose) {
        this.startPose = startPose;
    }

    public void execute(AutoRunSide runSide, Pose startPose) {
        setRunSide(runSide);
        setStartPose(startPose);
        execute();
    }

    public void setCommands(ArrayList<AutoCommand> commands) {
        this.commands = commands;
    }

    public void addCommands(AutoCommand... commands) {
        this.commands.addAll(Arrays.asList(commands));
    }

    public void addCommand(AutoCommand command) {
        this.commands.add(command);
    }

    public void execute() {
        AutoCommand currentCommand = null;
        AutoCommand lastCommand = null;
        AutoCommand nextCommand = null;

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
