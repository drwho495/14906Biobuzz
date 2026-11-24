package org.firstinspires.ftc.teamcode.Base.Auto;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.Base.RobotManager;
import java.util.ArrayList;
import java.util.Arrays;

public class AutoProgram {
    private final RobotManager robot;
    private ArrayList<AutoCommand> commands = new ArrayList<>();
    private AutoRunSide runSide;
    private String name = "N/A";

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

    public void execute(Pose startPose) {
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

    public AutoRunSide getRunSide() {
        return runSide;
    }
}
