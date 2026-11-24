package org.firstinspires.ftc.teamcode.Base.Auto;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.Base.RobotManager;

public abstract class AutoCommand {
    public abstract AutoCommandType getType();
    public abstract String getLabel();
    public abstract void execute(RobotManager robot, Pose startPose, AutoRunSide runSide, AutoCommand lastCommand, AutoCommand nextCommand);
}
