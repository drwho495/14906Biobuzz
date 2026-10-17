package org.firstinspires.ftc.teamcode.Base;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.Misc.Alliance;
import org.firstinspires.ftc.teamcode.Base.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Base.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class RobotManager {
    private HardwareTable hardwareTable;
    private IntakeSubsystem intake = new IntakeSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();
    private Follower follower;
    private ElapsedTime cacheResetTimer = new ElapsedTime();
    private ElapsedTime transferCycleTimer = new ElapsedTime();
    private boolean isTransfering = false;
    private double manualIntakePower = 0;
    private boolean scoreElementsCycleEnabled = false;
    private boolean autoTransferPollenDuringScoreCycle = true;
    private boolean teleopDriveActive = false;
    private boolean headingLockEnabled = false;
    private boolean scoreWaitForShooterVelocity = false;
    private boolean scoreWaitForHeadingLock = false;
    private boolean transferCycleTimerEnabled = false;
    private boolean isTransferOvercurrent = false;
    private boolean stopTransferIfOvercurrent = true;
    private ElapsedTime transferOvercurrentTimer = new ElapsedTime();
    private double headingLockAngle = 0;
    private double driverAngleOffset = 0;
    private Alliance robotAlliance = Alliance.RED;
    private List<LynxModule> lynxModules;

    public void initialize(HardwareMap hardwareMap) {
        hardwareTable = new HardwareTable(hardwareMap);
        follower = Constants.create(hardwareMap);

        setAlliance(
                StaticStorage.storedAlliance,
                false
        );

        shooter.initialize(hardwareMap, hardwareTable);
        intake.initialize(hardwareMap, hardwareTable);

        lynxModules = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : lynxModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void update() {
        if (scoreElementsCycleEnabled) {
            if (canShoot() && !isTransfering) {
                isTransfering = true;

                transferCycleTimer.reset();
                intake.setTransferRollersPower(Parameters.SCORE_TRANSFER_SPEED);
                shooter.openGate();
            }

            if (isTransfering) {
                if (autoTransferPollenDuringScoreCycle) {
                    if (transferCycleTimerEnabled && transferCycleTimer.time(TimeUnit.MILLISECONDS) >= Parameters.SHOOTER_TRANSFER_TIME) {
                        stopScoringCycle();
                    }
                } else {
                    intake.setTransferRollersPower(manualIntakePower);
                }
            }
        } else {
            intake.setFrontRollersPower(manualIntakePower);

            if (manualIntakePower < 0) {
                isTransferOvercurrent = false;
            }

            if (isTransferOvercurrent) {
                intake.setTransferRollersPower(0);
            } else {
                intake.setTransferRollersPower(manualIntakePower);

                if (stopTransferIfOvercurrent) {
                    if (intake.isTransferOvercurrent()) {
                        if (transferOvercurrentTimer.time(TimeUnit.MILLISECONDS) >= 750) {
                            isTransferOvercurrent = true;
                        }
                    } else {
                        transferOvercurrentTimer.reset();
                    }
                }
            }
        }

        shooter.setTargetVelocity(Parameters.SHOOTER_VELOCITY);

        if (cacheResetTimer.time(TimeUnit.MILLISECONDS) >= 5) {
            for (LynxModule module : lynxModules) {
                module.clearBulkCache();
            }

            cacheResetTimer.reset();
        }

        follower.update();
        shooter.update();
        intake.update();
    }

    private void setAlliance(Alliance alliance, boolean cacheAllianceValue) {
        robotAlliance = alliance;

        if (cacheAllianceValue) {
            StaticStorage.storedAlliance = alliance;
        }
    }

    public boolean canShoot() {
        if (scoreWaitForShooterVelocity && !shooter.ready()) {
            return false;
        }

        if (scoreWaitForHeadingLock && headingLockEnabled && getHeadingLockError() > Parameters.ACCEPTABLE_HEADING_LOCK_ERROR) {
            return false;
        }

        return true;
    }

    public void enableTransferEcoMode() {
        stopTransferIfOvercurrent = true;
        transferOvercurrentTimer.reset();
        isTransferOvercurrent = false;
    }

    public void disableTransferEcoMode() {
        stopTransferIfOvercurrent = false;
    }

    public void resetTransferOvercurrentFlag() {
        isTransferOvercurrent = false;
        transferOvercurrentTimer.reset();
    }

    public void setAlliance(Alliance alliance) {
        setAlliance(alliance, true);
    }

    public Alliance getAlliance() {
        return robotAlliance;
    }

    public void selectOtherAlliance() {
        if (robotAlliance == Alliance.RED) {
            setAlliance(Alliance.BLUE);
        } else if (robotAlliance == Alliance.BLUE) {
            setAlliance(Alliance.RED);
        }
    }

    public Pose getPose() {
        return follower.pose();
    }

    public void setDrivePowers(double x, double y, double heading, boolean fieldCentric) {
        if (follower.isBusy()) {
            follower.stop();
        }

        if (!teleopDriveActive) {
            teleopDriveActive = true;
        }

        double fieldCentricHeading = 0;

        if (fieldCentric) {
            fieldCentricHeading = follower.pose().heading() - driverAngleOffset;
        }

        DrivePowers drivePowers = ManualDrive.fieldCentric(
                y,
                x,
                heading,
                fieldCentricHeading
        );

        if (headingLockEnabled) {
            headingLockAngle = getAngleToGoal();

            drivePowers = ManualDrive.headingLock(
                    follower,
                    Constants.getHeadingFeedbackController(),
                    drivePowers,
                    headingLockAngle,
                    Constants.getHeadingBreakLinear(),
                    Constants.getHeadingBreakQuadratic(),
                    Parameters.HEADING_LOCK_STRENGTH
            );
        }

        follower.manual(
                drivePowers
        );
    }

    public void setDrivePowers(double x, double y, double heading) {
        setDrivePowers(x, y, heading, true);
    }

    public HardwareTable getHardwareTable() {
        return hardwareTable;
    }

    public boolean isScoringCycleEnabled() {
        return scoreElementsCycleEnabled;
    }

    public void startScoringCycle(boolean waitForShooterVelocity, boolean waitForHeadingLock) {
        intake.setFrontRollersPower(0);
        setIntakePower(0);

        scoreWaitForHeadingLock = waitForHeadingLock;
        scoreWaitForShooterVelocity = waitForShooterVelocity;
        scoreElementsCycleEnabled = true;
        isTransfering = false;
    }

    public void startScoringCycle() {
        startScoringCycle(true, true);
    }

    public void setIntakePower(double power) {
        if (!scoreElementsCycleEnabled || !autoTransferPollenDuringScoreCycle) {
            manualIntakePower = power;
        }
    }

    public void stopScoringCycle() {
        shooter.closeGate();
        scoreElementsCycleEnabled = false;
        isTransferOvercurrent = false;
    }

    public void toggleScoringElements() {
        if (scoreElementsCycleEnabled) {
            stopScoringCycle();
        } else {
            startScoringCycle();
        }
    }

    public void enableAutoTransferPollenDuringScoreCycle() {
        autoTransferPollenDuringScoreCycle = true;
    }

    public void disableAutoTransferPollenDuringScoreCycle() {
        autoTransferPollenDuringScoreCycle = false;
    }

    public boolean isAutoTransferPollenDuringScoreCycleEnabled() {
        return autoTransferPollenDuringScoreCycle;
    }

    public boolean isTransferCycleTimerEnabled() {
        return transferCycleTimerEnabled;
    }

    public void enableHeadingLock() {
        headingLockEnabled = true;
    }

    public void disableHeadingLock() {
        headingLockEnabled = false;
    }

    public boolean isHeadingLockEnabled() {
        return headingLockEnabled;
    }

    public double getShooterVelocity() {
        return shooter.getMotorVelocity();
    }

    public void setDriverAngle(double offset, AngleUnit unit) {
        driverAngleOffset = unit == AngleUnit.DEGREES ? Math.toRadians(offset) : offset;
    }

    /// assumes degrees
    public void setDriverAngle(double offset) {
        setDriverAngle(offset, AngleUnit.DEGREES);
    }

    public void calibrateIMU() {
        // this method calibrates the IMU if the pinpoint is used in Constants.
        follower.localizer.reset();
    }

    public Pose getGoalPose(Pose robotPose) {
        if (robotPose.y() > 70.75) {
            if (robotAlliance == Alliance.RED) {
                return Parameters.RED_AIM_GOAL_2;
            } else if (robotAlliance == Alliance.BLUE) {
                return Parameters.BLUE_AIM_GOAL_2;
            }
        } else {
            if (robotAlliance == Alliance.RED) {
                return Parameters.RED_AIM_GOAL_1;
            } else if (robotAlliance == Alliance.BLUE) {
                return Parameters.BLUE_AIM_GOAL_1;
            }
        }

        return new Pose(0, 0);
    }

    public Pose getGoalPose() {
        return getGoalPose(getPose());
    }

    public double getAngleToPose(Pose robotPose, Pose targetPose) {
        return Math.atan2(targetPose.y() - robotPose.y(), targetPose.x() - robotPose.x()) + Math.PI;
    }

    public double getAngleToGoal(Pose robotPose) {
        return getAngleToPose(robotPose, getGoalPose(robotPose));
    }

    public double getAngleToGoal() {
        return getAngleToGoal(getPose());
    }

    /// in radians
    public double getHeadingLockError() {
        return Math.abs(headingLockAngle - getPose().heading());
    }

    public double getHeadingLockAngle() {
        return headingLockAngle;
    }

    public void powerOnShooter() {
        shooter.powerOn();
    }

    public void powerOffShooter() {
        shooter.powerOff();
    }

    public void toggleShooter() {
        shooter.togglePowered();
    }

    public boolean isShooterPoweredOn() {
        return shooter.isPoweredOn();
    }

    public double getShooterMotor1Current() {
        return shooter.getMotor1Current();
    }

    public double getShooterMotor2Current() {
        return shooter.getMotor2Current();
    }

    public Follower getFollower() {
        return follower;
    }

    public boolean isFlowerMechanismDown() {
        return intake.isFlowerMechanismDown();
    }

    public void lowerFlowerMechanism() {
        intake.lowerFlowerMechanism();
    }

    public void raiseFlowerMechanism() {
        intake.raiseFlowerMechanism();
    }

    public void toggleFlowerMechanismPosition() {
        intake.toggleFlowerMechanismPosition();
    }
}
