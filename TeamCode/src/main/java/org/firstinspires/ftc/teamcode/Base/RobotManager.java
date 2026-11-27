package org.firstinspires.ftc.teamcode.Base;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.Policy.ShooterControlPolicy;
import org.firstinspires.ftc.teamcode.Base.Field.LocalFlowerLocation;
import org.firstinspires.ftc.teamcode.Base.Field.LocalHiveLocation;
import org.firstinspires.ftc.teamcode.Base.Misc.Alliance;
import org.firstinspires.ftc.teamcode.Base.Field.Flower;
import org.firstinspires.ftc.teamcode.Base.Field.Hive;
import org.firstinspires.ftc.teamcode.Base.Misc.PointsCurve;
import org.firstinspires.ftc.teamcode.Base.Subsystems.BrakeSubsystem;
import org.firstinspires.ftc.teamcode.Base.Utilities.MathUtils;
import org.firstinspires.ftc.teamcode.Base.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Base.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class RobotManager {
    private HardwareTable hardwareTable;
    private IntakeSubsystem intake = new IntakeSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();
    private BrakeSubsystem brake = new BrakeSubsystem();
    private Follower follower;
    private Foresight foresight = null;
    private ElapsedTime cacheResetTimer = new ElapsedTime();
    private ElapsedTime transferCycleTimer = new ElapsedTime();
    private ElapsedTime transferFrontTimer = new ElapsedTime();
    private ElapsedTime transferStartDelayTimer = new ElapsedTime();
    private boolean stageTransfer = false;
    private boolean isTransfering = false;
    private double manualIntakePower = 0;
    private double manualShooterVelocity = 0;
    private boolean scoreElementsCycleEnabled = false;
    private boolean autoTransferPollenDuringScoreCycle = true;
    private boolean teleopDriveActive = false;
    private boolean headingLockEnabled = false;
    private boolean aimToHiveEnabled = false;
    private Hive hiveToAimTo = null;
    private boolean scoreWaitForShooterVelocity = false;
    private boolean scoreWaitForHeading = false;
    private boolean transferCycleTimerEnabled = false;
    private boolean isTransferOvercurrent = false;
    private boolean stopTransferIfOvercurrent = true;
    private ElapsedTime transferOvercurrentTimer = new ElapsedTime();
    private double headingLockAngle = 0;
    private double driverAngleOffset = 0;
    private Alliance robotAlliance = Alliance.RED;
    private List<LynxModule> lynxModules;
    private boolean followerEndStalePathEnabled = true;
    private boolean pathTimeoutApplied = false;
    private boolean manualDriveEnabled = false;
    private boolean subsystemsEnabled = true;
    private double pathTimeout = 0;
    private LinearOpMode opMode;
    private ShooterControlPolicy shooterControlPolicy = ShooterControlPolicy.AUTOMATIC_DYNAMIC;
    private PointsCurve shooterVelocityCurve = new PointsCurve();

    public void initialize(LinearOpMode thisOpMode) {
        opMode = thisOpMode;
        hardwareTable = new HardwareTable(opMode.hardwareMap);
        follower = Constants.create(opMode.hardwareMap);

        Algorithm followerAlgorithm = follower.algorithm();
        if (followerAlgorithm instanceof Foresight) {
            foresight = (Foresight) followerAlgorithm;
        }

        setAlliance(StaticStorage.storedAlliance);

        shooter.initialize(hardwareTable);
        intake.initialize(hardwareTable);
        brake.initialize(hardwareTable);

        updateShooterCurve();

        lynxModules = hardwareTable.getLynxModules();

        for (LynxModule module : lynxModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void update() {
        Pose robotPose = getPose();

        if (scoreElementsCycleEnabled) {
            if (canShoot() && !stageTransfer && !isTransfering) {
                stageTransfer = true;

                transferStartDelayTimer.reset();
                shooter.openGate();
            }

            if (stageTransfer && transferStartDelayTimer.time(TimeUnit.MILLISECONDS) >= Parameters.TRANSFER_START_DELAY_TIME) {
                isTransfering = true;
                stageTransfer = false;

                transferCycleTimer.reset();
                transferFrontTimer.reset();
                transferCycleTimerEnabled = true;

                intake.setFrontRollersPower(Parameters.SCORE_TRANSFER_VELOCITY);
                intake.setTransferRollersVelocity(Parameters.SCORE_TRANSFER_VELOCITY);
            }

            if (isTransfering) {
                if (autoTransferPollenDuringScoreCycle) {
                    if (transferFrontTimer.time(TimeUnit.MILLISECONDS) > 50) {
                        intake.setFrontRollersPower(0);
                    }

                    if (transferCycleTimerEnabled && transferCycleTimer.time(TimeUnit.MILLISECONDS) >= Parameters.SHOOTER_TRANSFER_TIME) {
                        stopScoringCycle();
                    }
                } else {
                    intake.setFrontRollersPower(manualIntakePower);
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

        if (cacheResetTimer.time(TimeUnit.MILLISECONDS) >= 5) {
            for (LynxModule module : lynxModules) {
                module.clearBulkCache();
            }

            cacheResetTimer.reset();
        }

        if (aimToHiveEnabled && foresight != null) {
            double angleToHive = getAngleToPose(robotPose, hiveToAimTo.getCenterPose());

            follower.manual(
                    0,
                    0,
                    foresight.calculateHold(
                            follower.drivetrain,
                            robotPose.withHeading(angleToHive),
                            follower.localizer.state(),
                            true,
                            0
                    ).turn()
            );
        }

        if (shooterControlPolicy == ShooterControlPolicy.AUTOMATIC_DYNAMIC) {
            shooter.setTargetVelocity(getShooterVelocityFromRegression(robotPose));
        } else if (shooterControlPolicy == ShooterControlPolicy.AUTOMATIC_STATIC) {
            shooter.setTargetVelocity(Parameters.STATIC_SHOOTER_VELOCITY);
        } else if (shooterControlPolicy == ShooterControlPolicy.MANUAL) {
            shooter.setTargetVelocity(manualShooterVelocity);
        }

        if (follower.following()) {
            opMode.telemetry.addData("Robot X: ", robotPose.x());
            opMode.telemetry.addData("Robot Y: ", robotPose.y());
            opMode.telemetry.addData("Robot Heading: ", Math.toDegrees(robotPose.heading()));
            opMode.telemetry.update();
        }

        follower.update();

        if (subsystemsEnabled) {
            shooter.update();
            intake.update();
            brake.update();
        }
    }

    public void enableSubsystems() {
        subsystemsEnabled = true;
    }

    public void disableSubsystems() {
        subsystemsEnabled = false;
    }

    private void setAlliance(Alliance alliance) {
        robotAlliance = alliance;

        StaticStorage.storedAlliance = alliance;
    }

    public boolean canShoot() {
        if (!shooter.isPoweredOn()) {
            return false;
        }

        if (scoreWaitForShooterVelocity && !shooter.ready()) {
            return false;
        }

        if (scoreWaitForHeading && headingLockEnabled && !headingAtTarget()) {
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

    public Alliance getAlliance() {
        return robotAlliance;
    }

    public void switchAlliance() {
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

        manualDriveEnabled = true;

        if (!teleopDriveActive) {
            teleopDriveActive = true;
        }

        Pose robotPose = getPose();
        double fieldCentricHeading = 0;

        if (fieldCentric) {
            fieldCentricHeading = robotPose.heading() - driverAngleOffset;
        }

        DrivePowers drivePowers;

        if (headingLockEnabled && foresight != null) {
            headingLockAngle = getAngleToHive();

            drivePowers = ManualDrive.fieldCentric(
                    y,
                    x,
                    foresight.calculateHold(
                            follower.drivetrain,
                            robotPose.withHeading(headingLockAngle),
                            follower.localizer.state(),
                            true,
                            0
                    ).turn(),
                    fieldCentricHeading
            );
        } else {
            drivePowers = ManualDrive.fieldCentric(
                    y,
                    x,
                    heading,
                    fieldCentricHeading
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

    public void startScoringCycle(boolean waitForShooterVelocity, boolean waitForHeading) {
        intake.setFrontRollersPower(0);
        setIntakePower(0);

        scoreWaitForHeading = waitForHeading;
        scoreWaitForShooterVelocity = waitForShooterVelocity;
        scoreElementsCycleEnabled = true;
        isTransfering = false;
        stageTransfer = false;
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

    public void enableAimToHive() {
        enableAimToHive(getNearestAllianceHive());
    }

    public void enableAimToHive(Hive hive) {
        stopDriving();

        aimToHiveEnabled = true;
        hiveToAimTo = hive;
    }

    public void disableAimToHive() {
        aimToHiveEnabled = false;
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

    public Hive getNearestAllianceHive(Pose robotPose) {
        if (robotPose.y() > 70.75) {
            if (robotAlliance == Alliance.RED) {
                return Hive.UPPER_LEFT;
            } else {
                return Hive.UPPER_RIGHT;
            }
        } else {
            if (robotAlliance == Alliance.RED) {
                return Hive.LOWER_LEFT;
            } else {
                return Hive.LOWER_RIGHT;
            }
        }
    }

    public Hive getNearestAllianceHive() {
        return getNearestAllianceHive(getPose());
    }

    public double getAngleToHive(Pose robotPose, Hive hive) {
        return hive.getScoreHeading(robotPose);
    }

    public double getAngleToHive(Hive hive) {
        return getAngleToHive(getPose(), hive);
    }

    public double getAngleToHive() {
        Pose robotPose = getPose();

        return getAngleToHive(robotPose, getNearestAllianceHive(robotPose));
    }

    public double getAngleToPose(Pose robotPose, Pose targetPose) {
        return Math.atan2(targetPose.y() - robotPose.y(), targetPose.x() - robotPose.x()) + Math.PI;
    }

    /**
     * @return The heading error, normalized to a range of -PI -> PI, in radians
     */
    public double getHeadingError() {
        double error = 0;

        if (manualDriveEnabled && headingLockEnabled) {
            error = headingLockAngle - getPose().heading();
        } else if (follower.isBusy()) {
            error = follower.closestPose().heading() - getPose().heading();
        }

        return Angle.normalizeSigned(error);
    }

    public boolean headingAtTarget() {
        return Math.abs(getHeadingError()) < Parameters.ACCEPTABLE_HEADING_ERROR;
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

    public void setPose(Pose newPose) {
        follower.setPose(newPose);
    }

    /**
     * @param pose The pose local to the RED field side.
     * @return A pose that may be mirrored if the robot is driving on the BLUE alliance.
     */
    public Pose getFixedPose(Pose pose) {
        if (robotAlliance == Alliance.RED) {
            return pose;
        } else {
            return MathUtils.rotateAboutFieldCenter(pose, Math.PI);
        }
    }

    /**
     * @param x The x local to the RED field side.
     * @param y The y local to the RED field side.
     * @param heading The heading local to the RED field side.
     * @return A pose that may be mirrored if the robot is driving on the BLUE alliance.
     */
    public Pose getFixedPose(double x, double y, double heading) {
        Pose builtPose = new Pose(x, y, heading);

        if (robotAlliance == Alliance.RED) {
            return builtPose; // no mirroring is needed
        } else {
            return MathUtils.rotateAboutFieldCenter(builtPose, Math.PI);
        }
    }

    /**
     * @param x The x local to the RED field side.
     * @param y The y local to the RED field side.
     * @return A pose that may be mirrored if the robot is driving on the BLUE alliance.
     */
    public Pose getFixedPose(double x, double y) {
        return getFixedPose(x, y, 0);
    }

    /**
     * @param heading The heading local to the RED field side.
     * @param angleUnit The unit of heading.
     * @return A heading that may be mirrored if the robot is driving on the BLUE alliance.
     */
    public double getFixedHeading(double heading, AngleUnit angleUnit) {
        double correctedHeading = heading;

        if (angleUnit == AngleUnit.DEGREES) {
            correctedHeading = Math.toRadians(heading);
        }

        if (robotAlliance == Alliance.RED) {
            return correctedHeading; // no mirroring is needed
        } else {
            return correctedHeading + Math.PI;
        }
    }

    /**
     * @param heading The heading (in degrees) local to the RED field side.
     * @return A heading that may be mirrored if the robot is driving on the BLUE alliance.
     */
    public double getFixedHeading(double heading) {
        return getFixedHeading(heading, AngleUnit.DEGREES);
    }

    /**
     * @param parameter The parameter (X or Y coordinate) local to the RED field side.
     * @return A parameter that may be mirrored if the robot is driving on the BLUE alliance.
     */
    public double getFixedParameter(double parameter) {
        return getFixedPose(parameter, 0).x();
    }

    /**
     * @param parameter The parameter (X or Y coordinate) local to the RED field side.
     * @return A parameter with the correct sign to represent the direction relative to the RED or BLUE alliance.
     */
    public double fixParameterDirection(double parameter) {
        if (robotAlliance == Alliance.BLUE) {
            return -parameter;
        } else {
            return parameter;
        }
    }

    public void runPathBlocking(Path path, boolean correctAfterFinished) {
        stopDriving();

        follower.holdEnd.set(correctAfterFinished);
        follower.follow(path);

        ElapsedTime timer = new ElapsedTime();

        while (follower.following() && !opMode.isStopRequested() && opMode.opModeIsActive()) {
            Pose robotPose = getPose();

            if (pathTimeoutApplied && timer.time(TimeUnit.MILLISECONDS) >= pathTimeout) {
                break;
            }

            update();
        }

        pathTimeoutApplied = false;
    }

    public void runPathBlocking(Path path) {
        runPathBlocking(path, true);
    }

    public void sleep(long time) {
        ElapsedTime timer = new ElapsedTime();

        sleep(timer, time);
    }

    public void sleep(ElapsedTime timer, long time) {
        waitForCondition(() -> timer.time(TimeUnit.MILLISECONDS) >= time);
    }

    public void waitForShooter(double timeout) {
        ElapsedTime timer = new ElapsedTime();

        waitForCondition(() -> timer.time(TimeUnit.MILLISECONDS) >= timeout || shooter.ready());
    }

    public void waitForCondition(Supplier<Boolean> condition) {
        while (!condition.get() && opMode.opModeIsActive()) {
            update();
        }
    }

    public void addPathTimeout(double timeout) {
        pathTimeoutApplied = true;
        pathTimeout = timeout;
    }

    public Hive getHive(LocalHiveLocation fromLocalLocation) {
        if (fromLocalLocation == LocalHiveLocation.UPPER) {
            if (robotAlliance == Alliance.RED) {
                return Hive.UPPER_LEFT;
            } else {
                return Hive.LOWER_RIGHT;
            }
        } else { // LocalHiveLocation.LOWER
            if (robotAlliance == Alliance.RED) {
                return Hive.LOWER_LEFT;
            } else {
                return Hive.UPPER_RIGHT;
            }
        }
    }

    public Flower getFlower(LocalFlowerLocation fromLocalLocation) {
        if (fromLocalLocation == LocalFlowerLocation.UPPER) {
            if (robotAlliance == Alliance.RED) {
                return Flower.UPPER;
            } else {
                return Flower.LOWER;
            }
        } else if (fromLocalLocation == LocalFlowerLocation.LOWER) {
            if (robotAlliance == Alliance.RED) {
                return Flower.LOWER;
            } else {
                return Flower.UPPER;
            }
        } else {
            if (robotAlliance == Alliance.RED) {
                return Flower.LEFT;
            } else {
                return Flower.RIGHT;
            }
        }
    }

    public void lowerBrake() {
        brake.lower();
    }

    public void raiseBrake() {
        brake.raise();
    }

    public void toggleBrake() {
        brake.lower();
    }

    public boolean isBrakeLowered() {
        return brake.isLowered();
    }

    public void stopDriving() {
        if (follower.isBusy()) {
            follower.stop();
        }

        if (manualDriveEnabled) {
            if (headingLockEnabled) {
                disableHeadingLock();
            }

            setDrivePowers(0, 0, 0);
            manualDriveEnabled = false;
        }

        if (aimToHiveEnabled) {
            aimToHiveEnabled = false;
            follower.manual(0, 0, 0);
        }
    }

    public boolean areSubsystemsEnabled() {
        return subsystemsEnabled;
    }

    public void resetDriverAngle() {
        if (robotAlliance == Alliance.RED) {
            driverAngleOffset = 0;
        } else {
            driverAngleOffset = Math.PI;
        }
    }

    public void waitForPosition(long timeout, double error) {
        waitForCondition(() -> (!follower.isBusy()
                || follower.distanceToEndpoint() <= error));
    }

    public double getDistanceToHive(Pose robotPose, Hive hive) {
        return robotPose.distance(hive.getRegressionPose());
    }

    public double getDistanceToHive(Pose robotPose) {
        return getDistanceToHive(robotPose, getNearestAllianceHive(robotPose));
    }

    public double getDistanceToHive() {
        return getDistanceToHive(getPose());
    }

    public double getShooterVelocityFromRegression(Pose robotPose, Hive hive) {
        double distanceToGoal = getDistanceToHive(robotPose, hive);

        if (shooterVelocityCurve.isBuilt()) {
            return shooterVelocityCurve.getY(
                    Range.clip(distanceToGoal, shooterVelocityCurve.getMinX(), shooterVelocityCurve.getMaxX())
            );
        }

        return 0;
    }

    public double getShooterVelocityFromRegression(Pose robotPose) {
        return getShooterVelocityFromRegression(robotPose, getNearestAllianceHive(robotPose));
    }

    public double getShooterVelocityFromRegression() {
        return getShooterVelocityFromRegression(getPose());
    }

    public void setShooterControlPolicy(ShooterControlPolicy shooterControlPolicy) {
        this.shooterControlPolicy = shooterControlPolicy;
    }

    public void setShooterVelocity(double velocity) {
        manualShooterVelocity = velocity;
    }

    public ShooterControlPolicy getShooterControlPolicy() {
        return shooterControlPolicy;
    }

    public void updateShooterCurve() {
        shooterVelocityCurve.addPoint(48, 1405); // from wall
        shooterVelocityCurve.addPoint(58, 1440);
        shooterVelocityCurve.addPoint(66, 1520);

        shooterVelocityCurve.buildCurve();
    }

    public Pose getLocalOrigin() {
        if (robotAlliance == Alliance.RED) {
            return Parameters.ORIGIN_RED;
        } else {
            return Parameters.ORIGIN_BLUE;
        }
    }
}
