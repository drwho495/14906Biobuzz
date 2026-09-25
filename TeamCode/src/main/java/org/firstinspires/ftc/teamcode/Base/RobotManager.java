package org.firstinspires.ftc.teamcode.Base;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Base.Misc.Alliance;
import org.firstinspires.ftc.teamcode.Base.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Base.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.concurrent.TimeUnit;

public class RobotManager {
    private HardwareTable hardwareTable;
    private IntakeSubsystem intake = new IntakeSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();
    private Follower follower;
    private ElapsedTime transferCycleTimer = new ElapsedTime();
    private boolean isTransfering = false;
    private double manualIntakePower = 0;
    private boolean scoreElementsCycleEnabled = false;
    private boolean autoTransferPollenDuringScoreCycle = true;
    private boolean teleopDriveActive = false;
    private boolean headingLockEnabled = false;
    private double driverAngleOffset = 0;
    private Alliance robotAlliance = Alliance.RED;

    public void initialize(HardwareMap hardwareMap) {
        hardwareTable = new HardwareTable(hardwareMap);
        follower = Constants.create(hardwareMap);

        setAlliance(
                StaticStorage.storedAlliance,
                false
        );

        shooter.initialize(hardwareMap, hardwareTable);
        intake.initialize(hardwareMap, hardwareTable);
    }

    public void update() {
        if (scoreElementsCycleEnabled) {
            if (shooter.ready() && !isTransfering) {
                isTransfering = true;

                transferCycleTimer.reset();
                intake.setTransferRollersPower(Parameters.SCORE_TRANSFER_SPEED);
                shooter.openGate();
            }

            if (isTransfering) {
                if (autoTransferPollenDuringScoreCycle) {
                    if (transferCycleTimer.time(TimeUnit.MILLISECONDS) >= Parameters.SHOOTER_TRANSFER_TIME) {
                        intake.setFrontRollersPower(manualIntakePower);
                        intake.setTransferRollersPower(manualIntakePower);

                        stopScoringElements();
                    }
                } else {
                    intake.setTransferRollersPower(manualIntakePower);
                }
            }
        } else {
            intake.setFrontRollersPower(manualIntakePower);
            intake.setTransferRollersPower(manualIntakePower);
        }

        shooter.setTargetVelocity(Parameters.SHOOTER_VELOCITY);

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
            fieldCentricHeading = follower.pose().heading();
        }

        DrivePowers drivePowers = ManualDrive.fieldCentric(
                y,
                x,
                heading,
                fieldCentricHeading + driverAngleOffset
        );

        if (headingLockEnabled) {
            ManualDrive.headingLock(
                    follower,
                    Constants.getHeadingFeedbackController(),
                    drivePowers,
                    0,
                    Constants.getHeadingBreakLinear(),
                    Constants.getHeadingBreakQuadratic(),
                    Parameters.HEADING_LOCK_STRENGTH
            );
        } else {
            follower.manual(
                drivePowers
            );
        }
    }

    public void setDrivePowers(double x, double y, double heading) {
        setDrivePowers(x, y, heading, true);
    }

    public HardwareTable getHardwareTable() {
        return hardwareTable;
    }

    public boolean isScoringElements() {
        return scoreElementsCycleEnabled;
    }

    public void startScoringElements() {
        intake.setFrontRollersPower(0);
        shooter.closeGate();
        setIntakePower(0);

        scoreElementsCycleEnabled = true;
        isTransfering = false;
    }

    public void setIntakePower(double power) {
        if (!scoreElementsCycleEnabled || !autoTransferPollenDuringScoreCycle) {
            manualIntakePower = power;
        }
    }

    public void stopScoringElements() {
        scoreElementsCycleEnabled = false;
    }

    public void toggleScoringElements() {
        if (scoreElementsCycleEnabled) {
            stopScoringElements();
        } else {
            startScoringElements();
        }
    }

    public void enableAutoTransferPollenDuringScoreCycle() {
        autoTransferPollenDuringScoreCycle = true;
    }

    public void disableAutoTransferPollenDuringScoreCycle() {
        autoTransferPollenDuringScoreCycle = false;
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
}
