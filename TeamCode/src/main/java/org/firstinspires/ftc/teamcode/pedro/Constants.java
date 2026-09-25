package org.firstinspires.ftc.teamcode.pedro;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("motorLF");
        c.frontRightName.set("motorRF");
        c.backLeftName.set("motorLR");
        c.backRightName.set("motorRR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(3.241743553341843);
        c.yPodOffset.set(-3.1670253483329236);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3953667433079342);
                Controller secondaryTranslationalForward = Controller.proportional(0.1460774472583333);
                Controller primaryTranslationalLateral = Controller.proportional(0.6228521716058849);
                Controller secondaryTranslationalLateral = Controller.proportional(0.23012723449183217);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012714975754981537));
                c.brake.set(Controller.proportionalFeedforward(0.010807729391734307));

                c.headingFeedback.set(Controller.proportional(5.866789886935148));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06896151485482664, 0.0045723101633395125));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0825424319612417, 0.05550142123606644));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0018822725148517894, 0.002074946628591682));

                c.maxAchievableForwardVelocity.set(73.62159772813949);
                c.maxAchievableStrafeVelocity.set(65.4139146279758);
                c.naturalForwardDeceleration.set(44.18654802223922);
                c.naturalStrafeDeceleration.set(63.03038818486728);
            }
    );

    public static Controller getHeadingFeedbackController() {
        return Constants.foresightConfig.headingFeedback.get();
    }

    public static double getHeadingBreakLinear() {
        return Constants.foresightConfig.headingBrakeCoefficients.get().x();
    }

    public static double getHeadingBreakQuadratic() {
        return Constants.foresightConfig.headingBrakeCoefficients.get().y();
    }

    // we do this to carry localizer values through OpModes.
    public static Follower cachedFollower = null;

    public static Follower create(HardwareMap h) {
        if (cachedFollower == null) {
            cachedFollower = new Follower(
                    new PinpointLocalizer(h, localizerConfig),
                    new Mecanum(h, drivetrainConfig),
                    new Foresight(foresightConfig)
            );
        }

        return cachedFollower;
    }
}