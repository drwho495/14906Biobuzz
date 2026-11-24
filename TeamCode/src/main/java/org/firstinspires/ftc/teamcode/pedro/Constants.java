package org.firstinspires.ftc.teamcode.pedro;

//import com.bylazar.configurables.annotations.Configurable;

import com.acmerobotics.dashboard.config.Config;
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

//@Configurable
@Config
public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("motorLF");
        c.frontRightName.set("motorRF");
        c.backLeftName.set("motorLR");
        c.backRightName.set("motorRR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(64.0);
        c.yPodOffset.set(-86.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.MM);
    });

//    public static OctoQuadConfig localizerConfig = new OctoQuadConfig(c -> {
//        c.name.set("octoquad");
//        c.xPodPort.set(0);
//        c.yPodPort.set(1);
//        c.ticksPerUnit.set(505.316944406);
//        c.xPodOffset.set(3.6220472440944884);
//        c.yPodOffset.set(-4.68503937007874);
//        c.xPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
//        c.yPodDirection.set(OctoQuad.EncoderDirection.REVERSE);
//        c.globalDistanceUnit.set(DistanceUnit.INCH);
//        c.offsetUnits.set(DistanceUnit.INCH);
//        c.i2cRecoveryMode.set(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);
//        c.headingScalar.set(1.0316327390755264);
//    });


    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3077042444448746);
                Controller secondaryTranslationalForward = Controller.proportional(0.11368849631354266);
                Controller primaryTranslationalLateral = Controller.proportional(0.48165397390169584);
                Controller secondaryTranslationalLateral = Controller.proportional(0.17795827332546318);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010115287373132716));
                c.brake.set(Controller.proportionalFeedforward(0.008597994267162809));

                c.headingFeedback.set(Controller.proportional(4.9763514021624085));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06543258541292238, 0.005992226148788479));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0947576262775694, 0.04851018434340809));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0020691022111995997, 0.0022276325084210585));

                c.maxAchievableForwardVelocity.set(89.73643048605422);
                c.maxAchievableStrafeVelocity.set(65.13378374994176);
                c.naturalForwardDeceleration.set(25.397713377068225);
                c.naturalStrafeDeceleration.set(46.53004809193957);
            }
    );

    public static Controller getHeadingStaticFFController() {
        return Constants.foresightConfig.headingStaticFF.get();
    }

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
    public static PinpointLocalizer cachedLocalizer = null;

    public static Follower create(HardwareMap h) {
        if (Constants.cachedLocalizer == null) {
            Constants.cachedLocalizer = new PinpointLocalizer(h, localizerConfig);
        }

        return new Follower(
                Constants.cachedLocalizer,
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static double getHoldPointHeadingScalar() {
        return Constants.foresightConfig.holdPointHeadingScaling.get();
    }
}