package org.firstinspires.ftc.teamcode.pedro;

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

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FL");
        c.frontRightName.set("FR");
        c.backLeftName.set("BL");
        c.backRightName.set("BR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    //9 tall 5 wide

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odo");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-5.75);
        c.yPodOffset.set(-2.45);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2399927897707342);
                Controller secondaryTranslationalForward = Controller.proportional(0.08867092309483876);
                Controller primaryTranslationalLateral = Controller.proportional(0.29387985125743354);
                Controller secondaryTranslationalLateral = Controller.proportional(0.10858075242537245);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.018491377918890512));
                c.brake.set(Controller.proportionalFeedforward(0.015717671231056937));

                c.headingFeedback.set(Controller.proportional(3.7172119783204334));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04578732157668562, 0.0042231644508023075));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06864966441365261, 0.048859061488840824));
                c.quadraticBrakeCoefficients.set(Matrix.diag(8.770147465649998E-4, 0.0014075294086007156));

                c.maxAchievableForwardVelocity.set(58.95466251276149);
                c.maxAchievableStrafeVelocity.set(47.06262613004681);
                c.naturalForwardDeceleration.set(58.41039922695273);
                c.naturalStrafeDeceleration.set(77.77313667080718);
            }
    );

}

//public static ForesightConfig foresightConfig = new ForesightConfig(
//        c -> {
//            Controller primaryTranslationalForward = Controller.proportional(0.2249149167480223);
//            Controller secondaryTranslationalForward = Controller.proportional(0.08310005190113398);
//            Controller primaryTranslationalLateral = Controller.proportional(0.2527478666938251);
//            Controller secondaryTranslationalLateral = Controller.proportional(0.09338358319598845);
//
//            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//            c.coast.set(Controller.proportionalFeedforward(0.016047588837062834));
//            c.brake.set(Controller.proportionalFeedforward(0.013640450511503407));
//
//            c.headingFeedback.set(Controller.proportional(3.8861636503069055));
//            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.048575045028254146, 0.0025789406758087437));
//
//            c.linearBrakeCoefficients.set(Matrix.diag(0.039397124872052026, 0.027622423364473524));
//            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0017906889578534427, 0.002040440442742021));
//
//            c.maxAchievableForwardVelocity.set(63.883676724085284);
//            c.maxAchievableStrafeVelocity.set(53.84158622247183);
//            c.naturalForwardDeceleration.set(40.802591637056835);
//            c.naturalStrafeDeceleration.set(63.50465829123968);
//        }
//);