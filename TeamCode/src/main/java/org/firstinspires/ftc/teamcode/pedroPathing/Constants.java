package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Objects;

import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.localization.PinpointOdometryPredictor;
import edu.ftcphoenix.fw.integrations.pedro.PedroFieldTransform;
import edu.ftcphoenix.fw.integrations.pedro.PedroPathingRuntime;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;

/**
 * Project-specific Pedro tuning and pure Phoenix-to-Pedro configuration mapping.
 *
 * <p>{@link #phoenixAutoRuntimeConfig(PhoenixProfile)} snapshots only the Phoenix profile facts
 * that Pedro owns: Pinpoint configuration, drivetrain wiring, and the brake-mode choice. It does
 * not construct hardware or validate the draft. {@link PedroPathingRuntime#create(HardwareMap,
 * PedroPathingRuntime.Config)} remains the sole production effect boundary and authoritative
 * validation owner.</p>
 *
 * <p>Pedro's generated tuning menu and standalone test use the package-local native-Follower
 * factory so their vendor-owned update lifecycle remains compatible with Pedro's stock tools. That
 * exclusive diagnostic path is deliberately unavailable as a second production API.</p>
 */
public final class Constants {

    private Constants() {
        // Project constants/factories only.
    }

    /**
     * Map the Pedro-owned slice of a Phoenix profile into one independent raw runtime draft.
     *
     * <p>This method deliberately does not call {@link PhoenixProfile#copy()}. Unrelated malformed
     * profile sections cannot block this translation. Missing relevant nested sections and invalid
     * relevant values are preserved as raw configuration evidence for the runtime's single
     * {@code validatedCopy} boundary to diagnose before hardware acquisition.</p>
     *
     * @param profile selected Phoenix profile; retained nowhere and never mutated
     * @return a fresh, hardware-free Pedro runtime configuration draft
     * @throws NullPointerException if {@code profile} itself is null
     */
    public static PedroPathingRuntime.Config phoenixAutoRuntimeConfig(PhoenixProfile profile) {
        PhoenixProfile source = Objects.requireNonNull(profile, "profile");
        PedroPathingRuntime.Config mapped = PedroPathingRuntime.Config.defaults();

        mapped.predictor = source.localization == null || source.localization.predictor == null
                ? null
                : source.localization.predictor.copy();
        mapped.followerConstants = freshFollowerConstants();
        mapped.mecanumConstants = mecanumConstantsFrom(source);
        mapped.pathConstraints = freshPathConstraints();
        mapped.fieldTransform = PedroFieldTransform.decodeInvertedFtc();
        return mapped;
    }

    /**
     * Build Pedro's native Pinpoint/Follower graph for generated tuning tools only.
     *
     * <p>The complete runtime Config is snapshotted and validated before the first hardware
     * lookup. Construction then follows Mecanum, native Pinpoint localizer, and Follower order so a
     * returned drivetrain can be stopped best-effort if either later step fails. This follower owns
     * and polls Pinpoint inside {@link Follower#update()}; never pass it into Phoenix's managed Auto
     * graph, which instead uses {@link PedroPathingRuntime#create(HardwareMap,
     * PedroPathingRuntime.Config)}.</p>
     */
    static Follower createToolOnlyNativeFollower(HardwareMap hardwareMap) {
        return createToolOnlyNativeFollowerForTest(
                hardwareMap,
                PhoenixProfile.current(),
                nativeToolConstruction()
        );
    }

    /**
     * Package-local evidence seam for the native tool's exclusive construction transaction.
     *
     * <p>Generated/native tool OpModes call the one-argument factory above. Tests use this seam
     * to prove validation, order, and cleanup without acquiring FTC hardware.</p>
     */
    static Follower createToolOnlyNativeFollowerForTest(
            HardwareMap hardwareMap,
            PhoenixProfile profile,
            NativeToolConstruction construction
    ) {
        HardwareMap requiredHardwareMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
        PhoenixProfile requiredProfile = Objects.requireNonNull(profile, "profile");
        NativeToolConstruction requiredConstruction = Objects.requireNonNull(
                construction,
                "construction"
        );
        PedroPathingRuntime.Config snapshot = phoenixAutoRuntimeConfig(requiredProfile)
                .validatedCopy(null);

        Drivetrain drivetrain = null;
        try {
            drivetrain = requiredConstruction.createMecanum(
                    requiredHardwareMap,
                    snapshot.mecanumConstants
            );
            Localizer localizer = requiredConstruction.createPinpoint(
                    requiredHardwareMap,
                    pinpointConstantsFrom(snapshot.predictor)
            );
            return requiredConstruction.createFollower(
                    snapshot.followerConstants,
                    localizer,
                    drivetrain,
                    snapshot.pathConstraints
            );
        } catch (RuntimeException setupFailure) {
            if (drivetrain != null) {
                try {
                    drivetrain.breakFollowing();
                } catch (RuntimeException stopFailure) {
                    if (stopFailure != setupFailure) {
                        setupFailure.addSuppressed(stopFailure);
                    }
                }
            }
            throw new IllegalStateException(
                    "Pedro tool-only native Follower setup failed after validated configuration; "
                            + "check Phoenix Pinpoint and drivetrain hardware",
                    setupFailure
            );
        }
    }

    /** Package-local constructor roles used only by the native-tool transaction test seam. */
    interface NativeToolConstruction {
        Drivetrain createMecanum(HardwareMap hardwareMap, MecanumConstants constants);

        Localizer createPinpoint(HardwareMap hardwareMap, PinpointConstants constants);

        Follower createFollower(FollowerConstants constants,
                                Localizer localizer,
                                Drivetrain drivetrain,
                                PathConstraints constraints);
    }

    /** Bind the package-local construction roles to Pedro's concrete FTC implementations. */
    private static NativeToolConstruction nativeToolConstruction() {
        return new NativeToolConstruction() {
            @Override
            public Drivetrain createMecanum(HardwareMap hardwareMap,
                                            MecanumConstants constants) {
                return new Mecanum(hardwareMap, constants);
            }

            @Override
            public Localizer createPinpoint(HardwareMap hardwareMap,
                                            PinpointConstants constants) {
                return new PinpointLocalizer(hardwareMap, constants);
            }

            @Override
            public Follower createFollower(FollowerConstants constants,
                                           Localizer localizer,
                                           Drivetrain drivetrain,
                                           PathConstraints constraints) {
                return new Follower(constants, localizer, drivetrain, constraints);
            }
        };
    }

    /** Return a fresh copy of this project's checked-in Pedro follower tuning. */
    private static FollowerConstants freshFollowerConstants() {
        return new FollowerConstants().mass(9.616158);
    }

    /** Return a fresh copy of this project's checked-in route-completion/braking tuning. */
    private static PathConstraints freshPathConstraints() {
        return new PathConstraints(0.99, 100.0, 1.0, 1.0);
    }

    /** Map profile-owned physical wiring and brake mode onto fresh Pedro drivetrain tuning. */
    private static MecanumConstants mecanumConstantsFrom(PhoenixProfile profile) {
        if (profile.drive == null || profile.drive.wiring == null) {
            return null;
        }

        FtcDrives.MecanumWiringConfig wiring = profile.drive.wiring;
        MecanumConstants mapped = new MecanumConstants();
        mapped.maxPower = 1.0;
        mapped.useBrakeModeInTeleOp = profile.drive.enableZeroPowerBrake;
        mapped.leftFrontMotorName = wiring.frontLeftName;
        mapped.leftRearMotorName = wiring.backLeftName;
        mapped.rightFrontMotorName = wiring.frontRightName;
        mapped.rightRearMotorName = wiring.backRightName;
        mapped.leftFrontMotorDirection = toFtcDirection(wiring.frontLeftDirection);
        mapped.leftRearMotorDirection = toFtcDirection(wiring.backLeftDirection);
        mapped.rightFrontMotorDirection = toFtcDirection(wiring.frontRightDirection);
        mapped.rightRearMotorDirection = toFtcDirection(wiring.backRightDirection);
        return mapped;
    }

    /** Translate a validated predictor snapshot for Pedro's native Pinpoint localizer. */
    private static PinpointConstants pinpointConstantsFrom(
            PinpointOdometryPredictor.Config predictor
    ) {
        PinpointOdometryPredictor.Config source = Objects.requireNonNull(
                predictor,
                "predictor"
        );
        PinpointConstants target = new PinpointConstants()
                .forwardPodY(source.forwardPodOffsetLeftInches)
                .strafePodX(source.strafePodOffsetForwardInches)
                .distanceUnit(DistanceUnit.INCH)
                .hardwareMapName(source.hardwareMapName)
                .forwardEncoderDirection(source.forwardPodDirection)
                .strafeEncoderDirection(source.strafePodDirection);

        source.encoderResolution.applyTo(
                target::encoderResolution,
                target::customEncoderResolution
        );
        if (source.yawScalar != null) {
            target.yawScalar(source.yawScalar);
        }
        return target;
    }

    /** Preserve null authored directions for the runtime's authoritative validation boundary. */
    private static DcMotorSimple.Direction toFtcDirection(Direction direction) {
        if (direction == null) {
            return null;
        }
        return direction == Direction.REVERSE
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD;
    }
}
