package edu.ftcsushi.robots.phoenix.pedro;

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

import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.localization.PinpointOdometryPredictor;
import edu.ftcsushi.fw.integrations.pedro.PedroFieldTransform;
import edu.ftcsushi.fw.integrations.pedro.PedroPathingRuntime;
import edu.ftcsushi.robots.phoenix.PhoenixProfile;

/**
 * Project-specific Pedro tuning and pure Phoenix-to-Pedro configuration mapping.
 *
 * <p>{@link #phoenixAutoRuntimeConfig(PinpointOdometryPredictor.Config,
 * FtcDrives.MecanumWiringConfig, boolean)} snapshots only the Phoenix facts that Pedro owns:
 * Pinpoint configuration, drivetrain wiring, and the brake-mode choice. It does not construct
 * hardware or validate the draft. {@link PedroPathingRuntime#create(HardwareMap,
 * PedroPathingRuntime.Config)} remains the sole production effect boundary and authoritative
 * validation owner.</p>
 *
 * <p>Pedro's generated tuning menu and standalone test use the package-local native-Follower
 * factory so their vendor-owned update lifecycle remains compatible with Pedro's stock tools. That
 * exclusive diagnostic path is deliberately unavailable as a second production API.</p>
 */
public final class PhoenixPedroConfiguration {

    private PhoenixPedroConfiguration() {
        // Project constants/factories only.
    }

    /**
     * Map the Pedro-owned Phoenix facts into one independent raw runtime draft.
     *
     * <p>Unrelated profile sections are not accepted, so they cannot block this translation.
     * Missing or invalid relevant values are preserved as raw configuration evidence for the
     * runtime's single {@code validatedCopy} boundary to diagnose before hardware acquisition.</p>
     *
     * @param predictor selected Pinpoint draft; retained nowhere and never mutated
     * @param wiring selected drivetrain-wiring draft; retained nowhere and never mutated
     * @param enableZeroPowerBrake selected drivetrain brake-mode answer
     * @return a fresh, hardware-free Pedro runtime configuration draft
     */
    public static PedroPathingRuntime.Config phoenixAutoRuntimeConfig(
            PinpointOdometryPredictor.Config predictor,
            FtcDrives.MecanumWiringConfig wiring,
            boolean enableZeroPowerBrake
    ) {
        PedroPathingRuntime.Config mapped = PedroPathingRuntime.Config.defaults();

        mapped.predictor = predictor == null ? null : predictor.copy();
        mapped.followerConstants = freshFollowerConstants();
        mapped.mecanumConstants = mecanumConstantsFrom(wiring, enableZeroPowerBrake);
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
        PhoenixProfile profile = PhoenixProfile.current();
        return createToolOnlyNativeFollowerForTest(
                hardwareMap,
                phoenixAutoRuntimeConfig(
                        profile.localization == null ? null : profile.localization.predictor,
                        profile.drive == null ? null : profile.drive.wiring,
                        profile.drive != null && profile.drive.enableZeroPowerBrake
                ),
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
            PedroPathingRuntime.Config runtimeConfig,
            NativeToolConstruction construction
    ) {
        HardwareMap requiredHardwareMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
        NativeToolConstruction requiredConstruction = Objects.requireNonNull(
                construction,
                "construction"
        );
        PedroPathingRuntime.Config snapshot = Objects.requireNonNull(
                runtimeConfig,
                "runtimeConfig"
        ).validatedCopy(null);

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

    /** Map Phoenix physical wiring and brake mode onto fresh Pedro drivetrain tuning. */
    private static MecanumConstants mecanumConstantsFrom(
            FtcDrives.MecanumWiringConfig wiring,
            boolean enableZeroPowerBrake
    ) {
        if (wiring == null) {
            return null;
        }

        MecanumConstants mapped = new MecanumConstants();
        mapped.maxPower = 1.0;
        mapped.useBrakeModeInTeleOp = enableZeroPowerBrake;
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
