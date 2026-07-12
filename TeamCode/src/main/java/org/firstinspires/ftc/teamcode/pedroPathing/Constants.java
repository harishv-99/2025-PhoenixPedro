package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.HashSet;
import java.util.Objects;
import java.util.Set;

import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.ftc.FtcDrives;
import edu.ftcphoenix.fw.ftc.localization.PinpointOdometryPredictor;
import edu.ftcphoenix.fw.integrations.pedro.PedroFieldTransform;
import edu.ftcphoenix.fw.integrations.pedro.PedroPathingRuntime;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;

/**
 * Project-specific Pedro tuning and Phoenix/Pedro runtime factories.
 *
 * <p>Phoenix physical configuration has one checked-in authority: {@link PhoenixProfile}. These
 * factories derive Pedro motor wiring and Pinpoint calibration from a defensive profile copy.
 * This class retains only Pedro-specific follower, controller, drivetrain, and path tuning.</p>
 *
 * <p>Production Phoenix Auto must use {@link #createPhoenixAutoRuntime(HardwareMap,
 * PhoenixProfile)}. Pedro's generated tuning menu and the standalone Pedro test deliberately use
 * {@link #createToolOnlyNativeFollower(HardwareMap)} so their vendor-owned update lifecycle remains
 * compatible with Pedro's stock tuning code. The tool-only path is not a second production option.</p>
 */
public final class Constants {

    /** Pedro controller/follower tuning shared by the production lane and vendor tuning tools. */
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.616158);

    /** Pedro route completion and predictive-braking tuning. */
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    private Constants() {
        // Project constants/factories only.
    }

    /**
     * Build the complete production Phoenix/Pedro Auto runtime from one profile snapshot.
     *
     * @param hardwareMap FTC hardware registry
     * @param profile     selected Phoenix Auto profile; defensively copied
     * @return one runtime owning the Follower graph and sharing one Pinpoint predictor with Phoenix
     */
    public static PedroPathingRuntime createPhoenixAutoRuntime(HardwareMap hardwareMap,
                                                               PhoenixProfile profile) {
        HardwareMap requiredHardwareMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
        PhoenixProfile profileSnapshot = Objects.requireNonNull(profile, "profile").copy();
        validatePhysicalConfig(profileSnapshot, true);

        PinpointOdometryPredictor predictor;
        try {
            predictor = new PinpointOdometryPredictor(
                    requiredHardwareMap,
                    profileSnapshot.localization.predictor
            );
        } catch (RuntimeException setupFailure) {
            throw new IllegalStateException(
                    "Phoenix Pedro Pinpoint setup failed for hardwareMapName='"
                            + profileSnapshot.localization.predictor.hardwareMapName
                            + "'; check PhoenixProfile.localization.predictor",
                    setupFailure
            );
        }

        return PedroPathingRuntime.create(
                requiredHardwareMap,
                predictor,
                Objects.requireNonNull(followerConstants, "Constants.followerConstants"),
                mecanumConstantsFrom(profileSnapshot),
                Objects.requireNonNull(pathConstraints, "Constants.pathConstraints").copy(),
                PedroFieldTransform.decodeInvertedFtc()
        );
    }

    /**
     * Build Pedro's native Pinpoint/Follower graph for generated tuning tools only.
     *
     * <p>This follower owns and polls Pinpoint inside {@link Follower#update()}. Never pass it to
     * {@code PhoenixRobot.initAuto}; production uses {@link #createPhoenixAutoRuntime(HardwareMap,
     * PhoenixProfile)} so Phoenix and Pedro cannot become competing Pinpoint owners.</p>
     */
    public static Follower createToolOnlyNativeFollower(HardwareMap hardwareMap) {
        HardwareMap requiredHardwareMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
        PhoenixProfile profileSnapshot = PhoenixProfile.current().copy();
        validatePhysicalConfig(profileSnapshot, false);

        PinpointLocalizer localizer;
        try {
            localizer = new PinpointLocalizer(
                    requiredHardwareMap,
                    pinpointConstantsFrom(profileSnapshot)
            );
        } catch (RuntimeException setupFailure) {
            throw new IllegalStateException(
                    "Pedro tool-only Pinpoint setup failed for hardwareMapName='"
                            + profileSnapshot.localization.predictor.hardwareMapName
                            + "'; check PhoenixProfile.localization.predictor",
                    setupFailure
            );
        }

        Mecanum drivetrain = null;
        try {
            drivetrain = new Mecanum(requiredHardwareMap, mecanumConstantsFrom(profileSnapshot));
            return new Follower(
                    Objects.requireNonNull(followerConstants, "Constants.followerConstants"),
                    localizer,
                    drivetrain,
                    Objects.requireNonNull(pathConstraints, "Constants.pathConstraints").copy()
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
                    "Pedro tool-only drivetrain/Follower setup failed for PhoenixProfile.drive.wiring",
                    setupFailure
            );
        }
    }

    /** Pedro-specific drivetrain tuning plus profile-owned physical wiring. */
    static MecanumConstants mecanumConstantsFrom(PhoenixProfile profile) {
        FtcDrives.MecanumWiringConfig wiring = profile.drive.wiring;
        return new MecanumConstants()
                .maxPower(1.0)
                .useBrakeModeInTeleOp(profile.drive.zeroPowerBrake)
                .leftFrontMotorName(wiring.frontLeftName)
                .leftRearMotorName(wiring.backLeftName)
                .rightFrontMotorName(wiring.frontRightName)
                .rightRearMotorName(wiring.backRightName)
                .leftFrontMotorDirection(toFtcDirection(
                        wiring.frontLeftDirection,
                        "PhoenixProfile.drive.wiring.frontLeftDirection"
                ))
                .leftRearMotorDirection(toFtcDirection(
                        wiring.backLeftDirection,
                        "PhoenixProfile.drive.wiring.backLeftDirection"
                ))
                .rightFrontMotorDirection(toFtcDirection(
                        wiring.frontRightDirection,
                        "PhoenixProfile.drive.wiring.frontRightDirection"
                ))
                .rightRearMotorDirection(toFtcDirection(
                        wiring.backRightDirection,
                        "PhoenixProfile.drive.wiring.backRightDirection"
                ));
    }

    /** Profile-owned Pinpoint calibration translated only for Pedro's standalone native tools. */
    static PinpointConstants pinpointConstantsFrom(PhoenixProfile profile) {
        PinpointOdometryPredictor.Config source = profile.localization.predictor;
        PinpointConstants target = new PinpointConstants()
                .forwardPodY(source.forwardPodOffsetLeftInches)
                .strafePodX(source.strafePodOffsetForwardInches)
                .distanceUnit(DistanceUnit.INCH)
                .hardwareMapName(source.hardwareMapName)
                .forwardEncoderDirection(source.forwardPodDirection)
                .strafeEncoderDirection(source.strafePodDirection);

        if (source.customEncoderResolutionTicksPerInch != null) {
            target.customEncoderResolution(source.customEncoderResolutionTicksPerInch);
        } else {
            target.encoderResolution(source.encoderPods);
        }
        if (source.yawScalar != null) {
            target.yawScalar(source.yawScalar);
        }
        return target;
    }

    /** Fail before acquiring hardware when the physical profile cannot name one coherent graph. */
    static void validatePhysicalConfig(PhoenixProfile profile, boolean productionAuto) {
        Objects.requireNonNull(profile.drive, "PhoenixProfile.drive");
        FtcDrives.MecanumWiringConfig wiring = Objects.requireNonNull(
                profile.drive.wiring,
                "PhoenixProfile.drive.wiring"
        );

        Set<String> motorNames = new HashSet<String>();
        requireUniqueHardwareName(wiring.frontLeftName, "frontLeftName", motorNames);
        requireUniqueHardwareName(wiring.frontRightName, "frontRightName", motorNames);
        requireUniqueHardwareName(wiring.backLeftName, "backLeftName", motorNames);
        requireUniqueHardwareName(wiring.backRightName, "backRightName", motorNames);
        Objects.requireNonNull(wiring.frontLeftDirection,
                "PhoenixProfile.drive.wiring.frontLeftDirection");
        Objects.requireNonNull(wiring.frontRightDirection,
                "PhoenixProfile.drive.wiring.frontRightDirection");
        Objects.requireNonNull(wiring.backLeftDirection,
                "PhoenixProfile.drive.wiring.backLeftDirection");
        Objects.requireNonNull(wiring.backRightDirection,
                "PhoenixProfile.drive.wiring.backRightDirection");

        Objects.requireNonNull(profile.localization, "PhoenixProfile.localization");
        PinpointOdometryPredictor.Config predictor = Objects.requireNonNull(
                profile.localization.predictor,
                "PhoenixProfile.localization.predictor"
        );
        requireHardwareName(
                predictor.hardwareMapName,
                "PhoenixProfile.localization.predictor.hardwareMapName"
        );
        requireFinite(predictor.forwardPodOffsetLeftInches,
                "PhoenixProfile.localization.predictor.forwardPodOffsetLeftInches");
        requireFinite(predictor.strafePodOffsetForwardInches,
                "PhoenixProfile.localization.predictor.strafePodOffsetForwardInches");
        requireFinite(predictor.quality, "PhoenixProfile.localization.predictor.quality");
        if (predictor.quality < 0.0 || predictor.quality > 1.0) {
            throw new IllegalArgumentException(
                    "PhoenixProfile.localization.predictor.quality must be in [0, 1]"
            );
        }
        if (predictor.resetWaitMs < 0L) {
            throw new IllegalArgumentException(
                    "PhoenixProfile.localization.predictor.resetWaitMs must be >= 0"
            );
        }
        if (productionAuto && !predictor.enableResetOnInit) {
            throw new IllegalArgumentException(
                    "Phoenix Pedro Auto requires "
                            + "PhoenixProfile.localization.predictor.enableResetOnInit=true so the "
                            + "single Pinpoint owner performs the controlled INIT reset"
            );
        }
        Objects.requireNonNull(predictor.forwardPodDirection,
                "PhoenixProfile.localization.predictor.forwardPodDirection");
        Objects.requireNonNull(predictor.strafePodDirection,
                "PhoenixProfile.localization.predictor.strafePodDirection");
        if (predictor.customEncoderResolutionTicksPerInch != null) {
            requireFinitePositive(
                    predictor.customEncoderResolutionTicksPerInch,
                    "PhoenixProfile.localization.predictor.customEncoderResolutionTicksPerInch"
            );
        } else {
            Objects.requireNonNull(
                    predictor.encoderPods,
                    "PhoenixProfile.localization.predictor.encoderPods"
            );
        }
        if (predictor.yawScalar != null) {
            requireFinitePositive(
                    predictor.yawScalar,
                    "PhoenixProfile.localization.predictor.yawScalar"
            );
        }
    }

    private static void requireUniqueHardwareName(String name,
                                                  String fieldName,
                                                  Set<String> usedNames) {
        requireHardwareName(name, "PhoenixProfile.drive.wiring." + fieldName);
        if (!usedNames.add(name)) {
            throw new IllegalArgumentException(
                    "PhoenixProfile.drive.wiring." + fieldName
                            + " duplicates motor hardware name '" + name + "'"
            );
        }
    }

    private static void requireHardwareName(String name, String fieldName) {
        if (name == null || name.trim().isEmpty()) {
            throw new IllegalArgumentException(fieldName + " must be a non-blank hardware name");
        }
    }

    private static void requireFinite(double value, String fieldName) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(fieldName + " must be finite, got " + value);
        }
    }

    private static void requireFinitePositive(double value, String fieldName) {
        requireFinite(value, fieldName);
        if (value <= 0.0) {
            throw new IllegalArgumentException(fieldName + " must be > 0, got " + value);
        }
    }

    private static DcMotorSimple.Direction toFtcDirection(Direction direction, String fieldName) {
        Direction required = Objects.requireNonNull(direction, fieldName);
        return required == Direction.REVERSE
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD;
    }
}
