package edu.ftcphoenix.fw.integrations.pedro;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;

import edu.ftcphoenix.fw.ftc.localization.PinpointOdometryPredictor;
import edu.ftcphoenix.fw.localization.MotionPredictor;

/**
 * Production Pedro Auto runtime with one drivetrain writer and one Pinpoint hardware owner.
 *
 * <p>Robot-specific code supplies configuration, then uses three short operations:</p>
 * <pre>{@code
 * PedroPathingRuntime runtime = PedroPathingRuntime.create(...);
 * robot.initAuto(runtime.driveAdapter(), runtime.motionPredictor());
 * runtime.setStartingPose(paths.pedroStartPose);
 * }</pre>
 *
 * <p>The supplied {@link PinpointOdometryPredictor} remains the only object that initializes,
 * polls, resets, or rebases Pinpoint. Pedro receives a passive same-cycle view of its snapshots.
 * The returned {@link PedroPathingDriveAdapter} remains the only Follower heartbeat and drivetrain
 * stop owner.</p>
 */
public final class PedroPathingRuntime {

    private final PinpointOdometryPredictor motionPredictor;
    private final PedroPathingPassiveLocalizer passiveLocalizer;
    private final Follower follower;
    private final PedroPathingDriveAdapter driveAdapter;
    private final PathConstraints pathBuilderDefaults;

    private PedroPathingRuntime(PinpointOdometryPredictor motionPredictor,
                                PedroPathingPassiveLocalizer passiveLocalizer,
                                Follower follower,
                                PedroPathingDriveAdapter driveAdapter,
                                PathConstraints pathBuilderDefaults) {
        this.motionPredictor = motionPredictor;
        this.passiveLocalizer = passiveLocalizer;
        this.follower = follower;
        this.driveAdapter = driveAdapter;
        this.pathBuilderDefaults = pathBuilderDefaults;
    }

    /**
     * Builds and validates the complete production Pedro graph.
     *
     * <p>The pinned 2.1.2 {@code FollowerBuilder.pathConstraints(...)} method mutates Pedro's
     * process-wide default constraints. This boundary intentionally invokes Follower's equivalent
     * public four-argument constructor after validation, containing that vendor global-state trap
     * while retaining the exact configured drivetrain, localizer, and constraints.</p>
     *
     * <p>The four mecanum motor names must also be nonblank and distinct under the FTC SDK's
     * trimmed, case-sensitive lookup identity. That complete configuration check finishes before
     * Pedro constructs or configures drivetrain hardware.</p>
     *
     * @param hardwareMap FTC hardware registry used only to construct the native Pedro drivetrain
     * @param motionPredictor already-created sole Pinpoint owner with controlled INIT reset enabled
     * @param followerConstants Pedro controller/follower tuning
     * @param mecanumConstants Pedro drivetrain tuning plus physical motor wiring
     * @param pathConstraints Pedro route completion/braking constraints
     * @param fieldTransform explicit named Phoenix/Pedro field convention
     * @return validated production runtime
     */
    public static PedroPathingRuntime create(HardwareMap hardwareMap,
                                             PinpointOdometryPredictor motionPredictor,
                                             FollowerConstants followerConstants,
                                             MecanumConstants mecanumConstants,
                                             PathConstraints pathConstraints,
                                             PedroFieldTransform fieldTransform) {
        HardwareMap requiredHardwareMap = Objects.requireNonNull(hardwareMap, "hardwareMap");
        PinpointOdometryPredictor requiredPredictor = Objects.requireNonNull(
                motionPredictor,
                "motionPredictor"
        );
        FollowerConstants requiredFollowerConstants = Objects.requireNonNull(
                followerConstants,
                "followerConstants"
        );
        MecanumConstants requiredMecanumConstants = Objects.requireNonNull(
                mecanumConstants,
                "mecanumConstants"
        );
        PathConstraints requiredPathConstraints = Objects.requireNonNull(
                pathConstraints,
                "pathConstraints"
        );
        PedroFieldTransform requiredFieldTransform = Objects.requireNonNull(
                fieldTransform,
                "fieldTransform"
        );

        validatePredictorConfig(requiredPredictor.config());
        validateFollowerConstants(requiredFollowerConstants);
        validateMecanumConstants(requiredMecanumConstants);
        validatePathConstraints(requiredPathConstraints);

        PedroPathingPassiveLocalizer passiveLocalizer =
                new PedroPathingPassiveLocalizer(requiredPredictor, requiredFieldTransform);
        PathConstraints followerPathConstraints = requiredPathConstraints.copy();
        PathConstraints pathBuilderDefaults = requiredPathConstraints.copy();
        Mecanum drivetrain = null;
        try {
            drivetrain = new Mecanum(requiredHardwareMap, requiredMecanumConstants);
            Follower follower = new Follower(
                    requiredFollowerConstants,
                    passiveLocalizer,
                    drivetrain,
                    followerPathConstraints
            );
            passiveLocalizer.completeFollowerConstruction();

            PedroPathingDriveAdapter adapter = new PedroPathingDriveAdapter(
                    follower,
                    passiveLocalizer::prepareForHeartbeat
            );
            return new PedroPathingRuntime(
                    requiredPredictor,
                    passiveLocalizer,
                    follower,
                    adapter,
                    pathBuilderDefaults
            );
        } catch (RuntimeException constructionFailure) {
            if (drivetrain != null) {
                try {
                    drivetrain.breakFollowing();
                } catch (RuntimeException stopFailure) {
                    if (stopFailure != constructionFailure) {
                        constructionFailure.addSuppressed(stopFailure);
                    }
                }
            }
            throw new IllegalStateException(
                    "Pedro production runtime construction failed for field transform '"
                            + requiredFieldTransform.name() + "' and motors ["
                            + requiredMecanumConstants.leftFrontMotorName + ", "
                            + requiredMecanumConstants.leftRearMotorName + ", "
                            + requiredMecanumConstants.rightFrontMotorName + ", "
                            + requiredMecanumConstants.rightRearMotorName
                            + "]; check the configured voltage sensor, motor hardware names, "
                            + "directions, and Pedro tuning. Cause: "
                            + failureSummary(constructionFailure),
                    constructionFailure
            );
        }
    }

    /** Returns the exact sole Pinpoint owner supplied at construction. */
    public MotionPredictor motionPredictor() {
        return motionPredictor;
    }

    /** Returns the sole owned Follower heartbeat and drivetrain-stop adapter. */
    public PedroPathingDriveAdapter driveAdapter() {
        return driveAdapter;
    }

    /**
     * Starts a Pedro path builder with this runtime's validated path-constraint defaults.
     *
     * <p>Pinned Pedro 2.1.2's no-argument {@code Follower.pathBuilder()} reads a process-wide
     * mutable default rather than the constraints supplied to the Follower constructor. This
     * runtime method passes an independent copy explicitly and repairs the pinned builder's
     * {@code build()} behavior, which otherwise replaces every path's constraints with that same
     * global default. The workaround remains inside this vendor boundary, preserving configured
     * defaults and per-path overrides without global state or cross-path mutation.</p>
     */
    public PathBuilder pathBuilder() {
        return newPathBuilder(follower, pathBuilderDefaults);
    }

    /** Version-pinned helper kept package-private for pure integration verification. */
    static PathBuilder newPathBuilder(Follower follower, PathConstraints defaults) {
        return new IsolatedPathBuilder(
                Objects.requireNonNull(follower, "follower"),
                Objects.requireNonNull(defaults, "pathBuilderDefaults").copy()
        );
    }

    /**
     * Contains two pinned Pedro 2.1.2 constraint defects without adding a student-facing builder.
     *
     * <p>Pedro's no-argument builder reads a mutable process-wide default, and its
     * {@link PathBuilder#build()} path-chain constructor reapplies that global default after the
     * builder has already copied the explicitly supplied constraints into each path. This subclass
     * snapshots the builder's real per-path state, lets Pedro assemble callbacks and interpolation,
     * then restores the intended constraints. Fluent Pedro calls still return {@link PathBuilder}
     * and dispatch back to this override.</p>
     */
    private static final class IsolatedPathBuilder extends PathBuilder {
        private final List<Path> addedPaths = new ArrayList<Path>();
        private PathConstraints currentConstraints;

        IsolatedPathBuilder(Follower follower, PathConstraints ownedDefaults) {
            super(follower, ownedDefaults);
            currentConstraints = ownedDefaults;
        }

        @Override
        public PathBuilder addPath(Path path) {
            Path requiredPath = Objects.requireNonNull(path, "path");
            super.addPath(requiredPath);
            addedPaths.add(requiredPath);
            return this;
        }

        @Override
        public PathBuilder addPath(Curve curve) {
            return addPath(new Path(
                    Objects.requireNonNull(curve, "curve"),
                    currentConstraints
            ));
        }

        @Override
        public PathBuilder addPaths(Path... paths) {
            Objects.requireNonNull(paths, "paths");
            for (Path path : paths) {
                addPath(path);
            }
            return this;
        }

        @Override
        public PathBuilder addPaths(Curve... curves) {
            Objects.requireNonNull(curves, "curves");
            for (Curve curve : curves) {
                addPath(curve);
            }
            return this;
        }

        @Override
        public PathBuilder setConstraints(PathConstraints constraints) {
            currentConstraints = copiedConstraints(constraints);
            super.setConstraints(currentConstraints);
            return this;
        }

        @Override
        public PathBuilder setConstraintsForAll(PathConstraints constraints) {
            currentConstraints = copiedConstraints(constraints);
            super.setConstraintsForAll(currentConstraints);
            return this;
        }

        @Override
        public PathBuilder setConstraintsForLast(PathConstraints constraints) {
            currentConstraints = copiedConstraints(constraints);
            super.setConstraintsForLast(currentConstraints);
            return this;
        }

        @Override
        public PathChain build() {
            List<PathConstraints> intendedConstraints =
                    new ArrayList<PathConstraints>(addedPaths.size());
            for (Path path : addedPaths) {
                intendedConstraints.add(path.getConstraints().copy());
            }
            double brakingStart = currentConstraints.getBrakingStart();

            PathChain pathChain = super.build();
            if (pathChain.size() != addedPaths.size()) {
                throw new IllegalStateException(
                        "Pinned Pedro PathBuilder behavior changed: tracked "
                                + addedPaths.size() + " paths but build returned "
                                + pathChain.size()
                );
            }
            for (int i = 0; i < addedPaths.size(); i++) {
                Path path = pathChain.getPath(i);
                if (path != addedPaths.get(i)) {
                    throw new IllegalStateException(
                            "Pinned Pedro PathBuilder behavior changed: build replaced path " + i
                    );
                }
                path.setConstraints(intendedConstraints.get(i));
                path.setBrakingStart(brakingStart);
            }
            return pathChain;
        }

        private static PathConstraints copiedConstraints(PathConstraints constraints) {
            return Objects.requireNonNull(constraints, "constraints").copy();
        }
    }

    /**
     * Returns the same wrapped Follower for read-only advanced inspection.
     *
     * <p>Use {@link #pathBuilder()} for route construction so the runtime's checked constraints are
     * applied without Pedro global state. Do not call the Follower's update, drive, reset, or stop
     * lifecycle methods directly; route and drive lifecycle must go through
     * {@link #driveAdapter()}.</p>
     */
    public Follower follower() {
        return follower;
    }

    /**
     * Sets the coordinated Pedro/Phoenix starting pose before the first drive heartbeat.
     *
     * @param pedroStartPose pose explicitly expressed in Pedro field coordinates
     */
    public void setStartingPose(Pose pedroStartPose) {
        Pose requiredPose = Objects.requireNonNull(pedroStartPose, "pedroStartPose");
        passiveLocalizer.requireStartingPoseAllowed(requiredPose);
        follower.setStartingPose(requiredPose);
    }

    static void validatePredictorConfig(PinpointOdometryPredictor.Config config) {
        PinpointOdometryPredictor.Config value = Objects.requireNonNull(
                config,
                "motionPredictor.config()"
        );
        requireHardwareName(value.hardwareMapName, "motionPredictor.config().hardwareMapName");
        if (!value.enableResetOnInit) {
            throw new IllegalArgumentException(
                    "Pedro production Auto requires motionPredictor.config().enableResetOnInit=true "
                            + "so Phoenix performs the one controlled Pinpoint INIT reset before "
                            + "Pedro consumes its constructor reset"
            );
        }
        if (value.resetWaitMs < 0L) {
            throw new IllegalArgumentException(
                    "motionPredictor.config().resetWaitMs must be >= 0, got " + value.resetWaitMs
            );
        }
        requireFinite(value.forwardPodOffsetLeftInches,
                "motionPredictor.config().forwardPodOffsetLeftInches");
        requireFinite(value.strafePodOffsetForwardInches,
                "motionPredictor.config().strafePodOffsetForwardInches");
        requireRange(value.quality, 0.0, 1.0, "motionPredictor.config().quality");
        Objects.requireNonNull(
                value.forwardPodDirection,
                "motionPredictor.config().forwardPodDirection"
        );
        Objects.requireNonNull(
                value.strafePodDirection,
                "motionPredictor.config().strafePodDirection"
        );
        if (value.customEncoderResolutionTicksPerInch != null) {
            requirePositive(
                    value.customEncoderResolutionTicksPerInch,
                    "motionPredictor.config().customEncoderResolutionTicksPerInch"
            );
        } else {
            Objects.requireNonNull(value.encoderPods, "motionPredictor.config().encoderPods");
        }
        if (value.yawScalar != null) {
            requirePositive(value.yawScalar, "motionPredictor.config().yawScalar");
        }
    }

    /**
     * Validate Pedro drivetrain wiring and tuning before vendor hardware construction.
     */
    static void validateMecanumConstants(MecanumConstants constants) {
        MecanumConstants value = Objects.requireNonNull(constants, "mecanumConstants");
        Set<String> motorNames = new HashSet<String>();
        requireUniqueHardwareName(value.leftFrontMotorName,
                "mecanumConstants.leftFrontMotorName", motorNames);
        requireUniqueHardwareName(value.leftRearMotorName,
                "mecanumConstants.leftRearMotorName", motorNames);
        requireUniqueHardwareName(value.rightFrontMotorName,
                "mecanumConstants.rightFrontMotorName", motorNames);
        requireUniqueHardwareName(value.rightRearMotorName,
                "mecanumConstants.rightRearMotorName", motorNames);

        Objects.requireNonNull(value.leftFrontMotorDirection,
                "mecanumConstants.leftFrontMotorDirection");
        Objects.requireNonNull(value.leftRearMotorDirection,
                "mecanumConstants.leftRearMotorDirection");
        Objects.requireNonNull(value.rightFrontMotorDirection,
                "mecanumConstants.rightFrontMotorDirection");
        Objects.requireNonNull(value.rightRearMotorDirection,
                "mecanumConstants.rightRearMotorDirection");

        requirePositive(value.xVelocity, "mecanumConstants.xVelocity");
        requirePositive(value.yVelocity, "mecanumConstants.yVelocity");
        requirePositiveAtMostOne(value.maxPower, "mecanumConstants.maxPower");
        requireRange(value.motorCachingThreshold, 0.0, 1.0,
                "mecanumConstants.motorCachingThreshold");
        requirePositive(value.nominalVoltage, "mecanumConstants.nominalVoltage");
        requireNonNegative(value.staticFrictionCoefficient,
                "mecanumConstants.staticFrictionCoefficient");
        if (value.staticFrictionCoefficient >= 1.0) {
            throw new IllegalArgumentException(
                    "mecanumConstants.staticFrictionCoefficient must be < 1, got "
                            + value.staticFrictionCoefficient
            );
        }

        Vector frontLeftVector = Objects.requireNonNull(
                value.frontLeftVector,
                "mecanumConstants.frontLeftVector"
        );
        requireFinite(frontLeftVector.getXComponent(),
                "mecanumConstants.frontLeftVector.x");
        requireFinite(frontLeftVector.getYComponent(),
                "mecanumConstants.frontLeftVector.y");
        requirePositive(frontLeftVector.getMagnitude(),
                "mecanumConstants.frontLeftVector.magnitude");
    }

    static void validatePathConstraints(PathConstraints constraints) {
        PathConstraints value = Objects.requireNonNull(constraints, "pathConstraints");
        requirePositiveAtMostOne(value.getTValueConstraint(),
                "pathConstraints.tValueConstraint");
        requireNonNegative(value.getVelocityConstraint(),
                "pathConstraints.velocityConstraint");
        requireNonNegative(value.getTranslationalConstraint(),
                "pathConstraints.translationalConstraint");
        requireNonNegative(value.getHeadingConstraint(),
                "pathConstraints.headingConstraint");
        requireNonNegative(value.getTimeoutConstraint(),
                "pathConstraints.timeoutConstraint");
        requirePositive(value.getBrakingStrength(), "pathConstraints.brakingStrength");
        requireNonNegative(value.getBrakingStart(), "pathConstraints.brakingStart");
        if (value.getBEZIER_CURVE_SEARCH_LIMIT() <= 0) {
            throw new IllegalArgumentException(
                    "pathConstraints.BEZIER_CURVE_SEARCH_LIMIT must be > 0, got "
                            + value.getBEZIER_CURVE_SEARCH_LIMIT()
            );
        }
    }

    static void validateFollowerConstants(FollowerConstants constants) {
        FollowerConstants value = Objects.requireNonNull(constants, "followerConstants");
        validatePid(value.coefficientsTranslationalPIDF,
                "followerConstants.coefficientsTranslationalPIDF");
        validatePid(value.integralTranslational,
                "followerConstants.integralTranslational");
        validatePid(value.coefficientsHeadingPIDF,
                "followerConstants.coefficientsHeadingPIDF");
        validateFilteredPid(value.coefficientsDrivePIDF,
                "followerConstants.coefficientsDrivePIDF");
        validatePid(value.coefficientsSecondaryTranslationalPIDF,
                "followerConstants.coefficientsSecondaryTranslationalPIDF");
        validatePid(value.integralSecondaryTranslational,
                "followerConstants.integralSecondaryTranslational");
        validatePid(value.coefficientsSecondaryHeadingPIDF,
                "followerConstants.coefficientsSecondaryHeadingPIDF");
        validateFilteredPid(value.coefficientsSecondaryDrivePIDF,
                "followerConstants.coefficientsSecondaryDrivePIDF");
        validatePredictiveBraking(value.predictiveBrakingCoefficients,
                "followerConstants.predictiveBrakingCoefficients");

        requireFinite(value.headingPIDFSwitch, "followerConstants.headingPIDFSwitch");
        requireFinite(value.drivePIDFSwitch, "followerConstants.drivePIDFSwitch");
        requireFinite(value.holdPointTranslationalScaling,
                "followerConstants.holdPointTranslationalScaling");
        requireFinite(value.holdPointHeadingScaling,
                "followerConstants.holdPointHeadingScaling");
        requireFinite(value.translationalPIDFSwitch,
                "followerConstants.translationalPIDFSwitch");
        requireFinite(value.turnHeadingErrorThreshold,
                "followerConstants.turnHeadingErrorThreshold");
        requireFinite(value.centripetalScaling, "followerConstants.centripetalScaling");
        requirePositive(value.mass, "followerConstants.mass");
        requireFinite(value.forwardZeroPowerAcceleration,
                "followerConstants.forwardZeroPowerAcceleration");
        requireFinite(value.lateralZeroPowerAcceleration,
                "followerConstants.lateralZeroPowerAcceleration");
        requirePositive(value.driveKalmanFilterModelCovariance,
                "followerConstants.driveKalmanFilterModelCovariance");
        requirePositive(value.driveKalmanFilterDataCovariance,
                "followerConstants.driveKalmanFilterDataCovariance");
        requireNonNegative(value.stuckVelocity, "followerConstants.stuckVelocity");
        requireRange(value.stuckTValueLow, 0.0, 1.0,
                "followerConstants.stuckTValueLow");
        requireRange(value.stuckTValueHigh, 0.0, 1.0,
                "followerConstants.stuckTValueHigh");
        if (value.stuckTValueLow >= value.stuckTValueHigh) {
            throw new IllegalArgumentException(
                    "followerConstants.stuckTValueLow must be less than stuckTValueHigh"
            );
        }
        requireNonNegative(value.stuckTimeout, "followerConstants.stuckTimeout");
        if (value.BEZIER_CURVE_SEARCH_LIMIT <= 0) {
            throw new IllegalArgumentException(
                    "followerConstants.BEZIER_CURVE_SEARCH_LIMIT must be > 0, got "
                            + value.BEZIER_CURVE_SEARCH_LIMIT
            );
        }
    }

    private static void validatePid(PIDFCoefficients coefficients, String name) {
        PIDFCoefficients value = Objects.requireNonNull(coefficients, name);
        requireFinite(value.P, name + ".P");
        requireFinite(value.I, name + ".I");
        requireFinite(value.D, name + ".D");
        requireFinite(value.F, name + ".F");
    }

    private static void validateFilteredPid(FilteredPIDFCoefficients coefficients, String name) {
        FilteredPIDFCoefficients value = Objects.requireNonNull(coefficients, name);
        requireFinite(value.P, name + ".P");
        requireFinite(value.I, name + ".I");
        requireFinite(value.D, name + ".D");
        requireFinite(value.F, name + ".F");
        requireFinite(value.T, name + ".T");
    }

    private static void validatePredictiveBraking(PredictiveBrakingCoefficients coefficients,
                                                   String name) {
        PredictiveBrakingCoefficients value = Objects.requireNonNull(coefficients, name);
        requireFinite(value.P, name + ".P");
        requireFinite(value.kLinearBraking, name + ".kLinearBraking");
        requireFinite(value.kQuadraticFriction, name + ".kQuadraticFriction");
        requirePositiveAtMostOne(value.maximumBrakingPower,
                name + ".maximumBrakingPower");
    }

    private static void requireUniqueHardwareName(String name,
                                                  String fieldName,
                                                  Set<String> usedNames) {
        requireHardwareName(name, fieldName);
        String effectiveName = name.trim();
        if (!usedNames.add(effectiveName)) {
            throw new IllegalArgumentException(
                    fieldName + " duplicates motor hardware name after FTC trimming: raw='"
                            + name + "', effective='" + effectiveName
                            + "'; use a distinct configured hardware name"
            );
        }
    }

    private static void requireHardwareName(String value, String name) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(name + " must be a non-blank hardware name");
        }
    }

    private static void requireNonNegative(double value, String name) {
        requireFinite(value, name);
        if (value < 0.0) {
            throw new IllegalArgumentException(name + " must be >= 0, got " + value);
        }
    }

    private static void requirePositive(double value, String name) {
        requireFinite(value, name);
        if (value <= 0.0) {
            throw new IllegalArgumentException(name + " must be > 0, got " + value);
        }
    }

    private static void requirePositiveAtMostOne(double value, String name) {
        requireFinite(value, name);
        if (value <= 0.0 || value > 1.0) {
            throw new IllegalArgumentException(name + " must be in (0, 1], got " + value);
        }
    }

    private static void requireRange(double value, double min, double max, String name) {
        requireFinite(value, name);
        if (value < min || value > max) {
            throw new IllegalArgumentException(
                    name + " must be in [" + min + ", " + max + "], got " + value
            );
        }
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
    }

    private static String failureSummary(RuntimeException failure) {
        String message = failure.getMessage();
        return message == null || message.trim().isEmpty()
                ? failure.getClass().getSimpleName()
                : message;
    }
}
