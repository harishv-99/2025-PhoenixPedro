package edu.ftcphoenix.fw.integrations.pedro;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.drivetrain.Drivetrain;
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
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.ftc.localization.PinpointOdometryPredictor;
import edu.ftcphoenix.fw.localization.MotionPredictor;

/**
 * Production Pedro Auto runtime with one drivetrain writer and one Pinpoint hardware owner.
 *
 * <p>The following short form documents production Phoenix managed Auto:</p>
 * <pre>{@code
 * PhoenixProfile profile = PhoenixProfile.current();
 * PedroPathingRuntime runtime = PedroPathingRuntime.create(...);
 * robot.declareAuto(
 *         program,
 *         profile,
 *         runtime.driveAdapter(),
 *         runtime.motionPredictor(),
 *         frozenEligibleTagIds,
 *         BooleanSource.constant(true),
 *         BooleanSource.constant(false),
 *         () -&gt; runtime.setStartingPose(frozenPedroStartPose())
 * );
 * program.rootTask(rootRoutine);
 * }</pre>
 *
 * <p>The runtime-created {@link PinpointOdometryPredictor} is the only object that initializes,
 * polls, resets, or rebases Pinpoint. Pedro receives a passive same-cycle view of its snapshots.
 * The returned {@link PedroPathingDriveAdapter} remains the only Follower heartbeat and drivetrain
 * stop owner.</p>
 */
public final class PedroPathingRuntime {

    private static final String CONFIG_CONTEXT = Config.class.getCanonicalName();
    private static final double MIN_NORMALIZED_WHEEL_BASIS_DETERMINANT =
            64.0 * Math.ulp(4.0 * Math.PI);

    /** Package-private construction seam used only for effect-order verification. */
    interface ConstructionFactory {
        Drivetrain createDrivetrain(HardwareMap hardwareMap, MecanumConstants constants);

        PinpointOdometryPredictor createPredictor(
                HardwareMap hardwareMap,
                PinpointOdometryPredictor.Config config
        );

        Follower createFollower(FollowerConstants constants,
                                PedroPathingPassiveLocalizer localizer,
                                Drivetrain drivetrain,
                                PathConstraints constraints);
    }

    private static final ConstructionFactory PRODUCTION_CONSTRUCTION =
            new ConstructionFactory() {
                @Override
                public Drivetrain createDrivetrain(HardwareMap hardwareMap,
                                                   MecanumConstants constants) {
                    return new Mecanum(hardwareMap, constants);
                }

                @Override
                public PinpointOdometryPredictor createPredictor(
                        HardwareMap hardwareMap,
                        PinpointOdometryPredictor.Config config) {
                    return new PinpointOdometryPredictor(hardwareMap, config);
                }

                @Override
                public Follower createFollower(FollowerConstants constants,
                                               PedroPathingPassiveLocalizer localizer,
                                               Drivetrain drivetrain,
                                               PathConstraints constraints) {
                    return new Follower(constants, localizer, drivetrain, constraints);
                }
            };

    /**
     * Mutable, data-only authoring configuration for one production Pedro runtime.
     *
     * <p>{@link #defaults()} returns a valid software baseline, not reviewed drivetrain wiring,
     * Pinpoint placement, follower tuning, field alignment, or route safety. The runtime takes a
     * complete deep snapshot before any hardware lookup or vendor-global mutation.</p>
     */
    public static final class Config {

        /** Pinpoint hardware, pod placement, encoder, and evidence configuration. */
        public PinpointOdometryPredictor.Config predictor;

        /** Pedro follower/controller tuning. */
        public FollowerConstants followerConstants;

        /** Pedro mecanum wiring and drivetrain tuning. */
        public MecanumConstants mecanumConstants;

        /** Pedro path completion and braking constraints. */
        public PathConstraints pathConstraints;

        /** Explicit immutable conversion between Phoenix and Pedro field coordinates. */
        public PedroFieldTransform fieldTransform;

        private Config() {
        }

        /**
         * Returns a fresh mutable software-baseline draft.
         *
         * <p>The path constraints are authored directly and never read Pedro's mutable
         * {@link PathConstraints#defaultConstraints} process global.</p>
         */
        public static Config defaults() {
            Config c = new Config();
            c.predictor = PinpointOdometryPredictor.Config.defaults();
            c.followerConstants = new FollowerConstants();
            c.mecanumConstants = new MecanumConstants();
            c.pathConstraints = new PathConstraints(
                    0.995,
                    0.1,
                    0.1,
                    0.007,
                    100.0,
                    1.0,
                    10,
                    1.0
            );
            c.fieldTransform = PedroFieldTransform.decodeInvertedFtc();
            return c;
        }

        /**
         * Returns a raw deep copy without validating or activating this draft.
         *
         * <p>Null fields, including null nested Pedro coefficient objects, are deliberately
         * preserved so copying malformed authoring data never becomes a hidden effect boundary.</p>
         */
        public Config copy() {
            Config c = new Config();
            c.predictor = predictor == null ? null : predictor.copy();
            c.followerConstants = copyFollowerConstants(followerConstants);
            c.mecanumConstants = copyMecanumConstants(mecanumConstants);
            c.pathConstraints = pathConstraints == null ? null : pathConstraints.copy();
            c.fieldTransform = fieldTransform;
            return c;
        }

        /**
         * Takes and validates an independent snapshot without touching hardware or vendor globals.
         *
         * @param context diagnostic owner prefix; null or blank uses this Config's canonical name
         * @return independent validated snapshot
         * @throws NullPointerException for a required null object or nested collaborator
         * @throws IllegalArgumentException for a blank identity, invalid number, or incoherent
         *                                  cross-field combination
         */
        public Config validatedCopy(String context) {
            String owner = normalizedContext(context);
            Config c = copy();
            c.predictor = Objects.requireNonNull(
                    c.predictor,
                    owner + ".predictor must not be null"
            ).validatedCopy(owner + ".predictor");
            c.followerConstants = Objects.requireNonNull(
                    c.followerConstants,
                    owner + ".followerConstants must not be null"
            );
            c.mecanumConstants = Objects.requireNonNull(
                    c.mecanumConstants,
                    owner + ".mecanumConstants must not be null"
            );
            c.pathConstraints = Objects.requireNonNull(
                    c.pathConstraints,
                    owner + ".pathConstraints must not be null"
            );
            c.fieldTransform = Objects.requireNonNull(
                    c.fieldTransform,
                    owner + ".fieldTransform must not be null"
            );

            validateFollowerConstants(c.followerConstants, owner + ".followerConstants");
            validateMecanumConstants(c.mecanumConstants, owner + ".mecanumConstants");
            validatePathConstraints(
                    c.pathConstraints,
                    c.followerConstants,
                    c.mecanumConstants.xVelocity,
                    c.mecanumConstants.yVelocity,
                    owner + ".pathConstraints",
                    owner + ".followerConstants",
                    owner + ".mecanumConstants"
            );
            return c;
        }

        private static String normalizedContext(String context) {
            return context == null || context.trim().isEmpty() ? CONFIG_CONTEXT : context;
        }
    }

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
     * <p>The complete mutable draft is copied and validated before motor lookup, Pinpoint reset,
     * Follower construction, or Pedro global-state mutation. The pinned 2.1.2
     * {@code FollowerBuilder.pathConstraints(...)} method mutates Pedro's process-wide default
     * constraints, so this boundary instead invokes Follower's equivalent public four-argument
     * constructor with two private path-constraint copies.</p>
     *
     * <p>The four mecanum motor names must also be nonblank and distinct under the FTC SDK's
     * trimmed, case-sensitive lookup identity. That complete configuration check finishes before
     * Pedro constructs or configures drivetrain hardware.</p>
     *
     * @param hardwareMap FTC hardware registry for the owned Mecanum and Pinpoint graph
     * @param config complete data-only runtime configuration; it is defensively snapshotted
     * @return validated production runtime
     * @throws NullPointerException when the map, Config, or a required nested object is null
     * @throws IllegalArgumentException when an authored identity, number, or cross-field
     *                                  combination is invalid
     * @throws IllegalStateException when a valid configuration reaches an SDK/vendor construction
     *                               failure; a returned drivetrain is best-effort stopped and a
     *                               distinct cleanup failure is suppressed on the original cause
     */
    public static PedroPathingRuntime create(HardwareMap hardwareMap,
                                             Config config) {
        return createOwnedRuntime(hardwareMap, config, PRODUCTION_CONSTRUCTION);
    }

    /** Uses the production pipeline with injectable constructors; never a robot construction API. */
    static PedroPathingRuntime createForTest(HardwareMap hardwareMap,
                                             Config config,
                                             ConstructionFactory constructionFactory) {
        return createOwnedRuntime(hardwareMap, config, constructionFactory);
    }

    private static PedroPathingRuntime createOwnedRuntime(
            HardwareMap hardwareMap,
            Config config,
            ConstructionFactory constructionFactory) {
        HardwareMap requiredHardwareMap = Objects.requireNonNull(
                hardwareMap,
                "PedroPathingRuntime.create(hardwareMap, config) requires a non-null HardwareMap"
        );
        Config owned = Objects.requireNonNull(
                config,
                CONFIG_CONTEXT + " must not be null"
        ).validatedCopy(CONFIG_CONTEXT);
        ConstructionFactory factory = Objects.requireNonNull(
                constructionFactory,
                "constructionFactory"
        );

        PathConstraints followerPathConstraints = owned.pathConstraints.copy();
        PathConstraints pathBuilderDefaults = owned.pathConstraints.copy();
        Drivetrain drivetrain = null;
        try {
            drivetrain = Objects.requireNonNull(
                    factory.createDrivetrain(requiredHardwareMap, owned.mecanumConstants),
                    "Pedro construction factory returned a null drivetrain"
            );
            PinpointOdometryPredictor motionPredictor = Objects.requireNonNull(
                    factory.createPredictor(requiredHardwareMap, owned.predictor),
                    "Pedro construction factory returned a null Pinpoint predictor"
            );
            PedroPathingPassiveLocalizer passiveLocalizer =
                    new PedroPathingPassiveLocalizer(motionPredictor, owned.fieldTransform);
            Follower follower = Objects.requireNonNull(
                    factory.createFollower(
                            owned.followerConstants,
                            passiveLocalizer,
                            drivetrain,
                            followerPathConstraints
                    ),
                    "Pedro construction factory returned a null Follower"
            );
            passiveLocalizer.completeFollowerConstruction();

            PedroPathingDriveAdapter adapter = new PedroPathingDriveAdapter(
                    follower,
                    passiveLocalizer::prepareForHeartbeat
            );
            return new PedroPathingRuntime(
                    motionPredictor,
                    passiveLocalizer,
                    follower,
                    adapter,
                    pathBuilderDefaults
            );
        } catch (RuntimeException constructionFailure) {
            final Drivetrain drivetrainToStop = drivetrain;
            if (drivetrainToStop != null) {
                CleanupActions.attemptAllAfterFailure(
                        constructionFailure,
                        drivetrainToStop::breakFollowing
                );
            }
            throw new IllegalStateException(
                    "Pedro production runtime construction failed for field transform '"
                            + owned.fieldTransform.name() + "' and motors ["
                            + owned.mecanumConstants.leftFrontMotorName + ", "
                            + owned.mecanumConstants.leftRearMotorName + ", "
                            + owned.mecanumConstants.rightFrontMotorName + ", "
                            + owned.mecanumConstants.rightRearMotorName
                            + "]; check the configured voltage sensor, motor hardware names, "
                            + "directions, and Pedro tuning. Cause: "
                            + failureSummary(constructionFailure),
                    constructionFailure
            );
        }
    }

    /** Returns the exact sole Pinpoint owner created and retained by this runtime. */
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
        private final PathValidationContext validationContext;
        private PathConstraints currentConstraints;

        IsolatedPathBuilder(Follower follower, PathConstraints ownedDefaults) {
            this(BuilderInputs.capture(follower, ownedDefaults));
        }

        private IsolatedPathBuilder(BuilderInputs inputs) {
            super(inputs.follower, inputs.defaults);
            validationContext = inputs.validationContext;
            currentConstraints = inputs.defaults;
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
            currentConstraints = validatedConstraints(constraints);
            super.setConstraints(currentConstraints);
            return this;
        }

        @Override
        public PathBuilder setConstraintsForAll(PathConstraints constraints) {
            currentConstraints = validatedConstraints(constraints);
            super.setConstraintsForAll(currentConstraints);
            return this;
        }

        @Override
        public PathBuilder setConstraintsForLast(PathConstraints constraints) {
            currentConstraints = validatedConstraints(constraints);
            super.setConstraintsForLast(currentConstraints);
            return this;
        }

        @Override
        public PathBuilder setBrakingStrength(double brakingStrength) {
            PathConstraints candidate = lastPathConstraintsCopy();
            candidate.setBrakingStrength(brakingStrength);
            validationContext.validate(candidate);
            super.setBrakingStrength(brakingStrength);
            return this;
        }

        @Override
        public PathBuilder setBrakingStart(double brakingStart) {
            PathConstraints candidate = currentConstraints.copy();
            candidate.setBrakingStart(brakingStart);
            validationContext.validate(candidate);
            super.setBrakingStart(brakingStart);
            return this;
        }

        @Override
        public PathBuilder setGlobalDeceleration(double brakingStart) {
            PathConstraints candidate = currentConstraints.copy();
            candidate.setBrakingStart(brakingStart);
            validationContext.validate(candidate);
            super.setGlobalDeceleration(brakingStart);
            return this;
        }

        @Override
        public PathBuilder setVelocityConstraint(double velocityConstraint) {
            PathConstraints candidate = lastPathConstraintsCopy();
            candidate.setVelocityConstraint(velocityConstraint);
            validationContext.validate(candidate);
            super.setVelocityConstraint(velocityConstraint);
            return this;
        }

        @Override
        public PathBuilder setTranslationalConstraint(double translationalConstraint) {
            PathConstraints candidate = lastPathConstraintsCopy();
            candidate.setTranslationalConstraint(translationalConstraint);
            validationContext.validate(candidate);
            super.setTranslationalConstraint(translationalConstraint);
            return this;
        }

        @Override
        public PathBuilder setHeadingConstraint(double headingConstraint) {
            PathConstraints candidate = lastPathConstraintsCopy();
            candidate.setHeadingConstraint(headingConstraint);
            validationContext.validate(candidate);
            super.setHeadingConstraint(headingConstraint);
            return this;
        }

        @Override
        public PathBuilder setTValueConstraint(double tValueConstraint) {
            PathConstraints candidate = lastPathConstraintsCopy();
            candidate.setTValueConstraint(tValueConstraint);
            validationContext.validate(candidate);
            super.setTValueConstraint(tValueConstraint);
            return this;
        }

        @Override
        public PathBuilder setTimeoutConstraint(double timeoutConstraint) {
            PathConstraints candidate = lastPathConstraintsCopy();
            candidate.setTimeoutConstraint(timeoutConstraint);
            validationContext.validate(candidate);
            super.setTimeoutConstraint(timeoutConstraint);
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

        private PathConstraints validatedConstraints(PathConstraints constraints) {
            PathConstraints owned = Objects.requireNonNull(
                    constraints,
                    CONFIG_CONTEXT + ".pathConstraints must not be null"
            ).copy();
            validationContext.validate(owned);
            return owned;
        }

        private PathConstraints lastPathConstraintsCopy() {
            return addedPaths.get(addedPaths.size() - 1).getConstraints().copy();
        }

        /** Captures the private graph before the vendor builder can retain a collaborator. */
        private static final class BuilderInputs {
            final Follower follower;
            final PathConstraints defaults;
            final PathValidationContext validationContext;

            private BuilderInputs(Follower follower,
                                  PathConstraints defaults,
                                  PathValidationContext validationContext) {
                this.follower = follower;
                this.defaults = defaults;
                this.validationContext = validationContext;
            }

            static BuilderInputs capture(Follower follower, PathConstraints defaults) {
                Follower requiredFollower = Objects.requireNonNull(follower, "follower");
                FollowerConstants constants = Objects.requireNonNull(
                        requiredFollower.getConstants(),
                        CONFIG_CONTEXT + ".followerConstants must not be null"
                );
                PathValidationContext validationContext = new PathValidationContext(
                        constants.forwardZeroPowerAcceleration,
                        constants.lateralZeroPowerAcceleration,
                        requiredFollower.getDrivetrain().xVelocity(),
                        requiredFollower.getDrivetrain().yVelocity()
                );
                PathConstraints ownedDefaults = Objects.requireNonNull(
                        defaults,
                        CONFIG_CONTEXT + ".pathConstraints must not be null"
                ).copy();
                validationContext.validate(ownedDefaults);
                return new BuilderInputs(
                        requiredFollower,
                        ownedDefaults,
                        validationContext
                );
            }
        }
    }

    /**
     * Returns a defensive snapshot of Pedro's current cached field pose.
     *
     * <p>This operation does not update the Follower or poll Pinpoint. The returned Pedro
     * {@link Pose} instance is independent from the cached instance in the owned runtime graph.</p>
     *
     * @return fresh immutable snapshot preserving x, y, heading, and coordinate system
     */
    public Pose currentPedroPose() {
        Pose current = Objects.requireNonNull(
                follower.getPose(),
                "Pedro Follower returned a null cached pose"
        );
        return new Pose(
                current.getX(),
                current.getY(),
                current.getHeading(),
                current.getCoordinateSystem()
        );
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

    /** Deep-copy the exact mutable field closure of pinned Pedro 2.1.2 FollowerConstants. */
    private static FollowerConstants copyFollowerConstants(FollowerConstants source) {
        if (source == null) {
            return null;
        }
        FollowerConstants copy = new FollowerConstants();
        copy.coefficientsTranslationalPIDF = copyPid(source.coefficientsTranslationalPIDF);
        copy.integralTranslational = copyPid(source.integralTranslational);
        copy.coefficientsHeadingPIDF = copyPid(source.coefficientsHeadingPIDF);
        copy.coefficientsDrivePIDF = copyFilteredPid(source.coefficientsDrivePIDF);
        copy.coefficientsSecondaryTranslationalPIDF =
                copyPid(source.coefficientsSecondaryTranslationalPIDF);
        copy.integralSecondaryTranslational = copyPid(source.integralSecondaryTranslational);
        copy.headingPIDFSwitch = source.headingPIDFSwitch;
        copy.coefficientsSecondaryHeadingPIDF =
                copyPid(source.coefficientsSecondaryHeadingPIDF);
        copy.drivePIDFSwitch = source.drivePIDFSwitch;
        copy.coefficientsSecondaryDrivePIDF =
                copyFilteredPid(source.coefficientsSecondaryDrivePIDF);
        copy.predictiveBrakingCoefficients =
                copyPredictiveBraking(source.predictiveBrakingCoefficients);
        copy.usePredictiveBraking = source.usePredictiveBraking;
        copy.holdPointTranslationalScaling = source.holdPointTranslationalScaling;
        copy.holdPointHeadingScaling = source.holdPointHeadingScaling;
        copy.BEZIER_CURVE_SEARCH_LIMIT = source.BEZIER_CURVE_SEARCH_LIMIT;
        copy.useSecondaryTranslationalPIDF = source.useSecondaryTranslationalPIDF;
        copy.useSecondaryHeadingPIDF = source.useSecondaryHeadingPIDF;
        copy.useSecondaryDrivePIDF = source.useSecondaryDrivePIDF;
        copy.translationalPIDFSwitch = source.translationalPIDFSwitch;
        copy.turnHeadingErrorThreshold = source.turnHeadingErrorThreshold;
        copy.centripetalScaling = source.centripetalScaling;
        copy.automaticHoldEnd = source.automaticHoldEnd;
        copy.mass = source.mass;
        copy.forwardZeroPowerAcceleration = source.forwardZeroPowerAcceleration;
        copy.lateralZeroPowerAcceleration = source.lateralZeroPowerAcceleration;
        copy.driveKalmanFilterModelCovariance = source.driveKalmanFilterModelCovariance;
        copy.driveKalmanFilterDataCovariance = source.driveKalmanFilterDataCovariance;
        copy.stuckVelocity = source.stuckVelocity;
        copy.stuckTValueLow = source.stuckTValueLow;
        copy.stuckTValueHigh = source.stuckTValueHigh;
        copy.stuckTimeout = source.stuckTimeout;
        return copy;
    }

    /** Deep-copy the exact mutable field closure of pinned Pedro 2.1.2 MecanumConstants. */
    private static MecanumConstants copyMecanumConstants(MecanumConstants source) {
        if (source == null) {
            return null;
        }
        MecanumConstants copy = new MecanumConstants();
        copy.xVelocity = source.xVelocity;
        copy.yVelocity = source.yVelocity;
        copy.frontLeftVector = source.frontLeftVector == null
                ? null
                : new Vector(
                        source.frontLeftVector.getMagnitude(),
                        source.frontLeftVector.getTheta()
                );
        copy.maxPower = source.maxPower;
        copy.leftFrontMotorName = source.leftFrontMotorName;
        copy.leftRearMotorName = source.leftRearMotorName;
        copy.rightFrontMotorName = source.rightFrontMotorName;
        copy.rightRearMotorName = source.rightRearMotorName;
        copy.leftFrontMotorDirection = source.leftFrontMotorDirection;
        copy.leftRearMotorDirection = source.leftRearMotorDirection;
        copy.rightFrontMotorDirection = source.rightFrontMotorDirection;
        copy.rightRearMotorDirection = source.rightRearMotorDirection;
        copy.motorCachingThreshold = source.motorCachingThreshold;
        copy.useBrakeModeInTeleOp = source.useBrakeModeInTeleOp;
        copy.useVoltageCompensation = source.useVoltageCompensation;
        copy.nominalVoltage = source.nominalVoltage;
        copy.staticFrictionCoefficient = source.staticFrictionCoefficient;
        return copy;
    }

    private static PIDFCoefficients copyPid(PIDFCoefficients source) {
        return source == null ? null : new PIDFCoefficients(
                source.P,
                source.I,
                source.D,
                source.F
        );
    }

    private static FilteredPIDFCoefficients copyFilteredPid(
            FilteredPIDFCoefficients source) {
        return source == null ? null : new FilteredPIDFCoefficients(
                source.P,
                source.I,
                source.D,
                source.T,
                source.F
        );
    }

    private static PredictiveBrakingCoefficients copyPredictiveBraking(
            PredictiveBrakingCoefficients source) {
        if (source == null) {
            return null;
        }
        PredictiveBrakingCoefficients copy = new PredictiveBrakingCoefficients(
                source.P,
                source.kLinearBraking,
                source.kQuadraticFriction
        );
        copy.maximumBrakingPower = source.maximumBrakingPower;
        return copy;
    }

    /** Validate Pedro drivetrain wiring and tuning before vendor hardware construction. */
    static void validateMecanumConstants(MecanumConstants constants) {
        validateMecanumConstants(constants, CONFIG_CONTEXT + ".mecanumConstants");
    }

    private static void validateMecanumConstants(MecanumConstants constants, String name) {
        MecanumConstants value = Objects.requireNonNull(
                constants,
                name + " must not be null"
        );
        Map<String, HardwareNameClaim> motorNames = new HashMap<String, HardwareNameClaim>();
        requireUniqueHardwareName(value.leftFrontMotorName,
                name + ".leftFrontMotorName", motorNames);
        requireUniqueHardwareName(value.leftRearMotorName,
                name + ".leftRearMotorName", motorNames);
        requireUniqueHardwareName(value.rightFrontMotorName,
                name + ".rightFrontMotorName", motorNames);
        requireUniqueHardwareName(value.rightRearMotorName,
                name + ".rightRearMotorName", motorNames);

        Objects.requireNonNull(value.leftFrontMotorDirection,
                name + ".leftFrontMotorDirection must not be null");
        Objects.requireNonNull(value.leftRearMotorDirection,
                name + ".leftRearMotorDirection must not be null");
        Objects.requireNonNull(value.rightFrontMotorDirection,
                name + ".rightFrontMotorDirection must not be null");
        Objects.requireNonNull(value.rightRearMotorDirection,
                name + ".rightRearMotorDirection must not be null");

        requirePositive(value.xVelocity, name + ".xVelocity");
        requirePositive(value.yVelocity, name + ".yVelocity");
        requirePositiveAtMostOne(value.maxPower, name + ".maxPower");
        requireRangeUpperExclusive(
                value.motorCachingThreshold,
                0.0,
                1.0,
                name + ".motorCachingThreshold"
        );
        requirePositive(value.nominalVoltage, name + ".nominalVoltage");
        if (value.useVoltageCompensation) {
            double squaredNominalVoltage = value.nominalVoltage * value.nominalVoltage;
            if (!Double.isFinite(squaredNominalVoltage)) {
                throw new IllegalArgumentException(
                        name + ".nominalVoltage * " + name + ".nominalVoltage must remain finite "
                                + "when " + name + ".useVoltageCompensation=true, got "
                                + value.nominalVoltage + " and " + value.nominalVoltage
                );
            }
        }
        requireRangeUpperExclusive(
                value.staticFrictionCoefficient,
                0.0,
                1.0,
                name + ".staticFrictionCoefficient"
        );

        String vectorName = name + ".frontLeftVector";
        Vector frontLeftVector = Objects.requireNonNull(
                value.frontLeftVector,
                vectorName + " must not be null"
        );
        requireFinite(frontLeftVector.getXComponent(), vectorName + ".x");
        requireFinite(frontLeftVector.getYComponent(), vectorName + ".y");
        requirePositive(frontLeftVector.getMagnitude(), vectorName + ".magnitude");

        Vector normalizedFrontLeft = frontLeftVector.normalize();
        Vector normalizedMirrored = new Vector(
                normalizedFrontLeft.getMagnitude(),
                2.0 * Math.PI - normalizedFrontLeft.getTheta()
        );
        validateNormalizedWheelBasisDeterminant(
                normalizedFrontLeft.cross(normalizedMirrored),
                vectorName + ".normalizedBasisDeterminant"
        );
    }

    /** Validate a standalone constraints value against fresh pinned Pedro software defaults. */
    static void validatePathConstraints(PathConstraints constraints) {
        FollowerConstants followerConstants = new FollowerConstants();
        MecanumConstants mecanumConstants = new MecanumConstants();
        validatePathConstraints(
                constraints,
                followerConstants,
                mecanumConstants.xVelocity,
                mecanumConstants.yVelocity,
                CONFIG_CONTEXT + ".pathConstraints",
                CONFIG_CONTEXT + ".followerConstants",
                CONFIG_CONTEXT + ".mecanumConstants"
        );
    }

    private static void validatePathConstraints(PathConstraints constraints,
                                                FollowerConstants followerConstants,
                                                double xVelocity,
                                                double yVelocity,
                                                String pathName,
                                                String followerName,
                                                String mecanumName) {
        FollowerConstants follower = Objects.requireNonNull(
                followerConstants,
                followerName + " must not be null"
        );
        validatePathConstraints(
                constraints,
                follower.forwardZeroPowerAcceleration,
                follower.lateralZeroPowerAcceleration,
                xVelocity,
                yVelocity,
                pathName,
                followerName,
                mecanumName
        );
    }

    private static void validatePathConstraints(PathConstraints constraints,
                                                double forwardAcceleration,
                                                double lateralAcceleration,
                                                double xVelocity,
                                                double yVelocity,
                                                String pathName,
                                                String followerName,
                                                String mecanumName) {
        PathConstraints value = Objects.requireNonNull(
                constraints,
                pathName + " must not be null"
        );
        requirePositiveAtMostOne(value.getTValueConstraint(),
                pathName + ".tValueConstraint");
        requireNonNegative(value.getVelocityConstraint(),
                pathName + ".velocityConstraint");
        requireNonNegative(value.getTranslationalConstraint(),
                pathName + ".translationalConstraint");
        requireNonNegative(value.getHeadingConstraint(),
                pathName + ".headingConstraint");
        requireNonNegative(value.getTimeoutConstraint(),
                pathName + ".timeoutConstraint");
        requirePositive(value.getBrakingStrength(), pathName + ".brakingStrength");
        requireNonNegative(value.getBrakingStart(), pathName + ".brakingStart");
        if (value.getBEZIER_CURVE_SEARCH_LIMIT() <= 0) {
            throw new IllegalArgumentException(
                    pathName + ".BEZIER_CURVE_SEARCH_LIMIT must be > 0, got "
                            + value.getBEZIER_CURVE_SEARCH_LIMIT()
            );
        }

        validateScaledBrakingAcceleration(
                forwardAcceleration,
                followerName + ".forwardZeroPowerAcceleration",
                value.getBrakingStrength(),
                pathName + ".brakingStrength"
        );
        validateScaledBrakingAcceleration(
                lateralAcceleration,
                followerName + ".lateralZeroPowerAcceleration",
                value.getBrakingStrength(),
                pathName + ".brakingStrength"
        );
        validateStoppingDistance(
                xVelocity,
                mecanumName + ".xVelocity",
                forwardAcceleration,
                followerName + ".forwardZeroPowerAcceleration",
                value.getBrakingStart(),
                pathName + ".brakingStart"
        );
        validateStoppingDistance(
                yVelocity,
                mecanumName + ".yVelocity",
                forwardAcceleration,
                followerName + ".forwardZeroPowerAcceleration",
                value.getBrakingStart(),
                pathName + ".brakingStart"
        );
    }

    static void validateFollowerConstants(FollowerConstants constants) {
        validateFollowerConstants(constants, CONFIG_CONTEXT + ".followerConstants");
    }

    private static void validateFollowerConstants(FollowerConstants constants, String name) {
        FollowerConstants value = Objects.requireNonNull(
                constants,
                name + " must not be null"
        );
        validatePid(value.coefficientsTranslationalPIDF,
                name + ".coefficientsTranslationalPIDF");
        validatePid(value.integralTranslational, name + ".integralTranslational");
        validatePid(value.coefficientsHeadingPIDF, name + ".coefficientsHeadingPIDF");
        validateFilteredPid(value.coefficientsDrivePIDF, name + ".coefficientsDrivePIDF");
        validatePid(value.coefficientsSecondaryTranslationalPIDF,
                name + ".coefficientsSecondaryTranslationalPIDF");
        validatePid(value.integralSecondaryTranslational,
                name + ".integralSecondaryTranslational");
        validatePid(value.coefficientsSecondaryHeadingPIDF,
                name + ".coefficientsSecondaryHeadingPIDF");
        validateFilteredPid(value.coefficientsSecondaryDrivePIDF,
                name + ".coefficientsSecondaryDrivePIDF");
        validatePredictiveBraking(
                value.predictiveBrakingCoefficients,
                name + ".predictiveBrakingCoefficients",
                value.usePredictiveBraking,
                name + ".usePredictiveBraking"
        );

        validateSecondarySwitch(
                value.headingPIDFSwitch,
                name + ".headingPIDFSwitch",
                value.useSecondaryHeadingPIDF,
                name + ".useSecondaryHeadingPIDF"
        );
        validateSecondarySwitch(
                value.drivePIDFSwitch,
                name + ".drivePIDFSwitch",
                value.useSecondaryDrivePIDF,
                name + ".useSecondaryDrivePIDF"
        );
        validateSecondarySwitch(
                value.translationalPIDFSwitch,
                name + ".translationalPIDFSwitch",
                value.useSecondaryTranslationalPIDF,
                name + ".useSecondaryTranslationalPIDF"
        );
        requireNonNegative(value.holdPointTranslationalScaling,
                name + ".holdPointTranslationalScaling");
        requireNonNegative(value.holdPointHeadingScaling,
                name + ".holdPointHeadingScaling");
        requirePositive(value.turnHeadingErrorThreshold,
                name + ".turnHeadingErrorThreshold");
        requireNonNegative(value.centripetalScaling, name + ".centripetalScaling");
        requirePositive(value.mass, name + ".mass");
        double centripetalMassProduct = value.centripetalScaling * value.mass;
        if (!Double.isFinite(centripetalMassProduct)) {
            throw new IllegalArgumentException(
                    name + ".centripetalScaling * " + name + ".mass must remain finite, got "
                            + value.centripetalScaling + " and " + value.mass
            );
        }

        requireNegative(value.forwardZeroPowerAcceleration,
                name + ".forwardZeroPowerAcceleration");
        requireFiniteDouble(
                2.0 * value.forwardZeroPowerAcceleration,
                "2.0 * " + name + ".forwardZeroPowerAcceleration",
                value.forwardZeroPowerAcceleration
        );
        requireNegative(value.lateralZeroPowerAcceleration,
                name + ".lateralZeroPowerAcceleration");
        requireFiniteDouble(
                2.0 * value.lateralZeroPowerAcceleration,
                "2.0 * " + name + ".lateralZeroPowerAcceleration",
                value.lateralZeroPowerAcceleration
        );

        requireNonNegative(value.driveKalmanFilterModelCovariance,
                name + ".driveKalmanFilterModelCovariance");
        requireNonNegative(value.driveKalmanFilterDataCovariance,
                name + ".driveKalmanFilterDataCovariance");
        if (value.driveKalmanFilterModelCovariance == 0.0
                && value.driveKalmanFilterDataCovariance == 0.0) {
            throw new IllegalArgumentException(
                    name + ".driveKalmanFilterModelCovariance and "
                            + name + ".driveKalmanFilterDataCovariance must include at least one "
                            + "positive value, got 0.0 and 0.0"
            );
        }
        if (value.driveKalmanFilterModelCovariance > 0.0) {
            double recurrence = value.driveKalmanFilterModelCovariance
                    + 2.0 * value.driveKalmanFilterDataCovariance;
            if (!Double.isFinite(recurrence)) {
                throw new IllegalArgumentException(
                        name + ".driveKalmanFilterModelCovariance + 2 * "
                                + name + ".driveKalmanFilterDataCovariance must remain finite, got "
                                + value.driveKalmanFilterModelCovariance + " and "
                                + value.driveKalmanFilterDataCovariance
                );
            }
        }

        requireNonNegative(value.stuckVelocity, name + ".stuckVelocity");
        requireRange(value.stuckTValueLow, 0.0, 1.0, name + ".stuckTValueLow");
        requireRange(value.stuckTValueHigh, 0.0, 1.0, name + ".stuckTValueHigh");
        if (value.stuckTValueLow >= value.stuckTValueHigh) {
            throw new IllegalArgumentException(
                    name + ".stuckTValueLow must be < " + name + ".stuckTValueHigh, got "
                            + value.stuckTValueLow + " and " + value.stuckTValueHigh
            );
        }
        requireNonNegative(value.stuckTimeout, name + ".stuckTimeout");
        if (value.BEZIER_CURVE_SEARCH_LIMIT <= 0) {
            throw new IllegalArgumentException(
                    name + ".BEZIER_CURVE_SEARCH_LIMIT must be > 0, got "
                            + value.BEZIER_CURVE_SEARCH_LIMIT
            );
        }
    }

    private static void validatePid(PIDFCoefficients coefficients, String name) {
        PIDFCoefficients value = Objects.requireNonNull(
                coefficients,
                name + " must not be null"
        );
        requireFinite(value.P, name + ".P");
        requireFinite(value.I, name + ".I");
        requireFinite(value.D, name + ".D");
        requireFinite(value.F, name + ".F");
    }

    private static void validateFilteredPid(FilteredPIDFCoefficients coefficients, String name) {
        FilteredPIDFCoefficients value = Objects.requireNonNull(
                coefficients,
                name + " must not be null"
        );
        requireFinite(value.P, name + ".P");
        requireFinite(value.I, name + ".I");
        requireFinite(value.D, name + ".D");
        requireFinite(value.F, name + ".F");
        requireRange(value.T, 0.0, 1.0, name + ".T");
    }

    private static void validatePredictiveBraking(PredictiveBrakingCoefficients coefficients,
                                                   String name,
                                                   boolean enabled,
                                                   String enabledName) {
        PredictiveBrakingCoefficients value = Objects.requireNonNull(
                coefficients,
                name + " must not be null"
        );
        requireNonNegative(value.P, name + ".P");
        requireNonNegative(value.kLinearBraking, name + ".kLinearBraking");
        requireNonNegative(value.kQuadraticFriction, name + ".kQuadraticFriction");
        requirePositiveAtMostOne(value.maximumBrakingPower, name + ".maximumBrakingPower");
        if (enabled && value.P <= 0.0) {
            throw new IllegalArgumentException(
                    name + ".P must be > 0 when " + enabledName + "=true, got "
                            + value.P + " and true"
            );
        }
    }

    private static void validateSecondarySwitch(double value,
                                                String valueName,
                                                boolean enabled,
                                                String enabledName) {
        requireNonNegative(value, valueName);
        if (enabled && value <= 0.0) {
            throw new IllegalArgumentException(
                    valueName + " must be > 0 when " + enabledName + "=true, got "
                            + value + " and true"
            );
        }
    }

    private static void validateNormalizedWheelBasisDeterminant(double determinant, String name) {
        requireFinite(determinant, name);
        double absoluteDeterminant = Math.abs(determinant);
        if (absoluteDeterminant < MIN_NORMALIZED_WHEEL_BASIS_DETERMINANT) {
            throw new IllegalArgumentException(
                    name + " absolute value must be >= "
                            + MIN_NORMALIZED_WHEEL_BASIS_DETERMINANT + ", got " + determinant
            );
        }
        double conservativeInverse = 4.0 / absoluteDeterminant;
        if (!Double.isFinite(conservativeInverse)) {
            throw new IllegalArgumentException(
                    "4.0 / abs(" + name + ") must remain finite, got determinant "
                            + determinant + " and inverse " + conservativeInverse
            );
        }
    }

    private static void validateScaledBrakingAcceleration(double acceleration,
                                                          String accelerationName,
                                                          double brakingStrength,
                                                          String brakingStrengthName) {
        double scaled = acceleration * (brakingStrength * 4.0);
        if (!Double.isFinite(scaled)) {
            throw new IllegalArgumentException(
                    accelerationName + " * (" + brakingStrengthName
                            + " * 4.0) must remain finite, got " + acceleration + " and "
                            + brakingStrength
            );
        }
        double doubled = 2.0 * scaled;
        if (!Double.isFinite(doubled)) {
            throw new IllegalArgumentException(
                    "2.0 * (" + accelerationName + " * (" + brakingStrengthName
                            + " * 4.0)) must remain finite, got " + acceleration + " and "
                            + brakingStrength
            );
        }
    }

    private static void validateStoppingDistance(double velocity,
                                                 String velocityName,
                                                 double acceleration,
                                                 String accelerationName,
                                                 double brakingStart,
                                                 String brakingStartName) {
        double stoppingDistance = Math.abs(
                (0.0 - velocity * velocity) / (2.0 * acceleration)
        );
        if (!Double.isFinite(stoppingDistance)) {
            throw new IllegalArgumentException(
                    "abs((0.0 - " + velocityName + " * " + velocityName + ") / (2.0 * "
                            + accelerationName + ")) must remain finite, got " + velocity
                            + " and " + acceleration
            );
        }
        double brakingStartDistance = stoppingDistance * brakingStart;
        if (!Double.isFinite(brakingStartDistance)) {
            throw new IllegalArgumentException(
                    "stopping distance from " + velocityName + "=" + velocity + " and "
                            + accelerationName + "=" + acceleration + " times "
                            + brakingStartName + " must remain finite, got " + brakingStart
            );
        }
    }

    private static void requireUniqueHardwareName(String name,
                                                  String fieldName,
                                                  Map<String, HardwareNameClaim> usedNames) {
        requireHardwareName(name, fieldName);
        String effectiveName = name.trim();
        HardwareNameClaim first = usedNames.put(
                effectiveName,
                new HardwareNameClaim(fieldName, name)
        );
        if (first != null) {
            throw new IllegalArgumentException(
                    fieldName + "='" + name + "' duplicates motor hardware name "
                            + first.fieldName + "='" + first.rawName
                            + "' after case-sensitive FTC whitespace normalization to '"
                            + effectiveName + "'; use distinct hardware names"
            );
        }
    }

    private static void requireHardwareName(String value, String name) {
        if (value == null) {
            throw new NullPointerException(name + " must not be null");
        }
        if (value.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    name + " must contain a non-whitespace FTC hardware name, got '" + value + "'"
            );
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

    private static void requireNegative(double value, String name) {
        requireFinite(value, name);
        if (value >= 0.0) {
            throw new IllegalArgumentException(name + " must be < 0, got " + value);
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

    private static void requireRangeUpperExclusive(double value,
                                                   double min,
                                                   double max,
                                                   String name) {
        requireFinite(value, name);
        if (value < min || value >= max) {
            throw new IllegalArgumentException(
                    name + " must be in [" + min + ", " + max + "), got " + value
            );
        }
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
    }

    private static void requireFiniteDouble(double calculated,
                                            String expression,
                                            double input) {
        if (!Double.isFinite(calculated)) {
            throw new IllegalArgumentException(
                    expression + " must remain finite, got input " + input
            );
        }
    }

    /** Immutable arithmetic inputs used to validate every later PathBuilder constraints draft. */
    private static final class PathValidationContext {
        private final double forwardAcceleration;
        private final double lateralAcceleration;
        private final double xVelocity;
        private final double yVelocity;

        private PathValidationContext(double forwardAcceleration,
                                      double lateralAcceleration,
                                      double xVelocity,
                                      double yVelocity) {
            this.forwardAcceleration = forwardAcceleration;
            this.lateralAcceleration = lateralAcceleration;
            this.xVelocity = xVelocity;
            this.yVelocity = yVelocity;
        }

        private void validate(PathConstraints constraints) {
            validatePathConstraints(
                    constraints,
                    forwardAcceleration,
                    lateralAcceleration,
                    xVelocity,
                    yVelocity,
                    CONFIG_CONTEXT + ".pathConstraints",
                    CONFIG_CONTEXT + ".followerConstants",
                    CONFIG_CONTEXT + ".mecanumConstants"
            );
        }
    }

    private static final class HardwareNameClaim {
        private final String fieldName;
        private final String rawName;

        private HardwareNameClaim(String fieldName, String rawName) {
            this.fieldName = fieldName;
            this.rawName = rawName;
        }
    }

    private static String failureSummary(RuntimeException failure) {
        String message = failure.getMessage();
        return message == null || message.trim().isEmpty()
                ? failure.getClass().getSimpleName()
                : message;
    }
}
