package edu.ftcphoenix.robots.phoenix.autonomous.pedro;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import java.util.Objects;

import edu.ftcphoenix.fw.integrations.pedro.PedroPathingRuntime;
import edu.ftcphoenix.robots.phoenix.PhoenixCapabilities;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoSpec;

/**
 * Pedro-specific path construction for Phoenix autonomous routines.
 *
 * <p>This class owns Pedro geometry and path-builder calls. It does not decide which strategy should
 * run; {@link PhoenixPedroAutoRoutineFactory} makes that decision from {@link PhoenixAutoSpec}. The
 * current checked-in paths are still intentionally small integration placeholders, but the ownership
 * seam is now ready for real alliance/start/partner route geometry.</p>
 */
public final class PhoenixPedroPathFactory {

    private static final double STRUCTURAL_START_EPSILON = 1e-9;

    /**
     * Declares whether the selected Phoenix route geometry is suitable for match use.
     *
     * <p>The expected pose is expressed in Pedro field coordinates, inches, and radians. Pedro's
     * pinned {@link Pose} value is immutable, so this value object can safely expose it directly.
     * Construction stays behind {@link PhoenixPedroPathFactory#routeAvailabilityFor(PhoenixAutoSpec)}
     * so route maturity and its declared start pose have one authority.</p>
     */
    public static final class RouteAvailability {

        /** Route-geometry maturity relevant to autonomous arming policy. */
        public enum Maturity {
            /** Geometry has been implemented and validated for the selected match setup. */
            MATCH_READY,
            /** Geometry exists only to exercise the Pedro/Phoenix integration. */
            INTEGRATION_ONLY
        }

        /** Maturity of the route geometry for the exact selected Auto spec. */
        public final Maturity maturity;
        /** Declared starting pose in Pedro field coordinates, inches, and radians. */
        public final Pose expectedPedroStartPose;
        /** Human-facing explanation of the maturity classification. */
        public final String reason;

        private RouteAvailability(Maturity maturity,
                                  Pose expectedPedroStartPose,
                                  String reason) {
            this.maturity = Objects.requireNonNull(maturity, "maturity");
            this.expectedPedroStartPose = requireFinitePose(
                    expectedPedroStartPose,
                    "expectedPedroStartPose"
            );
            String cleanReason = reason == null ? "" : reason.trim();
            if (cleanReason.isEmpty()) {
                throw new IllegalArgumentException("route availability reason must not be blank");
            }
            this.reason = cleanReason;
        }

        /** Return whether this exact route selection is declared ready for match use. */
        public boolean isMatchReady() {
            return maturity == Maturity.MATCH_READY;
        }
    }

    /**
     * Fixed Pedro path set plus the poses used to initialize/debug the follower.
     *
     * <p>Only geometry whose start is known when the selected runtime is constructed belongs here.
     * Routes that depend on the robot's later pose are built by methods such as
     * {@link #buildReturnFromCurrentPose(Pose)} when their owning Task starts.</p>
     */
    public static final class Paths {
        /** Route maturity and the declared start-pose contract for this exact spec. */
        public final RouteAvailability routeAvailability;
        /**
         * Starting Pedro pose for the selected spec.
         */
        public final Pose pedroStartPose;
        /**
         * First placeholder outbound/preload path.
         */
        public final PathChain outboundPath;
        /**
         * Human-facing label for telemetry.
         */
        public final String label;

        private Paths(RouteAvailability routeAvailability,
                      PathChain outboundPath,
                      String label) {
            this.routeAvailability = Objects.requireNonNull(
                    routeAvailability,
                    "routeAvailability"
            );
            this.pedroStartPose = routeAvailability.expectedPedroStartPose;
            this.outboundPath = Objects.requireNonNull(outboundPath, "outboundPath");
            this.label = Objects.requireNonNull(label, "label");
        }
    }

    private final PedroPathingRuntime pedroRuntime;
    private final PhoenixProfile.AutoConfig autoCfg;

    /**
     * Create a path factory around the validated Pedro runtime used by the OpMode.
     *
     * @param pedroRuntime Pedro runtime that applies its validated path constraints explicitly
     * @param autoCfg  Phoenix Auto timing/path-placeholder config
     */
    public PhoenixPedroPathFactory(PedroPathingRuntime pedroRuntime,
                                   PhoenixProfile.AutoConfig autoCfg) {
        this.pedroRuntime = Objects.requireNonNull(pedroRuntime, "pedroRuntime");
        this.autoCfg = autoCfg == null ? new PhoenixProfile.AutoConfig() : autoCfg.copy();
    }

    /**
     * Return the route maturity and declared Pedro start pose for an exact Phoenix Auto spec.
     *
     * <p>This is the sole authority used both by preflight/readiness code and by
     * {@link #build(PhoenixAutoSpec, PhoenixCapabilities)}. Every checked-in route currently uses
     * the same configurable straight-line integration placeholder, so no selection is match-ready
     * yet.</p>
     *
     * @param spec exact alliance/start/partner/strategy selection
     * @return immutable maturity and expected-start declaration for {@code spec}
     */
    public static RouteAvailability routeAvailabilityFor(PhoenixAutoSpec spec) {
        PhoenixAutoSpec requiredSpec = Objects.requireNonNull(spec, "spec");
        return new RouteAvailability(
                RouteAvailability.Maturity.INTEGRATION_ONLY,
                expectedStartPoseFor(requiredSpec),
                "Only the checked-in configurable straight-line Pedro integration placeholder "
                        + "exists for "
                        + requiredSpec.summary()
                        + "; implement and validate this selection's match geometry in "
                        + "PhoenixPedroPathFactory before match use."
        );
    }

    /**
     * Build the currently checked-in fixed placeholder path set for the selected Auto spec.
     *
     * <p>The path labels and start pose already depend on the spec, but the geometry is still the
     * configured straight-line Pedro integration route. Replace this method's geometry branches
     * when real Decode routes are ready. The returned {@link Paths#pedroStartPose} is applied by the Auto
     * composition root through the Pedro integration lane so Pedro and Phoenix start from one
     * synchronized pose. Return/park geometry is deliberately not prebuilt here because its start
     * should be the robot's current pose when that Task begins. This factory never advances the
     * Follower.</p>
     */
    public Paths build(PhoenixAutoSpec spec, PhoenixCapabilities capabilities) {
        RouteAvailability routeAvailability = routeAvailabilityFor(spec);
        Objects.requireNonNull(capabilities, "capabilities");

        double distanceIn = autoCfg.pedroIntegrationTestDistanceIn;
        Pose pedroStartPose = routeAvailability.expectedPedroStartPose;
        Pose forwardPose = integrationPlaceholderEndPose(pedroStartPose, distanceIn);

        PathChain outbound = pedroRuntime.pathBuilder()
                .addPath(new BezierLine(pedroStartPose, forwardPose))
                .setLinearHeadingInterpolation(pedroStartPose.getHeading(), forwardPose.getHeading())
                .addParametricCallback(0.50, spinUpCallback(capabilities))
                .build();

        validateDeclaredStart(outbound, pedroStartPose);
        return new Paths(routeAvailability, outbound, labelFor(spec, distanceIn));
    }

    /**
     * Verify that fixed outbound geometry begins at its separately declared Pedro start pose.
     *
     * <p>The translation is read from the first curve at parametric zero. The heading is read
     * through the PathChain so a global chain interpolator, when present, remains authoritative;
     * it is compared modulo one turn. This inspection does not start, update, or otherwise mutate
     * the Follower lifecycle.</p>
     */
    static void validateDeclaredStart(PathChain outboundPath, Pose expectedPedroStartPose) {
        if (outboundPath == null) {
            throw new IllegalArgumentException("Phoenix Pedro outbound path must not be null");
        }
        if (outboundPath.size() == 0) {
            throw new IllegalArgumentException(
                    "Phoenix Pedro outbound path must contain at least one path"
            );
        }

        Path firstPath = outboundPath.firstPath();
        if (firstPath == null) {
            throw new IllegalArgumentException(
                    "Phoenix Pedro outbound path must expose its first path"
            );
        }

        Pose expectedStart = requireFinitePose(
                expectedPedroStartPose,
                "expectedPedroStartPose"
        );
        Pose actualTranslation = Objects.requireNonNull(
                firstPath.getPoint(0.0),
                "outboundPath first geometry"
        );
        requireFinite(
                actualTranslation.getX(),
                "outboundPath first geometry",
                "x"
        );
        requireFinite(
                actualTranslation.getY(),
                "outboundPath first geometry",
                "y"
        );
        double actualHeadingRad = outboundPath.getHeadingGoal(
                new PathChain.PathT(0, 0.0)
        );
        requireFinite(actualHeadingRad, "outboundPath first geometry", "heading");

        double translationErrorIn = Math.hypot(
                actualTranslation.getX() - expectedStart.getX(),
                actualTranslation.getY() - expectedStart.getY()
        );
        double headingErrorRad = wrappedHeadingErrorRad(
                actualHeadingRad,
                expectedStart.getHeading()
        );
        if (translationErrorIn > STRUCTURAL_START_EPSILON
                || headingErrorRad > STRUCTURAL_START_EPSILON) {
            throw new IllegalArgumentException(
                    "Phoenix Pedro outbound path does not start at its declared "
                            + "expectedPedroStartPose: expected "
                            + formatPose(expectedStart)
                            + " but first geometry starts at "
                            + formatPose(new Pose(
                                    actualTranslation.getX(),
                                    actualTranslation.getY(),
                                    actualHeadingRad,
                                    actualTranslation.getCoordinateSystem()
                            ))
                            + ". Update the route geometry and RouteAvailability together."
            );
        }
    }

    /**
     * Build a return path from a one-time snapshot of the Follower's current Pedro pose.
     *
     * <p>This operation is intended for a start-time route factory. It reads the current pose once,
     * copies both endpoints, and builds through {@link PedroPathingRuntime#pathBuilder()} so the
     * runtime's validated constraints remain in force. It performs no Follower lifecycle call;
     * route start, heartbeat, cancellation, and stop remain owned by the drive adapter.</p>
     *
     * @param pedroReturnPose return target expressed in Pedro field coordinates
     * @return newly built path from the sampled current pose to {@code pedroReturnPose}
     */
    public PathChain buildReturnFromCurrentPose(Pose pedroReturnPose) {
        Pose currentPose = snapshotPose(
                Objects.requireNonNull(
                        pedroRuntime.follower().getPose(),
                        "Pedro Follower current pose"
                )
        );
        Pose returnPose = snapshotPose(
                Objects.requireNonNull(pedroReturnPose, "pedroReturnPose")
        );

        return pedroRuntime.pathBuilder()
                .addPath(returnCurveFrom(currentPose, returnPose))
                .setLinearHeadingInterpolation(
                        currentPose.getHeading(),
                        returnPose.getHeading()
                )
                .build();
    }

    /**
     * Select the pinned Pedro curve that safely represents these snapshotted return endpoints.
     *
     * <p>Pedro 2.1.2's {@link BezierLine} divides by its length while finding the closest point, so
     * a line with coincident translation endpoints produces a non-finite path parameter. Its
     * purpose-built {@link BezierPoint} represents that already-at-position case without changing
     * the public Phoenix route API. The enclosing path still applies the requested target-heading
     * interpolation.</p>
     */
    static Curve returnCurveFrom(Pose sampledCurrentPose, Pose pedroReturnPose) {
        Pose currentPose = requireFinitePose(
                sampledCurrentPose,
                "Pedro Follower current pose"
        );
        Pose returnPose = requireFinitePose(pedroReturnPose, "pedroReturnPose");

        if (currentPose.getX() == returnPose.getX()
                && currentPose.getY() == returnPose.getY()) {
            return new BezierPoint(returnPose);
        }
        return new BezierLine(currentPose, returnPose);
    }

    /**
     * Build the current integration route's finite, displaced endpoint.
     *
     * <p>A zero-distance {@link BezierLine} triggers a known non-finite Pedro 2.1.2 calculation,
     * while non-finite or overflowing distance values poison later path geometry even though the
     * declared start still looks valid.</p>
     */
    static Pose integrationPlaceholderEndPose(Pose pedroStartPose, double distanceIn) {
        Pose startPose = requireFinitePose(pedroStartPose, "pedroStartPose");
        requireFinite(
                distanceIn,
                "PhoenixProfile.auto",
                "pedroIntegrationTestDistanceIn"
        );
        if (distanceIn <= 0.0) {
            throw new IllegalArgumentException(
                    "PhoenixProfile.auto.pedroIntegrationTestDistanceIn must be > 0 for the "
                            + "integration route, but was " + distanceIn
            );
        }

        return requireFinitePose(
                new Pose(
                        startPose.getX() + distanceIn,
                        startPose.getY(),
                        startPose.getHeading()
                ),
                "Phoenix Pedro integration route end"
        );
    }

    private Runnable spinUpCallback(final PhoenixCapabilities capabilities) {
        return new Runnable() {
            @Override
            public void run() {
                PhoenixCapabilities.Scoring scoring = capabilities.scoring();
                scoring.captureSuggestedShotVelocity();
                scoring.setFlywheelEnabled(true);
            }
        };
    }

    private static Pose expectedStartPoseFor(PhoenixAutoSpec spec) {
        // Placeholder Pedro frame: real field poses should replace these branches when routes are added.
        double y = spec.startPosition == PhoenixAutoSpec.StartPosition.AUDIENCE ? 0.0 : 24.0;
        double heading = spec.alliance == PhoenixAutoSpec.Alliance.RED ? 0.0 : Math.PI;
        return new Pose(0.0, y, heading);
    }

    private static String labelFor(PhoenixAutoSpec spec, double distanceIn) {
        return spec.alliance.label() + " "
                + spec.startPosition.label() + " placeholder "
                + String.format("%.1f in", distanceIn);
    }

    /** Copy a Pedro pose so each route endpoint is an explicit one-time value snapshot. */
    private static Pose snapshotPose(Pose pose) {
        return new Pose(
                pose.getX(),
                pose.getY(),
                pose.getHeading(),
                pose.getCoordinateSystem()
        );
    }

    /** Reject invalid live/target geometry before Pedro can propagate it into drive calculations. */
    private static Pose requireFinitePose(Pose pose, String endpointName) {
        Pose requiredPose = Objects.requireNonNull(pose, endpointName);
        requireFinite(requiredPose.getX(), endpointName, "x");
        requireFinite(requiredPose.getY(), endpointName, "y");
        requireFinite(requiredPose.getHeading(), endpointName, "heading");
        return requiredPose;
    }

    private static void requireFinite(double value, String endpointName, String fieldName) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(
                    endpointName + "." + fieldName + " must be finite, but was " + value
            );
        }
    }

    private static double wrappedHeadingErrorRad(double firstHeadingRad,
                                                 double secondHeadingRad) {
        double differenceRad = firstHeadingRad - secondHeadingRad;
        return Math.abs(Math.atan2(Math.sin(differenceRad), Math.cos(differenceRad)));
    }

    private static String formatPose(Pose pose) {
        return "(x=" + pose.getX()
                + " in, y=" + pose.getY()
                + " in, heading=" + pose.getHeading() + " rad)";
    }
}
