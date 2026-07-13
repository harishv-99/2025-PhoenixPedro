package edu.ftcphoenix.robots.phoenix.autonomous.pedro;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
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

    /**
     * Fixed Pedro path set plus the poses used to initialize/debug the follower.
     *
     * <p>Only geometry whose start is known during INIT belongs here. Routes that depend on the
     * robot's later pose are built by methods such as {@link #buildReturnFromCurrentPose(Pose)}
     * when their owning Task starts.</p>
     */
    public static final class Paths {
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

        private Paths(Pose pedroStartPose, PathChain outboundPath, String label) {
            this.pedroStartPose = pedroStartPose;
            this.outboundPath = outboundPath;
            this.label = label;
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
        this.autoCfg = autoCfg == null ? new PhoenixProfile.AutoConfig() : autoCfg;
    }

    /**
     * Build the currently checked-in fixed placeholder path set for the selected Auto spec.
     *
     * <p>The path labels and start pose already depend on the spec, but the geometry is still the
     * old twelve-inch Pedro integration route. Replace this method's geometry branches when real
     * Decode routes are ready. The returned {@link Paths#pedroStartPose} is applied by the Auto
     * composition root through the Pedro integration lane so Pedro and Phoenix start from one
     * synchronized pose. Return/park geometry is deliberately not prebuilt here because its start
     * should be the robot's current pose when that Task begins. This factory never advances the
     * Follower.</p>
     */
    public Paths build(PhoenixAutoSpec spec, PhoenixCapabilities capabilities) {
        Objects.requireNonNull(spec, "spec");
        Objects.requireNonNull(capabilities, "capabilities");

        double distanceIn = autoCfg.pedroIntegrationTestDistanceIn;
        Pose pedroStartPose = startPoseFor(spec);
        Pose forwardPose = new Pose(
                pedroStartPose.getX() + distanceIn,
                pedroStartPose.getY(),
                pedroStartPose.getHeading()
        );

        PathChain outbound = pedroRuntime.pathBuilder()
                .addPath(new BezierLine(pedroStartPose, forwardPose))
                .setLinearHeadingInterpolation(pedroStartPose.getHeading(), forwardPose.getHeading())
                .addParametricCallback(0.50, spinUpCallback(capabilities))
                .build();

        return new Paths(pedroStartPose, outbound, labelFor(spec, distanceIn));
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

    private static Pose startPoseFor(PhoenixAutoSpec spec) {
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
}
