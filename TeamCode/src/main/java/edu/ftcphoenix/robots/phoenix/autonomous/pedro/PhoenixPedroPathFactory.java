package edu.ftcphoenix.robots.phoenix.autonomous.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import java.util.Objects;

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
     * Built Pedro path set plus the poses used to initialize/debug the follower.
     */
    public static final class Paths {
        /**
         * Starting Pedro pose for the selected spec.
         */
        public final Pose startPose;
        /**
         * First placeholder outbound/preload path.
         */
        public final PathChain outboundPath;
        /**
         * Placeholder return/park path.
         */
        public final PathChain returnPath;
        /**
         * Human-facing label for telemetry.
         */
        public final String label;

        private Paths(Pose startPose, PathChain outboundPath, PathChain returnPath, String label) {
            this.startPose = startPose;
            this.outboundPath = outboundPath;
            this.returnPath = returnPath;
            this.label = label;
        }
    }

    private final Follower follower;
    private final PhoenixProfile.AutoConfig autoCfg;

    /**
     * Create a path factory around the Pedro follower used by the OpMode.
     *
     * @param follower Pedro follower used for path building and execution
     * @param autoCfg  Phoenix Auto timing/path-placeholder config
     */
    public PhoenixPedroPathFactory(Follower follower, PhoenixProfile.AutoConfig autoCfg) {
        this.follower = Objects.requireNonNull(follower, "follower");
        this.autoCfg = autoCfg == null ? new PhoenixProfile.AutoConfig() : autoCfg;
    }

    /**
     * Build the currently checked-in placeholder path set for the selected Auto spec.
     *
     * <p>The path labels and start pose already depend on the spec, but the geometry is still the
     * old twelve-inch Pedro integration route. Replace this method's geometry branches when real
     * Decode routes are ready.</p>
     */
    public Paths build(PhoenixAutoSpec spec, PhoenixCapabilities capabilities) {
        Objects.requireNonNull(spec, "spec");
        Objects.requireNonNull(capabilities, "capabilities");

        double distanceIn = autoCfg.pedroIntegrationTestDistanceIn;
        Pose startPose = startPoseFor(spec);
        Pose forwardPose = new Pose(startPose.getX() + distanceIn, startPose.getY(), startPose.getHeading());

        follower.setStartingPose(startPose);
        follower.update();

        PathChain outbound = follower.pathBuilder()
                .addPath(new BezierLine(startPose, forwardPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), forwardPose.getHeading())
                .addParametricCallback(0.50, spinUpCallback(capabilities))
                .build();

        PathChain back = follower.pathBuilder()
                .addPath(new BezierLine(forwardPose, startPose))
                .setLinearHeadingInterpolation(forwardPose.getHeading(), startPose.getHeading())
                .build();

        return new Paths(startPose, outbound, back, labelFor(spec, distanceIn));
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
}
