package edu.ftcphoenix.robots.examples.pedro.adaptive;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import java.util.Objects;
import java.util.function.BiFunction;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.drive.route.RouteFollower;
import edu.ftcphoenix.fw.integrations.pedro.PedroFieldTransform;
import edu.ftcphoenix.fw.integrations.pedro.PedroPathingRuntime;

/**
 * Owns Pedro geometry for one optional adaptive-collection attempt.
 *
 * <p>Collection and return routes each snapshot the live Pedro pose once when their start-built
 * route factory runs. Selected targets are converted from Phoenix field coordinates; unavailable
 * decisions use the explicit fallback. Native Pedro callbacks only latch {@link Milestones}; the
 * sibling attempt owns exit and cancellation policy.</p>
 */
public final class AdaptiveCollectionPaths {

    /** Mutable, data-only path geometry and milestone configuration. */
    public static final class Config {
        /** Transform shared with the adopting Pedro runtime's field convention. */
        public PedroFieldTransform fieldTransform;
        /** Selected-band target X in the Phoenix FTC field frame, in inches. */
        public double collectionFieldXInches;
        /** Selected-band target heading in the Phoenix FTC field frame, in radians. */
        public double collectionFieldHeadingRad;
        /** No-selection fallback target in the Phoenix FTC field frame. */
        public Pose2d fallbackFieldToRobotPose;
        /** Return target in the Phoenix FTC field frame. */
        public Pose2d returnFieldToRobotPose;
        /** Interior path progress that publishes safe-to-leave. */
        public double safeToLeavePathT;
        /** Later interior path progress that publishes near-end. */
        public double nearEndPathT;

        private Config() {
        }

        /** Returns a fresh finite baseline, not physically reviewed route geometry. */
        public static Config defaults() {
            Config c = new Config();
            c.fieldTransform = PedroFieldTransform.decodeInvertedFtc();
            c.collectionFieldXInches = 36.0;
            c.collectionFieldHeadingRad = 0.0;
            c.fallbackFieldToRobotPose = new Pose2d(36.0, 0.0, 0.0);
            c.returnFieldToRobotPose = new Pose2d(0.0, 0.0, 0.0);
            c.safeToLeavePathT = 0.55;
            c.nearEndPathT = 0.90;
            return c;
        }
    }

    /** Fresh per-attempt facts written only by the collection path callbacks. */
    static final class Milestones {
        private boolean safeToLeave;
        private boolean nearEnd;

        boolean safeToLeave() {
            return safeToLeave;
        }

        boolean nearEnd() {
            return nearEnd;
        }

        void latchSafeToLeave() {
            safeToLeave = true;
        }

        void latchNearEnd() {
            nearEnd = true;
        }
    }

    private final RouteFollower<PathChain> routeFollower;
    private final BiFunction<AdaptiveCollectionVisionService.Decision, Milestones, PathChain>
            collectionRouteBuilder;
    private final Supplier<PathChain> returnRouteBuilder;

    /**
     * Captures validated config around the OpMode's sole Pedro runtime without sampling pose.
     *
     * @throws NullPointerException for a required null object
     * @throws IllegalArgumentException for non-finite geometry or incoherent milestone positions
     */
    public AdaptiveCollectionPaths(PedroPathingRuntime runtime, Config config) {
        PedroPathingRuntime requiredRuntime = Objects.requireNonNull(runtime, "runtime");
        Config owned = snapshot(config);
        routeFollower = Objects.requireNonNull(requiredRuntime.driveAdapter(), "runtime.driveAdapter()");
        collectionRouteBuilder = (decision, milestones) ->
                buildCollectionRoute(requiredRuntime, owned, decision, milestones);
        returnRouteBuilder = () -> routeBuilder(
                requiredRuntime,
                owned,
                owned.returnFieldToRobotPose
        ).build();
    }

    /** Hardware-neutral package seam for the sibling attempt's route-lifecycle tests. */
    AdaptiveCollectionPaths(
            RouteFollower<PathChain> routeFollower,
            BiFunction<AdaptiveCollectionVisionService.Decision, Milestones, PathChain>
                    collectionRouteBuilder,
            Supplier<PathChain> returnRouteBuilder) {
        this.routeFollower = Objects.requireNonNull(routeFollower, "routeFollower");
        this.collectionRouteBuilder = Objects.requireNonNull(
                collectionRouteBuilder,
                "collectionRouteBuilder"
        );
        this.returnRouteBuilder = Objects.requireNonNull(returnRouteBuilder, "returnRouteBuilder");
    }

    /** Returns the exact follower owned by the supplied Pedro runtime. */
    RouteFollower<PathChain> routeFollower() {
        return routeFollower;
    }

    /** Returns a fresh, initially clear latch for one collection route. */
    Milestones newMilestones() {
        return new Milestones();
    }

    /** Builds selected or explicit-fallback geometry without starting the follower. */
    PathChain buildCollectionFromCurrentPose(
            AdaptiveCollectionVisionService.Decision decision,
            Milestones milestones) {
        PathChain route = collectionRouteBuilder.apply(
                Objects.requireNonNull(decision, "decision"),
                Objects.requireNonNull(milestones, "milestones")
        );
        return Objects.requireNonNull(route, "collectionRouteBuilder returned null");
    }

    /** Builds return geometry from the then-current pose without starting the follower. */
    PathChain buildReturnFromCurrentPose() {
        return Objects.requireNonNull(returnRouteBuilder.get(), "returnRouteBuilder returned null");
    }

    /** Adds the selected/fallback endpoint and both semantic callbacks to one fresh builder. */
    private static PathChain buildCollectionRoute(
            PedroPathingRuntime runtime,
            Config config,
            AdaptiveCollectionVisionService.Decision decision,
            Milestones milestones) {
        Pose2d target = decision.hasSelection()
                ? new Pose2d(
                        config.collectionFieldXInches,
                        decision.selectedBandCenterYInches(),
                        config.collectionFieldHeadingRad
                )
                : config.fallbackFieldToRobotPose;
        return routeBuilder(runtime, config, target)
                .addParametricCallback(config.safeToLeavePathT, milestones::latchSafeToLeave)
                .addParametricCallback(config.nearEndPathT, milestones::latchNearEnd)
                .build();
    }

    /** Samples current pose and obtains exactly one runtime path builder for one route. */
    private static PathBuilder routeBuilder(PedroPathingRuntime runtime,
                                            Config config,
                                            Pose2d phoenixFieldTarget) {
        Pose start = Objects.requireNonNull(
                runtime.currentPedroPose(),
                "runtime.currentPedroPose()"
        );
        Pose target = config.fieldTransform.phoenixFieldToPedroPose(phoenixFieldTarget);
        return Objects.requireNonNull(runtime.pathBuilder(), "runtime.pathBuilder()")
                .addPath(curveFrom(start, target))
                .setLinearHeadingInterpolation(start.getHeading(), target.getHeading());
    }

    /** Uses a finite point curve for coincident translation, avoiding Pedro's zero-length line. */
    static Curve curveFrom(Pose sampledPedroStartPose, Pose pedroTargetPose) {
        Pose start = requirePedroPose(sampledPedroStartPose, "sampledPedroStartPose");
        Pose target = requirePedroPose(pedroTargetPose, "pedroTargetPose");
        return start.getX() == target.getX() && start.getY() == target.getY()
                ? new BezierPoint(target)
                : new BezierLine(start, target);
    }

    /** Rejects non-Pedro or non-finite endpoints before vendor path calculations. */
    private static Pose requirePedroPose(Pose pose, String name) {
        Pose value = Objects.requireNonNull(pose, name);
        if (value.getCoordinateSystem() != PedroCoordinates.INSTANCE) {
            throw new IllegalArgumentException(
                    name + " must be explicitly tagged with PedroCoordinates, got "
                            + value.getCoordinateSystem()
            );
        }
        finite(value.getX(), name + ".x");
        finite(value.getY(), name + ".y");
        finite(value.getHeading(), name + ".heading");
        return value;
    }

    /** Copies every retained value and validates it before any runtime access. */
    private static Config snapshot(Config source) {
        Config value = Objects.requireNonNull(source, "AdaptiveCollectionPaths.Config is required");
        Config owned = new Config();
        owned.fieldTransform = Objects.requireNonNull(
                value.fieldTransform,
                "AdaptiveCollectionPaths.Config.fieldTransform must not be null"
        );
        owned.collectionFieldXInches = finite(
                value.collectionFieldXInches,
                "AdaptiveCollectionPaths.Config.collectionFieldXInches"
        );
        owned.collectionFieldHeadingRad = finite(
                value.collectionFieldHeadingRad,
                "AdaptiveCollectionPaths.Config.collectionFieldHeadingRad"
        );
        owned.fallbackFieldToRobotPose = copyPose(
                value.fallbackFieldToRobotPose,
                "AdaptiveCollectionPaths.Config.fallbackFieldToRobotPose"
        );
        owned.returnFieldToRobotPose = copyPose(
                value.returnFieldToRobotPose,
                "AdaptiveCollectionPaths.Config.returnFieldToRobotPose"
        );
        owned.safeToLeavePathT = pathT(
                value.safeToLeavePathT,
                "AdaptiveCollectionPaths.Config.safeToLeavePathT"
        );
        owned.nearEndPathT = pathT(
                value.nearEndPathT,
                "AdaptiveCollectionPaths.Config.nearEndPathT"
        );
        if (owned.safeToLeavePathT >= owned.nearEndPathT) {
            throw new IllegalArgumentException(
                    "AdaptiveCollectionPaths.Config.safeToLeavePathT must be less than "
                            + "nearEndPathT, got " + owned.safeToLeavePathT + " and "
                            + owned.nearEndPathT
            );
        }
        return owned;
    }

    /** Copies one finite immutable Phoenix-field pose. */
    private static Pose2d copyPose(Pose2d pose, String name) {
        Pose2d value = Objects.requireNonNull(pose, name + " must not be null");
        return new Pose2d(
                finite(value.xInches, name + ".xInches"),
                finite(value.yInches, name + ".yInches"),
                finite(value.headingRad, name + ".headingRad")
        );
    }

    /** Requires a finite scalar and returns it for compact capture expressions. */
    private static double finite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite, got " + value);
        }
        return value;
    }

    /** Requires a finite interior Pedro path parameter. */
    private static double pathT(double value, String name) {
        finite(value, name);
        if (value <= 0.0 || value >= 1.0) {
            throw new IllegalArgumentException(name + " must be > 0 and < 1, got " + value);
        }
        return value;
    }
}
