package edu.ftcphoenix.robots.examples.pedro;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import java.util.Objects;

import edu.ftcphoenix.fw.integrations.pedro.PedroPathingRuntime;

/** One fixed, short Pedro-coordinate practice path for the basic Auto reference. */
public final class BasicPedroAutoPaths {

    // Explicit Pedro field coordinates: inches and CCW-positive radians.
    private static final double START_X_INCHES = 24.0;
    private static final double START_Y_INCHES = 24.0;
    private static final double START_HEADING_RAD = 0.0;
    private static final double END_X_INCHES = 36.0;
    private static final double END_Y_INCHES = 24.0;
    private static final double END_HEADING_RAD = 0.0;

    private final Pose pedroStartPose;
    private final PathChain practiceRoute;

    /** Builds the fixed path eagerly through the validated runtime path builder. */
    public BasicPedroAutoPaths(PedroPathingRuntime runtime) {
        PedroPathingRuntime requiredRuntime = Objects.requireNonNull(runtime, "runtime");
        pedroStartPose = new Pose(START_X_INCHES, START_Y_INCHES, START_HEADING_RAD);
        Pose pedroEndPose = new Pose(END_X_INCHES, END_Y_INCHES, END_HEADING_RAD);
        practiceRoute = requiredRuntime.pathBuilder()
                .addPath(new BezierLine(pedroStartPose, pedroEndPose))
                .setLinearHeadingInterpolation(
                        pedroStartPose.getHeading(),
                        pedroEndPose.getHeading()
                )
                .build();
    }

    /** Returns a defensive copy of the declared Pedro-coordinate physical start pose. */
    public Pose pedroStartPose() {
        return new Pose(
                pedroStartPose.getX(),
                pedroStartPose.getY(),
                pedroStartPose.getHeading()
        );
    }

    /** Returns the package-owned fixed practice route; callers must not mutate it. */
    PathChain practiceRoute() {
        return practiceRoute;
    }
}
