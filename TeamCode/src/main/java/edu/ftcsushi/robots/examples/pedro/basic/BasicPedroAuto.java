package edu.ftcsushi.robots.examples.pedro.basic;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Objects;

import edu.ftcsushi.fw.core.lifecycle.CleanupActions;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.drive.route.RouteFollower;
import edu.ftcsushi.fw.drive.route.RouteTask;
import edu.ftcsushi.fw.drive.route.RouteTasks;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.integrations.pedro.PedroPathingRuntime;

/**
 * Smallest complete managed Pedro lesson: configure one runtime, author one fixed route, and
 * expose that exact route attempt's status and outcome.
 *
 * <p>The route is fixed, so it is built eagerly during configuration. The private service remains
 * active for the whole OpMode: it applies the starting pose at START, updates Pinpoint before the
 * Pedro adapter each cycle, and gives the adapter final drivetrain STOP ownership. The route
 * {@link RouteTask} may also reach the adapter in the Task phase; the adapter deduplicates that
 * same-cycle call.</p>
 *
 * <p>This compiling example is disabled and motion permission is false. Its defaults are software
 * placeholders, not reviewed wiring, odometry placement, tuning, route clearance, or safe motion.
 * Copy it into a team package and complete the documented physical review before enabling it.</p>
 */
@Autonomous(name = "FW Pedro Auto: One Route", group = "Framework Examples")
@Disabled
public final class BasicPedroAuto extends FtcRobotOpMode {

    private static final boolean ROBOT_MOTION_REVIEWED = false;
    private static final double ROUTE_TIMEOUT_SEC = 4.0;

    private static final double START_X_INCHES = 24.0;
    private static final double START_Y_INCHES = 24.0;
    private static final double END_X_INCHES = 36.0;
    private static final double END_Y_INCHES = 24.0;
    private static final double HEADING_RAD = 0.0;

    /** Construct the fixed route graph and declare its one managed lifecycle. */
    @Override
    protected void configure(RobotProgram program) {
        requireMotionReview();

        PedroPathingRuntime runtime = PedroPathingRuntime.create(
                hardwareMap,
                exampleRuntimeConfig()
        );
        Pose startPose = new Pose(START_X_INCHES, START_Y_INCHES, HEADING_RAD);

        // Register lifecycle ownership before later route construction can fail.
        registerServiceOrStop(program, new PedroHeartbeat(runtime, startPose));

        PathChain route = runtime.pathBuilder()
                .addPath(new BezierLine(
                        startPose,
                        new Pose(END_X_INCHES, END_Y_INCHES, HEADING_RAD)
                ))
                .setLinearHeadingInterpolation(HEADING_RAD, HEADING_RAD)
                .build();
        RouteTask<PathChain> routeTask = routeTask(runtime.driveAdapter(), route);
        program.rootTask(routeTask);

        program.presenter((clock, telemetry) -> {
            telemetry.addLine("Basic Pedro lesson: DISABLED TEST ONLY");
            telemetry.addLine("Review wiring, Pinpoint, tuning, route clearance, and STOP first.");
            telemetry.addData(
                    "route.expectedStartPedro",
                    "x=%.1f in, y=%.1f in, heading=%.1f deg",
                    START_X_INCHES,
                    START_Y_INCHES,
                    Math.toDegrees(HEADING_RAD)
            );
            telemetry.addData(
                    "route.status",
                    runtime.driveAdapter().getLatestRouteStatus()
            );
            telemetry.addData("route.outcome", routeTask.getOutcome());
        });
    }

    /**
     * Build one fresh Task for the already-authored route.
     *
     * <p>This package-local seam lets the hardware-free lesson test the route boundary without
     * constructing FTC or Pedro hardware. A fixed route is supplied eagerly; live-pose or
     * vision-dependent geometry instead belongs in a clearly named built-at-start route factory.</p>
     */
    static RouteTask<PathChain> routeTask(RouteFollower<PathChain> follower, PathChain route) {
        return RouteTasks.follow(
                "basicPedro.oneRoute",
                Objects.requireNonNull(follower, "follower"),
                Objects.requireNonNull(route, "route"),
                ROUTE_TIMEOUT_SEC
        );
    }

    /**
     * Author one independent software baseline; these values are not physical evidence.
     *
     * <p>Pedro restores its own following power when a path begins, so the {@code 0.25} initial
     * drivetrain value below is not a durable route-speed limit. Motion remains blocked until the
     * integration offers a reviewed persistent limit and every adopting-robot fact is validated.</p>
     */
    private static PedroPathingRuntime.Config exampleRuntimeConfig() {
        PedroPathingRuntime.Config config = PedroPathingRuntime.Config.defaults();
        config.mecanumConstants.leftFrontMotorName = "frontLeftMotor";
        config.mecanumConstants.leftRearMotorName = "backLeftMotor";
        config.mecanumConstants.rightFrontMotorName = "frontRightMotor";
        config.mecanumConstants.rightRearMotorName = "backRightMotor";
        config.mecanumConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        config.mecanumConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        config.mecanumConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        config.mecanumConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        config.mecanumConstants.maxPower = 0.25;
        config.mecanumConstants.useBrakeModeInTeleOp = true;
        return config;
    }

    /** Reject the example before any hardware lookup until a team completes physical review. */
    private static void requireMotionReview() {
        if (!ROBOT_MOTION_REVIEWED) {
            throw new IllegalStateException(
                    "ROBOT_MOTION_REVIEWED is false. Keep this Pedro example disabled: managed "
                            + "routes cannot yet set Pedro's persistent Follower power limit. Add "
                            + "and review that control, then review the drivetrain, Pinpoint, "
                            + "follower tuning, route, clear space, and STOP plan before motion."
            );
        }
    }

    /** Register the sole heartbeat owner, stopping its drive if registration itself fails. */
    private static void registerServiceOrStop(RobotProgram program, PedroHeartbeat heartbeat) {
        try {
            Objects.requireNonNull(program, "program").service(heartbeat);
        } catch (RuntimeException registrationFailure) {
            throw CleanupActions.attemptAllAfterFailure(
                    registrationFailure,
                    heartbeat::stop
            );
        }
    }

    /** Owns exact-start pose application, localization, Pedro heartbeat, and drivetrain STOP. */
    private static final class PedroHeartbeat implements RobotProgram.Service {
        private final PedroPathingRuntime runtime;
        private final Pose startPose;

        private PedroHeartbeat(PedroPathingRuntime runtime, Pose startPose) {
            this.runtime = Objects.requireNonNull(runtime, "runtime");
            Pose requiredPose = Objects.requireNonNull(startPose, "startPose");
            this.startPose = new Pose(
                    requiredPose.getX(),
                    requiredPose.getY(),
                    requiredPose.getHeading()
            );
        }

        @Override
        public void start(LoopClock clock) {
            runtime.setStartingPose(new Pose(
                    startPose.getX(),
                    startPose.getY(),
                    startPose.getHeading()
            ));
            update(clock);
        }

        @Override
        public void update(LoopClock clock) {
            runtime.motionPredictor().update(clock);
            runtime.driveAdapter().update(clock);
        }

        @Override
        public void stop() {
            runtime.driveAdapter().stop();
        }
    }
}
