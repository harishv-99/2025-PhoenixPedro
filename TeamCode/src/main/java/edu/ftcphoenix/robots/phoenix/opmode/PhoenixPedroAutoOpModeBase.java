package edu.ftcphoenix.robots.phoenix.opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Objects;

import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.integrations.pedro.PedroPathingDriveAdapter;
import edu.ftcphoenix.fw.integrations.pedro.PedroPathingRuntime;
import edu.ftcphoenix.fw.localization.MotionPredictor;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.robots.phoenix.PhoenixCapabilities;
import edu.ftcphoenix.robots.phoenix.PhoenixMatchHandoff;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;
import edu.ftcphoenix.robots.phoenix.PhoenixReadiness;
import edu.ftcphoenix.robots.phoenix.PhoenixRobot;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoProfiles;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoSpec;
import edu.ftcphoenix.robots.phoenix.autonomous.pedro.PhoenixPedroAutoContext;
import edu.ftcphoenix.robots.phoenix.autonomous.pedro.PhoenixPedroAutoRoutineFactory;
import edu.ftcphoenix.robots.phoenix.autonomous.pedro.PhoenixPedroPathFactory;

/**
 * Shared FTC lifecycle glue for Phoenix autonomous routines that use Pedro Pathing.
 *
 * <p>Concrete Driver Station entries should be tiny: return a {@link PhoenixAutoSpec} and let this
 * base class build the profile, Phoenix robot container, Pedro follower, fixed paths, retained path
 * factory, and routine. The goal is to keep annotated OpModes as named entry points, not strategy
 * scripts.</p>
 *
 * <p>The direct dependency on {@code Constants.createPhoenixAutoRuntime(hardwareMap, profile)}
 * remains here because the Pedro constants package is project-specific TeamCode setup. If a team
 * stores Pedro constants in a different package, update
 * {@link #createPedroRuntime(HardwareMap, PhoenixProfile)} in one place instead of every static
 * autonomous entry.</p>
 *
 * <p>Initialization is retry-safe during INIT. A failed attempt tears down any partially-created
 * Phoenix or Pedro runtime before another spec is built, which keeps selector telemetry, the
 * installed routine, and live hardware owners from drifting apart.</p>
 *
 * <p>This mode client asks the project factory for the team-specific runtime, then transfers its
 * adapter's recurring update/final-stop lifecycle and its one shared motion predictor to
 * {@link PhoenixRobot#initAuto(DriveCommandSink, MotionPredictor)}. Route geometry and routine
 * composition remain here on the Auto side; the retained path factory may later read a pose
 * snapshot for start-time geometry, but the OpMode does not create a second Pedro heartbeat,
 * Pinpoint owner, or shutdown path.</p>
 *
 * <p>Before constructing hardware, this base evaluates one retained {@link PhoenixReadiness}
 * result from the exact spec, route maturity, calibration acknowledgements, and Driver Station
 * purpose. A blocker prevents construction and remains effective at START. The expected Pedro
 * start is also displayed as an operator placement requirement; applying that pose synchronizes
 * software coordinates but does not measure physical placement.</p>
 *
 * <p>A new Phoenix Auto clears the process-local Auto-to-TeleOp handoff during INIT. After a match
 * Auto has successfully crossed both Phoenix START calls, {@link #stop()} captures the retained
 * backend-neutral pose before cleanup and publishes it only after cleanup succeeds. Test, blocked,
 * failed-start, invalid-pose, and failed-cleanup runs leave no snapshot for TeleOp.</p>
 */
public abstract class PhoenixPedroAutoOpModeBase extends OpMode {

    private PhoenixRobot robot;
    private PedroPathingRuntime pedroRuntime;
    private MotionPredictor motionPredictor;
    private Follower follower;
    private PedroPathingDriveAdapter driveAdapter;
    private PhoenixAutoSpec activeSpec;
    private PhoenixPedroPathFactory.RouteAvailability routeAvailability;
    private PhoenixReadiness.Result readiness;
    private Pose expectedPedroStartPose;
    private String pathLabel;
    private RuntimeException initFailure;
    private RuntimeException unrecoveredCleanupFailure;
    private boolean startBoundaryReached;
    private boolean autoStartedSuccessfully;
    private boolean stopBoundaryHandled;

    /**
     * Return the autonomous setup for static OpModes, or the current selector state.
     */
    protected abstract PhoenixAutoSpec autoSpec();

    /**
     * Whether this OpMode should build the robot immediately in {@link #init()}.
     *
     * <p>Static entries return true. Selector-style entries override this to false so they can build
     * only after the operator has chosen or confirmed a spec.</p>
     */
    protected boolean buildRobotInInit() {
        return true;
    }

    /**
     * Declare whether this Driver Station entry is a match Auto or the explicit Pedro test entry.
     *
     * <p>Competition entries inherit the fail-closed default. Only the visibly named Pedro
     * integration-test OpMode overrides this method.</p>
     */
    PhoenixReadiness.AutoPurpose autoPurpose() {
        return PhoenixReadiness.AutoPurpose.MATCH_AUTO;
    }

    /**
     * Initialize the static autonomous entry when appropriate.
     */
    @Override
    public void init() {
        PhoenixMatchHandoff.clear();
        autoStartedSuccessfully = false;
        stopBoundaryHandled = false;
        if (buildRobotInInit()) {
            initializeRobotForSpec(autoSpec());
        }
    }

    /** Keep the selected readiness result and expected physical placement visible during INIT. */
    @Override
    public void init_loop() {
        emitAutoReadinessTelemetry();
        telemetry.update();
    }

    /**
     * Start the selected autonomous routine.
     */
    @Override
    public void start() {
        // Capture the FTC START boundary before a last-chance INIT retry can consume match time.
        // The first regular loop then charges any such setup delay to the pre-park budget.
        final double startRuntimeSec = getRuntime();
        startBoundaryReached = true;
        if (!isAutoInitialized()
                && initFailure == null
                && (readiness == null || readiness.isAllowed())) {
            initializeRobotForSpec(autoSpec());
        }
        if (!isAutoInitialized()) {
            emitAutoReadinessTelemetry();
            telemetry.update();
            return;
        }

        autoStartedSuccessfully = false;
        try {
            // Reassert the declared software coordinate origin at the exact FTC START boundary,
            // before the shared clock or Pedro heartbeat begins. This does not claim to measure
            // physical field placement; the expected placement remains an operator check.
            pedroRuntime.setStartingPose(expectedPedroStartPose);
            robot.startAny(startRuntimeSec);
            robot.startAuto();
            autoStartedSuccessfully = true;
        } catch (RuntimeException startFailure) {
            RuntimeException cleanupFailure = clearAutoRuntime();
            attachSuppressed(startFailure, cleanupFailure);
            if (cleanupFailure != null) {
                unrecoveredCleanupFailure = startFailure;
            }
            initFailure = startFailure;
            PhoenixMatchHandoff.clear();
            emitAutoReadinessTelemetry();
            telemetry.update();
        }
    }

    /**
     * Advance the Phoenix Auto loop and compose Pedro debug rows into the same telemetry frame.
     *
     * <p>Phoenix's Auto telemetry presenter owns the final {@code telemetry.update()} call. Pedro
     * path/spec rows are added first so the Driver Station sees one coherent frame instead of a
     * Phoenix frame followed by a second Pedro-only update.</p>
     */
    @Override
    public void loop() {
        if (!isAutoInitialized()) {
            emitAutoReadinessTelemetry();
            telemetry.update();
            return;
        }

        robot.updateAny(getRuntime());
        emitAutoReadinessTelemetry();
        emitPedroDebugTelemetry();
        robot.updateAuto();
    }

    /**
     * Stop Phoenix Auto and any runtime left from a failed INIT retry.
     *
     * <p>The successfully initialized {@link PhoenixRobot} owns the Pedro adapter's recurring
     * heartbeat and final stop. A successfully started match Auto snapshots the already-cached
     * predictor pose before that cleanup, then publishes only after cleanup succeeds. Repeated stop
     * calls are inert so they cannot erase an already-published snapshot.</p>
     */
    @Override
    public void stop() {
        if (stopBoundaryHandled) {
            return;
        }
        // Close the boundary before invoking robot or carrier callbacks so a reentrant/repeated
        // stop cannot erase a snapshot that this invocation publishes.
        stopBoundaryHandled = true;

        boolean shouldPublish = false;
        PoseEstimate finalPose = null;
        RuntimeException primaryFailure = null;
        try {
            shouldPublish = autoStartedSuccessfully
                    && autoPurpose() == PhoenixReadiness.AutoPurpose.MATCH_AUTO;
            if (shouldPublish) {
                // Read only the predictor's retained software snapshot. Do not perform a new
                // hardware or follower update after the FTC stop boundary.
                finalPose = Objects.requireNonNull(
                        Objects.requireNonNull(
                                motionPredictor,
                                "Started Phoenix Auto is missing its motion predictor"
                        ).getEstimate(),
                        "Started Phoenix Auto motion predictor returned a null final estimate"
                );
            }
        } catch (RuntimeException captureOrPolicyFailure) {
            primaryFailure = captureOrPolicyFailure;
            shouldPublish = false;
        }

        RuntimeException cleanupFailure = clearAutoRuntime();
        if (primaryFailure == null) {
            primaryFailure = cleanupFailure;
        } else {
            attachSuppressed(primaryFailure, cleanupFailure);
        }
        clearSelectionState();
        unrecoveredCleanupFailure = null;
        startBoundaryReached = false;
        autoStartedSuccessfully = false;

        if (primaryFailure == null && shouldPublish) {
            try {
                // Publication is intentionally the final successful lifecycle operation: TeleOp
                // must never receive state from an Auto whose cleanup did not complete.
                PhoenixMatchHandoff.publishFromAuto(this, finalPose);
            } catch (RuntimeException publicationFailure) {
                PhoenixMatchHandoff.clear();
                primaryFailure = publicationFailure;
            }
        } else {
            PhoenixMatchHandoff.clear();
        }

        if (primaryFailure != null) {
            throw primaryFailure;
        }
    }

    /**
     * Return whether the Phoenix robot + Pedro routine have been built successfully.
     */
    protected final boolean isAutoInitialized() {
        return robot != null
                && pedroRuntime != null
                && motionPredictor != null
                && expectedPedroStartPose != null
                && readiness != null
                && readiness.isAllowed()
                && initFailure == null;
    }

    /**
     * Return the currently active spec, or null before initialization.
     */
    final PhoenixAutoSpec activeSpecOrNull() {
        return activeSpec;
    }

    /**
     * Return the retained initialization, start, or cleanup error, or null when none has occurred.
     */
    final String initErrorOrNull() {
        return initFailure == null ? null : failureSummary(initFailure);
    }

    /** Return the retained immutable readiness result, or null before it has been evaluated. */
    final PhoenixReadiness.Result readinessOrNull() {
        return readiness;
    }

    /** Return whether a cleanup failure makes every later INIT rebuild unsafe. */
    final boolean cleanupFailureBlocksRetry() {
        return unrecoveredCleanupFailure != null;
    }

    /**
     * Emit the shared Auto arming state without completing the telemetry frame.
     *
     * <p>Static entries, the selector, and the running test entry all use this same presenter path,
     * so a blocker cannot be hidden by a different OpMode screen.</p>
     */
    final void emitAutoReadinessTelemetry() {
        if (initFailure != null) {
            emitAutoSelectionTelemetry(activeSpec, routeAvailability);
            if (autoPurpose() == PhoenixReadiness.AutoPurpose.PEDRO_INTEGRATION_TEST) {
                telemetry.addData("auto.purpose", "TEST");
            }
            telemetry.addData("auto.readiness", "ERROR");
            telemetry.addLine("Phoenix Pedro Auto initialization/start failed: "
                    + failureSummary(initFailure));
            for (Throwable suppressed : initFailure.getSuppressed()) {
                telemetry.addLine("Cleanup also failed: " + failureSummary(suppressed));
            }
            if (unrecoveredCleanupFailure != null) {
                telemetry.addLine(
                        "Retry disabled: stop and restart this OpMode after verifying all outputs are safe."
                );
            }
            emitReadinessIssues(readiness);
            return;
        }
        emitAutoReadinessTelemetry(activeSpec, routeAvailability, readiness);
    }

    /**
     * Emit the same readiness block for a selector preview that has not constructed hardware yet.
     */
    final void emitAutoReadinessTelemetry(
            PhoenixAutoSpec displayedSpec,
            PhoenixPedroPathFactory.RouteAvailability displayedAvailability,
            PhoenixReadiness.Result displayedReadiness
    ) {
        emitAutoSelectionTelemetry(displayedSpec, displayedAvailability);
        if (autoPurpose() == PhoenixReadiness.AutoPurpose.PEDRO_INTEGRATION_TEST) {
            telemetry.addData("auto.purpose", "TEST");
        }
        if (displayedReadiness == null) {
            telemetry.addData("auto.readiness", "NOT EVALUATED");
            telemetry.addLine("Phoenix Pedro Auto has not been evaluated yet.");
            return;
        }

        String status;
        if (!displayedReadiness.isAllowed()) {
            status = "BLOCKED";
        } else if (autoPurpose() == PhoenixReadiness.AutoPurpose.PEDRO_INTEGRATION_TEST) {
            status = "TEST";
        } else if (displayedReadiness.hasWarnings()) {
            status = "WARN";
        } else {
            status = "READY";
        }
        telemetry.addData("auto.readiness", status);
        emitReadinessIssues(displayedReadiness);
    }

    private void emitReadinessIssues(PhoenixReadiness.Result displayedReadiness) {
        if (displayedReadiness == null) {
            return;
        }
        for (PhoenixReadiness.Issue issue : displayedReadiness.issues()) {
            telemetry.addLine(
                    "Auto [" + issue.severity() + "] " + issue.message()
                            + " | " + issue.remediation()
            );
        }
    }

    private void emitAutoSelectionTelemetry(
            PhoenixAutoSpec displayedSpec,
            PhoenixPedroPathFactory.RouteAvailability displayedAvailability
    ) {
        if (displayedSpec != null) {
            telemetry.addData("auto.spec", displayedSpec.summary());
        }
        if (displayedAvailability == null) {
            return;
        }

        telemetry.addData("auto.routeMaturity", displayedAvailability.maturity);
        Pose expectedStart = displayedAvailability.expectedPedroStartPose;
        telemetry.addData(
                "auto.expectedPhysicalStartPedro",
                "x=%.1f in, y=%.1f in, heading=%.1f deg",
                expectedStart.getX(),
                expectedStart.getY(),
                Math.toDegrees(expectedStart.getHeading())
        );
        if (!startBoundaryReached) {
            telemetry.addLine(
                    "Place the robot at auto.expectedPhysicalStartPedro "
                            + "(Pedro field coordinates) before START."
            );
        }
    }

    /**
     * Build Phoenix + Pedro runtime for a selected spec.
     *
     * <p>Selector OpModes may call this from {@code init_loop()} once the operator confirms a spec.
     * Repeated calls for the same successfully initialized spec are ignored so a second press of
     * confirm does not allocate another robot graph; a different spec is rejected. If an ordinary
     * previous attempt failed, this method clears the error, tears down any partial runtime, and
     * tries again from a clean state. A cleanup failure remains terminal for this OpMode so a retry
     * cannot create a competing hardware owner.</p>
     */
    protected final boolean initializeRobotForSpec(PhoenixAutoSpec spec) {
        PhoenixAutoSpec requestedSpec = Objects.requireNonNull(spec, "spec");
        if (isAutoInitialized()) {
            if (sameSpec(activeSpec, requestedSpec)) {
                return true;
            }
            throw new IllegalStateException(
                    "Phoenix Pedro Auto is already initialized for " + activeSpec.summary()
                            + "; cannot replace it with " + requestedSpec.summary()
                            + ". Create a new OpMode runtime for a different spec."
            );
        }

        if (unrecoveredCleanupFailure != null) {
            initFailure = unrecoveredCleanupFailure;
            emitAutoReadinessTelemetry();
            telemetry.update();
            return false;
        }

        RuntimeException previousStopFailure = clearAutoRuntime();
        clearSelectionState();
        activeSpec = requestedSpec;
        if (previousStopFailure != null) {
            initFailure = previousStopFailure;
            unrecoveredCleanupFailure = previousStopFailure;
            emitAutoReadinessTelemetry();
            telemetry.update();
            return false;
        }

        PhoenixRobot builtRobot = null;
        Follower builtFollower;
        PedroPathingRuntime builtPedroRuntime;
        PedroPathingDriveAdapter builtDriveAdapter = null;
        MotionPredictor builtMotionPredictor;

        try {
            PhoenixProfile baseProfile = PhoenixProfile.current();
            routeAvailability = PhoenixPedroPathFactory.routeAvailabilityFor(requestedSpec);
            expectedPedroStartPose = routeAvailability.expectedPedroStartPose;
            readiness = PhoenixReadiness.pedroAuto(
                    requestedSpec,
                    baseProfile,
                    autoPurpose()
            );
            if (!readiness.isAllowed()) {
                emitAutoReadinessTelemetry();
                telemetry.update();
                return false;
            }

            PhoenixProfile builtProfile = PhoenixAutoProfiles.profileFor(requestedSpec, baseProfile);

            builtRobot = new PhoenixRobot(hardwareMap, telemetry, gamepad1, gamepad2, builtProfile);
            builtRobot.initAny();

            builtPedroRuntime = createPedroRuntime(hardwareMap, builtProfile);
            builtFollower = builtPedroRuntime.follower();
            builtDriveAdapter = builtPedroRuntime.driveAdapter();
            builtMotionPredictor = builtPedroRuntime.motionPredictor();
            builtRobot.initAuto(
                    builtDriveAdapter,
                    builtMotionPredictor
            );
            PhoenixCapabilities builtCapabilities = builtRobot.capabilities();

            PhoenixPedroPathFactory pathFactory = new PhoenixPedroPathFactory(
                    builtPedroRuntime,
                    builtProfile.auto
            );
            PhoenixPedroPathFactory.Paths paths = pathFactory.build(requestedSpec, builtCapabilities);
            builtPedroRuntime.setStartingPose(paths.pedroStartPose);

            PhoenixPedroAutoContext ctx = new PhoenixPedroAutoContext(
                    requestedSpec,
                    builtProfile,
                    builtCapabilities,
                    builtDriveAdapter,
                    pathFactory,
                    paths
            );
            builtRobot.installAutoRoutine(PhoenixPedroAutoRoutineFactory.build(ctx));

            robot = builtRobot;
            pedroRuntime = builtPedroRuntime;
            motionPredictor = builtMotionPredictor;
            follower = builtFollower;
            driveAdapter = builtDriveAdapter;
            pathLabel = paths.label;

            emitAutoReadinessTelemetry();
            telemetry.addData("auto.paths", pathLabel);
            telemetry.addLine("Pedro runtime: one Phoenix Pinpoint owner + passive Follower localizer");
            telemetry.update();
            return true;
        } catch (RuntimeException e) {
            // A nested constructor may have failed its own rollback before publishing an owner
            // reference. CleanupActions records that uncertainty as a suppressed failure, and a
            // later successful stop cannot prove that hidden hardware ownership was released.
            boolean inheritedCleanupFailure = e.getSuppressed().length > 0;
            RuntimeException cleanupFailure = stopRobot(builtRobot);
            attachSuppressed(e, cleanupFailure);
            if (inheritedCleanupFailure || cleanupFailure != null) {
                unrecoveredCleanupFailure = e;
            }
            clearRuntimeOwnerReferences();
            initFailure = e;

            emitAutoReadinessTelemetry();
            telemetry.update();
            return false;
        }
    }

    /**
     * Factory hook for the project-specific single-owner Pedro Auto runtime.
     */
    protected PedroPathingRuntime createPedroRuntime(HardwareMap hardwareMap,
                                                     PhoenixProfile profile) {
        return Constants.createPhoenixAutoRuntime(
                Objects.requireNonNull(hardwareMap, "hardwareMap"),
                Objects.requireNonNull(profile, "profile")
        );
    }

    private void emitPedroDebugTelemetry() {
        if (activeSpec != null) {
            telemetry.addData("auto.spec", activeSpec.summary());
        }
        if (pathLabel != null) {
            telemetry.addData("auto.paths", pathLabel);
        }
        if (follower != null) {
            telemetry.addData("pedro.busy", follower.isBusy());
            telemetry.addData("pedro.x", follower.getPose().getX());
            telemetry.addData("pedro.y", follower.getPose().getY());
            telemetry.addData("pedro.headingDeg", Math.toDegrees(follower.getPose().getHeading()));
        }
        if (driveAdapter != null) {
            telemetry.addData("route.status", driveAdapter.getLatestRouteStatus());
        }
    }

    private RuntimeException clearAutoRuntime() {
        RuntimeException stopFailure = stopRobot(robot);
        clearRuntimeOwnerReferences();
        return stopFailure;
    }

    private void clearRuntimeOwnerReferences() {
        robot = null;
        pedroRuntime = null;
        motionPredictor = null;
        follower = null;
        driveAdapter = null;
        pathLabel = null;
    }

    private void clearSelectionState() {
        activeSpec = null;
        routeAvailability = null;
        readiness = null;
        expectedPedroStartPose = null;
        initFailure = null;
    }

    private static RuntimeException stopRobot(PhoenixRobot runtime) {
        if (runtime == null) {
            return null;
        }
        try {
            runtime.stop();
            return null;
        } catch (RuntimeException stopFailure) {
            return stopFailure;
        }
    }

    private static void attachSuppressed(RuntimeException primary,
                                         RuntimeException cleanupFailure) {
        if (cleanupFailure != null && cleanupFailure != primary) {
            primary.addSuppressed(cleanupFailure);
        }
    }

    private static String failureSummary(Throwable failure) {
        String message = failure.getMessage();
        return message == null || message.trim().isEmpty()
                ? failure.getClass().getSimpleName()
                : message;
    }

    private static boolean sameSpec(PhoenixAutoSpec first, PhoenixAutoSpec second) {
        return first != null
                && second != null
                && first.alliance == second.alliance
                && first.startPosition == second.startPosition
                && first.partnerPlan == second.partnerPlan
                && first.strategy == second.strategy;
    }
}
