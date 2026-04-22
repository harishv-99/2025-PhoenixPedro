package edu.ftcphoenix.robots.phoenix.opmode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Objects;

import edu.ftcphoenix.fw.integrations.pedro.PedroPathingDriveAdapter;
import edu.ftcphoenix.robots.phoenix.PhoenixCapabilities;
import edu.ftcphoenix.robots.phoenix.PhoenixProfile;
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
 * base class build the profile, Phoenix robot container, Pedro follower, path set, and routine. The
 * goal is to keep annotated OpModes as named entry points, not strategy scripts.</p>
 *
 * <p>The direct dependency on {@code Constants.createFollower(hardwareMap)} remains here because the
 * Pedro constants package is project-specific TeamCode setup. If a team stores Pedro constants in a
 * different package, update {@link #createPedroFollower(HardwareMap)} in one place instead of every
 * static autonomous entry.</p>
 *
 * <p>Initialization is retry-safe during INIT. A failed attempt tears down any partially-created
 * Phoenix or Pedro runtime before another spec is built, which keeps selector telemetry, queued
 * tasks, and live hardware owners from drifting apart.</p>
 */
public abstract class PhoenixPedroAutoOpModeBase extends OpMode {

    private PhoenixRobot robot;
    private PhoenixCapabilities capabilities;
    private Follower follower;
    private PedroPathingDriveAdapter driveAdapter;
    private PhoenixAutoSpec activeSpec;
    private PhoenixProfile activeProfile;
    private String pathLabel;
    private String initError;

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
     * Initialize the static autonomous entry when appropriate.
     */
    @Override
    public void init() {
        if (buildRobotInInit()) {
            initializeRobotForSpec(autoSpec());
        }
    }

    /**
     * Start the selected autonomous routine.
     */
    @Override
    public void start() {
        if (!isAutoInitialized() && initError == null) {
            initializeRobotForSpec(autoSpec());
        }
        if (initError != null || robot == null) {
            return;
        }
        robot.startAny(getRuntime());
        robot.startAuto();
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
        if (initError != null) {
            telemetry.addLine("Phoenix Pedro auto init failed");
            telemetry.addLine(initError);
            telemetry.update();
            return;
        }
        if (robot == null) {
            telemetry.addLine("Phoenix Pedro auto has not been initialized yet.");
            telemetry.update();
            return;
        }

        robot.updateAny(getRuntime());
        emitPedroDebugTelemetry();
        robot.updateAuto();
    }

    /**
     * Stop Phoenix Auto, the Pedro adapter, and any runtime left from a failed INIT retry.
     */
    @Override
    public void stop() {
        clearAutoRuntime();
        initError = null;
    }

    /**
     * Return whether the Phoenix robot + Pedro routine have been built successfully.
     */
    protected final boolean isAutoInitialized() {
        return robot != null && initError == null;
    }

    /**
     * Return the currently active spec, or null before initialization.
     */
    protected final PhoenixAutoSpec activeSpecOrNull() {
        return activeSpec;
    }

    /**
     * Return the current init error, or null if initialization has not failed.
     */
    protected final String initErrorOrNull() {
        return initError;
    }

    /**
     * Build Phoenix + Pedro runtime for a selected spec.
     *
     * <p>Selector OpModes may call this from {@code init_loop()} once the operator confirms a spec.
     * Repeated calls after a successful initialization are ignored so a second press of confirm does
     * not allocate another robot graph. If a previous attempt failed, this method clears the error,
     * tears down any partial runtime, and tries again from a clean state.</p>
     */
    protected final boolean initializeRobotForSpec(PhoenixAutoSpec spec) {
        if (isAutoInitialized()) {
            return true;
        }

        PhoenixAutoSpec requestedSpec = Objects.requireNonNull(spec, "spec");
        clearAutoRuntime();
        initError = null;

        PhoenixRobot builtRobot = null;
        Follower builtFollower = null;
        PedroPathingDriveAdapter builtDriveAdapter = null;

        try {
            PhoenixProfile builtProfile = PhoenixAutoProfiles.profileFor(requestedSpec, PhoenixProfile.current());

            builtRobot = new PhoenixRobot(hardwareMap, telemetry, gamepad1, gamepad2, builtProfile);
            builtRobot.initAny();
            builtRobot.initAuto();
            PhoenixCapabilities builtCapabilities = builtRobot.capabilities();

            builtFollower = createPedroFollower(hardwareMap);
            builtDriveAdapter = new PedroPathingDriveAdapter(builtFollower);

            PhoenixPedroPathFactory pathFactory = new PhoenixPedroPathFactory(builtFollower, builtProfile.auto);
            PhoenixPedroPathFactory.Paths paths = pathFactory.build(requestedSpec, builtCapabilities);

            PhoenixPedroAutoContext ctx = new PhoenixPedroAutoContext(
                    requestedSpec,
                    builtProfile,
                    builtCapabilities,
                    builtDriveAdapter,
                    paths
            );
            builtRobot.enqueueAuto(PhoenixPedroAutoRoutineFactory.build(ctx));

            activeSpec = requestedSpec;
            activeProfile = builtProfile;
            robot = builtRobot;
            capabilities = builtCapabilities;
            follower = builtFollower;
            driveAdapter = builtDriveAdapter;
            pathLabel = paths.label;

            telemetry.addLine("Phoenix Pedro auto ready");
            telemetry.addData("auto.spec", activeSpec.summary());
            telemetry.addData("auto.paths", pathLabel);
            telemetry.addLine("Follower factory: Constants.createFollower(hardwareMap)");
            telemetry.update();
            return true;
        } catch (RuntimeException e) {
            initError = buildInitError(e);
            safeStopDriveAdapter(builtDriveAdapter);
            safeStopRobot(builtRobot);
            clearRuntimeReferences();

            telemetry.addLine("Phoenix Pedro auto init failed");
            telemetry.addLine(initError);
            telemetry.update();
            return false;
        }
    }

    /**
     * Factory hook for the project-specific Pedro follower.
     */
    protected Follower createPedroFollower(HardwareMap hardwareMap) {
        return Constants.createFollower(Objects.requireNonNull(hardwareMap, "hardwareMap"));
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
    }

    private void clearAutoRuntime() {
        safeStopDriveAdapter(driveAdapter);
        safeStopRobot(robot);
        clearRuntimeReferences();
    }

    private void clearRuntimeReferences() {
        robot = null;
        capabilities = null;
        follower = null;
        driveAdapter = null;
        activeSpec = null;
        activeProfile = null;
        pathLabel = null;
    }

    private static void safeStopDriveAdapter(PedroPathingDriveAdapter adapter) {
        if (adapter == null) {
            return;
        }
        try {
            adapter.stop();
        } catch (RuntimeException ignored) {
            // Cleanup should never hide the original initialization or OpMode-stop reason.
        }
    }

    private static void safeStopRobot(PhoenixRobot runtime) {
        if (runtime == null) {
            return;
        }
        try {
            runtime.stopAuto();
        } catch (RuntimeException ignored) {
            // Continue stopping hardware owners even if the autonomous runner was only partly built.
        }
        try {
            runtime.stopAny();
        } catch (RuntimeException ignored) {
            // Cleanup should be best-effort during failed INIT retries and OpMode shutdown.
        }
    }

    private static String buildInitError(RuntimeException e) {
        String message = e.getMessage();
        if (message == null || message.isEmpty()) {
            message = e.getClass().getSimpleName();
        }
        return message + " | Check the Pedro Constants import or createPedroFollower(...) if your project uses a different package.";
    }
}
