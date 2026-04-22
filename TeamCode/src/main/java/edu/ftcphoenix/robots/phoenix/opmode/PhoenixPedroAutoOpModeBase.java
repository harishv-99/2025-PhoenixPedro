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
     * Advance the Phoenix Auto loop and Pedro debug telemetry.
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
        robot.updateAuto();

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
        telemetry.update();
    }

    /**
     * Stop Phoenix Auto and the Pedro adapter.
     */
    @Override
    public void stop() {
        if (robot != null) {
            robot.stopAuto();
            robot.stopAny();
            robot = null;
        }
        if (driveAdapter != null) {
            driveAdapter.stop();
            driveAdapter = null;
        }
        follower = null;
        capabilities = null;
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
     * not allocate another robot graph.</p>
     */
    protected final boolean initializeRobotForSpec(PhoenixAutoSpec spec) {
        if (isAutoInitialized()) {
            return true;
        }

        try {
            activeSpec = Objects.requireNonNull(spec, "spec");
            activeProfile = PhoenixAutoProfiles.profileFor(activeSpec, PhoenixProfile.current());

            robot = new PhoenixRobot(hardwareMap, telemetry, gamepad1, gamepad2, activeProfile);
            robot.initAny();
            robot.initAuto();
            capabilities = robot.capabilities();

            follower = createPedroFollower(hardwareMap);
            driveAdapter = new PedroPathingDriveAdapter(follower);

            PhoenixPedroPathFactory pathFactory = new PhoenixPedroPathFactory(follower, activeProfile.auto);
            PhoenixPedroPathFactory.Paths paths = pathFactory.build(activeSpec, capabilities);
            pathLabel = paths.label;

            PhoenixPedroAutoContext ctx = new PhoenixPedroAutoContext(
                    activeSpec,
                    activeProfile,
                    capabilities,
                    driveAdapter,
                    paths
            );
            robot.enqueueAuto(PhoenixPedroAutoRoutineFactory.build(ctx));

            telemetry.addLine("Phoenix Pedro auto ready");
            telemetry.addData("auto.spec", activeSpec.summary());
            telemetry.addData("auto.paths", pathLabel);
            telemetry.addLine("Follower factory: Constants.createFollower(hardwareMap)");
            telemetry.update();
            return true;
        } catch (RuntimeException e) {
            initError = buildInitError(e);
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

    private static String buildInitError(RuntimeException e) {
        String message = e.getMessage();
        if (message == null || message.isEmpty()) {
            message = e.getClass().getSimpleName();
        }
        return message + " | Check the Pedro Constants import or createPedroFollower(...) if your project uses a different package.";
    }
}
