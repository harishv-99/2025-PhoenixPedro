package edu.ftcphoenix.fw.integrations.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Objects;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.drive.DriveCommandSink;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.route.RouteFollower;

/**
 * Framework adapter that bridges Pedro Pathing into Phoenix's small route/drive seams.
 *
 * <p>This adapter intentionally plays two roles:</p>
 * <ul>
 *   <li>{@link RouteFollower}&lt;{@link PathChain}&gt; so Phoenix tasks can sequence Pedro routes.</li>
 *   <li>{@link DriveCommandSink} so Phoenix guidance tasks (such as aim tasks) can temporarily
 *       command the same follower directly using normalized robot-centric drive signals.</li>
 * </ul>
 *
 * <p>Typical usage:</p>
 * <pre>{@code
 * Follower follower = Constants.createFollower(hardwareMap);
 * PedroPathingDriveAdapter adapter = new PedroPathingDriveAdapter(follower);
 *
 * Task auto = Tasks.sequence(
 *         RouteTasks.follow(adapter, outboundPath, new RouteTask.Config()),
 *         phoenixRobot.aimTask(adapter, aimCfg),
 *         Tasks.runOnce(phoenixRobot::requestSingleShot)
 * );
 * }</pre>
 */
public final class PedroPathingDriveAdapter implements RouteFollower<PathChain>, DriveCommandSink {

    private final Follower follower;
    private boolean teleOpDriveStarted = false;

    /**
     * Creates a Phoenix adapter around one Pedro {@link Follower} instance.
     *
     * @param follower Pedro follower to wrap
     */
    public PedroPathingDriveAdapter(Follower follower) {
        this.follower = Objects.requireNonNull(follower, "follower");
    }

    /**
     * Returns the wrapped Pedro follower for route building or advanced inspection.
     */
    public Follower follower() {
        return follower;
    }

    @Override
    public void update(LoopClock clock) {
        follower.update();
    }

    @Override
    public void follow(PathChain route) {
        follower.followPath(Objects.requireNonNull(route, "route"));
        teleOpDriveStarted = false;
    }

    /**
     * Begins following a route while requesting Pedro's hold-end behavior when supported.
     *
     * @param route   path chain to follow
     * @param holdEnd whether Pedro should hold the final point when the path completes
     */
    public void follow(PathChain route, boolean holdEnd) {
        Objects.requireNonNull(route, "route");
        if (!holdEnd) {
            follow(route);
            return;
        }

        try {
            Method m = follower.getClass().getMethod("followPath", PathChain.class, boolean.class);
            m.invoke(follower, route, true);
        } catch (NoSuchMethodException e) {
            follower.followPath(route);
        } catch (IllegalAccessException | InvocationTargetException e) {
            throw new IllegalStateException("Unable to start Pedro hold-end follow", e);
        }
        teleOpDriveStarted = false;
    }

    @Override
    public boolean isBusy() {
        return follower.isBusy();
    }

    @Override
    public void cancel() {
        if (!invokeNoArgIfPresent("breakFollowing")) {
            invokeNoArgIfPresent("pausePathFollowing");
        }
        zeroTeleOpDrive();
    }

    @Override
    public void drive(DriveSignal signal) {
        DriveSignal cmd = (signal != null) ? signal.clamped() : DriveSignal.zero();
        ensureTeleOpDriveStarted();
        invokeTeleOpDrive(cmd.axial, cmd.lateral, cmd.omega);
    }

    @Override
    public void stop() {
        if (follower.isBusy()) {
            cancel();
            return;
        }
        zeroTeleOpDrive();
    }

    private void zeroTeleOpDrive() {
        ensureTeleOpDriveStarted();
        invokeTeleOpDrive(0.0, 0.0, 0.0);
    }

    private void ensureTeleOpDriveStarted() {
        if (teleOpDriveStarted) {
            return;
        }
        if (!invokeNoArgIfPresent("startTeleOpDrive")) {
            invokeNoArgOrThrow("startTeleopDrive");
        }
        teleOpDriveStarted = true;
    }

    private void invokeTeleOpDrive(double axial, double lateral, double omega) {
        try {
            Method current = follower.getClass().getMethod(
                    "setTeleOpDrive",
                    double.class,
                    double.class,
                    double.class,
                    boolean.class
            );
            current.invoke(follower, axial, lateral, omega, true);
            return;
        } catch (NoSuchMethodException ignored) {
            // Fall through to legacy signature.
        } catch (IllegalAccessException | InvocationTargetException e) {
            throw new IllegalStateException("Unable to command Pedro teleop drive", e);
        }

        try {
            Method legacy = follower.getClass().getMethod(
                    "setTeleOpMovementVectors",
                    double.class,
                    double.class,
                    double.class
            );
            legacy.invoke(follower, axial, lateral, omega);
        } catch (NoSuchMethodException e) {
            throw new IllegalStateException(
                    "Pedro Follower is missing both setTeleOpDrive(...) and setTeleOpMovementVectors(...)"
            );
        } catch (IllegalAccessException | InvocationTargetException e) {
            throw new IllegalStateException("Unable to command legacy Pedro teleop drive", e);
        }
    }

    private boolean invokeNoArgIfPresent(String methodName) {
        try {
            Method m = follower.getClass().getMethod(methodName);
            m.invoke(follower);
            return true;
        } catch (NoSuchMethodException e) {
            return false;
        } catch (IllegalAccessException | InvocationTargetException e) {
            throw new IllegalStateException("Unable to invoke Pedro method: " + methodName, e);
        }
    }

    private void invokeNoArgOrThrow(String methodName) {
        if (!invokeNoArgIfPresent(methodName)) {
            throw new IllegalStateException("Pedro Follower is missing method: " + methodName + "()");
        }
    }
}
