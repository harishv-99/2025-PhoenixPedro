package edu.ftcphoenix.robots.phoenix.autonomous.pedro;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcphoenix.fw.drive.route.RouteTask;
import edu.ftcphoenix.fw.drive.route.RouteTasks;
import edu.ftcphoenix.fw.integrations.pedro.PedroPathingDriveAdapter;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.robots.phoenix.PhoenixRobot;

/**
 * Minimal Phoenix autonomous used to validate Pedro Pathing integration.
 *
 * <p>This lives under {@code autonomous/pedro} on purpose so all Pedro-specific robot code is
 * easy to find today and easy to extract into a dedicated source set or module later.</p>
 *
 * <p>Routine shape:</p>
 * <ol>
 *   <li>Drive forward about 12 inches on a Pedro {@link PathChain}.</li>
 *   <li>Run a mid-path callback that captures the current Phoenix shot velocity suggestion and
 *       spins up the flywheel while the robot is still moving.</li>
 *   <li>Wait briefly for a selected scoring target, then execute Phoenix aim logic.</li>
 *   <li>Shoot one ball through the Phoenix scoring supervisor.</li>
 *   <li>Drive back to the starting pose.</li>
 * </ol>
 *
 * <p>This OpMode intentionally keeps follower construction as one direct, project-owned call to
 * {@code Constants.createFollower(hardwareMap)}. If your team stores the Pedro constants class in
 * a different package, change the import above or edit {@link #createPedroFollower(HardwareMap)}.
 * Phoenix no longer carries a reflective helper for this setup step.</p>
 */
@Autonomous(name = "Phoenix: Pedro Auto Test", group = "Phoenix")
public final class PhoenixPedroAutoTestOpMode extends OpMode {

    private static final double TEST_DISTANCE_IN = 12.0;

    /**
     * Pedro's forward axis is +X, so a zero-heading start pose plus +12 X produces a simple
     * straight-ahead test move. Adjust these poses to match your own Pedro field setup.
     */
    private static final Pose START_POSE = new Pose(0.0, 0.0, 0.0);
    private static final Pose FORWARD_POSE = new Pose(TEST_DISTANCE_IN, 0.0, 0.0);

    private PhoenixRobot robot;
    private Follower follower;
    private PedroPathingDriveAdapter driveAdapter;
    private PathChain outboundPath;
    private PathChain returnPath;
    private String initError;

    @Override
    public void init() {
        robot = new PhoenixRobot(hardwareMap, telemetry, gamepad1, gamepad2);
        robot.initAny();

        try {
            robot.initAuto();

            follower = createPedroFollower(hardwareMap);
            driveAdapter = new PedroPathingDriveAdapter(follower);
            follower.setStartingPose(START_POSE);
            follower.update();

            buildPaths();
            queueAutoRoutine();

            telemetry.addLine("Phoenix Pedro auto ready");
            telemetry.addLine("Follower factory: Constants.createFollower(hardwareMap)");
            telemetry.addData("test.distanceIn", TEST_DISTANCE_IN);
            telemetry.addLine("Mid-path callback: spin up + capture shot velocity");
        } catch (RuntimeException e) {
            initError = buildInitError(e);
            telemetry.addLine("Phoenix Pedro auto init failed");
            telemetry.addLine(initError);
        }
        telemetry.update();
    }

    @Override
    public void start() {
        if (initError != null) {
            return;
        }
        robot.startAny(getRuntime());
        robot.startAuto();
    }

    @Override
    public void loop() {
        if (initError != null) {
            telemetry.addLine("Phoenix Pedro auto init failed");
            telemetry.addLine(initError);
            telemetry.update();
            return;
        }

        robot.updateAny(getRuntime());
        robot.updateAuto();

        if (follower != null) {
            telemetry.addData("pedro.busy", follower.isBusy());
            telemetry.addData("pedro.x", follower.getPose().getX());
            telemetry.addData("pedro.y", follower.getPose().getY());
            telemetry.addData("pedro.headingDeg", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stopAuto();
            robot.stopAny();
        }
        if (driveAdapter != null) {
            driveAdapter.stop();
        }
    }

    private void buildPaths() {
        outboundPath = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, FORWARD_POSE))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), FORWARD_POSE.getHeading())
                .addParametricCallback(0.50, new Runnable() {
                    @Override
                    public void run() {
                        robot.captureSuggestedShotVelocity();
                        robot.setFlywheelEnabled(true);
                    }
                })
                .build();

        returnPath = follower.pathBuilder()
                .addPath(new BezierLine(FORWARD_POSE, START_POSE))
                .setLinearHeadingInterpolation(FORWARD_POSE.getHeading(), START_POSE.getHeading())
                .build();
    }

    private void queueAutoRoutine() {
        RouteTask.Config routeCfg = new RouteTask.Config();
        routeCfg.timeoutSec = 4.0;

        DriveGuidanceTask.Config aimCfg = new DriveGuidanceTask.Config();
        aimCfg.headingTolRad = Math.toRadians(2.0);
        aimCfg.timeoutSec = 1.75;
        aimCfg.maxNoGuidanceSec = 0.75;

        Task aimAndMaybeShoot = Tasks.branchOnOutcome(
                robot.waitForTargetSelection(0.75),
                Tasks.sequence(
                        Tasks.runOnce(robot::captureSuggestedShotVelocity),
                        Tasks.branchOnOutcome(
                                robot.aimTask(driveAdapter, aimCfg),
                                Tasks.sequence(
                                        Tasks.runOnce(robot::requestSingleShot),
                                        robot.waitForShotCompletion(2.5)
                                ),
                                Tasks.noop()
                        )
                ),
                Tasks.noop()
        );

        Task auto = Tasks.sequence(
                RouteTasks.follow("pedro.outbound12in", driveAdapter, outboundPath, routeCfg),
                aimAndMaybeShoot,
                RouteTasks.follow("pedro.returnToStart", driveAdapter, returnPath, routeCfg),
                Tasks.runOnce(new Runnable() {
                    @Override
                    public void run() {
                        robot.setFlywheelEnabled(false);
                    }
                })
        );

        robot.enqueueAuto(auto);
    }

    private static Follower createPedroFollower(HardwareMap hardwareMap) {
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
