package edu.ftcphoenix.robots.phoenix.opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import edu.ftcphoenix.fw.drive.guidance.DriveGuidanceTask;
import edu.ftcphoenix.fw.drive.route.RouteTask;
import edu.ftcphoenix.fw.drive.route.RouteTasks;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.Tasks;
import edu.ftcphoenix.robots.phoenix.PhoenixRobot;
import edu.ftcphoenix.robots.phoenix.pedro.PedroPathingDriveAdapter;
import edu.ftcphoenix.robots.phoenix.pedro.PedroPathingFollowers;

/**
 * Minimal Phoenix autonomous used to validate Pedro Pathing integration.
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
 * <p>This OpMode is intentionally a testing ground rather than a full match autonomous. Update
 * {@link #PEDRO_FOLLOWER_FACTORY_CLASSES} so one of the entries matches your team's Pedro constants
 * class if your project uses a different package layout.</p>
 */
@Autonomous(name = "Phoenix: Pedro Auto Test", group = "Phoenix")
public final class PhoenixPedroAutoTestOpMode extends OpMode {

    /**
     * Candidate team-owned classes that expose {@code static Follower createFollower(HardwareMap)}.
     *
     * <p>These cover the two most common Pedro package layouts used in the docs and sample code.
     * Add your own class name here if your project keeps its Pedro constants somewhere else.</p>
     */
    private static final String[] PEDRO_FOLLOWER_FACTORY_CLASSES = new String[]{
            "org.firstinspires.ftc.teamcode.pedroPathing.Constants",
            "org.firstinspires.ftc.teamcode.pedro.Constants"
    };

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

            follower = PedroPathingFollowers.createFollower(hardwareMap, PEDRO_FOLLOWER_FACTORY_CLASSES);
            driveAdapter = new PedroPathingDriveAdapter(follower);
            follower.setStartingPose(START_POSE);
            follower.update();

            buildPaths();
            queueAutoRoutine();

            telemetry.addLine("Phoenix Pedro auto ready");
            telemetry.addData("pedro.factoryCandidates", String.join(", ", PEDRO_FOLLOWER_FACTORY_CLASSES));
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

    private static String buildInitError(RuntimeException e) {
        String message = e.getMessage();
        if (message == null || message.isEmpty()) {
            message = e.getClass().getSimpleName();
        }
        return message + " | Check PEDRO_FOLLOWER_FACTORY_CLASSES if your Constants class lives elsewhere.";
    }
}
