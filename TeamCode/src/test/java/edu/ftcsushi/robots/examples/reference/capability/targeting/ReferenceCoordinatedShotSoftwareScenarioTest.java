package edu.ftcsushi.robots.examples.reference.capability.targeting;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;

import edu.ftcsushi.fw.actuation.PlantTargetResolution;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.math.InterpolatingTable1D;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.localization.MotionDelta;
import edu.ftcsushi.fw.localization.MotionPredictor;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/** End-to-end software-device story for one coordinated snapshot and bounded turret output. */
public final class ReferenceCoordinatedShotSoftwareScenarioTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void oneSnapshotCoordinatesIntentsDegradesExplicitlyAndFailsClosed() {
        List<String> hardwareEvents = new ArrayList<String>();
        ScriptedMotionPredictor localization = new ScriptedMotionPredictor(hardwareEvents);
        ReferenceCoordinatedShotService.Config shotConfig = shotConfig();
        ReferenceTurretMechanism.Config turretConfig = turretConfig();
        FtcTestHardware hardware = new FtcTestHardware(hardwareEvents);
        FtcTestHardware.MotorProbe turretMotor = hardware.addMotor(turretConfig.motorName);
        ManagedScenarioHost host = new ManagedScenarioHost(
                localization,
                shotConfig,
                turretConfig
        );
        host.hardwareMap = hardware;
        host.telemetry = telemetryProxy();
        host.runtimeSec = 5.0;
        host.init();
        hardwareEvents.clear();

        // At the current observation the target is 100 inches forward and 20 inches left.
        // The robot moved 2 inches left in 0.20 seconds, so the illustrative compensation
        // subtracts another 2 inches of inherited leftward velocity during flight.
        host.runtimeSec = 10.0;
        host.start();
        ReferenceCoordinatedShotService.Solution moving = host.shots.solution();

        assertSame("the output must consume the already-published immutable snapshot",
                moving, host.shots.solution());
        assertEquals(ReferenceCoordinatedShotService.Mode.MOVING_COMPENSATED, moving.mode);
        assertEquals(ReferenceCoordinatedShotService.Reason.NONE, moving.reason);
        assertSame(localization.lastPublishedTimestamp, moving.observationTimestamp);
        assertEquals(100.0, moving.effectiveForwardInches, EPSILON);
        assertEquals(18.0, moving.effectiveLeftInches, EPSILON);
        assertEquals(0.80, moving.quality, EPSILON);

        double movingDistance = Math.hypot(100.0, 18.0);
        assertEquals(1000.0 + 10.0 * movingDistance,
                moving.flywheelVelocityTicksPerSec, EPSILON);
        assertEquals(0.20 + 0.003 * movingDistance, moving.hoodPosition, EPSILON);
        assertEquals(Math.atan2(18.0, 100.0),
                host.turret.targetResolution().target(), EPSILON);
        assertTrue(host.turret.targetResolution().satisfiesIntent());
        assertFalse(host.turret.targetResolution().usedFallback());
        assertSame(moving.observationTimestamp,
                host.turret.targetResolution().selectedTimestamp());
        assertEquals((int) Math.round(Math.atan2(18.0, 100.0) * 1000.0),
                turretMotor.targetPositionTicks());
        assertEquals(1, localization.startCalls);
        assertEquals(0, localization.updateCalls);
        assertEquals(1, localization.estimateReads);
        assertEquals(1, localization.deltaReads);

        // A current pose without a usable interval remains a truthful stationary solution.
        localization.nextMode = ScriptedMotionPredictor.Mode.STATIONARY;
        host.runtimeSec = 10.02;
        host.loop();
        ReferenceCoordinatedShotService.Solution stationary = host.shots.solution();

        assertEquals(ReferenceCoordinatedShotService.Mode.STATIONARY_FALLBACK,
                stationary.mode);
        assertEquals(ReferenceCoordinatedShotService.Reason.MOTION_UNAVAILABLE,
                stationary.reason);
        assertEquals(100.0, stationary.effectiveForwardInches, EPSILON);
        assertEquals(20.0, stationary.effectiveLeftInches, EPSILON);
        assertTrue(host.turret.targetResolution().satisfiesIntent());
        assertEquals(1, localization.updateCalls);
        assertEquals(2, localization.estimateReads);
        assertEquals(2, localization.deltaReads);

        // Losing geometry removes the active request; the turret reports a measured hold instead.
        double measuredHoldRad = 0.30;
        turretMotor.setCurrentPositionTicks((int) Math.round(measuredHoldRad * 1000.0));
        localization.nextMode = ScriptedMotionPredictor.Mode.UNAVAILABLE;
        host.runtimeSec = 10.04;
        host.loop();
        ReferenceCoordinatedShotService.Solution unavailable = host.shots.solution();

        assertEquals(ReferenceCoordinatedShotService.Mode.UNAVAILABLE, unavailable.mode);
        assertEquals(ReferenceCoordinatedShotService.Reason.SPATIAL_UNAVAILABLE,
                unavailable.reason);
        assertFalse(unavailable.turretRequest.hasAlternatives());
        assertEquals(shotConfig.unavailableFlywheelVelocityTicksPerSec,
                unavailable.flywheelVelocityTicksPerSec, 0.0);
        assertEquals(shotConfig.unavailableHoodPosition, unavailable.hoodPosition, 0.0);
        assertEquals(PlantTargetResolution.Kind.HOLD_MEASURED_TARGET,
                host.turret.targetResolution().kind());
        assertTrue(host.turret.targetResolution().usedFallback());
        assertFalse(host.turret.targetResolution().satisfiesIntent());
        assertEquals(measuredHoldRad, host.turret.targetResolution().target(), EPSILON);
        assertEquals(2, localization.updateCalls);
        assertEquals(3, localization.estimateReads);
        assertEquals("unavailable geometry completes before motion is sampled",
                2, localization.deltaReads);

        // Managed cleanup stops the downstream output before invalidating the upstream snapshot.
        hardwareEvents.clear();
        localization.observeCleanup(host.shots, turretMotor);
        host.stop();
        assertEquals(0.0, turretMotor.power(), 0.0);
        assertEquals(ReferenceCoordinatedShotService.Reason.STOPPED,
                host.shots.solution().reason);
        assertEquals(1, localization.stopCalls);
        assertEquals(ReferenceCoordinatedShotService.Reason.STOPPED,
                localization.shotReasonObservedAtStop);
        assertEquals(0.0, localization.turretPowerObservedAtStop, 0.0);
        assertTrue(hardwareEvents.indexOf("power:coordinatedTurret:0.0")
                < hardwareEvents.indexOf("localization.stop"));

        int eventCount = hardwareEvents.size();
        host.stop();
        assertEquals(eventCount, hardwareEvents.size());
        assertEquals(1, localization.stopCalls);
    }

    private static ReferenceCoordinatedShotService.Config shotConfig() {
        ReferenceCoordinatedShotService.Config config =
                ReferenceCoordinatedShotService.Config.defaults();
        config.targetFieldXInches = 100.0;
        config.targetFieldYInches = 22.0;
        config.spatialMaxAgeSec = 0.10;
        config.spatialMinQuality = 0.50;
        config.motionMaxAgeSec = 0.10;
        config.motionMinQuality = 0.50;
        config.motionMaxIntervalSec = 0.25;
        config.illustrativeFlightTimeSec = 0.20;
        config.minimumModelDistanceInches = 0.0;
        config.maximumModelDistanceInches = 200.0;
        config.flywheelVelocityTicksPerSecByDistance =
                InterpolatingTable1D.ofSortedPairs(0.0, 1000.0, 200.0, 3000.0);
        config.hoodPositionByDistance =
                InterpolatingTable1D.ofSortedPairs(0.0, 0.20, 200.0, 0.80);
        config.minimumFlywheelVelocityTicksPerSec = 0.0;
        config.maximumFlywheelVelocityTicksPerSec = 3000.0;
        config.minimumHoodPosition = 0.0;
        config.maximumHoodPosition = 1.0;
        config.unavailableFlywheelVelocityTicksPerSec = 0.0;
        config.unavailableHoodPosition = 0.20;
        return config;
    }

    private static ReferenceTurretMechanism.Config turretConfig() {
        ReferenceTurretMechanism.Config config = ReferenceTurretMechanism.Config.defaults();
        config.motorName = "coordinatedTurret";
        config.minimumAngleRad = -Math.PI;
        config.maximumAngleRad = Math.PI;
        config.ticksPerRad = 1000.0;
        config.positionToleranceRad = 0.01;
        config.initialHoldAngleRad = 0.0;
        return config;
    }

    private static Pose3d pose(double xInches, double yInches, double yawRad) {
        return new Pose3d(xInches, yInches, 0.0, yawRad, 0.0, 0.0);
    }

    private static final class ManagedScenarioHost extends FtcRobotOpMode {
        private final ScriptedMotionPredictor localization;
        private final ReferenceCoordinatedShotService.Config shotConfig;
        private final ReferenceTurretMechanism.Config turretConfig;
        private double runtimeSec;
        private ReferenceCoordinatedShotService shots;
        private ReferenceTurretMechanism turret;

        private ManagedScenarioHost(
                ScriptedMotionPredictor localization,
                ReferenceCoordinatedShotService.Config shotConfig,
                ReferenceTurretMechanism.Config turretConfig) {
            this.localization = localization;
            this.shotConfig = shotConfig;
            this.turretConfig = turretConfig;
        }

        @Override
        protected void configure(RobotProgram program) {
            program.service(localization);
            shots = program.service(
                    new ReferenceCoordinatedShotService(localization, shotConfig));
            turret = program.output(
                    new ReferenceTurretMechanism(hardwareMap, turretConfig, shots));
        }

        @Override
        public double getRuntime() {
            return runtimeSec;
        }
    }

    private static final class ScriptedMotionPredictor
            implements MotionPredictor, RobotProgram.Service {
        private enum Mode {
            MOVING,
            STATIONARY,
            UNAVAILABLE
        }

        private PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        private MotionDelta delta = MotionDelta.none(LoopTimestamp.unavailable());
        private final List<String> events;
        private Mode nextMode = Mode.MOVING;
        private ReferenceCoordinatedShotService cleanupShotService;
        private FtcTestHardware.MotorProbe cleanupTurretMotor;
        private LoopTimestamp lastPublishedTimestamp = LoopTimestamp.unavailable();
        private ReferenceCoordinatedShotService.Reason shotReasonObservedAtStop;
        private double turretPowerObservedAtStop = Double.NaN;
        private int startCalls;
        private int updateCalls;
        private int stopCalls;
        private int estimateReads;
        private int deltaReads;

        private ScriptedMotionPredictor(List<String> events) {
            this.events = events;
        }

        @Override
        public void start(LoopClock clock) {
            startCalls++;
            publishNext(clock);
        }

        private void observeCleanup(ReferenceCoordinatedShotService shotService,
                                    FtcTestHardware.MotorProbe turretMotor) {
            cleanupShotService = shotService;
            cleanupTurretMotor = turretMotor;
        }

        @Override
        public void update(LoopClock clock) {
            updateCalls++;
            publishNext(clock);
        }

        @Override
        public void stop() {
            stopCalls++;
            shotReasonObservedAtStop = cleanupShotService.solution().reason;
            turretPowerObservedAtStop = cleanupTurretMotor.power();
            events.add("localization.stop");
        }

        @Override
        public PoseEstimate getEstimate() {
            estimateReads++;
            return estimate;
        }

        @Override
        public MotionDelta getLatestMotionDelta() {
            deltaReads++;
            return delta;
        }

        @Override
        public long trajectorySegmentId() {
            return 0L;
        }

        private void publishNext(LoopClock clock) {
            lastPublishedTimestamp = clock.nowTimestamp();
            if (nextMode == Mode.MOVING) {
                estimate = new PoseEstimate(
                        pose(0.0, 2.0, 0.0),
                        true,
                        0.90,
                        lastPublishedTimestamp
                );
                delta = new MotionDelta(
                        pose(0.0, 2.0, 0.0),
                        true,
                        0.80,
                        clock.timestampSecondsAgo(0.20),
                        lastPublishedTimestamp
                );
            } else if (nextMode == Mode.STATIONARY) {
                estimate = new PoseEstimate(
                        pose(0.0, 2.0, 0.0),
                        true,
                        0.90,
                        lastPublishedTimestamp
                );
                delta = MotionDelta.none(lastPublishedTimestamp);
            } else {
                estimate = new PoseEstimate(
                        Pose3d.zero(),
                        false,
                        0.0,
                        lastPublishedTimestamp
                );
                delta = MotionDelta.none(lastPublishedTimestamp);
            }
        }
    }

    private static Telemetry telemetryProxy() {
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (proxy, method, args) -> defaultValue(method)
        );
    }

    private static Object defaultValue(Method method) {
        Class<?> type = method.getReturnType();
        if (!type.isPrimitive()) {
            return null;
        }
        if (type == boolean.class) {
            return false;
        }
        if (type == char.class) {
            return '\0';
        }
        if (type == byte.class) {
            return (byte) 0;
        }
        if (type == short.class) {
            return (short) 0;
        }
        if (type == int.class) {
            return 0;
        }
        if (type == long.class) {
            return 0L;
        }
        if (type == float.class) {
            return 0.0f;
        }
        if (type == double.class) {
            return 0.0;
        }
        throw new AssertionError(
                "Unsupported primitive telemetry return type: " + type.getName());
    }
}
