package edu.ftcsushi.robots.examples.reference.capability.targeting;

import org.junit.Test;

import edu.ftcsushi.fw.actuation.PlantTargetResolution;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.localization.MotionDelta;
import edu.ftcsushi.fw.localization.MotionPredictor;
import edu.ftcsushi.fw.localization.PoseEstimate;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/** Contract tests for the optional Reference bounded turret realization owner. */
public final class ReferenceTurretMechanismTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void freshRequestSelectsNearestReachableFullTurnEquivalent() {
        ServiceFixture service = startedService();
        double requestedAngleRad = angleOf(service.service.solution());
        double expectedPhysicalAngleRad = requestedAngleRad + 2.0 * Math.PI;

        ReferenceTurretMechanism.Config config = mechanismConfig();
        config.minimumAngleRad = expectedPhysicalAngleRad - 0.50;
        config.maximumAngleRad = expectedPhysicalAngleRad + 0.50;
        config.initialHoldAngleRad = expectedPhysicalAngleRad;
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.MotorProbe motor = hardware.addMotor(config.motorName);
        motor.setCurrentPositionTicks(toTicks(expectedPhysicalAngleRad, config.ticksPerRad));
        ReferenceTurretMechanism mechanism =
                new ReferenceTurretMechanism(hardware, config, service.service);

        mechanism.update(service.time.clock());

        PlantTargetResolution resolution = mechanism.targetResolution();
        assertEquals(PlantTargetResolution.Kind.PLANNED_CANDIDATE, resolution.kind());
        assertTrue(resolution.satisfiesIntent());
        assertFalse(resolution.usedFallback());
        assertEquals("reference-coordinated-shot", resolution.selectedCandidateId());
        assertEquals(expectedPhysicalAngleRad, resolution.target(), EPSILON);
        assertSame(service.service.solution().observationTimestamp,
                resolution.selectedTimestamp());
        assertEquals(toTicks(expectedPhysicalAngleRad, config.ticksPerRad),
                motor.targetPositionTicks());
    }

    @Test
    public void acceptedMovingQualityCannotBeRejectedByTheSiblingPlannerGate() {
        ManualLoopClock time = new ManualLoopClock(10.0);
        FakeMotionPredictor predictor = new FakeMotionPredictor();
        ReferenceCoordinatedShotService.Config shotConfig =
                ReferenceCoordinatedShotService.Config.defaults();
        shotConfig.spatialMinQuality = 0.80;
        shotConfig.motionMinQuality = 0.20;
        LoopTimestamp end = time.clock().nowTimestamp();
        predictor.estimate = new PoseEstimate(Pose3d.zero(), true, 0.90, end);
        predictor.delta = new MotionDelta(
                Pose3d.zero(),
                true,
                0.30,
                time.clock().timestampSecondsAgo(0.05),
                end
        );
        ReferenceCoordinatedShotService service =
                new ReferenceCoordinatedShotService(predictor, shotConfig);
        service.start(time.clock());
        assertEquals(ReferenceCoordinatedShotService.Mode.MOVING_COMPENSATED,
                service.solution().mode);
        assertEquals(0.30, service.solution().quality, 0.0);

        ReferenceTurretMechanism.Config turretConfig = mechanismConfig();
        FtcTestHardware hardware = new FtcTestHardware();
        hardware.addMotor(turretConfig.motorName);
        ReferenceTurretMechanism mechanism =
                new ReferenceTurretMechanism(hardware, turretConfig, service);

        mechanism.update(time.clock());

        assertTrue(mechanism.targetResolution().satisfiesIntent());
        assertEquals(0.30, mechanism.targetResolution().selectedQuality(), 0.0);
    }

    @Test
    public void unreachableEquivalentFamilyTruthfullyHoldsMeasuredPosition() {
        ServiceFixture service = startedService();
        double requestedAngleRad = angleOf(service.service.solution());
        assertTrue("fixture request should stay outside the deliberately narrow cable range",
                requestedAngleRad < 1.50 || requestedAngleRad > 2.00);

        ReferenceTurretMechanism.Config config = mechanismConfig();
        config.minimumAngleRad = 1.50;
        config.maximumAngleRad = 2.00;
        config.initialHoldAngleRad = 1.75;
        double measuredAngleRad = 1.80;
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.MotorProbe motor = hardware.addMotor(config.motorName);
        motor.setCurrentPositionTicks(toTicks(measuredAngleRad, config.ticksPerRad));
        ReferenceTurretMechanism mechanism =
                new ReferenceTurretMechanism(hardware, config, service.service);

        mechanism.update(service.time.clock());

        PlantTargetResolution resolution = mechanism.targetResolution();
        assertEquals(PlantTargetResolution.Kind.HOLD_MEASURED_TARGET, resolution.kind());
        assertFalse(resolution.satisfiesIntent());
        assertTrue(resolution.usedFallback());
        assertEquals(measuredAngleRad, resolution.target(), EPSILON);
        assertEquals(toTicks(measuredAngleRad, config.ticksPerRad),
                motor.targetPositionTicks());
        assertTrue(resolution.reason().contains("reachable"));
    }

    @Test
    public void staleCachedObservationEntersMeasuredHoldAndFreshServicePublicationRecovers() {
        ServiceFixture service = startedService();
        ReferenceTurretMechanism.Config config = mechanismConfig();
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.MotorProbe motor = hardware.addMotor(config.motorName);
        ReferenceTurretMechanism mechanism =
                new ReferenceTurretMechanism(hardware, config, service.service);

        mechanism.update(service.time.clock());
        assertTrue(mechanism.targetResolution().satisfiesIntent());

        double heldMeasurementRad = 0.20;
        motor.setCurrentPositionTicks(toTicks(heldMeasurementRad, config.ticksPerRad));
        mechanism.update(service.time.nextCycle(0.11));

        PlantTargetResolution stale = mechanism.targetResolution();
        assertEquals(PlantTargetResolution.Kind.HOLD_MEASURED_TARGET, stale.kind());
        assertEquals(heldMeasurementRad, stale.target(), EPSILON);
        assertTrue(stale.reason().contains("age"));

        service.publishStationaryCurrentPose(service.time.nextCycle(0.01));
        service.service.update(service.time.clock());
        mechanism.update(service.time.clock());

        assertTrue(mechanism.targetResolution().satisfiesIntent());
        assertEquals(PlantTargetResolution.Kind.PLANNED_CANDIDATE,
                mechanism.targetResolution().kind());
    }

    @Test
    public void unavailableSolutionUsesHoldWithoutInventingIntent() {
        ManualLoopClock time = new ManualLoopClock();
        FakeMotionPredictor predictor = new FakeMotionPredictor();
        ReferenceCoordinatedShotService service = new ReferenceCoordinatedShotService(
                predictor,
                ReferenceCoordinatedShotService.Config.defaults()
        );
        ReferenceTurretMechanism.Config config = mechanismConfig();
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.MotorProbe motor = hardware.addMotor(config.motorName);
        double measuredAngleRad = -0.30;
        motor.setCurrentPositionTicks(toTicks(measuredAngleRad, config.ticksPerRad));
        ReferenceTurretMechanism mechanism =
                new ReferenceTurretMechanism(hardware, config, service);

        mechanism.update(time.clock());

        PlantTargetResolution resolution = mechanism.targetResolution();
        assertEquals(PlantTargetResolution.Kind.HOLD_MEASURED_TARGET, resolution.kind());
        assertEquals(measuredAngleRad, resolution.target(), EPSILON);
        assertFalse(resolution.satisfiesIntent());
        assertTrue(resolution.usedFallback());
        assertTrue(resolution.reason().contains("NOT_STARTED"));
    }

    @Test
    public void configurationIsValidatedBeforeHardwareLookupAndThenSnapshotted() {
        ServiceFixture service = startedService();
        ReferenceTurretMechanism.Config invalid = mechanismConfig();
        invalid.initialHoldAngleRad = invalid.maximumAngleRad + 0.01;
        FtcTestHardware rejectedHardware = new FtcTestHardware();

        IllegalArgumentException failure = assertThrows(
                IllegalArgumentException.class,
                () -> new ReferenceTurretMechanism(
                        rejectedHardware, invalid, service.service)
        );
        assertTrue(failure.getMessage().contains("initialHoldAngleRad"));
        assertEquals(0, rejectedHardware.lookupCalls());

        ReferenceTurretMechanism.Config config = mechanismConfig();
        config.motorName = "  turret  ";
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.MotorProbe originalMotor = hardware.addMotor(config.motorName);
        ReferenceTurretMechanism mechanism =
                new ReferenceTurretMechanism(hardware, config, service.service);
        config.motorName = "mutatedMotor";
        config.ticksPerRad = 1.0;
        config.minimumAngleRad = 100.0;
        config.maximumAngleRad = 101.0;
        config.initialHoldAngleRad = 100.5;

        mechanism.update(service.time.clock());

        double requestedAngleRad = angleOf(service.service.solution());
        assertEquals(toTicks(requestedAngleRad, 100.0), originalMotor.targetPositionTicks());
        assertEquals(1, hardware.lookupCalls());
        assertEquals("turret", hardware.lastLookupName());
    }

    @Test
    public void everyOwnedNumericConfigurationBoundaryRejectsInvalidInputBeforeLookup() {
        ServiceFixture service = startedService();
        assertInvalidBeforeLookup(service, c -> c.motorName = " ", "motorName");
        assertInvalidBeforeLookup(service, c -> c.direction = null, "direction");
        assertInvalidBeforeLookup(service, c -> c.minimumAngleRad = Double.NaN,
                "minimumAngleRad");
        assertInvalidBeforeLookup(service, c -> c.maximumAngleRad = c.minimumAngleRad,
                "minimumAngleRad < maximumAngleRad");
        assertInvalidBeforeLookup(service, c -> c.ticksPerRad = 0.0, "ticksPerRad");
        assertInvalidBeforeLookup(service, c -> c.positionToleranceRad = -0.01,
                "positionToleranceRad");
        assertInvalidBeforeLookup(service, c -> c.initialHoldAngleRad = Double.NaN,
                "initialHoldAngleRad");
    }

    @Test
    public void stopIsTerminalIdempotentAndDoesNotOwnTheUpstreamService() {
        ServiceFixture service = startedService();
        ReferenceTurretMechanism.Config config = mechanismConfig();
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.MotorProbe motor = hardware.addMotor(config.motorName);
        ReferenceTurretMechanism mechanism =
                new ReferenceTurretMechanism(hardware, config, service.service);
        mechanism.update(service.time.clock());
        ReferenceCoordinatedShotService.Solution published = service.service.solution();
        int targetWrites = motor.targetPositionWrites();

        mechanism.stop();
        mechanism.stop();
        mechanism.update(service.time.nextCycle(0.02));

        assertEquals(0.0, motor.power(), 0.0);
        assertEquals(targetWrites, motor.targetPositionWrites());
        assertEquals(PlantTargetResolution.Kind.UNAVAILABLE,
                mechanism.targetResolution().kind());
        assertTrue(mechanism.targetResolution().reason().contains("stopped"));
        assertSame("stopping the output must not reset or replace the service snapshot",
                published, service.service.solution());

        service.publishStationaryCurrentPose(service.time.nextCycle(0.02));
        service.service.update(service.time.clock());
        assertTrue(service.service.solution().mode
                != ReferenceCoordinatedShotService.Mode.UNAVAILABLE);
    }

    private static ServiceFixture startedService() {
        ManualLoopClock time = new ManualLoopClock();
        FakeMotionPredictor predictor = new FakeMotionPredictor();
        ServiceFixture fixture = new ServiceFixture(time, predictor);
        fixture.publishStationaryCurrentPose(time.clock());
        fixture.service.start(time.clock());
        assertEquals(ReferenceCoordinatedShotService.Mode.STATIONARY_FALLBACK,
                fixture.service.solution().mode);
        return fixture;
    }

    private static ReferenceTurretMechanism.Config mechanismConfig() {
        ReferenceTurretMechanism.Config config = ReferenceTurretMechanism.Config.defaults();
        config.motorName = "turret";
        config.minimumAngleRad = -1.5 * Math.PI;
        config.maximumAngleRad = 1.5 * Math.PI;
        config.ticksPerRad = 100.0;
        config.positionToleranceRad = 0.01;
        config.initialHoldAngleRad = 0.0;
        return config;
    }

    private static double angleOf(ReferenceCoordinatedShotService.Solution solution) {
        return Math.atan2(solution.effectiveLeftInches, solution.effectiveForwardInches);
    }

    private static int toTicks(double angleRad, double ticksPerRad) {
        return (int) Math.round(angleRad * ticksPerRad);
    }

    private static void assertInvalidBeforeLookup(ServiceFixture service,
                                                  ConfigMutation mutation,
                                                  String messageFragment) {
        ReferenceTurretMechanism.Config config = mechanismConfig();
        mutation.apply(config);
        FtcTestHardware hardware = new FtcTestHardware();
        RuntimeException failure = assertThrows(
                RuntimeException.class,
                () -> new ReferenceTurretMechanism(hardware, config, service.service)
        );
        assertTrue("Expected message containing " + messageFragment + ", got: "
                        + failure.getMessage(),
                failure.getMessage() != null
                        && failure.getMessage().contains(messageFragment));
        assertEquals(0, hardware.lookupCalls());
    }

    @FunctionalInterface
    private interface ConfigMutation {
        void apply(ReferenceTurretMechanism.Config config);
    }

    private static final class ServiceFixture {
        private final ManualLoopClock time;
        private final FakeMotionPredictor predictor;
        private final ReferenceCoordinatedShotService service;

        private ServiceFixture(ManualLoopClock time, FakeMotionPredictor predictor) {
            this.time = time;
            this.predictor = predictor;
            service = new ReferenceCoordinatedShotService(
                    predictor,
                    ReferenceCoordinatedShotService.Config.defaults()
            );
        }

        private void publishStationaryCurrentPose(LoopClock clock) {
            predictor.publishStationary(clock.nowTimestamp());
        }
    }

    private static final class FakeMotionPredictor implements MotionPredictor {
        private PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());
        private MotionDelta delta = MotionDelta.none(LoopTimestamp.unavailable());

        private void publishStationary(LoopTimestamp timestamp) {
            estimate = new PoseEstimate(Pose3d.zero(), true, 1.0, timestamp);
            delta = MotionDelta.none(timestamp);
        }

        @Override
        public void update(LoopClock clock) {
            throw new AssertionError("the coordinated-shot service must not update its predictor");
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }

        @Override
        public long trajectorySegmentId() {
            return 0L;
        }

        @Override
        public MotionDelta getLatestMotionDelta() {
            return delta;
        }
    }
}
