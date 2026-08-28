package edu.ftcphoenix.robots.examples.reference.capability.inventory;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.junit.Test;

import edu.ftcphoenix.fw.testing.ManualLoopClock;
import edu.ftcphoenix.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/** Contract tests for the optional Reference inventory-status owner. */
public final class ReferenceInventoryStatusServiceTest {

    @Test
    public void constructionSnapshotsConfigResolvesEverythingBeforeEffectsAndDoesNotRead() {
        ReferenceInventoryStatusService.Config config = zeroDelayConfig();
        FtcTestHardware incomplete = new FtcTestHardware();
        FtcTestHardware.DigitalProbe first = incomplete.addDigitalInput(config.firstPositionSensorName);
        FtcTestHardware.DigitalProbe second = incomplete.addDigitalInput(config.secondPositionSensorName);

        assertThrows(IllegalArgumentException.class,
                () -> new ReferenceInventoryStatusService(incomplete, config));
        assertEquals(0, first.modeWriteCalls());
        assertEquals(0, second.modeWriteCalls());

        Fixture fixture = new Fixture(config);
        assertEquals(0, fixture.first.stateReadCalls());
        assertEquals(0, fixture.second.stateReadCalls());
        assertEquals(0, fixture.third.stateReadCalls());
        assertEquals(DigitalChannel.Mode.INPUT, fixture.first.mode());
        assertEquals(1, fixture.first.modeWriteCalls());
        assertFalse(fixture.service.status().observed);
        assertSame(fixture.service.fullSource(), fixture.service.fullSource());

        config.firstPositionSensorName = "changed";
        fixture.start();
        assertEquals(1, fixture.first.stateReadCalls());
    }

    @Test
    public void configurationRejectsInvalidValuesNamesAndResolvedIdentity() {
        ReferenceInventoryStatusService.Config invalidDelay = zeroDelayConfig();
        invalidDelay.occupiedDebounceSec = Double.NaN;
        assertThrows(IllegalArgumentException.class,
                () -> new ReferenceInventoryStatusService(new FtcTestHardware(), invalidDelay));

        ReferenceInventoryStatusService.Config duplicateNames = zeroDelayConfig();
        duplicateNames.secondPositionSensorName =
                " " + duplicateNames.firstPositionSensorName + " ";
        assertThrows(IllegalArgumentException.class,
                () -> new ReferenceInventoryStatusService(new FtcTestHardware(), duplicateNames));

        ReferenceInventoryStatusService.Config duplicateDevices = zeroDelayConfig();
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.DigitalProbe shared =
                hardware.addDigitalInput(duplicateDevices.firstPositionSensorName);
        hardware.addDigitalInputAlias(duplicateDevices.secondPositionSensorName, shared);
        hardware.addDigitalInput(duplicateDevices.thirdPositionSensorName);
        assertThrows(IllegalArgumentException.class,
                () -> new ReferenceInventoryStatusService(hardware, duplicateDevices));
        assertEquals(0, shared.modeWriteCalls());
    }

    @Test
    public void allBitPatternsPublishExactCountFullAndOrderIssue() {
        assertPattern(false, false, false, 0, false,
                ReferenceInventoryStatusService.OrderIssue.NONE);
        assertPattern(true, false, false, 1, false,
                ReferenceInventoryStatusService.OrderIssue.NONE);
        assertPattern(false, true, false, 1, false,
                ReferenceInventoryStatusService.OrderIssue.SECOND_WITHOUT_FIRST);
        assertPattern(false, false, true, 1, false,
                ReferenceInventoryStatusService.OrderIssue.THIRD_WITHOUT_SECOND);
        assertPattern(true, true, false, 2, false,
                ReferenceInventoryStatusService.OrderIssue.NONE);
        assertPattern(true, false, true, 2, false,
                ReferenceInventoryStatusService.OrderIssue.THIRD_WITHOUT_SECOND);
        assertPattern(false, true, true, 2, false,
                ReferenceInventoryStatusService.OrderIssue.SECOND_WITHOUT_FIRST);
        assertPattern(true, true, true, 3, true,
                ReferenceInventoryStatusService.OrderIssue.NONE);
    }

    @Test
    public void positiveDebounceStartsAtStartBoundaryAndOwnersAdvanceIndependently() {
        ReferenceInventoryStatusService.Config config = ReferenceInventoryStatusService.Config.defaults();
        config.occupiedDebounceSec = 0.02;
        config.vacatedDebounceSec = 0.02;
        Fixture fixture = new Fixture(config);
        fixture.first.setHigh(false);
        fixture.second.setHigh(true);
        fixture.third.setHigh(true);
        fixture.start();

        assertTrue(fixture.service.status().observed);
        assertFalse(fixture.service.status().firstPositionOccupied);
        fixture.advance(0.011);
        fixture.second.setHigh(false);
        fixture.advance(0.011);
        assertTrue(fixture.service.status().firstPositionOccupied);
        assertFalse(fixture.service.status().secondPositionOccupied);
        fixture.advance(0.011);
        assertTrue(fixture.service.status().secondPositionOccupied);
    }

    @Test
    public void sameCycleSourceReadsAreCachedAndSnapshotsStayImmutable() {
        Fixture fixture = new Fixture(zeroDelayConfig());
        fixture.start();
        ReferenceInventoryStatusService.Status empty = fixture.service.status();
        int reads = fixture.totalReads();

        fixture.first.setHigh(false);
        fixture.service.update(fixture.time.clock());
        assertNotSame(empty, fixture.service.status());
        assertFalse(fixture.service.status().firstPositionOccupied);
        assertEquals(reads, fixture.totalReads());

        fixture.advance(0.01);
        assertNotSame(empty, fixture.service.status());
        assertFalse(empty.firstPositionOccupied);
        assertTrue(fixture.service.status().firstPositionOccupied);
    }

    @Test
    public void failedObservationDoesNotPartiallyPublishAndMayRetrySameCycle() {
        Fixture fixture = new Fixture(zeroDelayConfig());
        fixture.start();
        ReferenceInventoryStatusService.Status prior = fixture.service.status();
        RuntimeException failure = new RuntimeException("second failed");
        fixture.second.setReadFailure(failure);
        fixture.time.nextCycle(0.01);

        assertSame(failure, assertThrows(RuntimeException.class,
                () -> fixture.service.update(fixture.time.clock())));
        int readsAfterFailure = fixture.totalReads();
        assertSame(prior, fixture.service.status());

        fixture.second.setReadFailure(null);
        fixture.service.update(fixture.time.clock());
        assertTrue(fixture.service.status().observed);
        assertNotSame(prior, fixture.service.status());
        assertEquals(readsAfterFailure + 2, fixture.totalReads());
    }

    @Test
    public void stopClearsStatusAndResetsSources() {
        Fixture fixture = new Fixture(zeroDelayConfig());
        fixture.first.setHigh(false);
        fixture.start();
        assertTrue(fixture.service.status().firstPositionOccupied);
        int reads = fixture.totalReads();
        fixture.service.stop();
        assertFalse(fixture.service.status().observed);
        assertFalse(fixture.service.fullSource().getAsBoolean(fixture.time.clock()));
        assertEquals(reads, fixture.totalReads());
    }

    private static void assertPattern(boolean first, boolean second, boolean third, int count,
                                      boolean full,
                                      ReferenceInventoryStatusService.OrderIssue issue) {
        Fixture fixture = new Fixture(zeroDelayConfig());
        fixture.first.setHigh(!first);
        fixture.second.setHigh(!second);
        fixture.third.setHigh(!third);
        fixture.start();
        ReferenceInventoryStatusService.Status status = fixture.service.status();
        assertTrue(status.observed);
        assertEquals(first, status.firstPositionOccupied);
        assertEquals(second, status.secondPositionOccupied);
        assertEquals(third, status.thirdPositionOccupied);
        assertEquals(count, status.conditionedOccupiedPositionCount);
        assertEquals(full, status.full);
        assertEquals(full, fixture.service.fullSource().getAsBoolean(fixture.time.clock()));
        assertEquals(issue, status.orderIssue);
    }

    private static ReferenceInventoryStatusService.Config zeroDelayConfig() {
        ReferenceInventoryStatusService.Config config = ReferenceInventoryStatusService.Config.defaults();
        config.occupiedDebounceSec = 0.0;
        config.vacatedDebounceSec = 0.0;
        return config;
    }

    private static final class Fixture {
        private final ManualLoopClock time = new ManualLoopClock();
        private final FtcTestHardware.DigitalProbe first;
        private final FtcTestHardware.DigitalProbe second;
        private final FtcTestHardware.DigitalProbe third;
        private final ReferenceInventoryStatusService service;

        private Fixture(ReferenceInventoryStatusService.Config config) {
            FtcTestHardware hardware = new FtcTestHardware();
            first = hardware.addDigitalInput(config.firstPositionSensorName);
            second = hardware.addDigitalInput(config.secondPositionSensorName);
            third = hardware.addDigitalInput(config.thirdPositionSensorName);
            service = new ReferenceInventoryStatusService(hardware, config);
        }

        private void start() {
            service.start(time.clock());
        }

        private void advance(double dtSec) {
            service.update(time.nextCycle(dtSec));
        }

        private int totalReads() {
            return first.stateReadCalls() + second.stateReadCalls() + third.stateReadCalls();
        }
    }
}
