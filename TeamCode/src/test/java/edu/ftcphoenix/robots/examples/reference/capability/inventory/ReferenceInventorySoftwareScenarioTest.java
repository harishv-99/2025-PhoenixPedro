package edu.ftcphoenix.robots.examples.reference.capability.inventory;

import org.junit.Test;

import edu.ftcphoenix.fw.testing.ManualLoopClock;
import edu.ftcphoenix.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Reactive software-device scenario for ordered active-low inventory observations. */
public final class ReferenceInventorySoftwareScenarioTest {

    @Test
    public void authoredLevelsShowBounceAcquisitionRemovalAndOrderIssue() {
        ReferenceInventoryStatusService.Config config =
                ReferenceInventoryStatusService.Config.defaults();
        config.occupiedDebounceSec = 0.02;
        config.vacatedDebounceSec = 0.02;
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.DigitalProbe first = hardware.addDigitalInput(config.firstPositionSensorName);
        FtcTestHardware.DigitalProbe second = hardware.addDigitalInput(config.secondPositionSensorName);
        FtcTestHardware.DigitalProbe third = hardware.addDigitalInput(config.thirdPositionSensorName);
        ManualLoopClock time = new ManualLoopClock();
        ReferenceInventoryStatusService inventory =
                new ReferenceInventoryStatusService(hardware, config);

        first.setHigh(true);
        second.setHigh(true);
        third.setHigh(true);
        inventory.start(time.clock());
        assertEquals(0, inventory.status().conditionedOccupiedPositionCount);

        first.setHigh(false);
        inventory.update(time.nextCycle(0.011));
        first.setHigh(true); // Sub-delay LOW bounce does not become occupied.
        inventory.update(time.nextCycle(0.008));
        assertEquals(0, inventory.status().conditionedOccupiedPositionCount);

        first.setHigh(false);
        inventory.update(time.nextCycle(0.011));
        inventory.update(time.nextCycle(0.011));
        assertEquals(1, inventory.status().conditionedOccupiedPositionCount);

        second.setHigh(false);
        inventory.update(time.nextCycle(0.011));
        inventory.update(time.nextCycle(0.011));
        third.setHigh(false);
        inventory.update(time.nextCycle(0.011));
        inventory.update(time.nextCycle(0.011));
        assertEquals(3, inventory.status().conditionedOccupiedPositionCount);
        assertTrue(inventory.status().full);

        first.setHigh(true); // Authored reverse change: ejection/shot meaning remains external.
        inventory.update(time.nextCycle(0.011));
        inventory.update(time.nextCycle(0.011));
        assertFalse(inventory.status().full);
        assertEquals(ReferenceInventoryStatusService.OrderIssue.SECOND_WITHOUT_FIRST,
                inventory.status().orderIssue);

        second.setHigh(true);
        inventory.update(time.nextCycle(0.011));
        inventory.update(time.nextCycle(0.011));
        assertEquals(ReferenceInventoryStatusService.OrderIssue.THIRD_WITHOUT_SECOND,
                inventory.status().orderIssue);
    }
}
