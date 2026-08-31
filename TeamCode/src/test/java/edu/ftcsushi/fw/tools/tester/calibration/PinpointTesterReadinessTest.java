package edu.ftcsushi.fw.tools.tester.calibration;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Locks the stationary/readiness gates used by the Pinpoint calibration tools. */
public final class PinpointTesterReadinessTest {

    @Test
    public void axisSamplesRequireReadyPose() {
        long currentCycle = 42L;
        for (GoBildaPinpointDriver.DeviceStatus status
                : GoBildaPinpointDriver.DeviceStatus.values()) {
            assertFalse(PinpointAxisDirectionTester.hasReadyPoseForSample(
                    status, false, currentCycle, currentCycle));
            if (status == GoBildaPinpointDriver.DeviceStatus.READY) {
                assertTrue(PinpointAxisDirectionTester.hasReadyPoseForSample(
                        status, true, currentCycle, currentCycle));
            } else {
                assertFalse(PinpointAxisDirectionTester.hasReadyPoseForSample(
                        status, true, currentCycle, currentCycle));
            }
        }
        assertFalse(PinpointAxisDirectionTester.hasReadyPoseForSample(
                GoBildaPinpointDriver.DeviceStatus.READY,
                true,
                currentCycle - 1L,
                currentCycle
        ));
        assertFalse(PinpointAxisDirectionTester.hasReadyPoseForSample(
                null, true, currentCycle, currentCycle));
    }

    @Test
    public void podOffsetMotionRequiresCurrentReadyPoseAndVelocity() {
        long currentCycle = 42L;
        assertTrue(PinpointPodOffsetCalibrator.hasCurrentReadyKinematics(
                GoBildaPinpointDriver.DeviceStatus.READY,
                true,
                true,
                currentCycle,
                currentCycle
        ));

        assertFalse(PinpointPodOffsetCalibrator.hasCurrentReadyKinematics(
                GoBildaPinpointDriver.DeviceStatus.CALIBRATING,
                true,
                true,
                currentCycle,
                currentCycle
        ));
        assertFalse(PinpointPodOffsetCalibrator.hasCurrentReadyKinematics(
                GoBildaPinpointDriver.DeviceStatus.READY,
                false,
                true,
                currentCycle,
                currentCycle
        ));
        assertFalse(PinpointPodOffsetCalibrator.hasCurrentReadyKinematics(
                GoBildaPinpointDriver.DeviceStatus.READY,
                true,
                false,
                currentCycle,
                currentCycle
        ));
        assertFalse(PinpointPodOffsetCalibrator.hasCurrentReadyKinematics(
                GoBildaPinpointDriver.DeviceStatus.READY,
                true,
                true,
                currentCycle - 1L,
                currentCycle
        ));
        assertFalse(PinpointPodOffsetCalibrator.hasCurrentReadyKinematics(
                null,
                true,
                true,
                currentCycle,
                currentCycle
        ));
    }
}
