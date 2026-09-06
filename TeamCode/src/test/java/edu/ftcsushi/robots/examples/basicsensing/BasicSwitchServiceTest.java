package edu.ftcsushi.robots.examples.basicsensing;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.junit.Test;

import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;
import edu.ftcsushi.fw.testing.ftc.FtcTestTelemetry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/** Maintainer contracts for input ownership, configuration, cached publication, and cleanup. */
public final class BasicSwitchServiceTest {

    @Test
    public void constructionConfiguresOneInputAndSnapshotsEveryConfigField() {
        BasicSwitchTestRig rig = new BasicSwitchTestRig();
        rig.mode.init();
        assertEquals(1, rig.hardware.lookupCalls());
        assertEquals(DigitalChannel.Mode.INPUT, rig.input.mode());
        assertEquals(1, rig.input.modeWriteCalls());
        assertEquals(0, rig.input.stateReadCalls());
        rig.config.switchName = "changed";
        rig.config.pressedDebounceSec = 0.0;
        rig.config.releasedDebounceSec = 0.0;
        rig.mode.start();
        rig.observeAt(0.011, false);
        assertFalse(rig.status().pressed);
        rig.observeAt(0.022, false);
        assertTrue(rig.status().pressed);
        rig.observeAt(0.033, true);
        assertTrue(rig.status().pressed);
        rig.observeAt(0.044, true);
        assertFalse(rig.status().pressed);
        assertEquals(1, rig.hardware.lookupCalls());
    }

    @Test
    public void invalidConfigurationFailsBeforeHardwareLookup() {
        for (String name : new String[]{null, "", "   "}) {
            BasicSwitchService.Config config = BasicSwitchService.Config.defaults();
            config.switchName = name;
            assertRejectedBeforeLookup(config, "switchName");
        }
        for (double delay : new double[]{-0.01, Double.NaN, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY}) {
            BasicSwitchService.Config pressed = BasicSwitchService.Config.defaults();
            pressed.pressedDebounceSec = delay;
            assertRejectedBeforeLookup(pressed, "pressedDebounceSec");
            BasicSwitchService.Config released = BasicSwitchService.Config.defaults();
            released.releasedDebounceSec = delay;
            assertRejectedBeforeLookup(released, "releasedDebounceSec");
        }
        assertThrows(NullPointerException.class,
                () -> new BasicSwitchService(new FtcTestHardware(), null));
        assertThrows(NullPointerException.class,
                () -> new BasicSwitchService(null, BasicSwitchService.Config.defaults()));
    }

    @Test
    public void zeroDelaysAreImmediateAndSameCycleReadsShareOneObservation() {
        BasicSwitchService.Config config = BasicSwitchService.Config.defaults();
        config.switchName = " lessonSwitch ";
        config.pressedDebounceSec = 0.0;
        config.releasedDebounceSec = 0.0;
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.DigitalProbe input = hardware.addDigitalInput("lessonSwitch");
        BasicSwitchService service = new BasicSwitchService(hardware, config);
        ManualLoopClock time = new ManualLoopClock();
        input.setHigh(false);
        service.start(time.clock());
        BasicSwitchService.Status pressed = service.status();
        assertTrue(pressed.rawPressed);
        assertTrue(pressed.pressed);
        assertEquals(1, input.stateReadCalls());
        input.setHigh(true);
        service.update(time.clock());
        assertTrue(service.status().pressed);
        assertEquals(1, input.stateReadCalls());
        service.update(time.nextCycle(0.01));
        assertFalse(service.status().pressed);
        assertTrue(pressed.pressed);
        assertEquals(2, input.stateReadCalls());
        assertSame(service.status(), service.status());
    }

    @Test
    public void failedReadLeavesPriorSnapshotAndMayRetrySameCycle() {
        BasicSwitchService.Config config = BasicSwitchService.Config.defaults();
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.DigitalProbe input = hardware.addDigitalInput(config.switchName);
        BasicSwitchService service = new BasicSwitchService(hardware, config);
        ManualLoopClock time = new ManualLoopClock();
        service.start(time.clock());
        BasicSwitchService.Status prior = service.status();
        RuntimeException failure = new IllegalStateException("switch read failed");
        input.setReadFailure(failure);
        time.nextCycle(0.03);
        assertSame(failure, assertThrows(RuntimeException.class,
                () -> service.update(time.clock())));
        assertSame(prior, service.status());
        input.setReadFailure(null);
        input.setHigh(false);
        service.update(time.clock());
        assertTrue(service.status().pressed);
        assertFalse(prior.pressed);
        assertEquals(3, input.stateReadCalls());
    }

    @Test
    public void managedReadFailureClearsStatusAndSuppressesLaterLoops() {
        BasicSwitchTestRig rig = new BasicSwitchTestRig();
        rig.mode.init();
        rig.mode.start();
        rig.observeAt(0.03, false);
        BasicSwitchService.Status pressed = rig.status();
        RuntimeException failure = new IllegalStateException("switch read failed");
        rig.input.setReadFailure(failure);
        int commits = rig.commits;
        assertSame(failure, assertThrows(RuntimeException.class,
                () -> rig.observeAt(0.04, false)));
        assertFalse(rig.status().observed);
        assertTrue(pressed.pressed);
        assertEquals(commits, rig.commits);
        int reads = rig.input.stateReadCalls();
        rig.input.setReadFailure(null);
        rig.observeAt(0.08, true);
        rig.mode.stop();
        assertEquals(reads, rig.input.stateReadCalls());
    }

    @Test
    public void startDoesNotChargeTheTimeSpentInInitToDebounce() {
        BasicSwitchTestRig rig = new BasicSwitchTestRig();
        rig.input.setHigh(false);
        rig.mode.init();
        rig.mode.runtimeSec = 10.0;
        rig.mode.init_loop();
        rig.mode.runtimeSec = 11.0;
        rig.mode.start();
        assertEquals(1, rig.input.stateReadCalls());
        assertTrue(rig.status().rawPressed);
        assertFalse(rig.status().pressed);
        rig.observeAt(11.011, false);
        assertFalse(rig.status().pressed);
        rig.observeAt(11.022, false);
        assertTrue(rig.status().pressed);
    }

    @Test
    public void presentersUseTheServiceObservationAndCommitExactlyOncePerFrame() {
        BasicSwitchTestRig rig = new BasicSwitchTestRig();
        rig.input.setHigh(true);
        rig.mode.init();
        assertEquals(1, rig.commits);
        rig.mode.start();
        assertEquals(1, rig.commits);
        rig.observeAt(0.011, false);
        assertEquals(2, rig.input.stateReadCalls());
        assertEquals(2, rig.commits);
        assertEquals(true, rig.rows.get("switch.observed"));
        assertEquals(true, rig.rows.get("switch.rawPressed"));
        assertEquals(false, rig.rows.get("switch.pressed"));
        rig.observeAt(0.022, false);
        assertEquals(3, rig.input.stateReadCalls());
        assertEquals(3, rig.commits);
        assertEquals(true, rig.rows.get("switch.pressed"));
    }

    @Test
    public void stopBeforeStartAndRepeatedCleanupDoNotPollOrReconfigureInput() {
        BasicSwitchTestRig rig = new BasicSwitchTestRig();
        rig.mode.init();
        rig.mode.stop();
        rig.mode.limitSwitch.stop();
        rig.mode.start();
        rig.mode.loop();
        assertFalse(rig.status().observed);
        assertEquals(0, rig.input.stateReadCalls());
        assertEquals(1, rig.input.modeWriteCalls());
    }

    @Test
    public void productionHostConstructsTheSameSingleInputGraph() {
        FtcTestHardware hardware = new FtcTestHardware();
        FtcTestHardware.DigitalProbe input = hardware.addDigitalInput(
                BasicSwitchService.Config.defaults().switchName);
        BasicSwitchTeleOp mode = new BasicSwitchTeleOp();
        mode.hardwareMap = hardware;
        mode.telemetry = FtcTestTelemetry.silent();
        mode.init();
        assertEquals(1, hardware.lookupCalls());
        assertEquals(0, input.stateReadCalls());
        mode.start();
        assertEquals(1, input.stateReadCalls());
        mode.stop();
        mode.loop();
        assertEquals(1, input.stateReadCalls());
    }

    private static void assertRejectedBeforeLookup(BasicSwitchService.Config config, String field) {
        FtcTestHardware hardware = new FtcTestHardware();
        IllegalArgumentException failure = assertThrows(IllegalArgumentException.class,
                () -> new BasicSwitchService(hardware, config));
        assertTrue(failure.getMessage().contains(field));
        assertEquals(0, hardware.lookupCalls());
    }
}
