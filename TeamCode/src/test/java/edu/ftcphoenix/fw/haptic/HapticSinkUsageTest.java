package edu.ftcphoenix.fw.haptic;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.input.binding.Bindings;
import edu.ftcphoenix.fw.supervisor.Cooldown;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

/** Verifies the intended robot-owned event and repeat-policy call sites with an SDK-free sink. */
public final class HapticSinkUsageTest {

    @Test
    public void bindingRequestsOneCueForEachRealEventEdge() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Bindings bindings = new Bindings();
        RecordingHapticSink haptics = new RecordingHapticSink();
        boolean[] intakeFull = {false};

        bindings.onRise(
                BooleanSource.of(() -> intakeFull[0]),
                () -> haptics.pulse(0.75, 0.20));

        bindings.update(manualClock.clock());
        intakeFull[0] = true;
        bindings.update(manualClock.nextCycle(0.02));
        bindings.update(manualClock.clock());
        bindings.update(manualClock.nextCycle(0.02));

        assertEquals(1, haptics.pulses.size());
        assertPulse(haptics.pulses.get(0), 0.75, 0.20);

        intakeFull[0] = false;
        bindings.update(manualClock.nextCycle(0.02));
        intakeFull[0] = true;
        bindings.update(manualClock.nextCycle(0.02));

        assertEquals(2, haptics.pulses.size());
        assertPulse(haptics.pulses.get(1), 0.75, 0.20);
    }

    @Test
    public void eventSpecificReminderCooldownCannotSuppressDirectUrgentCue() {
        ManualLoopClock manualClock = new ManualLoopClock();
        Cooldown reminderCooldown = new Cooldown(5.0);
        RecordingHapticSink haptics = new RecordingHapticSink();

        requestReminderIfReady(haptics, reminderCooldown, manualClock);
        assertFalse(reminderCooldown.isReady(manualClock.clock()));

        haptics.pulse(1.0, 0.50);
        requestReminderIfReady(haptics, reminderCooldown, manualClock);

        assertEquals(2, haptics.pulses.size());
        assertPulse(haptics.pulses.get(0), 0.25, 0.10);
        assertPulse(haptics.pulses.get(1), 1.0, 0.50);

        manualClock.nextCycle(5.0);
        requestReminderIfReady(haptics, reminderCooldown, manualClock);

        assertEquals(3, haptics.pulses.size());
        assertPulse(haptics.pulses.get(2), 0.25, 0.10);
    }

    private static void requestReminderIfReady(RecordingHapticSink haptics,
                                               Cooldown reminderCooldown,
                                               ManualLoopClock manualClock) {
        if (reminderCooldown.isReady(manualClock.clock())) {
            haptics.pulse(0.25, 0.10);
            reminderCooldown.trigger(manualClock.clock());
        }
    }

    private static void assertPulse(Pulse pulse,
                                    double expectedStrength,
                                    double expectedDurationSec) {
        assertEquals(expectedStrength, pulse.strength, 0.0);
        assertEquals(expectedDurationSec, pulse.durationSec, 0.0);
    }

    private static final class RecordingHapticSink implements HapticSink {
        private final List<Pulse> pulses = new ArrayList<>();

        @Override
        public void pulse(double strength, double durationSec) {
            pulses.add(new Pulse(strength, durationSec));
        }

        @Override
        public void stop() {
            // No external device is owned by this test fake.
        }
    }

    private static final class Pulse {
        private final double strength;
        private final double durationSec;

        private Pulse(double strength, double durationSec) {
            this.strength = strength;
            this.durationSec = durationSec;
        }
    }
}
