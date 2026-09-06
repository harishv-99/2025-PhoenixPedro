package edu.ftcsushi.robots.examples.basicsensing;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Small software experiments using the production owner, configuration, graph, and managed loop. */
public final class BasicSwitchSoftwareScenarioTest {

    @Test
    public void authoredLevelsBecomeCachedPressedStatusAfterSampledDebounce() {
        // ARRANGE: HIGH means released for the selected active-low interpretation.
        BasicSwitchTestRig rig = new BasicSwitchTestRig();
        rig.input.setHigh(true);
        rig.mode.init();
        rig.mode.start();
        assertFalse(rig.status().pressed);

        // INJECT EVIDENCE + HEARTBEAT: a brief LOW followed by HIGH does not latch pressed.
        rig.observeAt(0.010, false);
        assertTrue(rig.status().rawPressed);
        assertFalse(rig.status().pressed);
        rig.observeAt(0.015, true);
        assertFalse(rig.status().pressed);

        // HEARTBEAT: two LOW samples contribute 0.022 seconds to the 0.02-second debounce.
        rig.observeAt(0.026, false);
        assertFalse(rig.status().pressed);
        rig.observeAt(0.037, false);
        assertTrue(rig.status().pressed);
        assertEquals(true, rig.rows.get("switch.pressed"));

        // RELEASE: the raw meaning changes before the debounced meaning clears.
        rig.observeAt(0.048, true);
        assertFalse(rig.status().rawPressed);
        assertTrue(rig.status().pressed);
        rig.observeAt(0.059, true);
        assertFalse(rig.status().pressed);
        rig.mode.stop();
        // NEXT GATE: physical wiring and switch behavior still need their own no-motion check.
    }

    @Test
    public void initAndStopDoNotInventSwitchObservations() {
        // ARRANGE: the input is LOW, but INIT is only configuration and cached presentation.
        BasicSwitchTestRig rig = new BasicSwitchTestRig();
        rig.input.setHigh(false);
        rig.mode.init();
        rig.mode.init_loop();
        assertEquals(0, rig.input.stateReadCalls());
        assertEquals(false, rig.rows.get("switch.observed"));

        // START: sample at zero elapsed time; observing LOW is not yet debounced pressed.
        rig.mode.start();
        assertTrue(rig.status().observed);
        assertTrue(rig.status().rawPressed);
        assertFalse(rig.status().pressed);
        assertEquals(1, rig.input.stateReadCalls());

        // STOP: clear evidence, then let the managed host suppress every later loop.
        rig.mode.stop();
        assertFalse(rig.status().observed);
        rig.observeAt(1.0, true);
        rig.mode.stop();
        assertEquals(1, rig.input.stateReadCalls());
        // NEXT GATE: cached false without observed=true must never be called a released switch.
    }
}
