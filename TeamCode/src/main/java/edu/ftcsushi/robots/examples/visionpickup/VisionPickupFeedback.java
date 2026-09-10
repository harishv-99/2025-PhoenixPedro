package edu.ftcsushi.robots.examples.visionpickup;

import java.util.Objects;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.haptic.HapticSink;

/**
 * Example-owned driver cues for the difference between heading alignment and lost assistance.
 *
 * <p>Declare this output after pickup evaluation and the source-driven drive output. It reads one
 * cached {@link VisionPickup.Status}, never polls vision/localization, and requests at most one
 * pulse per update. One short pulse marks the first observed alignment in each aim session; a
 * long pulse marks newly observed loss. Loss wins if both become observable together, and consumes
 * that session's alignment cue rather than playing a misleading short pulse on the next loop.
 * Heading-tolerance chatter cannot repeat the short cue. If an intentional release/override or
 * STOP has already left assistance off when this output observes a loss, it consumes that event
 * without a late alarm. Intentional cancellation, pickup completion, and STOP do not themselves
 * request a pulse.</p>
 *
 * <p>These durations are illustrative robot policy, not a controller guarantee. A returning
 * {@link HapticSink#pulse(double, double)} only accepts a command locally; the latest undelivered
 * FTC request may replace an earlier one. Check support and recognizability on the actual driver
 * controller. Keep {@link VisionPickupPresenter} visible because a pulse is neither a delivery
 * receipt nor the only indication of lost assistance.</p>
 */
public final class VisionPickupFeedback implements RobotProgram.Output {
    /** Example normalized strength, sent equally to the FTC controller's two channels. */
    public static final double PULSE_STRENGTH = 1.0;
    /** One short first-alignment request, in seconds; test recognition on the real controller. */
    public static final double ALIGNED_PULSE_SEC = 0.10;
    /** One longer assist-loss request, in seconds; test recognition on the real controller. */
    public static final double LOST_PULSE_SEC = 0.50;

    private final VisionPickup pickup;
    private final HapticSink driver;
    private LoopClock ownerClock;
    private long lastCycle = Long.MIN_VALUE;
    private long observedLossCount;
    private long alignedSession = Long.MIN_VALUE;
    private boolean updating;
    private boolean stopped;
    private RuntimeException failure;

    /**
     * Creates the one feedback lifecycle owner for the supplied driver recipient.
     * Construction submits no command; the program owns updates and terminal stop.
     */
    public VisionPickupFeedback(VisionPickup pickup, HapticSink driver) {
        this.pickup = Objects.requireNonNull(pickup, "pickup");
        this.driver = Objects.requireNonNull(driver, "driver haptic sink");
    }

    /**
     * Observe one published status after drive evaluation and submit at most one semantic cue.
     * The cycle is claimed before any sink effect. Repeated successful updates are inert; a
     * RuntimeException is retained without retry, including a caught recursive update failure.
     * A stopped owner is inert and cannot restart through another update.
     */
    @Override
    public void update(LoopClock clock) {
        if (stopped) return;
        if (failure != null) throw failure;
        Objects.requireNonNull(clock, "clock");
        if (ownerClock == null) ownerClock = clock;
        if (ownerClock != clock) {
            throw new IllegalArgumentException("feedback must use the same shared LoopClock");
        }
        if (updating) {
            failure = new IllegalStateException("driver feedback update is reentrant");
            throw failure;
        }
        if (lastCycle == clock.cycle()) return;
        lastCycle = clock.cycle();
        updating = true;
        try {
            VisionPickup.Status snapshot = pickup.status();
            boolean lost = snapshot.assistLossCount != observedLossCount;
            observedLossCount = snapshot.assistLossCount;
            if (snapshot.assistState == VisionPickup.AssistState.IDLE
                    || snapshot.assistState == VisionPickup.AssistState.STOPPED) return;
            if (lost) {
                alignedSession = snapshot.aimSessionId;
                driver.pulse(PULSE_STRENGTH, LOST_PULSE_SEC);
            } else if (snapshot.assistState == VisionPickup.AssistState.ALIGNED
                    && alignedSession != snapshot.aimSessionId) {
                alignedSession = snapshot.aimSessionId;
                driver.pulse(PULSE_STRENGTH, ALIGNED_PULSE_SEC);
            }
            // A sink callback may catch its recursive-update error. Do not hide that failure.
            if (failure != null) throw failure;
        } catch (RuntimeException caught) {
            if (failure == null) failure = caught;
            else if (failure != caught) failure.addSuppressed(caught);
            throw failure;
        } finally {
            updating = false;
        }
    }

    /**
     * Terminal, idempotent, best-effort stop request, including before the first update.
     * The owner becomes inert before calling the sink, so a callback or failure cannot rearm it.
     * A normally returning stop is not proof of physical controller delivery or stop latency.
     */
    @Override
    public void stop() {
        if (stopped) return;
        stopped = true;
        driver.stop();
    }
}
