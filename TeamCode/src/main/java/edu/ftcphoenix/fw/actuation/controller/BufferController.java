package edu.ftcphoenix.fw.actuation.controller;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Objects;
import java.util.function.BooleanSupplier;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.core.debug.DebugSink;

/**
 * Controller that turns high-level "buffer" commands (SEND / EJECT) into
 * timed pulses on a single {@link Plant}, typically a power plant driving
 * an intake or feeder.
 *
 * <p>Intended usage:</p>
 *
 * <pre>{@code
 * // Example: CR-servo feeder with timed pulses and shooter-ready gating.
 * Plant bufferPlant = Actuators.plant(hw)
 *         .motor("feeder", Direction.FORWARD)
 *         .power()
 *         .build();
 *
 * BufferController buffer = new BufferController(
 *     bufferPlant,
 *     +1.0,              // forwardTarget (SEND)
 *     -1.0,              // reverseTarget (EJECT)
 *     0.0,               // idleTarget
 *     0.35,              // pulseSeconds
 *     shooter::isReady   // downstreamReady (may be null)
 * );
 *
 * // Gamepad bindings:
 * bindings.onPress(pads.p1().rb(), () -> buffer.handle(BufferController.Cmd.SEND));
 * bindings.onPress(pads.p1().b(),  () -> buffer.handle(BufferController.Cmd.EJECT));
 *
 * // In your loop:
 * buffer.update(dtSec); // drives the underlying plant
 * }</pre>
 *
 * <p>Semantics:</p>
 * <ul>
 *   <li>Commands are queued in FIFO order.</li>
 *   <li>At most one command is "active" at a time; it drives the plant at
 *       {@code forwardTarget} or {@code reverseTarget} for {@code pulseSeconds}
 *       seconds.</li>
 *   <li>Between pulses (and whenever idle), the plant is held at
 *       {@code idleTarget}.</li>
 *   <li>If a {@link #getDownstreamReady() downstreamReady} gate is provided,
 *       a queued command will not start until it returns true.</li>
 *   <li>{@link Cmd#CANCEL_ALL} immediately clears the queue and forces the
 *       plant to {@code idleTarget}.</li>
 * </ul>
 *
 * <p><b>Important:</b> when using this controller, your robot code should
 * call {@link #update(double)} on the controller and <b>not</b> call
 * {@link Plant#update(double)} on the wrapped plant directly. The controller
 * takes ownership of calling {@link Plant#setTarget(double)} and
 * {@link Plant#update(double)}.</p>
 */
public final class BufferController {

    /**
     * High-level commands accepted by {@link #handle(Cmd)}.
     */
    public enum Cmd {
        /**
         * Queue a forward "send" pulse (e.g., feed one game piece).
         */
        SEND,

        /**
         * Queue a reverse "eject" pulse.
         */
        EJECT,

        /**
         * Cancel all queued commands and immediately force the plant to
         * {@code idleTarget}.
         */
        CANCEL_ALL
    }

    private final Plant plant;
    private final double forwardTarget;
    private final double reverseTarget;
    private final double idleTarget;
    private final double pulseSeconds;
    private final BooleanSupplier downstreamReady;

    private final Deque<Cmd> queue = new ArrayDeque<>();

    private Cmd activeCmd = null;
    private double remainingSec = 0.0;

    // Debug: last sampled downstream readiness (only evaluated when a queued pulse is eligible to start).
    private boolean lastDownstreamReady = true;

    /**
     * Construct a {@link BufferController} with explicit targets and an optional
     * downstream readiness gate.
     *
     * @param plant           plant to drive (typically a power plant)
     * @param forwardTarget   target value for {@link Cmd#SEND} pulses
     * @param reverseTarget   target value for {@link Cmd#EJECT} pulses
     * @param idleTarget      target value when idle (no active pulse)
     * @param pulseSeconds    duration of each pulse; must be &gt;= 0
     * @param downstreamReady optional gate; if non-null, a queued command
     *                        will not start until this supplier returns true
     */
    public BufferController(Plant plant,
                            double forwardTarget,
                            double reverseTarget,
                            double idleTarget,
                            double pulseSeconds,
                            BooleanSupplier downstreamReady) {
        this.plant = Objects.requireNonNull(plant, "plant is required");
        if (pulseSeconds < 0.0) {
            throw new IllegalArgumentException("pulseSeconds must be >= 0");
        }
        this.forwardTarget = forwardTarget;
        this.reverseTarget = reverseTarget;
        this.idleTarget = idleTarget;
        this.pulseSeconds = pulseSeconds;
        this.downstreamReady = downstreamReady;

        // Start idle.
        plant.setTarget(idleTarget);
    }

    /**
     * Convenience constructor for a common case:
     *
     * <ul>
     *   <li>{@code forwardTarget = +1.0}</li>
     *   <li>{@code reverseTarget = -1.0}</li>
     *   <li>{@code idleTarget = 0.0}</li>
     *   <li>no downstream ready gate</li>
     * </ul>
     */
    public BufferController(Plant plant, double pulseSeconds) {
        this(plant, +1.0, -1.0, 0.0, pulseSeconds, null);
    }

    /**
     * Enqueue a buffer command.
     *
     * <p>Supported commands:</p>
     * <ul>
     *   <li>{@link Cmd#SEND} – queue a forward pulse.</li>
     *   <li>{@link Cmd#EJECT} – queue a reverse pulse.</li>
     *   <li>{@link Cmd#CANCEL_ALL} – clear the queue and immediately
     *       force the plant idle.</li>
     * </ul>
     *
     * <p>This method is cheap and intended to be called from button bindings
     * (e.g., {@code onPress} handlers).</p>
     */
    public void handle(Cmd cmd) {
        Objects.requireNonNull(cmd, "cmd is required");

        switch (cmd) {
            case SEND:
            case EJECT:
                queue.addLast(cmd);
                break;

            case CANCEL_ALL:
                queue.clear();
                activeCmd = null;
                remainingSec = 0.0;
                lastDownstreamReady = true;
                plant.setTarget(idleTarget);
                break;
        }
    }

    /**
     * @return true if a pulse is currently active (in-progress).
     */
    public boolean isBusy() {
        return activeCmd != null;
    }

    /**
     * @return true if there are queued commands waiting to be executed.
     */
    public boolean hasQueuedCommands() {
        return !queue.isEmpty();
    }

    /**
     * Clear all queued commands and stop any active pulse, forcing the plant
     * to {@code idleTarget}.
     */
    public void cancelAll() {
        handle(Cmd.CANCEL_ALL);
    }

    /**
     * Advance the controller and the wrapped plant by {@code dtSec} seconds.
     *
     * <p>Typical call site is your TeleOp / Auto loop:</p>
     *
     * <pre>{@code
     * double dt = clock.dtSec();
     * bufferController.update(dt);
     * }</pre>
     *
     * <p>Responsibilities:</p>
     * <ul>
     *   <li>If there is an active pulse, decrement its remaining time and
     *       return to {@code idleTarget} when the pulse completes.</li>
     *   <li>If there is no active pulse and there are queued commands,</li>
     *   <li>and {@code downstreamReady} (if provided) is true, start the next
     *       pulse by setting the plant target to {@code forwardTarget} or
     *       {@code reverseTarget}.</li>
     *   <li>Always call {@link Plant#update(double)} exactly once per call.</li>
     * </ul>
     *
     * @param dtSec time step in seconds
     */
    public void update(double dtSec) {
        if (downstreamReady == null) {
            lastDownstreamReady = true;
        }

        // 1) Advance current pulse (if any).
        if (activeCmd != null) {
            remainingSec -= dtSec;
            if (remainingSec <= 0.0) {
                // Pulse complete: go idle and clear active command.
                activeCmd = null;
                remainingSec = 0.0;
                lastDownstreamReady = true;
                plant.setTarget(idleTarget);
            }
        }

        // 2) If idle, consider starting the next queued pulse.
        if (activeCmd == null && !queue.isEmpty()) {
            boolean ready = (downstreamReady == null) || downstreamReady.getAsBoolean();
            lastDownstreamReady = ready;
            if (ready) {
                Cmd next = queue.removeFirst();
                activeCmd = next;
                remainingSec = pulseSeconds;

                double tgt;
                switch (next) {
                    case SEND:
                        tgt = forwardTarget;
                        break;
                    case EJECT:
                        tgt = reverseTarget;
                        break;
                    case CANCEL_ALL:
                        // CANCEL_ALL should never be enqueued; it's handled eagerly.
                        tgt = idleTarget;
                        break;
                    default:
                        tgt = idleTarget;
                        break;
                }
                plant.setTarget(tgt);
            }
        }

        // 3) Always advance the plant.
        plant.update(dtSec);
    }

    /**
     * Reset the controller and underlying plant to a known idle state.
     *
     * <p>Effect:</p>
     * <ul>
     *   <li>Cancels all queued commands.</li>
     *   <li>Clears any active pulse.</li>
     *   <li>Calls {@link Plant#reset()}.</li>
     *   <li>Sets the plant target to {@code idleTarget}.</li>
     * </ul>
     */
    public void reset() {
        queue.clear();
        activeCmd = null;
        remainingSec = 0.0;
        lastDownstreamReady = true;
        plant.reset();
        plant.setTarget(idleTarget);
    }

    /**
     * @return the underlying {@link Plant} (for advanced use only).
     */
    public Plant getPlant() {
        return plant;
    }

    /**
     * @return the downstream readiness gate, or {@code null} if none.
     */
    public BooleanSupplier getDownstreamReady() {
        return downstreamReady;
    }

    /**
     * @return configured pulse duration in seconds.
     */
    public double getPulseSeconds() {
        return pulseSeconds;
    }

    /**
     * @return configured forward target value.
     */
    public double getForwardTarget() {
        return forwardTarget;
    }

    /**
     * @return configured reverse target value.
     */
    public double getReverseTarget() {
        return reverseTarget;
    }

    /**
     * @return configured idle target value.
     */
    public double getIdleTarget() {
        return idleTarget;
    }


    /**
     * Debug helper: emit current queue / pulse state and the wrapped plant's debug output.
     */
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "buffer" : prefix;

        dbg.addLine(p)
                .addData(p + ".activeCmd", activeCmd)
                .addData(p + ".remainingSec", remainingSec)
                .addData(p + ".queueSize", queue.size())
                .addData(p + ".downstreamReady", lastDownstreamReady)
                .addData(p + ".forwardTarget", forwardTarget)
                .addData(p + ".reverseTarget", reverseTarget)
                .addData(p + ".idleTarget", idleTarget)
                .addData(p + ".pulseSeconds", pulseSeconds);

        plant.debugDump(dbg, p + ".plant");
    }

}
