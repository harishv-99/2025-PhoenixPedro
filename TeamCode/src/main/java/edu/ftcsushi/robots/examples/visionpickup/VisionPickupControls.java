package edu.ftcsushi.robots.examples.visionpickup;

import java.util.Objects;

import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.input.binding.CallbackBindings;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskBindings;
import edu.ftcsushi.fw.task.Tasks;

/**
 * Independent TeleOp meanings over the same mode-neutral pickup Task that Auto can create.
 * These borrowed sources can be gamepad inputs or software fixtures; construction registers nothing.
 */
public final class VisionPickupControls {
    private final BooleanSource aimHeld;
    private final BooleanSource pickupHeld;
    private final BooleanSource driverOverride;
    private Object pickupGesture;
    private boolean overridden;
    private boolean bound;

    /** A held pickup request owns permission; release or driver override also rejects queued work. */
    public VisionPickupControls(BooleanSource aimHeld, BooleanSource pickupHeld,
                                BooleanSource driverOverride) {
        this.aimHeld = Objects.requireNonNull(aimHeld, "aimHeld");
        this.pickupHeld = Objects.requireNonNull(pickupHeld, "pickupHeld");
        this.driverOverride = Objects.requireNonNull(driverOverride, "driverOverride");
    }

    /**
     * Declares held heading assistance, one fresh Task per pickup press, and immediate cancellation
     * on release/override. Each queued request retains its original gesture's permission: a later
     * press cannot revive released work. A press during override is ignored, not saved for later.
     *
     * <p>Aim observes the original held input. Releasing override alone cannot manufacture an aim
     * press; release/repress aim to request a new session. The managed program remains the only
     * Task runner and final drive owner.</p>
     */
    public void bind(CallbackBindings callbacks, TaskBindings tasks, VisionPickup pickup) {
        if (bound) throw new IllegalStateException("bind VisionPickupControls once");
        Objects.requireNonNull(callbacks, "callbacks");
        Objects.requireNonNull(tasks, "tasks");
        Objects.requireNonNull(pickup, "pickup");
        bound = true;
        callbacks.mirrorOnChange(driverOverride, value -> {
            overridden = value;
            if (value) {
                invalidatePickup(pickup);
                pickup.setAimEnabled(false);
            }
        });
        callbacks.mirrorOnChange(aimHeld,
                enabled -> pickup.setAimEnabled(enabled && !overridden));
        callbacks.onFall(pickupHeld, () -> invalidatePickup(pickup));
        tasks.onRise(pickupHeld, () -> createPickup(pickup));
    }

    /** Withdraw this gesture without clearing unrelated program work or cancelling unstarted Tasks. */
    private void invalidatePickup(VisionPickup pickup) {
        pickupGesture = null;
        pickup.cancelPickup();
    }

    /** Capture one original press; the final identity check also covers cancellation inside a read. */
    private Task createPickup(VisionPickup pickup) {
        if (overridden) return Tasks.noop();
        Object gesture = new Object();
        pickupGesture = gesture;
        return pickup.createPickupTask(clock -> {
            boolean held = pickupHeld.getAsBoolean(clock);
            boolean inhibited = driverOverride.getAsBoolean(clock);
            return pickupGesture == gesture && held && !inhibited;
        });
    }
}
