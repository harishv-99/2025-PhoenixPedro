package edu.ftcsushi.robots.examples.cameraonlypickup;

import java.util.Objects;

import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.input.binding.CallbackBindings;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskBindings;
import edu.ftcsushi.fw.task.Tasks;

/** One press requests one attempt; release or override revokes that gesture, including queued work. */
public final class CameraOnlyPickupControls {
    private final BooleanSource pickupHeld;
    private final BooleanSource driverOverride;
    private Object gesture;
    private boolean overridden;
    private boolean bound;

    /** Borrows input meanings; construction registers nothing and starts no Task. */
    public CameraOnlyPickupControls(BooleanSource pickupHeld, BooleanSource driverOverride) {
        this.pickupHeld = Objects.requireNonNull(pickupHeld, "pickupHeld");
        this.driverOverride = Objects.requireNonNull(driverOverride, "driverOverride");
    }

    /**
     * Declares exactly once. Callback bindings revoke permission before queued Tasks run; an old
     * released press cannot gain permission from a newer press. Override ignores new presses.
     */
    public void bind(CallbackBindings callbacks, TaskBindings tasks, CameraOnlyPickup pickup) {
        if (bound) throw new IllegalStateException("bind CameraOnlyPickupControls once");
        Objects.requireNonNull(callbacks, "callbacks");
        Objects.requireNonNull(tasks, "tasks");
        Objects.requireNonNull(pickup, "pickup");
        bound = true;
        callbacks.mirrorOnChange(driverOverride, value -> {
            overridden = value;
            if (value) invalidate(pickup);
        });
        callbacks.onFall(pickupHeld, () -> invalidate(pickup));
        tasks.onRise(pickupHeld, () -> createPickup(pickup));
    }

    /** Withdraws intent without clearing unrelated work in the program runner. */
    private void invalidate(CameraOnlyPickup pickup) {
        gesture = null;
        pickup.cancelPickup();
    }

    /** Captures this press's identity; permission is checked again when a queued attempt starts. */
    private Task createPickup(CameraOnlyPickup pickup) {
        if (overridden) return Tasks.noop();
        Object requestedGesture = new Object();
        gesture = requestedGesture;
        return pickup.createPickupTask(clock -> {
            boolean held = pickupHeld.getAsBoolean(clock);
            boolean override = driverOverride.getAsBoolean(clock);
            return gesture == requestedGesture && held && !override;
        });
    }
}
