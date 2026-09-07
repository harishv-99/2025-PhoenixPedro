package edu.ftcsushi.robots.examples.visionpickup;

import java.util.Objects;

import edu.ftcsushi.fw.core.source.BooleanSource;
import edu.ftcsushi.fw.input.binding.CallbackBindings;
import edu.ftcsushi.fw.task.TaskBindings;

/**
 * Independent TeleOp meanings over the same mode-neutral pickup Task that Auto can create.
 * These borrowed sources can be gamepad inputs or software fixtures; construction registers nothing.
 */
public final class VisionPickupControls {
    private final BooleanSource aimHeld;
    private final BooleanSource pickupHeld;
    private final BooleanSource driverOverride;
    private final BooleanSource pickupPermission;
    private boolean bound;

    /** A held pickup request owns permission; release or driver override also rejects queued work. */
    public VisionPickupControls(BooleanSource aimHeld, BooleanSource pickupHeld,
                                BooleanSource driverOverride) {
        this.aimHeld = Objects.requireNonNull(aimHeld, "aimHeld");
        this.pickupHeld = Objects.requireNonNull(pickupHeld, "pickupHeld");
        this.driverOverride = Objects.requireNonNull(driverOverride, "driverOverride");
        this.pickupPermission = clock -> this.pickupHeld.getAsBoolean(clock)
                && !this.driverOverride.getAsBoolean(clock);
    }

    /**
     * Declares held heading assistance, one fresh Task per pickup press, and immediate cancellation
     * on release/override. The managed program remains the only Task runner and final drive owner.
     */
    public void bind(CallbackBindings callbacks, TaskBindings tasks, VisionPickup pickup) {
        if (bound) throw new IllegalStateException("bind VisionPickupControls once");
        Objects.requireNonNull(callbacks, "callbacks");
        Objects.requireNonNull(tasks, "tasks");
        Objects.requireNonNull(pickup, "pickup");
        bound = true;
        callbacks.mirrorOnChange(clock -> aimHeld.getAsBoolean(clock)
                && !driverOverride.getAsBoolean(clock), pickup::setAimEnabled);
        callbacks.onFall(pickupHeld, pickup::cancelPickup);
        callbacks.whileHigh(driverOverride, pickup::cancelPickup);
        tasks.onRise(pickupHeld, () -> pickup.createPickupTask(pickupPermission));
    }
}
