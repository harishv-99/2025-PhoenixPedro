package edu.ftcphoenix.fw.drive;

import java.util.ArrayList;
import java.util.Objects;
import java.util.function.BooleanSupplier;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Builder for stacking multiple {@link DriveOverlay}s on top of a base {@link DriveSource}.
 *
 * <p>Phoenix drive overlays are intentionally small and composable, but writing nested
 * {@link DriveSource#overlayWhen(BooleanSupplier, DriveOverlay, DriveOverlayMask)} calls can get
 * noisy as you add more assists. {@code DriveOverlayStack} provides a structured, readable way to
 * apply many overlays in one place.</p>
 *
 * <h2>Semantics</h2>
 * <ul>
 *   <li>The base {@link DriveSource} runs every loop.</li>
 *   <li>Each overlay layer is evaluated in the order it was added.</li>
 *   <li>When enabled, a layer may override some DOFs of the current command, based on:
 *       <ul>
 *         <li>the layer's requested mask, and</li>
 *         <li>the overlay's dynamic mask returned in {@link DriveOverlayOutput}.</li>
 *       </ul>
 *   </li>
 *   <li>If multiple enabled layers claim the same DOF, <b>the last layer wins</b> for that DOF.</li>
 *   <li>Each overlay still receives {@link DriveOverlay#onEnable(LoopClock)} and
 *       {@link DriveOverlay#onDisable(LoopClock)} when it toggles.</li>
 * </ul>
 *
 * <h2>Example</h2>
 * <pre>{@code
 * DriveSource drive = DriveOverlayStack.on(stickDrive)
 *     .add("shootBrace", () -> shootBraceEnabled,
 *          DriveGuidance.poseLock(poseEstimator),
 *          DriveOverlayMask.TRANSLATION_ONLY)
 *     .add("autoAim", gamepads.p2().leftBumper()::isHeld,
 *          aimPlan.overlay(),
 *          DriveOverlayMask.OMEGA_ONLY)
 *     .build();
 * }</pre>
 */
public final class DriveOverlayStack {

    private DriveOverlayStack() {
        // static utility
    }

    /**
     * Start building an overlay stack on top of {@code base}.
     */
    public static Builder on(DriveSource base) {
        return new Builder(base);
    }

    /**
     * Builder that collects overlay layers and produces a composed {@link DriveSource}.
     */
    public static final class Builder {
        private final DriveSource base;
        private final ArrayList<Layer> layers = new ArrayList<Layer>();

        private Builder(DriveSource base) {
            this.base = Objects.requireNonNull(base, "base");
        }

        /**
         * Add a named overlay layer.
         *
         * @param name          name used for debug output (non-null, non-empty)
         * @param enabledWhen   condition to enable this overlay (non-null)
         * @param overlay       overlay implementation (non-null)
         * @param requestedMask requested override mask (non-null, not {@link DriveOverlayMask#NONE})
         */
        public Builder add(String name,
                           BooleanSupplier enabledWhen,
                           DriveOverlay overlay,
                           DriveOverlayMask requestedMask) {
            if (name == null || name.trim().isEmpty()) {
                throw new IllegalArgumentException("name must be non-null and non-empty");
            }
            Objects.requireNonNull(enabledWhen, "enabledWhen");
            Objects.requireNonNull(overlay, "overlay");
            Objects.requireNonNull(requestedMask, "requestedMask");
            if (requestedMask.isNone()) {
                throw new IllegalArgumentException("requestedMask must not be NONE");
            }
            layers.add(new Layer(name, enabledWhen, overlay, requestedMask));
            return this;
        }

        /**
         * Add an unnamed overlay layer. A stable auto-generated name will be used for debug.
         */
        public Builder add(BooleanSupplier enabledWhen,
                           DriveOverlay overlay,
                           DriveOverlayMask requestedMask) {
            String name = "overlay" + layers.size();
            return add(name, enabledWhen, overlay, requestedMask);
        }

        /**
         * Convenience overload: requested mask defaults to {@link DriveOverlayMask#ALL}.
         */
        public Builder add(String name,
                           BooleanSupplier enabledWhen,
                           DriveOverlay overlay) {
            return add(name, enabledWhen, overlay, DriveOverlayMask.ALL);
        }

        /**
         * Convenience overload: requested mask defaults to {@link DriveOverlayMask#ALL}.
         */
        public Builder add(BooleanSupplier enabledWhen,
                           DriveOverlay overlay) {
            return add(enabledWhen, overlay, DriveOverlayMask.ALL);
        }

        /**
         * Finish the builder.
         *
         * <p>If no layers were added, this returns the original {@code base} source.</p>
         */
        public DriveSource build() {
            if (layers.isEmpty()) {
                return base;
            }
            Layer[] arr = layers.toArray(new Layer[0]);
            return new StackedDriveSource(base, arr);
        }
    }

    /**
     * Immutable layer definition.
     */
    private static final class Layer {
        final String name;
        final BooleanSupplier enabledWhen;
        final DriveOverlay overlay;
        final DriveOverlayMask requestedMask;

        // Mutable state for lifecycle + debug.
        boolean lastEnabled = false;
        DriveOverlayOutput lastOut = DriveOverlayOutput.zero();
        DriveOverlayMask lastEffectiveMask = DriveOverlayMask.NONE;

        Layer(String name,
              BooleanSupplier enabledWhen,
              DriveOverlay overlay,
              DriveOverlayMask requestedMask) {
            this.name = name;
            this.enabledWhen = enabledWhen;
            this.overlay = overlay;
            this.requestedMask = requestedMask;
        }
    }

    /**
     * DriveSource implementation that applies a list of overlay layers.
     */
    private static final class StackedDriveSource implements DriveSource {
        private final DriveSource base;
        private final Layer[] layers;

        private DriveSignal lastBase = DriveSignal.zero();
        private DriveSignal lastOut = DriveSignal.zero();

        StackedDriveSource(DriveSource base, Layer[] layers) {
            this.base = Objects.requireNonNull(base, "base");
            this.layers = Objects.requireNonNull(layers, "layers");
        }

        @Override
        public DriveSignal get(LoopClock clock) {
            DriveSignal cmd = base.get(clock);
            lastBase = cmd;

            for (int i = 0; i < layers.length; i++) {
                Layer layer = layers[i];

                boolean enabled = layer.enabledWhen.getAsBoolean();

                if (!enabled) {
                    if (layer.lastEnabled) {
                        layer.overlay.onDisable(clock);
                        layer.lastEnabled = false;
                    }
                    layer.lastOut = DriveOverlayOutput.zero();
                    layer.lastEffectiveMask = DriveOverlayMask.NONE;
                    continue;
                }

                if (!layer.lastEnabled) {
                    layer.overlay.onEnable(clock);
                    layer.lastEnabled = true;
                }

                DriveOverlayOutput out = layer.overlay.get(clock);
                if (out == null) {
                    out = DriveOverlayOutput.zero();
                }

                DriveOverlayMask eff = out.mask.intersect(layer.requestedMask);
                layer.lastOut = out;
                layer.lastEffectiveMask = eff;

                if (eff.isNone()) {
                    continue;
                }

                double axial = eff.axial ? out.signal.axial : cmd.axial;
                double lateral = eff.lateral ? out.signal.lateral : cmd.lateral;
                double omega = eff.omega ? out.signal.omega : cmd.omega;
                cmd = new DriveSignal(axial, lateral, omega);
            }

            lastOut = cmd;
            return cmd;
        }

        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) {
                return;
            }
            String p = (prefix == null || prefix.isEmpty()) ? "drive" : prefix;
            dbg.addData(p + ".class", "DriveOverlayStack")
                    .addData(p + ".layers", layers.length)
                    .addData(p + ".lastBase", lastBase)
                    .addData(p + ".lastOut", lastOut);

            base.debugDump(dbg, p + ".base");

            for (int i = 0; i < layers.length; i++) {
                Layer layer = layers[i];
                String lp = p + ".layer[" + i + "]";
                dbg.addData(lp + ".name", layer.name);
                dbg.addData(lp + ".enabled", layer.lastEnabled);
                dbg.addData(lp + ".requestedMask", layer.requestedMask.toString());
                dbg.addData(lp + ".effectiveMask", layer.lastEffectiveMask.toString());
                dbg.addData(lp + ".lastOut", layer.lastOut);
                layer.overlay.debugDump(dbg, lp + ".overlay");
            }
        }

        @Override
        public String toString() {
            return "DriveOverlayStack{" + "layers=" + layers.length + ", base=" + base + "}";
        }
    }
}
