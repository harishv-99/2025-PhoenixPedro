package edu.ftcphoenix.fw.core.source;

import java.util.ArrayList;
import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Builder for layering multiple scalar overrides on top of a base {@link ScalarSource}.
 *
 * <p>This is the scalar analogue of {@link edu.ftcphoenix.fw.drive.DriveOverlayStack}.
 * It exists to make the "single-writer" plant rule easy to follow:
 * build one final target source that encodes your priority rules, then write that
 * single target to the plant each loop.</p>
 *
 * <h2>Semantics</h2>
 * <ul>
 *   <li>The base source is sampled every loop.</li>
 *   <li>Each layer is evaluated in the order it was added.</li>
 *   <li>If a layer is enabled, it replaces the current value with its own value.</li>
 *   <li>If multiple enabled layers exist, <b>the last enabled layer wins</b>.</li>
 * </ul>
 *
 * <h2>When to use</h2>
 * <p>Use this for mechanisms where you have a <b>continuous base target</b> and one or
 * more <b>temporary overrides</b> (like an {@link edu.ftcphoenix.fw.task.OutputTaskRunner}
 * that should override while active).</p>
 *
 * <h2>Example</h2>
 * <pre>{@code
 * OutputTaskRunner feederQueue = Tasks.outputQueue(0.0);
 * ScalarSource baseFeeder = ScalarSource.constant(0.0);
 *
 * ScalarSource finalFeeder = ScalarOverlayStack.on(baseFeeder)
 *     .add("queue", feederQueue.activeSource(), feederQueue)
 *     .add("eject", ejectRequested, ScalarSource.constant(-1.0))
 *     .build();
 *
 * feederPlant.setTarget(finalFeeder.getAsDouble(clock));
 * }</pre>
 */
public final class ScalarOverlayStack {

    private ScalarOverlayStack() {
        // static utility
    }

    /**
     * Start building an overlay stack on top of {@code base}.
     */
    public static Builder on(ScalarSource base) {
        return new Builder(base);
    }

    /**
     * Builder that collects scalar layers and produces a composed {@link ScalarSource}.
     */
    public static final class Builder {
        private final ScalarSource base;
        private final ArrayList<Layer> layers = new ArrayList<Layer>();

        private Builder(ScalarSource base) {
            this.base = Objects.requireNonNull(base, "base");
        }

        /**
         * Add a named scalar override layer.
         *
         * @param name        name used for debug output (non-null, non-empty)
         * @param enabledWhen condition to enable this override (non-null)
         * @param overlay     scalar to use when enabled (non-null)
         */
        public Builder add(String name, BooleanSource enabledWhen, ScalarSource overlay) {
            if (name == null || name.trim().isEmpty()) {
                throw new IllegalArgumentException("name must be non-null and non-empty");
            }
            Objects.requireNonNull(enabledWhen, "enabledWhen");
            Objects.requireNonNull(overlay, "overlay");
            layers.add(new Layer(name, enabledWhen, overlay));
            return this;
        }

        /**
         * Add an unnamed scalar override layer. A stable auto-generated name will be used for debug.
         */
        public Builder add(BooleanSource enabledWhen, ScalarSource overlay) {
            String name = "overlay" + layers.size();
            return add(name, enabledWhen, overlay);
        }

        /**
         * Finish the builder.
         *
         * <p>If no layers were added, this returns the original {@code base} source.</p>
         */
        public ScalarSource build() {
            if (layers.isEmpty()) {
                return base;
            }
            Layer[] arr = layers.toArray(new Layer[0]);
            return new StackedScalarSource(base, arr);
        }
    }

    /**
     * Immutable layer definition.
     */
    private static final class Layer {
        final String name;
        final BooleanSource enabledWhen;
        final ScalarSource overlay;

        // Mutable for debug.
        boolean lastEnabled = false;
        double lastValue = 0.0;

        Layer(String name, BooleanSource enabledWhen, ScalarSource overlay) {
            this.name = name;
            this.enabledWhen = enabledWhen;
            this.overlay = overlay;
        }
    }

    /**
     * ScalarSource implementation that applies a list of override layers.
     */
    private static final class StackedScalarSource implements ScalarSource {
        private final ScalarSource base;
        private final Layer[] layers;

        private long lastCycle = Long.MIN_VALUE;
        private double lastBase = 0.0;
        private double lastOut = 0.0;

        StackedScalarSource(ScalarSource base, Layer[] layers) {
            this.base = Objects.requireNonNull(base, "base");
            this.layers = Objects.requireNonNull(layers, "layers");
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public double getAsDouble(LoopClock clock) {
            long cyc = clock.cycle();
            if (cyc == lastCycle) {
                return lastOut;
            }
            lastCycle = cyc;

            double out = base.getAsDouble(clock);
            lastBase = out;

            for (int i = 0; i < layers.length; i++) {
                Layer layer = layers[i];

                boolean enabled = layer.enabledWhen.getAsBoolean(clock);
                layer.lastEnabled = enabled;

                if (!enabled) {
                    layer.lastValue = 0.0;
                    continue;
                }

                out = layer.overlay.getAsDouble(clock);
                layer.lastValue = out;
            }

            lastOut = out;
            return out;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void reset() {
            base.reset();
            for (int i = 0; i < layers.length; i++) {
                Layer layer = layers[i];
                layer.enabledWhen.reset();
                layer.overlay.reset();
                layer.lastEnabled = false;
                layer.lastValue = 0.0;
            }

            lastCycle = Long.MIN_VALUE;
            lastBase = 0.0;
            lastOut = 0.0;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void debugDump(DebugSink dbg, String prefix) {
            if (dbg == null) {
                return;
            }
            String p = (prefix == null || prefix.isEmpty()) ? "scalar" : prefix;
            dbg.addData(p + ".class", "ScalarOverlayStack")
                    .addData(p + ".layers", layers.length)
                    .addData(p + ".lastBase", lastBase)
                    .addData(p + ".lastOut", lastOut);

            base.debugDump(dbg, p + ".base");

            for (int i = 0; i < layers.length; i++) {
                Layer layer = layers[i];
                String lp = p + ".layer[" + i + "]";
                dbg.addData(lp + ".name", layer.name);
                dbg.addData(lp + ".enabled", layer.lastEnabled);
                dbg.addData(lp + ".lastValue", layer.lastValue);
                layer.enabledWhen.debugDump(dbg, lp + ".enabledWhen");
                layer.overlay.debugDump(dbg, lp + ".overlay");
            }
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public String toString() {
            return "ScalarOverlayStack{" + "layers=" + layers.length + ", base=" + base + "}";
        }
    }
}
