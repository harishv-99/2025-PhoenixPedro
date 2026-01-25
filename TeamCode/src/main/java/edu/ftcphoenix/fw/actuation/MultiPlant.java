package edu.ftcphoenix.fw.actuation;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;

/**
 * A {@link Plant} that fans out a single logical target to multiple child plants.
 *
 * <p>Each child may optionally apply a linear mapping:</p>
 *
 * <pre>{@code
 * childTarget = scale * target + bias
 * }</pre>
 *
 * <p>This is useful for multi-motor mechanisms where one motor needs a small
 * constant calibration factor, or for mirrored mechanisms where one side should
 * receive a slightly different command.</p>
 *
 * <h2>Setpoint semantics</h2>
 * <ul>
 *   <li>{@link #atSetpoint()} is {@code true} only if <b>all</b> children are at setpoint.</li>
 *   <li>{@link #hasFeedback()} is {@code true} only if <b>all</b> children report feedback.</li>
 * </ul>
 */
final class MultiPlant implements Plant {

    static Builder builder() {
        return new Builder();
    }

    static MultiPlant of(Plant... plants) {
        Builder b = builder();
        if (plants != null) {
            for (Plant p : plants) {
                b.add(p);
            }
        }
        return b.build();
    }

    /**
     * Small builder for {@link MultiPlant}.
     *
     * <p>This is internal framework plumbing used by higher-level builders
     * (for example {@link Actuators}) to assemble multi-actuator mechanisms.</p>
     */
    static final class Builder {
        private final List<Plant> plants = new ArrayList<>();
        private final List<Double> scales = new ArrayList<>();
        private final List<Double> biases = new ArrayList<>();

        Builder add(Plant plant) {
            return add(plant, 1.0, 0.0);
        }

        Builder add(Plant plant, double scale) {
            return add(plant, scale, 0.0);
        }

        Builder add(Plant plant, double scale, double bias) {
            plants.add(Objects.requireNonNull(plant, "plant"));
            scales.add(scale);
            biases.add(bias);
            return this;
        }

        Builder addAll(Plant... plants) {
            if (plants == null) {
                return this;
            }
            for (Plant p : plants) {
                add(p);
            }
            return this;
        }

        MultiPlant build() {
            if (plants.isEmpty()) {
                throw new IllegalStateException("MultiPlant requires at least one child plant");
            }
            Plant[] ps = plants.toArray(new Plant[0]);
            double[] sc = new double[scales.size()];
            double[] bi = new double[biases.size()];
            for (int i = 0; i < sc.length; i++) {
                sc[i] = scales.get(i);
                bi[i] = biases.get(i);
            }
            return new MultiPlant(ps, sc, bi);
        }
    }

    private final Plant[] plants;
    private final double[] scale;
    private final double[] bias;

    private double target;

    private MultiPlant(Plant[] plants, double[] scale, double[] bias) {
        this.plants = Objects.requireNonNull(plants, "plants");
        this.scale = Objects.requireNonNull(scale, "scale");
        this.bias = Objects.requireNonNull(bias, "bias");
        if (plants.length == 0) {
            throw new IllegalArgumentException("MultiPlant requires at least one child plant");
        }
        if (scale.length != plants.length || bias.length != plants.length) {
            throw new IllegalArgumentException("scale/bias arrays must match plant count");
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setTarget(double target) {
        this.target = target;
        for (int i = 0; i < plants.length; i++) {
            plants[i].setTarget(scale[i] * target + bias[i]);
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double getTarget() {
        return target;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void update(double dtSec) {
        for (Plant p : plants) {
            p.update(dtSec);
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void reset() {
        for (Plant p : plants) {
            p.reset();
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void stop() {
        for (Plant p : plants) {
            p.stop();
        }
        target = 0.0;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean atSetpoint() {
        for (Plant p : plants) {
            if (!p.atSetpoint()) {
                return false;
            }
        }
        return true;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean hasFeedback() {
        for (Plant p : plants) {
            if (!p.hasFeedback()) {
                return false;
            }
        }
        return true;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "plant" : prefix;
        dbg.addData(p + ".target", target)
                .addData(p + ".count", plants.length)
                .addData(p + ".atSetpoint", atSetpoint())
                .addData(p + ".hasFeedback", hasFeedback());

        // Emit child targets in a compact form.
        for (int i = 0; i < plants.length; i++) {
            dbg.addData(p + ".child" + i + ".target", plants[i].getTarget());
        }
    }
}
