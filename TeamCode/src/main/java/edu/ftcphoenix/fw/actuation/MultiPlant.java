package edu.ftcphoenix.fw.actuation;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * A {@link Plant} that fans a single logical target out to multiple child plants.
 *
 * <p>Each child may apply a linear mapping:</p>
 *
 * <pre>{@code
 * childTarget = scale * groupTarget + bias
 * }</pre>
 *
 * <p>This is useful for device-managed groups such as mirrored lift motors or multi-servo linkages.
 * The group's authoritative measurement is computed by inverse-mapping each child measurement back
 * into group units and averaging the results.</p>
 */
public final class MultiPlant implements Plant {

    /**
     * Create a new builder.
     */
    public static Builder builder() {
        return new Builder();
    }

    /**
     * Small builder for {@link MultiPlant}.
     */
    public static final class Builder {
        private final List<Plant> plants = new ArrayList<>();
        private final List<Double> scales = new ArrayList<>();
        private final List<Double> biases = new ArrayList<>();

        public Builder add(Plant plant) {
            return add(plant, 1.0, 0.0);
        }

        public Builder add(Plant plant, double scale) {
            return add(plant, scale, 0.0);
        }

        public Builder add(Plant plant, double scale, double bias) {
            plants.add(Objects.requireNonNull(plant, "plant"));
            scales.add(scale);
            biases.add(bias);
            return this;
        }

        public Builder addAll(Plant... plants) {
            if (plants == null) {
                return this;
            }
            for (Plant plant : plants) {
                add(plant);
            }
            return this;
        }

        public MultiPlant build() {
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
    private final double[] scales;
    private final double[] biases;

    private double target;
    private double lastMeasurement = Double.NaN;

    private MultiPlant(Plant[] plants, double[] scales, double[] biases) {
        this.plants = Objects.requireNonNull(plants, "plants");
        this.scales = Objects.requireNonNull(scales, "scales");
        this.biases = Objects.requireNonNull(biases, "biases");
        if (plants.length == 0) {
            throw new IllegalArgumentException("MultiPlant requires at least one child plant");
        }
        if (plants.length != scales.length || plants.length != biases.length) {
            throw new IllegalArgumentException("plants/scales/biases must have matching lengths");
        }
    }

    @Override
    public void setTarget(double target) {
        this.target = target;
        for (int i = 0; i < plants.length; i++) {
            plants[i].setTarget(scales[i] * target + biases[i]);
        }
    }

    @Override
    public double getTarget() {
        return target;
    }

    @Override
    public void update(LoopClock clock) {
        for (Plant plant : plants) {
            plant.update(clock);
        }
        lastMeasurement = computeAggregateMeasurement();
    }

    @Override
    public void reset() {
        for (Plant plant : plants) {
            plant.reset();
        }
        lastMeasurement = Double.NaN;
    }

    @Override
    public void stop() {
        for (Plant plant : plants) {
            plant.stop();
        }
        lastMeasurement = Double.NaN;
    }

    @Override
    public boolean atSetpoint() {
        for (Plant plant : plants) {
            if (!plant.atSetpoint()) {
                return false;
            }
        }
        return true;
    }

    @Override
    public boolean hasFeedback() {
        for (Plant plant : plants) {
            if (!plant.hasFeedback()) {
                return false;
            }
        }
        return true;
    }

    @Override
    public double getMeasurement() {
        return lastMeasurement;
    }

    private double computeAggregateMeasurement() {
        if (!hasFeedback()) {
            return Double.NaN;
        }
        double sum = 0.0;
        int count = 0;
        for (int i = 0; i < plants.length; i++) {
            double scale = scales[i];
            if (Math.abs(scale) < 1e-9) {
                return Double.NaN;
            }
            double childMeasurement = plants[i].getMeasurement();
            if (!Double.isFinite(childMeasurement)) {
                return Double.NaN;
            }
            sum += (childMeasurement - biases[i]) / scale;
            count++;
        }
        return count > 0 ? (sum / count) : Double.NaN;
    }

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "plant" : prefix;
        dbg.addData(p + ".target", getTarget())
                .addData(p + ".count", plants.length)
                .addData(p + ".hasFeedback", hasFeedback())
                .addData(p + ".atSetpoint", atSetpoint())
                .addData(p + ".measurement", getMeasurement())
                .addData(p + ".error", getError());
        for (int i = 0; i < plants.length; i++) {
            dbg.addData(p + ".child" + i + ".target", plants[i].getTarget())
                    .addData(p + ".child" + i + ".measurement", plants[i].getMeasurement())
                    .addData(p + ".child" + i + ".error", plants[i].getError());
        }
    }
}
