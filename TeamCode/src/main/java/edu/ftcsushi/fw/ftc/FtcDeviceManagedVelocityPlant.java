package edu.ftcsushi.fw.ftc;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Objects;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.actuation.PlantTargetResolution;
import edu.ftcsushi.fw.actuation.PlantTargetStatus;
import edu.ftcsushi.fw.actuation.ScalarRange;
import edu.ftcsushi.fw.core.debug.DebugSink;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.core.hal.VelocityOutput;
import edu.ftcsushi.fw.core.lifecycle.CleanupActions;
import edu.ftcsushi.fw.core.source.ScalarSource;
import edu.ftcsushi.fw.core.source.ScalarTarget;
import edu.ftcsushi.fw.core.time.LoopClock;

/** Package-private FTC identity and evidence carrier for device-managed velocity Plants. */
final class FtcDeviceManagedVelocityPlant implements Plant {

    private final Plant delegate;
    private final FtcDeviceManagedVelocityBinding binding;
    private final ScalarRange targetRange;
    private boolean controllerClaimed;
    private boolean updateAttempted;
    private boolean stopped;
    private LoopClock lastSuccessfulUpdateClock;
    private long lastSuccessfulUpdateCycle = Long.MIN_VALUE;

    FtcDeviceManagedVelocityPlant(Plant delegate,
                                  FtcDeviceManagedVelocityBinding binding,
                                  ScalarRange targetRange) {
        this.delegate = Objects.requireNonNull(delegate, "delegate");
        this.binding = Objects.requireNonNull(binding, "binding");
        this.targetRange = Objects.requireNonNull(targetRange, "targetRange");
    }

    FtcDeviceManagedVelocityBinding binding() {
        return binding;
    }

    ScalarRange targetRange() {
        return targetRange;
    }

    synchronized void claimController() {
        if (controllerClaimed) {
            throw new IllegalStateException(
                    "This FTC device-managed velocity Plant already has a controller-tuning "
                            + "owner; create a fresh Plant for another session");
        }
        if (stopped) {
            throw new IllegalStateException("Claim the FTC device-managed velocity controller "
                    + "before the Plant is stopped");
        }
        if (updateAttempted) {
            throw new IllegalStateException("Claim the FTC device-managed velocity controller "
                    + "before the Plant's first update(clock) heartbeat");
        }
        controllerClaimed = true;
    }

    synchronized void requireEvidenceAfterUpdate(LoopClock clock, String operation) {
        Objects.requireNonNull(clock, "clock");
        requireControllerConfigurationActive(operation);
        if (lastSuccessfulUpdateClock != clock
                || lastSuccessfulUpdateCycle != clock.cycle()) {
            throw new IllegalStateException(operation + " requires the bound Plant to complete "
                    + "update(clock) successfully earlier in the same clock cycle");
        }
    }

    synchronized void requireControllerConfigurationActive(String operation) {
        if (stopped) {
            throw new IllegalStateException(operation + " is unavailable after the bound Plant "
                    + "was stopped; only best-effort restoration remains available");
        }
    }

    @Override
    public void update(LoopClock clock) {
        synchronized (this) {
            updateAttempted = true;
        }
        delegate.update(clock);
        synchronized (this) {
            if (!stopped) {
                lastSuccessfulUpdateClock = clock;
                lastSuccessfulUpdateCycle = clock.cycle();
            }
        }
    }

    @Override public double getRequestedTarget() { return delegate.getRequestedTarget(); }
    @Override public double getAppliedTarget() { return delegate.getAppliedTarget(); }
    @Override public PlantTargetResolution getTargetResolution() { return delegate.getTargetResolution(); }
    @Override public PlantTargetStatus getTargetStatus() { return delegate.getTargetStatus(); }
    @Override public boolean hasFeedback() { return delegate.hasFeedback(); }
    @Override public double getMeasurement() { return delegate.getMeasurement(); }
    @Override public double getRequestedTargetError() { return delegate.getRequestedTargetError(); }
    @Override public double getAppliedTargetError() { return delegate.getAppliedTargetError(); }
    @Override public boolean atTarget() { return delegate.atTarget(); }
    @Override public boolean atTarget(double target) { return delegate.atTarget(target); }
    @Override public boolean hasCommandTarget() { return delegate.hasCommandTarget(); }
    @Override public ScalarTarget commandTarget() { return delegate.commandTarget(); }
    @Override
    public void stop() {
        synchronized (this) {
            stopped = true;
            lastSuccessfulUpdateClock = null;
            lastSuccessfulUpdateCycle = Long.MIN_VALUE;
        }
        delegate.stop();
    }
    @Override public void debugDump(DebugSink dbg, String prefix) { delegate.debugDump(dbg, prefix); }
}

/**
 * One resolved motor group shared by the Plant realization and its configuration-only handle.
 * The exact per-child measurement sources are memoized once and consumed by both paths.
 */
final class FtcDeviceManagedVelocityBinding {

    private final List<DcMotorEx> motors;
    private final List<String> names;
    private final double[] scales;
    private final double nativePerPlantUnit;
    private final double plantTolerance;
    private final double[] constructionPidf;
    private final List<ScalarSource> nativeMeasurements;
    private final GroupOutput output;

    FtcDeviceManagedVelocityBinding(List<DcMotorEx> motors,
                                    List<String> names,
                                    List<Direction> directions,
                                    double[] scales,
                                    double nativePerPlantUnit,
                                    double plantTolerance,
                                    double[] constructionPidf) {
        this.motors = immutableCopy(motors, "motors");
        this.names = immutableCopy(names, "names");
        List<Direction> checkedDirections = immutableCopy(directions, "directions");
        this.scales = Objects.requireNonNull(scales, "scales").clone();
        if (this.motors.isEmpty()
                || this.motors.size() != this.names.size()
                || this.motors.size() != checkedDirections.size()
                || this.motors.size() != this.scales.length) {
            throw new IllegalArgumentException(
                    "FTC device-managed velocity binding children must be non-empty and aligned");
        }
        this.nativePerPlantUnit = requireFiniteNonZero(
                nativePerPlantUnit, "nativePerPlantUnit");
        if (!Double.isFinite(plantTolerance) || plantTolerance < 0.0) {
            throw new IllegalArgumentException(
                    "plantTolerance must be finite and >= 0, got " + plantTolerance);
        }
        this.plantTolerance = plantTolerance;
        if (constructionPidf == null) {
            this.constructionPidf = null;
        } else {
            if (constructionPidf.length != 4) {
                throw new IllegalArgumentException(
                        "constructionPidf must contain exactly kP, kI, kD, and kF");
            }
            this.constructionPidf =
                    FtcControllerConfigurationValidation.requireControllerPidf(
                            constructionPidf[0],
                            constructionPidf[1],
                            constructionPidf[2],
                            constructionPidf[3],
                            "FTC velocity binding construction candidate");
        }

        IdentityHashMap<DcMotorEx, String> seen = new IdentityHashMap<>();
        for (int index = 0; index < this.motors.size(); index++) {
            DcMotorEx motor = Objects.requireNonNull(this.motors.get(index),
                    "motors[" + index + "]");
            String earlierName = seen.put(motor, this.names.get(index));
            if (earlierName != null) {
                throw new IllegalStateException("FTC device-managed velocity motors '"
                        + earlierName + "' and '" + this.names.get(index)
                        + "' resolve to the same DcMotorEx object; each group member must be "
                        + "a distinct configured device");
            }
            requireFiniteNonZero(this.scales[index],
                    "scale for motor '" + this.names.get(index) + "'");
            double nativeTolerance =
                    Math.abs(this.nativePerPlantUnit * this.scales[index]) * this.plantTolerance;
            if (!Double.isFinite(nativeTolerance)) {
                throw new IllegalArgumentException("Mapped native tolerance for motor '"
                        + this.names.get(index) + "' must be finite, got " + nativeTolerance);
            }
        }

        List<VelocityOutput> childOutputs = new ArrayList<>(this.motors.size());
        List<ScalarSource> childMeasurements = new ArrayList<>(this.motors.size());
        for (int index = 0; index < this.motors.size(); index++) {
            childOutputs.add(FtcHardware.motorVelocity(
                    this.motors.get(index), checkedDirections.get(index)));
            childMeasurements.add(FtcSensors.motorVelocityTicksPerSec(
                    this.motors.get(index)));
        }
        nativeMeasurements = Collections.unmodifiableList(childMeasurements);
        output = new GroupOutput(childOutputs);
    }

    List<DcMotorEx> motors() { return motors; }
    List<String> names() { return names; }
    int size() { return motors.size(); }
    double[] constructionPidf() {
        return constructionPidf == null ? null : constructionPidf.clone();
    }

    VelocityOutput output() { return output; }

    ScalarSource sharedNativeMeasurement() {
        return clock -> {
            double[] inverseMapped = new double[nativeMeasurements.size()];
            for (int index = 0; index < nativeMeasurements.size(); index++) {
                double nativeMeasurement = nativeMeasurements.get(index).getAsDouble(clock);
                double sharedMeasurement = nativeMeasurement / scales[index];
                if (!Double.isFinite(sharedMeasurement)) {
                    return Double.NaN;
                }
                inverseMapped[index] = sharedMeasurement;
            }
            double maximumMagnitude = 0.0;
            for (double value : inverseMapped) {
                maximumMagnitude = Math.max(maximumMagnitude, Math.abs(value));
            }
            if (maximumMagnitude == 0.0) return 0.0;
            double scaledSum = 0.0;
            for (double value : inverseMapped) {
                scaledSum += value / maximumMagnitude;
            }
            double average = (scaledSum / inverseMapped.length) * maximumMagnitude;
            return Double.isFinite(average) ? average : Double.NaN;
        };
    }

    double nativeCommandedTarget(int index) { return output.childCommand(index); }

    double nativeMeasurement(int index, LoopClock clock) {
        return nativeMeasurements.get(index).getAsDouble(clock);
    }

    double nativeTolerance(int index) {
        double tolerance = Math.abs(nativePerPlantUnit * scales[index]) * plantTolerance;
        return Double.isFinite(tolerance) ? tolerance : Double.NaN;
    }

    private final class GroupOutput implements VelocityOutput {
        private final List<VelocityOutput> outputs;
        private final double[] lastChildCommands;
        private double lastSharedCommand;

        private GroupOutput(List<VelocityOutput> outputs) {
            this.outputs = new ArrayList<>(outputs);
            lastChildCommands = new double[outputs.size()];
        }

        @Override
        public synchronized void setVelocity(double velocity) {
            if (!Double.isFinite(velocity)) {
                throw new IllegalArgumentException(
                        "motor velocity group command must be finite, got " + velocity);
            }
            double[] candidates = new double[outputs.size()];
            for (int index = 0; index < outputs.size(); index++) {
                candidates[index] = velocity * scales[index];
                if (!Double.isFinite(candidates[index])) {
                    throw new IllegalStateException("motor velocity runtime child "
                            + (index + 1) + " ('" + names.get(index)
                            + "') maps to non-finite native velocity " + candidates[index]);
                }
            }

            try {
                for (int index = 0; index < outputs.size(); index++) {
                    outputs.get(index).setVelocity(candidates[index]);
                    lastChildCommands[index] = candidates[index];
                }
            } catch (RuntimeException failure) {
                lastSharedCommand = Double.NaN;
                throw failure;
            }
            lastSharedCommand = velocity;
        }

        @Override
        public synchronized double getCommandedVelocity() {
            return lastSharedCommand;
        }

        private synchronized double childCommand(int index) {
            return lastChildCommands[index];
        }

        @Override
        public synchronized void stop() {
            Runnable[] actions = new Runnable[outputs.size()];
            for (int index = 0; index < outputs.size(); index++) {
                final int childIndex = index;
                actions[index] = () -> {
                    outputs.get(childIndex).stop();
                    synchronized (GroupOutput.this) {
                        lastChildCommands[childIndex] = 0.0;
                    }
                };
            }
            try {
                CleanupActions.attemptAll(actions);
                lastSharedCommand = 0.0;
            } catch (RuntimeException failure) {
                lastSharedCommand = Double.NaN;
                throw failure;
            }
        }
    }

    private static double requireFiniteNonZero(double value, String name) {
        if (!Double.isFinite(value) || value == 0.0) {
            throw new IllegalArgumentException(name + " must be finite and non-zero, got " + value);
        }
        return value;
    }

    private static <T> List<T> immutableCopy(List<T> values, String name) {
        return Collections.unmodifiableList(new ArrayList<>(
                Objects.requireNonNull(values, name)));
    }
}
