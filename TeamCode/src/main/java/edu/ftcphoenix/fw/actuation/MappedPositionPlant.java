package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Source-driven {@link PositionPlant} that maps caller-facing plant units to native hardware units.
 *
 * <p>Most robot code should create this through the guided {@code FtcActuators.plant(...)} builder.
 * This class is public so FTC and non-FTC boundary builders can share the same plant-domain mapping
 * implementation.</p>
 *
 * <h2>Coordinate model</h2>
 * <pre>{@code
 * native = nativeReference + nativePerPlantUnit * (plant - plantReference)
 * }</pre>
 *
 * <p>The target source is sampled once per {@link #update(LoopClock)}. Static plant range and
 * reference validity are enforced first; dynamic hardware protection such as interlocks and target
 * rate limits are applied by {@link PlantTargetGuards}; then one applied target is sent to hardware
 * or the framework regulator.</p>
 */
public final class MappedPositionPlant implements PositionPlant {

    /**
     * Reference initialization policy for the native/plant coordinate map.
     */
    public enum ReferenceMode {
        /**
         * The static {@code plantReference -> nativeReference} mapping is known at build time.
         */
        STATIC,
        /**
         * On first update, sample native feedback and treat it as the configured plant position.
         */
        ASSUME_CURRENT,
        /**
         * The coordinate is invalid until a task establishes a reference.
         */
        NEEDS_REFERENCE
    }

    private final PositionOutput positionOut;
    private final PowerOutput regulatedPowerOut;
    private final ScalarRegulator regulator;
    private final ScalarSource nativeMeasurement;
    private final PowerOutput searchPowerOut;
    private final ScalarSource targetSource;
    private final ScalarTarget writableTarget;
    private final PlantTargetGuards targetGuards;
    private final Topology topology;
    private final double period;
    private final ScalarRange configuredRange;
    private final double nativePerPlantUnit;
    private final double tolerance;
    private final ReferenceMode referenceMode;
    private final String unreferencedReason;
    private final double assumedPlantPosition;

    private boolean referenced;
    private boolean pendingAssume;
    private double plantReference;
    private double nativeReference;
    private double requestedTarget;
    private double appliedTarget;
    private double lastMeasurement = Double.NaN;
    private double lastNativeMeasurement = Double.NaN;
    private boolean lastAtTarget;
    private boolean searchActive;
    private double searchPower;
    private double lastRegulatorOutput;
    private PlantTargetStatus targetStatus = PlantTargetStatus.STOPPED;

    private MappedPositionPlant(PositionOutput positionOut,
                                PowerOutput regulatedPowerOut,
                                ScalarRegulator regulator,
                                ScalarSource nativeMeasurement,
                                PowerOutput searchPowerOut,
                                ScalarSource targetSource,
                                ScalarTarget writableTarget,
                                PlantTargetGuards targetGuards,
                                Topology topology,
                                double period,
                                ScalarRange configuredRange,
                                double nativePerPlantUnit,
                                double tolerance,
                                ReferenceMode referenceMode,
                                double plantReference,
                                double nativeReference,
                                double assumedPlantPosition,
                                String unreferencedReason) {
        this.positionOut = positionOut;
        this.regulatedPowerOut = regulatedPowerOut;
        this.regulator = regulator;
        this.nativeMeasurement = nativeMeasurement != null ? nativeMeasurement.memoized() : null;
        this.searchPowerOut = searchPowerOut;
        this.targetSource = Objects.requireNonNull(targetSource, "targetSource").memoized();
        this.writableTarget = writableTarget;
        this.targetGuards = targetGuards == null ? PlantTargetGuards.none() : targetGuards;
        this.topology = Objects.requireNonNull(topology, "topology");
        this.period = period;
        this.configuredRange = Objects.requireNonNull(configuredRange, "configuredRange");
        this.nativePerPlantUnit = nativePerPlantUnit;
        this.tolerance = tolerance;
        this.referenceMode = Objects.requireNonNull(referenceMode, "referenceMode");
        this.plantReference = plantReference;
        this.nativeReference = nativeReference;
        this.assumedPlantPosition = assumedPlantPosition;
        this.unreferencedReason = (unreferencedReason == null || unreferencedReason.trim().isEmpty())
                ? "position reference not established"
                : unreferencedReason.trim();
        this.referenced = referenceMode == ReferenceMode.STATIC;
        this.pendingAssume = referenceMode == ReferenceMode.ASSUME_CURRENT;
        validate();
    }

    /**
     * Starts configuring a commanded-position plant such as a standard servo.
     */
    public static Builder commanded(PositionOutput out) {
        return new Builder(Objects.requireNonNull(out, "out"), null, null, null);
    }

    /**
     * Starts configuring a position-output plant with native feedback.
     */
    public static Builder positionOutput(PositionOutput out, ScalarSource nativeMeasurement) {
        return new Builder(Objects.requireNonNull(out, "out"), null, null,
                Objects.requireNonNull(nativeMeasurement, "nativeMeasurement"));
    }

    /**
     * Starts configuring a framework-regulated position plant that drives raw power.
     */
    public static Builder regulated(PowerOutput powerOut,
                                    ScalarSource nativeMeasurement,
                                    ScalarRegulator regulator) {
        return new Builder(null,
                Objects.requireNonNull(powerOut, "powerOut"),
                Objects.requireNonNull(regulator, "regulator"),
                Objects.requireNonNull(nativeMeasurement, "nativeMeasurement"));
    }

    /**
     * Advanced builder. FTC robot code should normally use {@code FtcActuators}.
     */
    public static final class Builder {
        private final PositionOutput positionOut;
        private final PowerOutput regulatedPowerOut;
        private final ScalarRegulator regulator;
        private final ScalarSource nativeMeasurement;
        private PowerOutput searchPowerOut;
        private ScalarSource targetSource;
        private ScalarTarget writableTarget;
        private PlantTargetGuards targetGuards = PlantTargetGuards.none();
        private Topology topology = Topology.LINEAR;
        private double period = Double.NaN;
        private ScalarRange configuredRange = ScalarRange.unbounded();
        private double nativePerPlantUnit = 1.0;
        private double tolerance = 10.0;
        private ReferenceMode referenceMode = ReferenceMode.STATIC;
        private double plantReference = 0.0;
        private double nativeReference = 0.0;
        private double assumedPlantPosition = 0.0;
        private String unreferencedReason = "position reference not established";

        private Builder(PositionOutput positionOut,
                        PowerOutput regulatedPowerOut,
                        ScalarRegulator regulator,
                        ScalarSource nativeMeasurement) {
            this.positionOut = positionOut;
            this.regulatedPowerOut = regulatedPowerOut;
            this.regulator = regulator;
            this.nativeMeasurement = nativeMeasurement;
        }

        /**
         * Sets linear/periodic topology. Period must be finite and positive for periodic plants.
         */
        public Builder topology(Topology topology, double period) {
            this.topology = Objects.requireNonNull(topology, "topology");
            this.period = topology == Topology.PERIODIC ? period : Double.NaN;
            return this;
        }

        /**
         * Sets the static legal target range in plant units.
         */
        public Builder range(ScalarRange range) {
            this.configuredRange = Objects.requireNonNull(range, "range");
            return this;
        }

        /**
         * Sets how many native units correspond to one plant unit.
         */
        public Builder nativePerPlantUnit(double nativePerPlantUnit) {
            if (!Double.isFinite(nativePerPlantUnit) || Math.abs(nativePerPlantUnit) < 1.0e-12)
                throw new IllegalArgumentException("nativePerPlantUnit must be finite and non-zero");
            this.nativePerPlantUnit = nativePerPlantUnit;
            return this;
        }

        /**
         * Sets the completion tolerance in plant units.
         */
        public Builder positionTolerance(double tolerance) {
            if (tolerance < 0.0 || !Double.isFinite(tolerance))
                throw new IllegalArgumentException("positionTolerance must be finite and >= 0");
            this.tolerance = tolerance;
            return this;
        }

        /**
         * Allows calibration-search tasks to temporarily drive the mechanism using open-loop power.
         */
        public Builder searchPowerOutput(PowerOutput searchPowerOut) {
            this.searchPowerOut = searchPowerOut;
            return this;
        }

        /**
         * Uses a static reference: {@code plantPosition} maps to {@code nativePosition}.
         */
        public Builder plantPositionMapsToNative(double plantPosition, double nativePosition) {
            this.referenceMode = ReferenceMode.STATIC;
            this.plantReference = plantPosition;
            this.nativeReference = nativePosition;
            return this;
        }

        /**
         * Assumes the current native measurement corresponds to {@code plantPosition} on first update.
         */
        public Builder assumeCurrentPositionIs(double plantPosition) {
            this.referenceMode = ReferenceMode.ASSUME_CURRENT;
            this.assumedPlantPosition = plantPosition;
            return this;
        }

        /**
         * Requires an explicit calibration/reference task before position targets can be applied.
         */
        public Builder needsReference(String reason) {
            this.referenceMode = ReferenceMode.NEEDS_REFERENCE;
            this.unreferencedReason = reason;
            return this;
        }

        /**
         * Sets dynamic plant target guards.
         */
        public Builder targetGuards(PlantTargetGuards targetGuards) {
            this.targetGuards = targetGuards == null ? PlantTargetGuards.none() : targetGuards;
            return this;
        }

        /**
         * Use a final target source. If the source is a {@link ScalarTarget}, it is auto-registered for tasks.
         */
        public Builder targetedBy(ScalarSource targetSource) {
            this.targetSource = Objects.requireNonNull(targetSource, "targetSource");
            if (targetSource instanceof ScalarTarget)
                this.writableTarget = (ScalarTarget) targetSource;
            return this;
        }

        /**
         * Register the command target that plant tasks may write when the final source is composed.
         */
        public Builder writableTarget(ScalarTarget writableTarget) {
            this.writableTarget = Objects.requireNonNull(writableTarget, "writableTarget");
            return this;
        }

        /**
         * Builds the mapped position plant.
         */
        public MappedPositionPlant build() {
            if (targetSource == null)
                throw new IllegalStateException("MappedPositionPlant requires targetedBy(...)");
            return new MappedPositionPlant(positionOut, regulatedPowerOut, regulator, nativeMeasurement,
                    searchPowerOut, targetSource, writableTarget, targetGuards, topology, period,
                    configuredRange, nativePerPlantUnit, tolerance, referenceMode, plantReference,
                    nativeReference, assumedPlantPosition, unreferencedReason);
        }
    }

    private void validate() {
        if (positionOut == null && regulatedPowerOut == null)
            throw new IllegalStateException("MappedPositionPlant requires either a position output or regulated power output");
        if (positionOut != null && regulatedPowerOut != null)
            throw new IllegalStateException("MappedPositionPlant cannot use both a position output and regulated power output");
        if (regulatedPowerOut != null && regulator == null)
            throw new IllegalStateException("Regulated position plants require a regulator");
        if (topology == Topology.PERIODIC && (!(period > 0.0) || !Double.isFinite(period)))
            throw new IllegalArgumentException("Periodic position plants require finite period > 0");
        if (!Double.isFinite(nativePerPlantUnit) || Math.abs(nativePerPlantUnit) < 1.0e-12)
            throw new IllegalArgumentException("nativePerPlantUnit must be finite and non-zero");
        if (tolerance < 0.0 || !Double.isFinite(tolerance))
            throw new IllegalArgumentException("positionTolerance must be finite and >= 0");
        if ((referenceMode == ReferenceMode.ASSUME_CURRENT || referenceMode == ReferenceMode.NEEDS_REFERENCE) && nativeMeasurement == null) {
            throw new IllegalStateException(referenceMode + " requires native feedback so a reference can be established");
        }
    }

    @Override
    public void update(LoopClock clock) {
        if (pendingAssume) {
            double nativeNow = sampleNative(clock);
            if (Double.isFinite(nativeNow))
                establishReferenceFromNative(assumedPlantPosition, nativeNow);
        }

        requestedTarget = targetSource.getAsDouble(clock);

        if (searchActive) {
            samplePlantMeasurement(clock);
            if (searchPowerOut != null) searchPowerOut.setPower(searchPower);
            lastAtTarget = false;
            targetStatus = PlantTargetStatus.holdingLast("calibration search active");
            return;
        }

        ScalarRange range = targetRange();
        double candidate = Double.isFinite(requestedTarget) ? requestedTarget : appliedTarget;
        PlantTargetStatus status = PlantTargetStatus.ACCEPTED;
        if (!range.valid) {
            status = PlantTargetStatus.referenceNotEstablished(range.reason);
            candidate = appliedTarget;
        } else {
            double clamped = range.clamp(candidate);
            if (Math.abs(clamped - candidate) > 1e-9)
                status = PlantTargetStatus.clampedToRange("target clamped to position range");
            candidate = clamped;
        }

        PlantTargetGuards.Result guarded = targetGuards.apply(candidate, status, appliedTarget, clock);
        appliedTarget = guarded.target;
        targetStatus = guarded.status;

        if (!isReferenced()) {
            stopNormalPositionOutput();
            samplePlantMeasurement(clock);
            lastAtTarget = false;
            return;
        }

        samplePlantMeasurement(clock);
        if (positionOut != null) {
            positionOut.setPosition(toNative(appliedTarget));
        } else if (regulatedPowerOut != null) {
            lastRegulatorOutput = regulator.update(appliedTarget, lastMeasurement, clock);
            regulatedPowerOut.setPower(lastRegulatorOutput);
        }
        lastAtTarget = atTarget(requestedTarget);
    }

    @Override
    public void reset() {
        targetSource.reset();
        targetGuards.reset();
        if (nativeMeasurement != null) nativeMeasurement.reset();
        if (regulator != null) regulator.reset();
        lastMeasurement = Double.NaN;
        lastNativeMeasurement = Double.NaN;
        lastAtTarget = false;
        lastRegulatorOutput = 0.0;
        targetStatus = PlantTargetStatus.STOPPED;
        if (referenceMode == ReferenceMode.ASSUME_CURRENT) {
            referenced = false;
            pendingAssume = true;
        }
    }

    @Override
    public void stop() {
        if (positionOut != null) positionOut.stop();
        if (regulatedPowerOut != null) regulatedPowerOut.stop();
        if (searchPowerOut != null) searchPowerOut.stop();
        if (regulator != null) regulator.reset();
        targetGuards.reset();
        searchActive = false;
        lastRegulatorOutput = 0.0;
        targetStatus = PlantTargetStatus.STOPPED;
    }

    @Override
    public double getRequestedTarget() {
        return requestedTarget;
    }

    @Override
    public double getAppliedTarget() {
        return appliedTarget;
    }

    @Override
    public PlantTargetStatus getTargetStatus() {
        return targetStatus;
    }

    @Override
    public boolean hasWritableTarget() {
        return writableTarget != null;
    }

    @Override
    public ScalarTarget writableTarget() {
        if (writableTarget == null) return PositionPlant.super.writableTarget();
        return writableTarget;
    }

    @Override
    public boolean hasFeedback() {
        return nativeMeasurement != null;
    }

    @Override
    public double getMeasurement() {
        return lastMeasurement;
    }

    @Override
    public boolean atTarget() {
        return lastAtTarget;
    }

    @Override
    public boolean atTarget(double target) {
        return hasFeedback()
                && Double.isFinite(target)
                && Double.isFinite(lastMeasurement)
                && Math.abs(requestedTarget - target) <= tolerance
                && Math.abs(appliedTarget - target) <= tolerance
                && Math.abs(lastMeasurement - target) <= tolerance
                && targetStatus.kind() == PlantTargetStatus.Kind.ACCEPTED;
    }

    @Override
    public Topology topology() {
        return topology;
    }

    @Override
    public double period() {
        return topology == Topology.PERIODIC ? period : Double.NaN;
    }

    @Override
    public ScalarRange targetRange() {
        if (!isReferenced()) return ScalarRange.invalid(referenceStatus());
        return configuredRange;
    }

    @Override
    public boolean isReferenced() {
        return referenced;
    }

    @Override
    public String referenceStatus() {
        if (referenced) return "referenced";
        if (pendingAssume) return "reference pending first update";
        return unreferencedReason;
    }

    @Override
    public void establishReferenceAt(double plantPosition) {
        if (!Double.isFinite(lastNativeMeasurement))
            throw new IllegalStateException("Cannot establish position reference before a finite native measurement has been sampled");
        establishReferenceFromNative(plantPosition, lastNativeMeasurement);
    }

    @Override
    public void establishReferenceAt(double plantPosition, LoopClock clock) {
        double nativeNow = sampleNative(clock);
        if (!Double.isFinite(nativeNow))
            throw new IllegalStateException("Cannot establish position reference from non-finite native measurement");
        establishReferenceFromNative(plantPosition, nativeNow);
    }

    @Override
    public boolean supportsCalibrationSearch() {
        return searchPowerOut != null;
    }

    @Override
    public void beginCalibrationSearch(double power) {
        if (searchPowerOut == null)
            throw new IllegalStateException("This PositionPlant does not support calibration search drive");
        stopNormalPositionOutput();
        searchActive = true;
        searchPower = power;
        searchPowerOut.setPower(power);
        targetStatus = PlantTargetStatus.holdingLast("calibration search active");
    }

    @Override
    public void endCalibrationSearch(boolean stopOutput) {
        if (stopOutput && searchPowerOut != null) searchPowerOut.stop();
        searchActive = false;
    }

    private void stopNormalPositionOutput() {
        if (positionOut != null) positionOut.stop();
        if (regulatedPowerOut != null) regulatedPowerOut.stop();
        if (regulator != null) regulator.reset();
    }

    private double sampleNative(LoopClock clock) {
        if (nativeMeasurement == null) {
            lastNativeMeasurement = Double.NaN;
            return Double.NaN;
        }
        lastNativeMeasurement = nativeMeasurement.getAsDouble(clock);
        return lastNativeMeasurement;
    }

    private void samplePlantMeasurement(LoopClock clock) {
        double nativeValue = sampleNative(clock);
        lastMeasurement = isReferenced() && Double.isFinite(nativeValue) ? toPlant(nativeValue) : Double.NaN;
    }

    private void establishReferenceFromNative(double requestedPlantReference, double nativeAtReference) {
        double resolvedPlantReference = requestedPlantReference;
        if (topology == Topology.PERIODIC && referenced && Double.isFinite(lastMeasurement)) {
            double k = Math.rint((lastMeasurement - requestedPlantReference) / period);
            resolvedPlantReference = requestedPlantReference + k * period;
        }
        plantReference = resolvedPlantReference;
        nativeReference = nativeAtReference;
        referenced = true;
        pendingAssume = false;
        lastMeasurement = resolvedPlantReference;
        lastNativeMeasurement = nativeAtReference;
    }

    private double toNative(double plantPosition) {
        return nativeReference + nativePerPlantUnit * (plantPosition - plantReference);
    }

    private double toPlant(double nativePosition) {
        return plantReference + (nativePosition - nativeReference) / nativePerPlantUnit;
    }

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) return;
        PositionPlant.super.debugDump(dbg, prefix);
        String p = (prefix == null || prefix.isEmpty()) ? "positionPlant" : prefix;
        dbg.addData(p + ".topology", topology)
                .addData(p + ".period", period())
                .addData(p + ".range", targetRange())
                .addData(p + ".referenced", referenced)
                .addData(p + ".referenceStatus", referenceStatus())
                .addData(p + ".nativePerPlantUnit", nativePerPlantUnit)
                .addData(p + ".plantReference", plantReference)
                .addData(p + ".nativeReference", nativeReference)
                .addData(p + ".lastNativeMeasurement", lastNativeMeasurement)
                .addData(p + ".searchActive", searchActive)
                .addData(p + ".lastRegulatorOutput", lastRegulatorOutput);
        targetGuards.debugDump(dbg, p + ".targetGuards");
        if (regulator != null) regulator.debugDump(dbg, p + ".regulator");
    }
}
