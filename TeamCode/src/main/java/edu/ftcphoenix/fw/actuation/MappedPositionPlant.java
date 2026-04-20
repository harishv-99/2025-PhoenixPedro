package edu.ftcphoenix.fw.actuation;

import java.util.Objects;

import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.core.hal.PositionOutput;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.time.LoopClock;

/**
 * Advanced implementation of {@link PositionPlant} that maps caller-facing plant units to native
 * hardware/control units.
 *
 * <p>Most robot code should create this through the guided {@code FtcActuators.plant(...)} builder
 * rather than constructing it directly. The class is public so FTC and non-FTC boundary builders can
 * share the same plant-domain mapping implementation.</p>
 *
 * <h2>Coordinate model</h2>
 *
 * <p>The mapping is affine:</p>
 *
 * <pre>{@code
 * native = nativeReference + nativePerPlantUnit * (plant - plantReference)
 * }</pre>
 *
 * <p>{@code nativePerPlantUnit} is known at build time. The reference may be static, assumed from
 * the current native measurement on first update, or established later by a homing/indexing task.
 * All public plant targets, measurements, ranges, periods, and tolerances are in plant units.</p>
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
         * The coordinate is invalid until a task calls {@link #establishReferenceAt(double, LoopClock)}.
         */
        NEEDS_REFERENCE
    }

    private final PositionOutput positionOut;
    private final PowerOutput regulatedPowerOut;
    private final ScalarRegulator regulator;
    private final ScalarSource nativeMeasurement;
    private final PowerOutput searchPowerOut;
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
    private double target = 0.0;
    private double desiredTarget = 0.0;
    private double lastMeasurement = Double.NaN;
    private double lastNativeMeasurement = Double.NaN;
    private boolean lastAtSetpoint;
    private boolean searchActive;
    private double searchPower;
    private double lastRegulatorOutput;
    private String lastBlockReason = "";

    private MappedPositionPlant(PositionOutput positionOut,
                                PowerOutput regulatedPowerOut,
                                ScalarRegulator regulator,
                                ScalarSource nativeMeasurement,
                                PowerOutput searchPowerOut,
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
        this.topology = Objects.requireNonNull(topology, "topology");
        this.period = period;
        this.configuredRange = Objects.requireNonNull(configuredRange, "configuredRange");
        this.nativePerPlantUnit = nativePerPlantUnit;
        this.tolerance = tolerance;
        this.referenceMode = Objects.requireNonNull(referenceMode, "referenceMode");
        this.plantReference = plantReference;
        this.nativeReference = nativeReference;
        this.assumedPlantPosition = assumedPlantPosition;
        this.unreferencedReason = (unreferencedReason == null || unreferencedReason.isEmpty())
                ? "position reference not established"
                : unreferencedReason;
        this.referenced = referenceMode == ReferenceMode.STATIC;
        this.pendingAssume = referenceMode == ReferenceMode.ASSUME_CURRENT;
        validate();
    }

    /**
     * Starts configuring a commanded-position plant such as a standard servo.
     *
     * @param out native position output; commands are produced in native units
     */
    public static Builder commanded(PositionOutput out) {
        return new Builder(Objects.requireNonNull(out, "out"), null, null, null);
    }

    /**
     * Starts configuring a position-output plant with native feedback, such as FTC RUN_TO_POSITION.
     *
     * @param out               native position output; commands are produced in native units
     * @param nativeMeasurement authoritative native-position measurement source
     */
    public static Builder positionOutput(PositionOutput out, ScalarSource nativeMeasurement) {
        return new Builder(Objects.requireNonNull(out, "out"), null, null,
                Objects.requireNonNull(nativeMeasurement, "nativeMeasurement"));
    }

    /**
     * Starts configuring a framework-regulated position plant that drives power.
     *
     * <p>The regulator receives setpoint and measurement in plant units. The native measurement
     * source is converted through the configured coordinate map before the regulator is updated.</p>
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
     * Builder for advanced boundary adapters. Robot code should normally use {@code FtcActuators}.
     */
    public static final class Builder {
        private final PositionOutput positionOut;
        private final PowerOutput regulatedPowerOut;
        private final ScalarRegulator regulator;
        private final ScalarSource nativeMeasurement;
        private PowerOutput searchPowerOut;
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
         * Sets the plant coordinate topology. Period must be finite and positive for periodic plants.
         */
        public Builder topology(Topology topology, double period) {
            this.topology = Objects.requireNonNull(topology, "topology");
            this.period = topology == Topology.PERIODIC ? period : Double.NaN;
            return this;
        }

        /**
         * Sets the valid plant-unit target range.
         */
        public Builder range(ScalarRange range) {
            this.configuredRange = Objects.requireNonNull(range, "range");
            return this;
        }

        /**
         * Sets how many native units correspond to one plant unit.
         */
        public Builder nativePerPlantUnit(double nativePerPlantUnit) {
            if (!Double.isFinite(nativePerPlantUnit) || Math.abs(nativePerPlantUnit) < 1.0e-12) {
                throw new IllegalArgumentException("nativePerPlantUnit must be finite and non-zero");
            }
            this.nativePerPlantUnit = nativePerPlantUnit;
            return this;
        }

        /**
         * Sets the plant-level completion tolerance in plant units.
         */
        public Builder positionTolerance(double tolerance) {
            if (tolerance < 0.0 || !Double.isFinite(tolerance)) {
                throw new IllegalArgumentException("positionTolerance must be finite and >= 0");
            }
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
         * Samples native feedback on first update and treats that sample as {@code plantPosition}.
         */
        public Builder assumeCurrentPositionIs(double plantPosition) {
            this.referenceMode = ReferenceMode.ASSUME_CURRENT;
            this.assumedPlantPosition = plantPosition;
            this.unreferencedReason = "position reference pending first update";
            return this;
        }

        /**
         * Starts invalid until a task establishes a reference.
         */
        public Builder needsReference(String reason) {
            this.referenceMode = ReferenceMode.NEEDS_REFERENCE;
            this.unreferencedReason = reason;
            return this;
        }

        /**
         * Builds the mapped position plant.
         */
        public MappedPositionPlant build() {
            return new MappedPositionPlant(positionOut,
                    regulatedPowerOut,
                    regulator,
                    nativeMeasurement,
                    searchPowerOut,
                    topology,
                    period,
                    configuredRange,
                    nativePerPlantUnit,
                    tolerance,
                    referenceMode,
                    plantReference,
                    nativeReference,
                    assumedPlantPosition,
                    unreferencedReason);
        }
    }

    private void validate() {
        if (positionOut == null && regulatedPowerOut == null) {
            throw new IllegalStateException("MappedPositionPlant requires either a position output or a regulated power output");
        }
        if (positionOut != null && regulatedPowerOut != null) {
            throw new IllegalStateException("MappedPositionPlant cannot use both a position output and regulated power output");
        }
        if (regulatedPowerOut != null && regulator == null) {
            throw new IllegalStateException("Regulated position plants require a regulator");
        }
        if (topology == Topology.PERIODIC && (!(period > 0.0) || !Double.isFinite(period))) {
            throw new IllegalArgumentException("Periodic position plants require finite period > 0");
        }
        if (!Double.isFinite(nativePerPlantUnit) || Math.abs(nativePerPlantUnit) < 1.0e-12) {
            throw new IllegalArgumentException("nativePerPlantUnit must be finite and non-zero");
        }
        if ((referenceMode == ReferenceMode.ASSUME_CURRENT || referenceMode == ReferenceMode.NEEDS_REFERENCE)
                && nativeMeasurement == null) {
            throw new IllegalStateException(referenceMode + " requires native feedback so a reference can be established");
        }
    }

    @Override
    public void setTarget(double target) {
        desiredTarget = target;
        if (searchActive) {
            lastBlockReason = "calibration search active";
            return;
        }
        ScalarRange range = targetRange();
        if (!range.valid) {
            lastBlockReason = range.reason;
            return;
        }
        double applied = range.clamp(target);
        this.target = applied;
        lastBlockReason = applied == target ? "" : "target clamped to plant range";
        if (positionOut != null) {
            positionOut.setPosition(toNative(applied));
        }
    }

    @Override
    public double getTarget() {
        return target;
    }

    /**
     * Returns the latest requested target, even if a range/reference/search guard blocked it.
     */
    public double getDesiredTarget() {
        return desiredTarget;
    }

    @Override
    public void update(LoopClock clock) {
        if (clock == null) {
            return;
        }
        if (pendingAssume) {
            double nativeNow = sampleNative(clock);
            if (Double.isFinite(nativeNow)) {
                establishReferenceFromNative(assumedPlantPosition, nativeNow);
                pendingAssume = false;
                if (Double.isFinite(desiredTarget)) {
                    setTarget(desiredTarget);
                }
            }
        }

        if (searchActive) {
            samplePlantMeasurement(clock);
            if (searchPowerOut != null) {
                searchPowerOut.setPower(searchPower);
            }
            lastAtSetpoint = false;
            return;
        }

        if (regulatedPowerOut != null) {
            samplePlantMeasurement(clock);
            if (!isReferenced()) {
                regulatedPowerOut.stop();
                lastRegulatorOutput = 0.0;
                lastAtSetpoint = false;
                return;
            }
            lastRegulatorOutput = regulator.update(target, lastMeasurement, clock);
            regulatedPowerOut.setPower(lastRegulatorOutput);
        } else {
            samplePlantMeasurement(clock);
        }
        lastAtSetpoint = Double.isFinite(lastMeasurement) && Math.abs(target - lastMeasurement) <= tolerance;
    }

    @Override
    public void reset() {
        if (nativeMeasurement != null) {
            nativeMeasurement.reset();
        }
        if (regulator != null) {
            regulator.reset();
        }
        lastMeasurement = Double.NaN;
        lastNativeMeasurement = Double.NaN;
        lastAtSetpoint = false;
        lastRegulatorOutput = 0.0;
    }

    @Override
    public void stop() {
        if (positionOut != null) {
            positionOut.stop();
        }
        if (regulatedPowerOut != null) {
            regulatedPowerOut.stop();
        }
        if (searchPowerOut != null) {
            searchPowerOut.stop();
        }
        if (regulator != null) {
            regulator.reset();
        }
        searchActive = false;
        lastRegulatorOutput = 0.0;
    }

    @Override
    public boolean atSetpoint() {
        return lastAtSetpoint;
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
    public Topology topology() {
        return topology;
    }

    @Override
    public double period() {
        return topology == Topology.PERIODIC ? period : Double.NaN;
    }

    @Override
    public ScalarRange targetRange() {
        if (!isReferenced()) {
            return ScalarRange.invalid(referenceStatus());
        }
        return configuredRange;
    }

    @Override
    public boolean isReferenced() {
        return referenced;
    }

    @Override
    public String referenceStatus() {
        if (referenced) {
            return "referenced";
        }
        if (pendingAssume) {
            return "reference pending first update";
        }
        return unreferencedReason;
    }

    @Override
    public void establishReferenceAt(double plantPosition) {
        if (!Double.isFinite(lastNativeMeasurement)) {
            throw new IllegalStateException("Cannot establish position reference before a finite native measurement has been sampled");
        }
        establishReferenceFromNative(plantPosition, lastNativeMeasurement);
        if (Double.isFinite(desiredTarget)) {
            setTarget(desiredTarget);
        }
    }

    @Override
    public void establishReferenceAt(double plantPosition, LoopClock clock) {
        double nativeNow = sampleNative(clock);
        if (!Double.isFinite(nativeNow)) {
            throw new IllegalStateException("Cannot establish position reference from non-finite native measurement");
        }
        establishReferenceFromNative(plantPosition, nativeNow);
        if (Double.isFinite(desiredTarget)) {
            setTarget(desiredTarget);
        }
    }

    @Override
    public boolean supportsCalibrationSearch() {
        return searchPowerOut != null;
    }

    @Override
    public void beginCalibrationSearch(double power) {
        if (searchPowerOut == null) {
            throw new IllegalStateException("This PositionPlant does not support calibration search drive");
        }
        stopNormalPositionOutput();
        searchActive = true;
        searchPower = power;
        searchPowerOut.setPower(power);
        lastBlockReason = "calibration search active";
    }

    @Override
    public void endCalibrationSearch(boolean stopOutput) {
        if (stopOutput && searchPowerOut != null) {
            searchPowerOut.stop();
        }
        searchActive = false;
        lastBlockReason = "";
    }

    private void stopNormalPositionOutput() {
        if (positionOut != null) {
            positionOut.stop();
        }
        if (regulatedPowerOut != null) {
            regulatedPowerOut.stop();
        }
        if (regulator != null) {
            regulator.reset();
        }
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
        this.plantReference = resolvedPlantReference;
        this.nativeReference = nativeAtReference;
        this.referenced = true;
        this.pendingAssume = false;
        this.lastMeasurement = resolvedPlantReference;
        this.lastNativeMeasurement = nativeAtReference;
    }

    private double toNative(double plantPosition) {
        return nativeReference + nativePerPlantUnit * (plantPosition - plantReference);
    }

    private double toPlant(double nativePosition) {
        return plantReference + (nativePosition - nativeReference) / nativePerPlantUnit;
    }

    @Override
    public void debugDump(DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "positionPlant" : prefix;
        dbg.addData(p + ".target", getTarget())
                .addData(p + ".desiredTarget", desiredTarget)
                .addData(p + ".measurement", getMeasurement())
                .addData(p + ".error", getError())
                .addData(p + ".atSetpoint", atSetpoint())
                .addData(p + ".hasFeedback", hasFeedback())
                .addData(p + ".topology", topology)
                .addData(p + ".period", period())
                .addData(p + ".range", targetRange())
                .addData(p + ".referenced", referenced)
                .addData(p + ".referenceStatus", referenceStatus())
                .addData(p + ".nativePerPlantUnit", nativePerPlantUnit)
                .addData(p + ".plantReference", plantReference)
                .addData(p + ".nativeReference", nativeReference)
                .addData(p + ".lastNativeMeasurement", lastNativeMeasurement)
                .addData(p + ".searchActive", searchActive)
                .addData(p + ".lastRegulatorOutput", lastRegulatorOutput)
                .addData(p + ".lastBlockReason", lastBlockReason);
        if (regulator != null) {
            regulator.debugDump(dbg, p + ".regulator");
        }
    }
}
