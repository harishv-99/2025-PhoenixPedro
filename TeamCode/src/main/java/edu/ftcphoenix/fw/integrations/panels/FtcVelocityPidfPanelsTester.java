package edu.ftcphoenix.fw.integrations.panels;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;
import java.util.function.Function;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.ScalarRange;
import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.ftc.FtcMotorControllers;
import edu.ftcphoenix.fw.ftc.FtcMotorVelocityPidf;
import edu.ftcphoenix.fw.tools.tester.BaseTeleOpTester;

/** Framework-owned implementation behind {@link FtcPanelsTuners#velocityPidf}. */
final class FtcVelocityPidfPanelsTester extends BaseTeleOpTester {

    static final String GRAPH_SEGMENT_ID = "tune.velocityPidf.segmentId";
    static final String GRAPH_TARGET = "tune.velocityPidf.requestedTarget";
    static final String GRAPH_MEASUREMENT = "tune.velocityPidf.measuredAbs";
    static final String GRAPH_ERROR = "tune.velocityPidf.error";
    static final String GRAPH_ABSOLUTE_SPEED_RATE =
            "tune.velocityPidf.measuredAbsRatePerSec";

    private static final double DEFAULT_AUTO_STOP_AFTER_SEC = 5.0;
    private static final double DRAFT_QUIESCENCE_SEC = 0.10;
    private static final double DRAFT_STABILITY_TIMEOUT_SEC = 0.75;

    enum TransitionType {
        COLD_START,
        HOT_UPDATE
    }

    interface ControllerSession {
        void apply(Gains gains);

        Gains readback();

        void restoreInitial();
    }

    interface DraftPort {
        Candidate read();

        void seedAndRefresh(Gains gains, double target, double autoStopAfterSec);
    }

    static final class Gains {
        final double kP;
        final double kI;
        final double kD;
        final double kF;

        Gains(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }

        boolean sameValues(Gains other) {
            return other != null
                    && sameDouble(kP, other.kP)
                    && sameDouble(kI, other.kI)
                    && sameDouble(kD, other.kD)
                    && sameDouble(kF, other.kF);
        }

        String compact() {
            return "kP=" + kP + ", kI=" + kI + ", kD=" + kD + ", kF=" + kF;
        }
    }

    static final class Candidate {
        final Gains gains;
        final double target;
        final double autoStopAfterSec;

        Candidate(double kP,
                  double kI,
                  double kD,
                  double kF,
                  double target,
                  double autoStopAfterSec) {
            this(new Gains(kP, kI, kD, kF), target, autoStopAfterSec);
        }

        Candidate(Gains gains, double target, double autoStopAfterSec) {
            this.gains = Objects.requireNonNull(gains, "gains");
            this.target = target;
            this.autoStopAfterSec = autoStopAfterSec;
        }

        boolean sameValues(Candidate other) {
            return other != null
                    && gains.sameValues(other.gains)
                    && sameDouble(target, other.target)
                    && sameDouble(autoStopAfterSec, other.autoStopAfterSec);
        }

        String compact() {
            return gains.compact()
                    + ", target=" + target
                    + ", autoStop=" + autoStopAfterSec + " s";
        }
    }

    static final class Segment {
        final long id;
        final TransitionType transitionType;
        final Candidate requested;
        final Gains controllerReadback;
        final double startSec;
        final String changes;

        Segment(long id,
                TransitionType transitionType,
                Candidate requested,
                Gains controllerReadback,
                double startSec,
                String changes) {
            this.id = id;
            this.transitionType = transitionType;
            this.requested = requested;
            this.controllerReadback = controllerReadback;
            this.startSec = startSec;
            this.changes = changes;
        }
    }

    static final class CompletedSegment {
        final Segment segment;
        final double endSec;
        final String reason;

        CompletedSegment(Segment segment, double endSec, String reason) {
            this.segment = segment;
            this.endSec = endSec;
            this.reason = reason;
        }
    }

    private static final class PendingCapture {
        Candidate lastSample;
        long lastSampleCycle;
        double unchangedSinceSec;
        final double requestedAtSec;

        PendingCapture(Candidate firstSample, long cycle, double requestedAtSec) {
            lastSample = firstSample;
            lastSampleCycle = cycle;
            unchangedSinceSec = requestedAtSec;
            this.requestedAtSec = requestedAtSec;
        }
    }

    private static final class FtcControllerSession implements ControllerSession {
        private final FtcMotorVelocityPidf controller;

        FtcControllerSession(FtcMotorVelocityPidf controller) {
            this.controller = Objects.requireNonNull(controller, "controller");
        }

        @Override
        public void apply(Gains gains) {
            controller.setGains(gains.kP, gains.kI, gains.kD, gains.kF);
        }

        @Override
        public Gains readback() {
            return new Gains(
                    controller.getKP(),
                    controller.getKI(),
                    controller.getKD(),
                    controller.getKF());
        }

        @Override
        public void restoreInitial() {
            controller.restoreInitial();
        }
    }

    private static final class PanelsDraftPort implements DraftPort {
        private static final PanelsDraftPort INSTANCE = new PanelsDraftPort();

        @Override
        public Candidate read() {
            return FtcPanelsTuners.readVelocityDraft();
        }

        @Override
        public void seedAndRefresh(Gains gains, double target, double autoStopAfterSec) {
            FtcPanelsTuners.seedVelocityDraft(gains, target, autoStopAfterSec);
        }
    }

    private final String testerName;
    private final ScalarRange testTargetRange;
    private final double initialTestTarget;
    private final Function<HardwareMap, Plant> plantFactory;
    private final boolean usesInjectedOwnership;

    private Plant plant;
    private ControllerSession controller;
    private DraftPort draft;

    private Gains lastControllerReadback;
    private PendingCapture pendingCapture;
    private Candidate pendingColdCandidate;
    private Segment currentSegment;
    private CompletedSegment lastCompletedSegment;
    private long nextSegmentId = 1L;

    private boolean started;
    private boolean cleanupClaimed;
    private boolean finiteMeasurementAvailable;
    private boolean finiteAbsoluteSpeedRateAvailable;
    private double measured;
    private double measuredAbs;
    private double absoluteSpeedRate;
    private double previousMeasuredAbs;
    private String statusMessage = "INIT: start this OpMode when the mechanism is safe to test";

    FtcVelocityPidfPanelsTester(String testerName,
                               ScalarRange testTargetRange,
                               Function<HardwareMap, Plant> plantFactory) {
        this.testerName = requireNonBlank("testerName", testerName);
        this.testTargetRange = requireTestRange(testTargetRange);
        this.initialTestTarget = this.testTargetRange.minValue;
        this.plantFactory = Objects.requireNonNull(plantFactory, "plantFactory");
        this.usesInjectedOwnership = false;
    }

    /** Package-local deterministic-test seam. */
    FtcVelocityPidfPanelsTester(String testerName,
                               ScalarRange testTargetRange,
                               Plant plant,
                               ControllerSession controller,
                               DraftPort draft) {
        this.testerName = requireNonBlank("testerName", testerName);
        this.testTargetRange = requireTestRange(testTargetRange);
        this.initialTestTarget = this.testTargetRange.minValue;
        this.plantFactory = null;
        this.plant = Objects.requireNonNull(plant, "plant");
        this.controller = Objects.requireNonNull(controller, "controller");
        this.draft = Objects.requireNonNull(draft, "draft");
        this.usesInjectedOwnership = true;
    }

    @Override
    public String name() {
        return testerName;
    }

    @Override
    protected void onInit() {
        if (!usesInjectedOwnership) {
            acquireOwnership();
        }
        requireTunablePlant(plant);

        started = false;
        cleanupClaimed = false;
        finiteMeasurementAvailable = false;
        finiteAbsoluteSpeedRateAvailable = false;
        measured = 0.0;
        measuredAbs = 0.0;
        absoluteSpeedRate = 0.0;
        previousMeasuredAbs = 0.0;
        pendingCapture = null;
        pendingColdCandidate = null;
        currentSegment = null;
        lastCompletedSegment = null;
        nextSegmentId = 1L;

        // Establish the tester's inactive request without actuating the Plant during FTC INIT.
        plant.commandTarget().set(0.0);
        lastControllerReadback = Objects.requireNonNull(
                controller.readback(),
                "velocity PIDF controller initial readback returned null");
        draft.seedAndRefresh(
                lastControllerReadback,
                initialTestTarget,
                DEFAULT_AUTO_STOP_AFTER_SEC);

        // A is declared before B/BACK so a simultaneous safety action wins.
        bindings.onRise(gamepads.p1().a(), this::beginCandidateCapture);
        bindings.onRise(gamepads.p1().b(), () -> requestActiveZero("B pressed"));
        bindings.onRise(gamepads.p1().back(), this::stopFromBack);

        renderTelemetry();
    }

    @Override
    protected void onInitLoop(double dtSec) {
        renderTelemetry();
    }

    @Override
    protected void onStart() {
        if (cleanupClaimed) {
            started = false;
            statusMessage = "STOPPED: create a new OpMode instance for another tuning session";
            return;
        }
        started = true;
        statusMessage = "ZERO REQUESTED: finish Update All, verify Draft values, then press A";
    }

    @Override
    protected void onLoop(double dtSec) {
        processPendingCapture();
        processPendingColdStart();
        processAutomaticStop();

        if (!cleanupClaimed) {
            plant.update(clock);
            refreshFeedbackEvidence();
        }
        renderTelemetry();
    }

    @Override
    protected void onStop() {
        started = false;
        cleanupOwnedSession();
    }

    private void acquireOwnership() {
        Plant builtPlant = null;
        try {
            builtPlant = plantFactory.apply(ctx.hw);
            requireTunablePlant(builtPlant);
            FtcMotorVelocityPidf builtController =
                    FtcMotorControllers.velocityPidf(builtPlant);
            requireTestRangeInsidePlant(
                    testTargetRange,
                    builtController.plantTargetRange());
            plant = builtPlant;
            controller = new FtcControllerSession(builtController);
            draft = PanelsDraftPort.INSTANCE;
        } catch (RuntimeException primaryFailure) {
            final Plant rollbackPlant = builtPlant;
            throw CleanupActions.attemptAllAfterFailure(
                    primaryFailure,
                    () -> stopIfConstructed(rollbackPlant));
        }
    }

    private void beginCandidateCapture() {
        if (cleanupClaimed) {
            // BACK/stop is terminal for this tester instance. Later sampled inputs must not make
            // its presentation look merely pre-start or suggest that it can be rearmed.
            return;
        }
        if (!started) {
            statusMessage = "Press FTC START before applying a candidate";
            return;
        }

        Candidate firstSample = draft.read();
        pendingCapture = new PendingCapture(firstSample, clock.cycle(), clock.nowSec());
        pendingColdCandidate = null;
        statusMessage = "CAPTURING: waiting for the complete Panels draft to stay unchanged";
    }

    private void processPendingCapture() {
        if (pendingCapture == null || cleanupClaimed) {
            return;
        }

        double nowSec = clock.nowSec();
        if (nowSec - pendingCapture.requestedAtSec > DRAFT_STABILITY_TIMEOUT_SEC) {
            pendingCapture = null;
            statusMessage = "REJECTED: Panels draft kept changing; finish Update All, then press A";
            return;
        }

        long cycle = clock.cycle();
        if (cycle == pendingCapture.lastSampleCycle) {
            return;
        }

        Candidate sample = draft.read();
        pendingCapture.lastSampleCycle = cycle;
        if (!sample.sameValues(pendingCapture.lastSample)) {
            pendingCapture.lastSample = sample;
            pendingCapture.unchangedSinceSec = nowSec;
            return;
        }

        if (nowSec - pendingCapture.unchangedSinceSec < DRAFT_QUIESCENCE_SEC) {
            return;
        }

        Candidate captured = pendingCapture.lastSample;
        pendingCapture = null;
        String rejection = validateNonControllerCandidate(captured);
        if (rejection != null) {
            statusMessage = "REJECTED: " + rejection;
            return;
        }

        if (currentSegment != null) {
            applyCandidate(captured, TransitionType.HOT_UPDATE);
            return;
        }

        pendingColdCandidate = captured;
        statusMessage = "COLD START WAIT: finite feedback and Plant atTarget(0.0) required";
    }

    private void processPendingColdStart() {
        if (pendingColdCandidate == null || currentSegment != null || cleanupClaimed) {
            return;
        }
        if (!finiteMeasurementAvailable) {
            statusMessage = "COLD START WAIT: finite velocity feedback is unavailable";
            return;
        }
        if (!plant.atTarget(0.0)) {
            statusMessage = "COLD START WAIT: Plant has not truthfully reached target 0.0";
            return;
        }

        Candidate captured = pendingColdCandidate;
        pendingColdCandidate = null;
        applyCandidate(captured, TransitionType.COLD_START);
    }

    private void applyCandidate(Candidate candidate, TransitionType transitionType) {
        try {
            controller.apply(candidate.gains);
        } catch (IllegalArgumentException rejectedGains) {
            statusMessage = "REJECTED: " + describe(rejectedGains);
            return;
        } catch (RuntimeException controllerFailure) {
            throw terminateAfterFailure(
                    "applying controller PIDF",
                    controllerFailure);
        }

        final Gains readback;
        try {
            readback = Objects.requireNonNull(
                    controller.readback(),
                    "velocity PIDF controller applied readback returned null");
        } catch (RuntimeException readbackFailure) {
            // Apply already returned successfully, so even an IllegalArgumentException here is a
            // post-write uncertainty, not a recoverable candidate-validation rejection.
            throw terminateAfterFailure(
                    "reading back applied controller PIDF",
                    readbackFailure);
        }

        Segment previous = currentSegment;
        String changes = describeChanges(previous, candidate);
        try {
            // The Plant remains the sole target realization path. Hardware changes only when the
            // owning tester's downstream update runs after the complete controller readback.
            plant.commandTarget().set(candidate.target);
        } catch (RuntimeException requestFailure) {
            throw terminateAfterFailure(
                    "committing the captured velocity target",
                    requestFailure);
        }

        double nowSec = clock.nowSec();
        if (previous != null) {
            lastCompletedSegment = new CompletedSegment(previous, nowSec, "HOT_UPDATE");
        }
        currentSegment = new Segment(
                nextSegmentId++,
                transitionType,
                candidate,
                readback,
                nowSec,
                changes);
        lastControllerReadback = readback;
        statusMessage = transitionType + " APPLIED: segment " + currentSegment.id
                + (candidate.autoStopAfterSec == 0.0
                ? " runs until B/BACK/stop"
                : " auto-stops after " + candidate.autoStopAfterSec + " s");
    }

    private void processAutomaticStop() {
        if (currentSegment == null || cleanupClaimed) {
            return;
        }
        double durationSec = currentSegment.requested.autoStopAfterSec;
        if (durationSec == 0.0 || clock.nowSec() - currentSegment.startSec < durationSec) {
            return;
        }
        requestActiveZero("automatic stop after " + durationSec + " s");
    }

    private void requestActiveZero(String reason) {
        if (!started || cleanupClaimed) {
            return;
        }

        pendingCapture = null;
        pendingColdCandidate = null;
        Segment ending = currentSegment;
        try {
            plant.commandTarget().set(0.0);
        } catch (RuntimeException zeroRequestFailure) {
            throw terminateAfterFailure("requesting active zero", zeroRequestFailure);
        }

        if (ending != null) {
            lastCompletedSegment = new CompletedSegment(ending, clock.nowSec(), reason);
            currentSegment = null;
        }
        statusMessage = "ZERO REQUESTED: " + reason
                + "; next A waits for finite feedback and Plant atTarget(0.0)";
    }

    private void stopFromBack() {
        if (cleanupClaimed) {
            return;
        }
        started = false;
        cleanupOwnedSession();
        statusMessage = "STOPPED: BACK stopped the Plant and restored the controller session";
    }

    private String validateNonControllerCandidate(Candidate candidate) {
        double target = candidate.target;
        if (!Double.isFinite(target)) {
            return "testTarget must be finite";
        }
        if (target <= 0.0) {
            return "testTarget must be > 0; use B to request zero";
        }
        if (!testTargetRange.contains(target)) {
            return "testTarget must be inside [" + testTargetRange.minValue
                    + ", " + testTargetRange.maxValue + "]";
        }
        double autoStop = candidate.autoStopAfterSec;
        if (!Double.isFinite(autoStop) || autoStop < 0.0) {
            return "autoStopAfterSec must be finite and >= 0; 0 means no automatic stop";
        }
        return null;
    }

    private void refreshFeedbackEvidence() {
        double rawMeasurement = plant.getMeasurement();
        finiteMeasurementAvailable = Double.isFinite(rawMeasurement);
        measured = finiteMeasurementAvailable ? rawMeasurement : 0.0;
        measuredAbs = Math.abs(measured);

        double dtSec = clock.dtSec();
        if (finiteMeasurementAvailable && dtSec > 1e-6 && Double.isFinite(dtSec)) {
            double derivedRate = (measuredAbs - previousMeasuredAbs) / dtSec;
            finiteAbsoluteSpeedRateAvailable = Double.isFinite(derivedRate);
            absoluteSpeedRate = finiteAbsoluteSpeedRateAvailable ? derivedRate : 0.0;
        } else {
            finiteAbsoluteSpeedRateAvailable = false;
            absoluteSpeedRate = 0.0;
        }
        previousMeasuredAbs = measuredAbs;
    }

    private RuntimeException terminateAfterFailure(String operation, RuntimeException cause) {
        IllegalStateException terminal = new IllegalStateException(
                testerName + " failed while " + operation
                        + "; the Plant was terminally stopped and restoration of the session's "
                        + "starting controller configuration was attempted. Controller and "
                        + "physical state may be uncertain: " + describe(cause),
                cause);
        try {
            cleanupOwnedSession();
        } catch (RuntimeException cleanupFailure) {
            if (cleanupFailure != terminal) {
                terminal.addSuppressed(cleanupFailure);
            }
        }
        return terminal;
    }

    private void cleanupOwnedSession() {
        if (cleanupClaimed) {
            return;
        }
        cleanupClaimed = true;
        pendingCapture = null;
        pendingColdCandidate = null;
        currentSegment = null;
        finiteMeasurementAvailable = false;
        finiteAbsoluteSpeedRateAvailable = false;
        CleanupActions.attemptAll(
                () -> stopIfConstructed(plant),
                () -> restoreIfConstructed(controller));
    }

    private void renderTelemetry() {
        if (ctx == null) {
            return;
        }
        Candidate browserDraft = draft != null ? draft.read() : null;

        telemHeader(testerName);
        ctx.telemetry.addLine(
                "Finish Update All, verify Draft values, then A; capture uses best-effort quiescence.");
        ctx.telemetry.addLine("A: apply/hot update | B: active zero | BACK: stop + restore");
        ctx.telemetry.addData("Status", statusMessage);
        ctx.telemetry.addData("Browser draft", draftStatus(browserDraft));
        ctx.telemetry.addData("Velocity feedback",
                finiteMeasurementAvailable ? "FINITE" : "UNAVAILABLE");
        ctx.telemetry.addData("Absolute-speed rate",
                finiteAbsoluteSpeedRateAvailable ? "FINITE" : "UNAVAILABLE");
        if (browserDraft != null) {
            ctx.telemetry.addData("Draft values", browserDraft.compact());
        }
        if (pendingColdCandidate != null) {
            ctx.telemetry.addData("Waiting candidate", pendingColdCandidate.compact());
        }

        if (currentSegment == null) {
            ctx.telemetry.addData("Current segment", "NONE (active-zero request)");
        } else {
            ctx.telemetry.addData("Current segment",
                    currentSegment.id + " / " + currentSegment.transitionType);
            ctx.telemetry.addData("Requested candidate", currentSegment.requested.compact());
            ctx.telemetry.addData("Changed from previous segment", currentSegment.changes);
            ctx.telemetry.addData("Controller readback", currentSegment.controllerReadback.compact());
            ctx.telemetry.addData("Segment elapsed sec",
                    Math.max(0.0, clock.nowSec() - currentSegment.startSec));
        }

        if (lastCompletedSegment == null) {
            ctx.telemetry.addData("Last completed segment", "NONE");
        } else {
            CompletedSegment completed = lastCompletedSegment;
            ctx.telemetry.addData("Last completed segment",
                    completed.segment.id + " / " + completed.segment.transitionType
                            + " / " + completed.reason);
            ctx.telemetry.addData("Last completed requested", completed.segment.requested.compact());
            ctx.telemetry.addData("Last completed controller readback",
                    completed.segment.controllerReadback.compact());
            ctx.telemetry.addData("Last completed run sec",
                    Math.max(0.0, completed.endSec - completed.segment.startSec));
        }

        long segmentId = currentSegment != null ? currentSegment.id : 0L;
        double target = finiteGraphValue(currentSegment != null
                ? currentSegment.requested.target : 0.0);
        double graphMeasurement = finiteGraphValue(measuredAbs);
        ctx.telemetry.addData(GRAPH_SEGMENT_ID, segmentId);
        ctx.telemetry.addData(GRAPH_TARGET, target);
        ctx.telemetry.addData(GRAPH_MEASUREMENT, graphMeasurement);
        ctx.telemetry.addData(GRAPH_ERROR, finiteGraphValue(target - graphMeasurement));
        ctx.telemetry.addData(
                GRAPH_ABSOLUTE_SPEED_RATE,
                finiteGraphValue(absoluteSpeedRate));

        if (lastControllerReadback != null) {
            ctx.telemetry.addLine("");
            ctx.telemetry.addLine("Accepted controller readback — copy into checked-in config:");
            ctx.telemetry.addLine("kP = " + Double.toString(lastControllerReadback.kP));
            ctx.telemetry.addLine("kI = " + Double.toString(lastControllerReadback.kI));
            ctx.telemetry.addLine("kD = " + Double.toString(lastControllerReadback.kD));
            ctx.telemetry.addLine("kF = " + Double.toString(lastControllerReadback.kF));
        }
        telemUpdate();
    }

    private String draftStatus(Candidate browserDraft) {
        if (pendingCapture != null) {
            return "CAPTURING AFTER A";
        }
        if (browserDraft == null) {
            return "UNAVAILABLE";
        }
        if (currentSegment != null && browserDraft.sameValues(currentSegment.requested)) {
            return "UNCHANGED FROM CURRENT SEGMENT";
        }
        if (lastCompletedSegment != null
                && browserDraft.sameValues(lastCompletedSegment.segment.requested)) {
            return "UNCHANGED FROM LAST COMPLETED SEGMENT";
        }
        return "CHANGED / NOT USED — PRESS A TO CAPTURE";
    }

    private static String describeChanges(Segment previous, Candidate next) {
        if (previous == null) {
            return "initial candidate";
        }
        Candidate prior = previous.requested;
        StringBuilder changes = new StringBuilder();
        appendChange(changes, "kP", prior.gains.kP, next.gains.kP);
        appendChange(changes, "kI", prior.gains.kI, next.gains.kI);
        appendChange(changes, "kD", prior.gains.kD, next.gains.kD);
        appendChange(changes, "kF", prior.gains.kF, next.gains.kF);
        appendChange(changes, "target", prior.target, next.target);
        appendChange(changes, "autoStop", prior.autoStopAfterSec, next.autoStopAfterSec);
        return changes.length() == 0 ? "none (new numbered segment)" : changes.toString();
    }

    private static void appendChange(StringBuilder destination,
                                     String name,
                                     double before,
                                     double after) {
        if (sameDouble(before, after)) {
            return;
        }
        if (destination.length() > 0) {
            destination.append(", ");
        }
        destination.append(name)
                .append(' ')
                .append(before)
                .append(" -> ")
                .append(after);
    }

    private static ScalarRange requireTestRange(ScalarRange range) {
        if (range == null) {
            throw new NullPointerException("testTargetRange");
        }
        if (!range.valid
                || !Double.isFinite(range.minValue)
                || !Double.isFinite(range.maxValue)
                || range.minValue <= 0.0) {
            throw new IllegalArgumentException(
                    "FtcPanelsTuners.velocityPidf requires a finite bounded testTargetRange "
                            + "with minValue > 0, but received " + range);
        }
        return range;
    }

    static void requireTestRangeInsidePlant(ScalarRange testRange,
                                            ScalarRange plantRange) {
        if (plantRange == null
                || !plantRange.valid
                || !plantRange.contains(0.0)
                || !plantRange.contains(testRange.minValue)
                || !plantRange.contains(testRange.maxValue)) {
            throw new IllegalArgumentException(
                    "Velocity PIDF testTargetRange " + testRange
                            + " must lie inside a bound Plant target range that also contains "
                            + "the required zero stop target; Plant range=" + plantRange);
        }
    }

    private static String requireNonBlank(String argument, String value) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "FtcPanelsTuners.velocityPidf requires a nonblank " + argument);
        }
        return value;
    }

    private static void requireTunablePlant(Plant plant) {
        if (plant == null) {
            throw new IllegalStateException("velocity PIDF plantFactory returned null");
        }
        if (!plant.hasCommandTarget()) {
            throw new IllegalArgumentException(
                    "Velocity PIDF tuning requires a Plant with one graph-owned command target");
        }
        if (!plant.hasFeedback()) {
            throw new IllegalArgumentException(
                    "Velocity PIDF tuning requires a feedback Plant");
        }
    }

    private static boolean sameDouble(double first, double second) {
        return Double.doubleToLongBits(first) == Double.doubleToLongBits(second);
    }

    private static double finiteGraphValue(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    private static String describe(RuntimeException failure) {
        String message = failure.getMessage();
        return failure.getClass().getSimpleName()
                + (message == null || message.trim().isEmpty() ? "" : ": " + message);
    }

    private static void stopIfConstructed(Plant plant) {
        if (plant != null) {
            plant.stop();
        }
    }

    private static void restoreIfConstructed(ControllerSession controller) {
        if (controller != null) {
            controller.restoreInitial();
        }
    }
}
