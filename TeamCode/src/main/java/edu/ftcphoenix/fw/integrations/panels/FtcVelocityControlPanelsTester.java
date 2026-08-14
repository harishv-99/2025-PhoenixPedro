package edu.ftcphoenix.fw.integrations.panels;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Function;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.ScalarRange;
import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.tools.tester.BaseTeleOpTester;

/** Framework-owned implementation behind {@link FtcPanelsTuners#velocityControl}. */
final class FtcVelocityControlPanelsTester extends BaseTeleOpTester {

    static final String GRAPH_SEGMENT_ID = "tune.velocityControl.segmentId";
    static final String GRAPH_TARGET = "tune.velocityControl.requestedTarget";
    static final String GRAPH_MEASUREMENT = "tune.velocityControl.measurement";
    static final String GRAPH_ERROR = "tune.velocityControl.error";
    static final String METRIC_FIRST_AT_TARGET_SEC =
            "tune.velocityControl.firstAtTargetSec";
    static final String METRIC_SETTLING_SEC = "tune.velocityControl.settlingSec";
    static final String METRIC_DIRECTIONAL_DROOP =
            "tune.velocityControl.directionalDroop";
    static final String METRIC_OVERSHOOT = "tune.velocityControl.overshoot";
    static final String METRIC_OUTPUT_LIMITED_SEC =
            "tune.velocityControl.outputLimitedSec";
    static final String METRIC_DISTURBANCE_PEAK =
            "tune.velocityControl.disturbancePeakError";
    static final String METRIC_DISTURBANCE_RECOVERY_SEC =
            "tune.velocityControl.disturbanceRecoverySec";

    private static final String FIELD_TARGET = "targetVelocity";
    private static final String FIELD_AUTO_STOP_SEC = "autoStopAfterSec";
    private static final double DEFAULT_AUTO_STOP_AFTER_SEC = 5.0;
    private static final double DRAFT_QUIESCENCE_SEC = 0.10;
    private static final double DRAFT_STABILITY_TIMEOUT_SEC = 0.75;
    private static final int HISTORY_ROWS = 6;

    interface DraftPort {
        Map<String, Double> read();

        void seed(ControlTuningModel.Parameters controller,
                  Map<String, Double> experiment);
    }

    static final class Candidate {
        final ControlTuningModel.Parameters controller;
        final double target;
        final double autoStopAfterSec;

        Candidate(ControlTuningModel.Parameters controller,
                  double target,
                  double autoStopAfterSec) {
            this.controller = Objects.requireNonNull(controller, "controller");
            this.target = target;
            this.autoStopAfterSec = autoStopAfterSec;
        }

        boolean sameTarget(Candidate other) {
            return other != null
                    && ControlTuningModel.sameDouble(target, other.target);
        }

        boolean sameFullRequest(Candidate other) {
            return other != null
                    && controller.sameValues(other.controller)
                    && sameTarget(other)
                    && ControlTuningModel.sameDouble(
                            autoStopAfterSec, other.autoStopAfterSec);
        }

        Map<String, Double> experimentRequest() {
            LinkedHashMap<String, Double> request = new LinkedHashMap<String, Double>();
            request.put(FIELD_TARGET, target);
            request.put(FIELD_AUTO_STOP_SEC, autoStopAfterSec);
            return request;
        }

        String compact() {
            return controller.compact() + ", target=" + target
                    + ", autoStop=" + autoStopAfterSec + " s";
        }
    }

    private static final class PendingCapture {
        Map<String, Double> lastSample;
        long lastSampleCycle;
        double unchangedSinceSec;
        final double requestedAtSec;

        PendingCapture(Map<String, Double> firstSample, long cycle, double requestedAtSec) {
            lastSample = firstSample;
            lastSampleCycle = cycle;
            unchangedSinceSec = requestedAtSec;
            this.requestedAtSec = requestedAtSec;
        }
    }

    private static final class PendingReconfiguration {
        final Candidate candidate;
        final ControlExperimentHistory.Transition transition;
        final boolean controllerChanged;
        final boolean targetChanged;
        final boolean requireControllerZeroEvidence;

        PendingReconfiguration(Candidate candidate,
                               ControlExperimentHistory.Transition transition,
                               boolean controllerChanged,
                               boolean targetChanged,
                               boolean requireControllerZeroEvidence) {
            this.candidate = candidate;
            this.transition = transition;
            this.controllerChanged = controllerChanged;
            this.targetChanged = targetChanged;
            this.requireControllerZeroEvidence = requireControllerZeroEvidence;
        }
    }

    private static final class ActiveSegment {
        final long id;
        final ControlExperimentHistory.Transition transition;
        final Candidate candidate;
        final List<ControlTuningModel.Readback> readbacks;
        final double startSec;
        final ControlResponseMetrics.Velocity metrics;
        boolean realizedByPlantUpdate;

        ActiveSegment(long id,
                      ControlExperimentHistory.Transition transition,
                      Candidate candidate,
                      List<ControlTuningModel.Readback> readbacks,
                      double startSec,
                      double initialMeasurement) {
            this.id = id;
            this.transition = transition;
            this.candidate = candidate;
            this.readbacks = ControlTuningModel.immutableReadbacks(readbacks);
            this.startSec = startSec;
            metrics = new ControlResponseMetrics.Velocity(
                    candidate.target, startSec, initialMeasurement);
        }
    }

    private static final class PlantFacts {
        final double requestedTarget;
        final double appliedTarget;
        final double measurement;

        PlantFacts(double requestedTarget, double appliedTarget, double measurement) {
            this.requestedTarget = requestedTarget;
            this.appliedTarget = appliedTarget;
            this.measurement = measurement;
        }

        void addTo(Map<String, Double> destination) {
            destination.put("finalRequestedTarget", requestedTarget);
            destination.put("finalAppliedTarget", appliedTarget);
            destination.put("finalMeasurement", measurement);
        }
    }

    private static final class PanelsDraftPort implements DraftPort {
        private static final PanelsDraftPort INSTANCE = new PanelsDraftPort();

        @Override
        public Map<String, Double> read() {
            return FtcPanelsTuners.readActiveDraft();
        }

        @Override
        public void seed(ControlTuningModel.Parameters controller,
                         Map<String, Double> experiment) {
            FtcPanelsTuners.seedActiveDraft(controller, experiment);
        }
    }

    private final String testerName;
    private final ScalarRange testTargetRange;
    private final double initialTestTarget;
    private final Function<HardwareMap, Plant> plantFactory;
    private final Function<Plant, ControlTuningModel.Session> sessionFactory;
    private final boolean usesInjectedOwnership;

    private Plant plant;
    private ControlTuningModel.Session controller;
    private DraftPort draft;
    private ControlTuningModel.Parameters controllerSchema;
    private List<ControlTuningModel.Readback> lastReadbacks;
    private Candidate lastAcceptedCandidate;
    private PendingCapture pendingCapture;
    private PendingReconfiguration pendingReconfiguration;
    private ActiveSegment activeSegment;
    private PlantFacts pendingEndingFacts;
    private ControlExperimentHistory history;
    private long nextSegmentId;
    private boolean started;
    private boolean cleanupClaimed;
    private boolean measurementAvailable;
    private double measurement;
    private String statusMessage = "INIT: start this OpMode when the mechanism is safe to test";

    FtcVelocityControlPanelsTester(String testerName,
                                   ScalarRange testTargetRange,
                                   Function<HardwareMap, Plant> plantFactory) {
        this(testerName,
                testTargetRange,
                plantFactory,
                ControlTuningAdapters::claimVelocity,
                null,
                null,
                null);
    }

    /** Package-local deterministic-test seam. */
    FtcVelocityControlPanelsTester(String testerName,
                                   ScalarRange testTargetRange,
                                   Plant plant,
                                   ControlTuningModel.Session controller,
                                   DraftPort draft) {
        this(testerName, testTargetRange, null, null, plant, controller, draft);
    }

    private FtcVelocityControlPanelsTester(
            String testerName,
            ScalarRange testTargetRange,
            Function<HardwareMap, Plant> plantFactory,
            Function<Plant, ControlTuningModel.Session> sessionFactory,
            Plant plant,
            ControlTuningModel.Session controller,
            DraftPort draft
    ) {
        this.testerName = requireNonBlank("testerName", testerName);
        this.testTargetRange = requireFiniteRange("velocityControl", testTargetRange);
        this.initialTestTarget = this.testTargetRange.clamp(0.0);
        this.plantFactory = plantFactory;
        this.sessionFactory = sessionFactory;
        this.plant = plant;
        this.controller = controller;
        this.draft = draft;
        usesInjectedOwnership = plant != null;
        if (usesInjectedOwnership) {
            Objects.requireNonNull(controller, "controller");
            Objects.requireNonNull(draft, "draft");
        } else {
            Objects.requireNonNull(plantFactory, "plantFactory");
            Objects.requireNonNull(sessionFactory, "sessionFactory");
        }
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
        cleanupClaimed = false;
        try {
        requireVelocityPlant(plant);
        validateTestRangeInsidePlant(testTargetRange, controller.plantTargetRange());

        started = false;
        measurementAvailable = false;
        measurement = Double.NaN;
        pendingCapture = null;
        pendingReconfiguration = null;
        activeSegment = null;
        pendingEndingFacts = null;
        lastAcceptedCandidate = null;
        history = new ControlExperimentHistory();
        nextSegmentId = 1L;

        controllerSchema = Objects.requireNonNull(
                controller.initialCandidate(), "controller initialCandidate");
        String schemaError = controllerSchema.validationError();
        if (schemaError != null) {
            throw new IllegalStateException("Controller initial candidate is invalid: " + schemaError);
        }
        lastReadbacks = checkedReadbacks(controller.readbacks(), controllerSchema);

        // Establish the inactive safe request without advancing the Plant during FTC INIT.
        plant.commandTarget().set(0.0);
        LinkedHashMap<String, Double> experiment = new LinkedHashMap<String, Double>();
        experiment.put(FIELD_TARGET, initialTestTarget);
        experiment.put(FIELD_AUTO_STOP_SEC, DEFAULT_AUTO_STOP_AFTER_SEC);
        draft.seed(controllerSchema, experiment);

        bindings.onRise(gamepads.p1().a(), this::beginCandidateCapture);
        bindings.onRise(gamepads.p1().b(), () -> requestZero("B_PRESSED"));
        bindings.onRise(gamepads.p1().back(), this::stopFromBack);
        renderTelemetry();
        } catch (RuntimeException failure) {
            throw CleanupActions.attemptAllAfterFailure(
                    failure, () -> cleanupOwnedSession("INIT_FAILURE"));
        }
    }

    @Override
    protected void onInitLoop(double dtSec) {
        renderTelemetry();
    }

    @Override
    protected void onStart() {
        if (cleanupClaimed) {
            statusMessage = "STOPPED: start a fresh OpMode for another tuning session";
            return;
        }
        started = true;
        statusMessage = "ZERO REQUESTED: verify the active draft, then press A";
    }

    @Override
    protected void onLoop(double dtSec) {
        try {
            processPendingCapture();
            processAutomaticStop();
            if (!cleanupClaimed) {
                plant.update(clock);
                if (activeSegment != null) {
                    activeSegment.realizedByPlantUpdate = true;
                }
                refreshEvidence();
                processPendingReconfiguration();
            }
            renderTelemetry();
        } catch (RuntimeException failure) {
            if (cleanupClaimed) {
                throw failure;
            }
            throw terminateAfterFailure("updating the velocity experiment", failure);
        }
    }

    @Override
    protected void onStop() {
        started = false;
        cleanupOwnedSession("OPMODE_STOP");
    }

    List<ControlExperimentHistory.Record> historyRecords() {
        return history == null
                ? Collections.<ControlExperimentHistory.Record>emptyList()
                : history.records();
    }

    String sessionId() {
        return history == null ? "UNASSIGNED" : history.sessionId();
    }

    private void acquireOwnership() {
        Plant builtPlant = null;
        try {
            builtPlant = plantFactory.apply(ctx.hw);
            requireVelocityPlant(builtPlant);
            plant = builtPlant;
            controller = Objects.requireNonNull(
                    sessionFactory.apply(builtPlant), "velocity controller tuning session");
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
            return;
        }
        if (!started) {
            statusMessage = "Press FTC START before applying an experiment";
            return;
        }
        if (pendingReconfiguration != null) {
            statusMessage = "WAIT: an accepted candidate is already waiting at zero";
            return;
        }
        Map<String, Double> firstSample = draft.read();
        pendingCapture = new PendingCapture(firstSample, clock.cycle(), clock.nowSec());
        statusMessage = "CAPTURING: waiting for the complete active draft to stay unchanged";
    }

    private void processPendingCapture() {
        if (pendingCapture == null || cleanupClaimed) {
            return;
        }
        double nowSec = clock.nowSec();
        if (nowSec - pendingCapture.requestedAtSec > DRAFT_STABILITY_TIMEOUT_SEC) {
            pendingCapture = null;
            statusMessage = "REJECTED: draft kept changing; finish Update All, then press A";
            return;
        }
        if (clock.cycle() == pendingCapture.lastSampleCycle) {
            return;
        }
        Map<String, Double> sample = draft.read();
        pendingCapture.lastSampleCycle = clock.cycle();
        if (!sameDraft(sample, pendingCapture.lastSample)) {
            pendingCapture.lastSample = sample;
            pendingCapture.unchangedSinceSec = nowSec;
            return;
        }
        if (nowSec - pendingCapture.unchangedSinceSec < DRAFT_QUIESCENCE_SEC) {
            return;
        }

        Map<String, Double> captured = pendingCapture.lastSample;
        pendingCapture = null;
        String schemaError = validateDraftSchema(captured, controllerSchema, velocityFields());
        if (schemaError != null) {
            statusMessage = "REJECTED: " + schemaError;
            return;
        }
        Candidate candidate = readCandidate(captured);
        String validationError = validateCandidate(candidate);
        if (validationError != null) {
            statusMessage = "REJECTED: " + validationError;
            return;
        }
        acceptCandidate(candidate);
    }

    private Candidate readCandidate(Map<String, Double> captured) {
        return new Candidate(
                FtcPanelsTuners.readControllerFields(captured, controllerSchema),
                FtcPanelsTuners.readExperimentField(captured, FIELD_TARGET),
                FtcPanelsTuners.readExperimentField(captured, FIELD_AUTO_STOP_SEC));
    }

    private String validateCandidate(Candidate candidate) {
        String parameterError = candidate.controller.validationError();
        if (parameterError != null) {
            return parameterError;
        }
        try {
            controller.validate(candidate.controller);
        } catch (IllegalArgumentException rejected) {
            return describe(rejected);
        }
        if (!Double.isFinite(candidate.target) || !testTargetRange.contains(candidate.target)) {
            return FIELD_TARGET + " must be finite and inside " + testTargetRange;
        }
        if (!Double.isFinite(candidate.autoStopAfterSec) || candidate.autoStopAfterSec < 0.0) {
            return FIELD_AUTO_STOP_SEC + " must be finite and >= 0; 0 disables automatic stop";
        }
        return null;
    }

    private void acceptCandidate(Candidate candidate) {
        ControlTuningModel.Parameters comparisonController = lastAcceptedCandidate == null
                ? controllerSchema : lastAcceptedCandidate.controller;
        boolean controllerChanged = (lastAcceptedCandidate == null
                && controller.requiresInitialApply())
                || !candidate.controller.sameValues(comparisonController);
        boolean targetChanged = lastAcceptedCandidate == null
                || !candidate.sameTarget(lastAcceptedCandidate);
        ControlExperimentHistory.Transition transition = transition(
                controllerChanged, targetChanged);

        boolean controllerRequiresZero = controllerChanged
                && controller.reconfigurationPolicy()
                == ControlTuningModel.ReconfigurationPolicy.ZERO_AND_SETTLE;
        boolean coldBoundary = activeSegment == null;
        if (controllerRequiresZero || coldBoundary) {
            if (activeSegment != null) {
                pendingEndingFacts = capturePlantFacts();
            }
            try {
                plant.commandTarget().set(0.0);
            } catch (RuntimeException failure) {
                throw terminateAfterFailure("staging zero for cold reconfiguration", failure);
            }
            endActiveSegment("CONTROLLER_RECONFIGURATION_ZERO");
            pendingReconfiguration = new PendingReconfiguration(
                    candidate,
                    transition,
                    controllerChanged,
                    targetChanged,
                    controllerRequiresZero);
            statusMessage = controllerRequiresZero
                    ? "ZERO WAIT: every grouped controller must prove mapped zero before "
                            + "the complete candidate is applied"
                    : "COLD START WAIT: finite feedback and Plant atTarget(0.0) required";
            return;
        }
        applyAndStart(candidate, transition, controllerChanged, targetChanged);
    }

    private void processPendingReconfiguration() {
        if (pendingReconfiguration == null || cleanupClaimed) {
            return;
        }
        if (!measurementAvailable || !plant.atTarget(0.0)) {
            statusMessage = "COLD START WAIT: finite feedback and Plant atTarget(0.0) required";
            return;
        }
        if (pendingReconfiguration.requireControllerZeroEvidence
                && !controller.readyForReconfiguration(clock)) {
            statusMessage = "ZERO WAIT: every grouped controller must prove mapped zero";
            return;
        }
        PendingReconfiguration ready = pendingReconfiguration;
        pendingReconfiguration = null;
        applyAndStart(
                ready.candidate,
                ready.transition,
                ready.controllerChanged,
                ready.targetChanged);
    }

    private void applyAndStart(Candidate candidate,
                               ControlExperimentHistory.Transition transition,
                               boolean controllerChanged,
                               boolean targetChanged) {
        boolean targetNeedsCommit = targetChanged
                || activeSegment == null
                || !ControlTuningModel.sameDouble(plant.commandTarget().get(), candidate.target);
        if (activeSegment != null) {
            pendingEndingFacts = capturePlantFacts();
        }
        if (controllerChanged) {
            try {
                controller.apply(candidate.controller, clock);
            } catch (RuntimeException failure) {
                throw terminateAfterFailure("applying the complete controller candidate", failure);
            }
            try {
                lastReadbacks = checkedReadbacks(controller.readbacks(), candidate.controller);
            } catch (RuntimeException failure) {
                throw terminateAfterFailure("reading back the applied controller candidate", failure);
            }
        }

        // A repeat after B must re-arm the unchanged experiment. Otherwise a controller-only A
        // deliberately leaves the persistent target untouched.
        if (targetNeedsCommit) {
            try {
                plant.commandTarget().set(candidate.target);
            } catch (RuntimeException failure) {
                throw terminateAfterFailure("committing the captured velocity target", failure);
            }
        }

        endActiveSegment("NEXT_ACCEPTED_SEGMENT");

        lastAcceptedCandidate = candidate;
        double nowSec = clock.nowSec();
        activeSegment = new ActiveSegment(
                nextSegmentId++,
                transition,
                candidate,
                lastReadbacks,
                nowSec,
                measurementAvailable ? measurement : Double.NaN);
        statusMessage = transition + " ACCEPTED: session " + history.sessionId()
                + ", segment " + activeSegment.id;
    }

    private void processAutomaticStop() {
        if (activeSegment == null || cleanupClaimed) {
            return;
        }
        double durationSec = activeSegment.candidate.autoStopAfterSec;
        if (durationSec > 0.0
                && activeSegment.realizedByPlantUpdate
                && clock.nowSec() - activeSegment.startSec >= durationSec) {
            requestZero("AUTO_STOP_TIMEOUT");
        }
    }

    private void requestZero(String reason) {
        if (!started || cleanupClaimed) {
            return;
        }
        pendingCapture = null;
        pendingReconfiguration = null;
        if (activeSegment != null) {
            pendingEndingFacts = capturePlantFacts();
        }
        try {
            plant.commandTarget().set(0.0);
        } catch (RuntimeException failure) {
            throw terminateAfterFailure("requesting active zero", failure);
        }
        endActiveSegment(reason);
        statusMessage = "ZERO REQUESTED: " + reason;
    }

    private void refreshEvidence() {
        double latestMeasurement = plant.getMeasurement();
        measurementAvailable = Double.isFinite(latestMeasurement);
        measurement = measurementAvailable ? latestMeasurement : Double.NaN;
        if (activeSegment == null) {
            return;
        }
        ControlTuningModel.Evidence evidence = Objects.requireNonNull(
                controller.evidence(clock), "controller evidence");
        if (!measurementAvailable) {
            activeSegment.metrics.retainEvidence(evidence);
            return;
        }
        activeSegment.metrics.update(
                clock.nowSec(),
                measurement,
                controller.experimentAtTarget(
                        plant.atTarget(activeSegment.candidate.target), clock),
                evidence);
    }

    private void endActiveSegment(String reason) {
        if (activeSegment == null) {
            return;
        }
        ActiveSegment ending = activeSegment;
        PlantFacts finalFacts = pendingEndingFacts == null
                ? capturePlantFacts() : pendingEndingFacts;
        ending.metrics.finish(clock.nowSec());
        Map<String, Double> metrics = new LinkedHashMap<String, Double>(ending.metrics.snapshot());
        finalFacts.addTo(metrics);
        history.add(new ControlExperimentHistory.Record(
                history.sessionId(),
                ending.id,
                "VELOCITY",
                ending.transition,
                ending.candidate.controller,
                ending.readbacks,
                ending.candidate.experimentRequest(),
                metrics,
                ending.metrics.evidence(),
                ending.startSec,
                clock.nowSec(),
                reason));
        if (activeSegment == ending) {
            activeSegment = null;
        }
        pendingEndingFacts = null;
    }

    private void stopFromBack() {
        if (cleanupClaimed) {
            return;
        }
        started = false;
        cleanupOwnedSession("BACK_STOP");
        statusMessage = "STOPPED: BACK terminally stopped and restored the tuning session";
    }

    private RuntimeException terminateAfterFailure(String operation, RuntimeException cause) {
        IllegalStateException terminal = new IllegalStateException(
                testerName + " failed while " + operation
                        + "; the Plant was terminally stopped and restoration was attempted. "
                        + "Controller and physical state may be uncertain: " + describe(cause),
                cause);
        try {
            cleanupOwnedSession("TERMINAL_FAILURE");
        } catch (RuntimeException cleanupFailure) {
            if (cleanupFailure != terminal) {
                terminal.addSuppressed(cleanupFailure);
            }
        }
        return terminal;
    }

    private void cleanupOwnedSession(String reason) {
        if (cleanupClaimed) {
            return;
        }
        cleanupClaimed = true;
        pendingCapture = null;
        pendingReconfiguration = null;
        if (activeSegment != null && pendingEndingFacts == null) {
            pendingEndingFacts = capturePlantFacts();
        }
        CleanupActions.attemptAll(
                () -> stopIfConstructed(plant),
                () -> restoreIfConstructed(controller),
                () -> endActiveSegment(reason));
    }

    private void restoreIfConstructed(ControlTuningModel.Session session) {
        if (session != null) {
            session.restoreInitial(clock);
        }
    }

    private void renderTelemetry() {
        if (ctx == null) {
            return;
        }
        Map<String, Double> browserDraft = draft == null
                ? Collections.<String, Double>emptyMap() : draft.read();
        telemHeader(testerName);
        ctx.telemetry.addLine("Update the complete active draft, verify it, then press A.");
        ctx.telemetry.addLine("A: accept one experiment | B: zero | BACK: stop + restore");
        ctx.telemetry.addData("Status", statusMessage);
        ctx.telemetry.addData("Session ID", sessionId());
        ctx.telemetry.addData("Controller topology",
                controller == null ? "UNCLAIMED" : controller.topology());
        ctx.telemetry.addData("Draft", draftStatus(browserDraft));
        ctx.telemetry.addData("Velocity feedback",
                measurementAvailable ? measurement : "UNAVAILABLE");
        ctx.telemetry.addData("History records", historyRecords().size());

        Map<String, Double> metrics = Collections.emptyMap();
        if (activeSegment == null) {
            ctx.telemetry.addData("Current segment", "NONE");
        } else {
            ctx.telemetry.addData("Current segment",
                    activeSegment.id + " / " + activeSegment.transition);
            ctx.telemetry.addData("Captured request", activeSegment.candidate.compact());
            ctx.telemetry.addData("Controller readbacks", compactReadbacks(activeSegment.readbacks));
            metrics = activeSegment.metrics.snapshot();
            ctx.telemetry.addData("Response metrics", compactMetrics(metrics));
            ControlTuningTelemetry.addEvidence(
                    ctx.telemetry,
                    "tune.velocityControl.controller.",
                    activeSegment.metrics.controllerNumericEvidence(),
                    activeSegment.metrics.evidence());
        }
        renderHistoryRows();

        double target = activeSegment == null ? 0.0 : activeSegment.candidate.target;
        ctx.telemetry.addData(GRAPH_SEGMENT_ID,
                activeSegment == null ? 0.0 : activeSegment.id);
        ControlTuningTelemetry.addOptionalNumber(ctx.telemetry, GRAPH_TARGET,
                activeSegment == null ? null : target);
        ControlTuningTelemetry.addOptionalNumber(ctx.telemetry, GRAPH_MEASUREMENT,
                measurementAvailable ? measurement : null);
        ControlTuningTelemetry.addOptionalNumber(ctx.telemetry, GRAPH_ERROR,
                activeSegment != null && measurementAvailable ? target - measurement : null);
        ControlTuningTelemetry.addOptionalNumber(ctx.telemetry, METRIC_FIRST_AT_TARGET_SEC,
                metrics.get("firstAtTargetSec"));
        ControlTuningTelemetry.addOptionalNumber(ctx.telemetry, METRIC_SETTLING_SEC,
                metrics.get("settlingSec"));
        ControlTuningTelemetry.addOptionalNumber(ctx.telemetry, METRIC_DIRECTIONAL_DROOP,
                metrics.get("directionalDroop"));
        ControlTuningTelemetry.addOptionalNumber(ctx.telemetry, METRIC_OVERSHOOT,
                metrics.get("overshoot"));
        ControlTuningTelemetry.addOptionalNumber(ctx.telemetry, METRIC_OUTPUT_LIMITED_SEC,
                metrics.get("outputLimitedDurationSec"));
        ControlTuningTelemetry.addOptionalNumber(ctx.telemetry, METRIC_DISTURBANCE_PEAK,
                metrics.get("disturbancePeakAbsError"));
        ControlTuningTelemetry.addOptionalNumber(
                ctx.telemetry, METRIC_DISTURBANCE_RECOVERY_SEC,
                metrics.get("disturbanceRecoverySec"));
        telemUpdate();
    }

    private void renderHistoryRows() {
        List<ControlExperimentHistory.Record> records = historyRecords();
        int first = Math.max(0, records.size() - HISTORY_ROWS);
        for (int i = first; i < records.size(); i++) {
            ControlExperimentHistory.Record record = records.get(i);
            ctx.telemetry.addData("History " + record.segmentId,
                    record.transition + " / " + record.terminationReason
                            + " / target=" + record.experimentRequest.get(FIELD_TARGET)
                            + " / settle=" + record.metrics.get("settlingSec"));
            ControlTuningTelemetry.addEvidence(
                    ctx.telemetry,
                    "tune.velocityControl.history." + record.segmentId + ".controller.",
                    controllerMetrics(record.metrics),
                    record.evidence);
        }
    }

    private static Map<String, Double> controllerMetrics(Map<String, Double> metrics) {
        LinkedHashMap<String, Double> controller = new LinkedHashMap<String, Double>();
        for (Map.Entry<String, Double> entry : metrics.entrySet()) {
            if (entry.getKey().startsWith("controller.")) {
                controller.put(entry.getKey().substring("controller.".length()),
                        entry.getValue());
            }
        }
        return controller;
    }

    private String draftStatus(Map<String, Double> browserDraft) {
        if (pendingCapture != null) {
            return "CAPTURING AFTER A";
        }
        String schemaError = controllerSchema == null
                ? "controller not claimed"
                : validateDraftSchema(browserDraft, controllerSchema, velocityFields());
        if (schemaError != null) {
            return "INVALID: " + schemaError;
        }
        Candidate browser = readCandidate(browserDraft);
        if (activeSegment != null && browser.sameFullRequest(activeSegment.candidate)) {
            return "UNCHANGED FROM CURRENT SEGMENT";
        }
        if (lastAcceptedCandidate != null && browser.sameFullRequest(lastAcceptedCandidate)) {
            return "UNCHANGED REPEAT — A STARTS A NEW OBSERVATION";
        }
        Candidate comparison = activeSegment == null
                ? lastAcceptedCandidate : activeSegment.candidate;
        if (comparison != null
                && browser.controller.sameValues(comparison.controller)
                && browser.sameTarget(comparison)) {
            return "ANCILLARY CHANGE — A STARTS A REPEAT WITHOUT REWRITING TARGET";
        }
        return "CHANGED / NOT ACCEPTED";
    }

    static ScalarRange requireFiniteRange(String workflow, ScalarRange range) {
        if (range == null) {
            throw new NullPointerException("allowedTestTargetRange");
        }
        if (!range.valid || !Double.isFinite(range.minValue)
                || !Double.isFinite(range.maxValue)) {
            throw new IllegalArgumentException(
                    "FtcPanelsTuners." + workflow
                            + " requires a finite bounded allowed target range, but received "
                            + range);
        }
        return range;
    }

    static void validateTestRangeInsidePlant(ScalarRange testRange, ScalarRange plantRange) {
        if (plantRange == null || !plantRange.valid || !plantRange.contains(0.0)
                || !plantRange.contains(testRange.minValue)
                || !plantRange.contains(testRange.maxValue)) {
            throw new IllegalArgumentException(
                    "Velocity allowed target range " + testRange
                            + " must lie inside a valid Plant target range that also contains "
                            + "the required zero request; Plant range=" + plantRange);
        }
    }

    static String validateDraftSchema(Map<String, Double> draft,
                                      ControlTuningModel.Parameters controllerSchema,
                                      List<String> experimentFields) {
        if (draft == null) {
            return "active Panels draft is unavailable";
        }
        List<String> expected = new ArrayList<String>();
        for (String field : controllerSchema.values().keySet()) {
            expected.add(FtcPanelsTuners.controllerKey(field));
        }
        for (String field : experimentFields) {
            expected.add(FtcPanelsTuners.experimentKey(field));
        }
        if (!new ArrayList<String>(draft.keySet()).equals(expected)) {
            return "active draft schema changed; expected " + expected
                    + " but found " + draft.keySet();
        }
        for (String field : expected) {
            Double value = draft.get(field);
            if (value == null || !Double.isFinite(value)) {
                return field + " must be finite";
            }
        }
        return null;
    }

    private static List<String> velocityFields() {
        List<String> fields = new ArrayList<String>(2);
        fields.add(FIELD_TARGET);
        fields.add(FIELD_AUTO_STOP_SEC);
        return fields;
    }

    private static List<ControlTuningModel.Readback> checkedReadbacks(
            List<ControlTuningModel.Readback> readbacks,
            ControlTuningModel.Parameters schema
    ) {
        List<ControlTuningModel.Readback> checked =
                ControlTuningModel.immutableReadbacks(readbacks);
        if (!ControlTuningModel.sameReadbackSchema(schema, checked)) {
            throw new IllegalStateException(
                    "Controller readback schema does not match the claimed active topology");
        }
        for (ControlTuningModel.Readback readback : checked) {
            String error = readback.parameters.validationError();
            if (error != null) {
                throw new IllegalStateException(
                        "Controller readback for " + readback.owner + " is invalid: " + error);
            }
        }
        return checked;
    }

    private static ControlExperimentHistory.Transition transition(
            boolean controllerChanged,
            boolean targetChanged
    ) {
        if (controllerChanged && targetChanged) {
            return ControlExperimentHistory.Transition.CONTROLLER_AND_TARGET_CHANGE;
        }
        if (controllerChanged) {
            return ControlExperimentHistory.Transition.CONTROLLER_CHANGE;
        }
        if (targetChanged) {
            return ControlExperimentHistory.Transition.TARGET_CHANGE;
        }
        return ControlExperimentHistory.Transition.REPEAT;
    }

    private static void requireVelocityPlant(Plant plant) {
        if (plant == null) {
            throw new IllegalStateException("velocity plantFactory returned null");
        }
        if (!plant.hasCommandTarget()) {
            throw new IllegalArgumentException(
                    "Velocity control tuning requires one graph-owned command target");
        }
        if (!plant.hasFeedback()) {
            throw new IllegalArgumentException("Velocity control tuning requires feedback");
        }
    }

    private static String requireNonBlank(String argument, String value) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "FtcPanelsTuners requires a nonblank " + argument);
        }
        return value;
    }

    private static boolean sameDraft(Map<String, Double> first, Map<String, Double> second) {
        if (first == null || second == null || !first.keySet().equals(second.keySet())) {
            return false;
        }
        for (Map.Entry<String, Double> entry : first.entrySet()) {
            Double other = second.get(entry.getKey());
            if (entry.getValue() == null || other == null
                    || !ControlTuningModel.sameDouble(entry.getValue(), other)) {
                return false;
            }
        }
        return true;
    }

    private static String compactReadbacks(List<ControlTuningModel.Readback> readbacks) {
        StringBuilder result = new StringBuilder();
        for (ControlTuningModel.Readback readback : readbacks) {
            if (result.length() > 0) {
                result.append(" | ");
            }
            result.append(readback.compact());
        }
        return result.toString();
    }

    private static String compactMetrics(Map<String, Double> metrics) {
        return "firstAtTarget=" + metrics.get("firstAtTargetSec")
                + ", settle=" + metrics.get("settlingSec")
                + ", peakError=" + metrics.get("peakAbsError")
                + ", droop=" + metrics.get("directionalDroop")
                + ", overshoot=" + metrics.get("overshoot")
                + ", recovery=" + metrics.get("disturbanceRecoverySec");
    }

    private PlantFacts capturePlantFacts() {
        return new PlantFacts(
                safeRequestedTarget(plant),
                safeAppliedTarget(plant),
                safeMeasurement(plant));
    }

    private static double safeRequestedTarget(Plant plant) {
        try {
            return plant == null ? Double.NaN : plant.getRequestedTarget();
        } catch (RuntimeException ignored) {
            return Double.NaN;
        }
    }

    private static double safeAppliedTarget(Plant plant) {
        try {
            return plant == null ? Double.NaN : plant.getAppliedTarget();
        } catch (RuntimeException ignored) {
            return Double.NaN;
        }
    }

    private static double safeMeasurement(Plant plant) {
        try {
            return plant == null ? Double.NaN : plant.getMeasurement();
        } catch (RuntimeException ignored) {
            return Double.NaN;
        }
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
}
