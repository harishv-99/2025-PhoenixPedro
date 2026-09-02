package edu.ftcsushi.fw.integrations.panels;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BiFunction;
import java.util.function.Function;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.actuation.PlantSnapshot;
import edu.ftcsushi.fw.actuation.PositionPlant;
import edu.ftcsushi.fw.actuation.ScalarRange;
import edu.ftcsushi.fw.core.lifecycle.CleanupActions;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.tools.tester.BaseTeleOpTester;

/** Framework-owned implementation behind {@link FtcPanelsTuners#positionControl}. */
final class FtcPositionControlPanelsTester extends BaseTeleOpTester {

    static final String GRAPH_SEGMENT_ID = "tune.positionControl.segmentId";
    static final String GRAPH_TARGET = "tune.positionControl.requestedTarget";
    static final String GRAPH_MEASUREMENT = "tune.positionControl.measurement";
    static final String GRAPH_ERROR = "tune.positionControl.error";
    static final String METRIC_FIRST_AT_TARGET_SEC =
            "tune.positionControl.firstAtTargetSec";
    static final String METRIC_SETTLING_SEC = "tune.positionControl.settlingSec";
    static final String METRIC_OVERSHOOT = "tune.positionControl.overshoot";
    static final String METRIC_PEAK_RATE = "tune.positionControl.peakMeasuredRate";
    static final String METRIC_HOLD_ERROR = "tune.positionControl.holdError";
    static final String METRIC_HOLD_DRIFT = "tune.positionControl.holdDrift";
    static final String METRIC_OUTPUT_LIMITED_SEC =
            "tune.positionControl.outputLimitedSec";
    static final String METRIC_DISTURBANCE_DISPLACEMENT =
            "tune.positionControl.disturbanceDisplacement";
    static final String METRIC_DISTURBANCE_RECOVERY_SEC =
            "tune.positionControl.disturbanceRecoverySec";

    private static final String FIELD_ENDPOINT_A = "endpointA";
    private static final String FIELD_ENDPOINT_B = "endpointB";
    private static final String FIELD_AUTO_HOLD_SEC = "autoHoldAfterSec";
    private static final String FIELD_SELECTED_TARGET = "selectedTarget";
    private static final String FIELD_SELECTED_ENDPOINT = "selectedEndpoint";
    private static final double DEFAULT_AUTO_HOLD_AFTER_SEC = 5.0;
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
        final double endpointA;
        final double endpointB;
        final double autoHoldAfterSec;
        final boolean selectsEndpointA;

        Candidate(ControlTuningModel.Parameters controller,
                  double endpointA,
                  double endpointB,
                  double autoHoldAfterSec,
                  boolean selectsEndpointA) {
            this.controller = Objects.requireNonNull(controller, "controller");
            this.endpointA = endpointA;
            this.endpointB = endpointB;
            this.autoHoldAfterSec = autoHoldAfterSec;
            this.selectsEndpointA = selectsEndpointA;
        }

        double selectedTarget() {
            return selectsEndpointA ? endpointA : endpointB;
        }

        boolean sameSelectedTarget(Candidate other) {
            return other != null
                    && ControlTuningModel.sameDouble(selectedTarget(), other.selectedTarget());
        }

        boolean sameFullRequest(Candidate other) {
            return other != null
                    && controller.sameValues(other.controller)
                    && ControlTuningModel.sameDouble(endpointA, other.endpointA)
                    && ControlTuningModel.sameDouble(endpointB, other.endpointB)
                    && ControlTuningModel.sameDouble(
                            autoHoldAfterSec, other.autoHoldAfterSec)
                    && selectsEndpointA == other.selectsEndpointA;
        }

        Map<String, Double> experimentRequest() {
            LinkedHashMap<String, Double> request = new LinkedHashMap<String, Double>();
            request.put(FIELD_ENDPOINT_A, endpointA);
            request.put(FIELD_ENDPOINT_B, endpointB);
            request.put(FIELD_AUTO_HOLD_SEC, autoHoldAfterSec);
            request.put(FIELD_SELECTED_TARGET, selectedTarget());
            request.put(FIELD_SELECTED_ENDPOINT, selectsEndpointA ? 1.0 : 2.0);
            return request;
        }

        String compact() {
            return controller.compact() + ", endpoints=[" + endpointA + ", " + endpointB
                    + "], selected=" + (selectsEndpointA ? "A" : "B")
                    + "(" + selectedTarget() + "), autoHold=" + autoHoldAfterSec + " s";
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

    private static final class ActiveSegment {
        final long id;
        final ControlExperimentHistory.Transition transition;
        final Candidate candidate;
        final List<ControlTuningModel.Readback> readbacks;
        final double startSec;
        final ControlResponseMetrics.Position metrics;
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
            metrics = new ControlResponseMetrics.Position(
                    candidate.selectedTarget(), startSec, initialMeasurement);
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
    private final ScalarRange allowedPhysicalTargetRange;
    private final Function<HardwareMap, PositionPlant> plantFactory;
    private final BiFunction<HardwareMap, PositionPlant, Task> referenceTaskFactory;
    private final Function<PositionPlant, ControlTuningModel.Session> sessionFactory;
    private final boolean usesInjectedOwnership;

    private PositionPlant plant;
    private ControlTuningModel.Session controller;
    private DraftPort draft;
    private ControlTuningModel.Parameters controllerSchema;
    private List<ControlTuningModel.Readback> lastReadbacks;
    private Candidate lastAcceptedCandidate;
    private PendingCapture pendingCapture;
    private String pendingHoldReason;
    private ActiveSegment activeSegment;
    private PlantSnapshot pendingEndingSnapshot;
    private boolean pendingEndingSnapshotAttempted;
    private ControlExperimentHistory history;
    private Task referenceTask;
    private long nextSegmentId;
    private boolean nextEndpointA;
    private boolean started;
    private boolean cleanupClaimed;
    private boolean referenceAuthorized;
    private boolean holdPrepared;
    private boolean measurementAvailable;
    private double measurement;
    private String statusMessage = "INIT: start this OpMode when the mechanism is safe to test";

    FtcPositionControlPanelsTester(
            String testerName,
            ScalarRange allowedPhysicalTargetRange,
            Function<HardwareMap, PositionPlant> plantFactory,
            BiFunction<HardwareMap, PositionPlant, Task> referenceTaskFactory
    ) {
        this(testerName,
                allowedPhysicalTargetRange,
                plantFactory,
                referenceTaskFactory,
                ControlTuningAdapters::claimPosition,
                null,
                null,
                null);
    }

    /** Package-local deterministic-test seam. */
    FtcPositionControlPanelsTester(
            String testerName,
            ScalarRange allowedPhysicalTargetRange,
            PositionPlant plant,
            ControlTuningModel.Session controller,
            DraftPort draft,
            BiFunction<HardwareMap, PositionPlant, Task> referenceTaskFactory
    ) {
        this(testerName,
                allowedPhysicalTargetRange,
                null,
                referenceTaskFactory,
                null,
                plant,
                controller,
                draft);
    }

    private FtcPositionControlPanelsTester(
            String testerName,
            ScalarRange allowedPhysicalTargetRange,
            Function<HardwareMap, PositionPlant> plantFactory,
            BiFunction<HardwareMap, PositionPlant, Task> referenceTaskFactory,
            Function<PositionPlant, ControlTuningModel.Session> sessionFactory,
            PositionPlant plant,
            ControlTuningModel.Session controller,
            DraftPort draft
    ) {
        this.testerName = requireNonBlank("testerName", testerName);
        this.allowedPhysicalTargetRange = FtcVelocityControlPanelsTester.requireFiniteRange(
                "positionControl", allowedPhysicalTargetRange);
        this.plantFactory = plantFactory;
        this.referenceTaskFactory = referenceTaskFactory;
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
        requirePositionPlant(plant);
        if (plant.isReferenced()) {
            validatePhysicalRange();
        }

        started = false;
        referenceAuthorized = plant.isReferenced();
        holdPrepared = false;
        measurementAvailable = false;
        measurement = Double.NaN;
        pendingCapture = null;
        pendingHoldReason = null;
        activeSegment = null;
        pendingEndingSnapshot = null;
        pendingEndingSnapshotAttempted = false;
        referenceTask = null;
        lastAcceptedCandidate = null;
        history = new ControlExperimentHistory();
        nextSegmentId = 1L;
        nextEndpointA = true;

        controllerSchema = Objects.requireNonNull(
                controller.initialCandidate(), "controller initialCandidate");
        String schemaError = controllerSchema.validationError();
        if (schemaError != null) {
            throw new IllegalStateException("Controller initial candidate is invalid: " + schemaError);
        }
        lastReadbacks = checkedReadbacks(controller.readbacks(), controllerSchema);

        LinkedHashMap<String, Double> experiment = new LinkedHashMap<String, Double>();
        experiment.put(FIELD_ENDPOINT_A, allowedPhysicalTargetRange.minValue);
        experiment.put(FIELD_ENDPOINT_B, allowedPhysicalTargetRange.maxValue);
        experiment.put(FIELD_AUTO_HOLD_SEC, DEFAULT_AUTO_HOLD_AFTER_SEC);
        draft.seed(controllerSchema, experiment);

        bindings.onRise(gamepads.p1().a(), this::beginAction);
        bindings.onRise(gamepads.p1().b(), () -> queueHold("B_PRESSED"));
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
        statusMessage = plant.isReferenced()
                ? "PREPARING HOLD: A remains inert until current-position hold is proven"
                : referenceTaskFactory == null
                ? "PREPARING ASSUME-CURRENT HOLD: A remains inert until reference is proven"
                : "REFERENCE REQUIRED: press A to start one fresh reference Task";
    }

    @Override
    protected void onLoop(double dtSec) {
        try {
            processReferenceTask();
            prepareHoldIfReady();
            processPendingCapture();
            processAutomaticHold();
            processPendingHold();
            if (!cleanupClaimed && (referenceTask != null || holdPrepared)) {
                plant.update(clock);
                if (activeSegment != null) {
                    activeSegment.realizedByPlantUpdate = true;
                }
                refreshEvidence();
            }
            renderTelemetry();
        } catch (RuntimeException failure) {
            if (cleanupClaimed) {
                throw failure;
            }
            throw terminateAfterFailure("updating the position experiment", failure);
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
        PositionPlant builtPlant = null;
        try {
            builtPlant = plantFactory.apply(ctx.hw);
            requirePositionPlant(builtPlant);
            plant = builtPlant;
            controller = Objects.requireNonNull(
                    sessionFactory.apply(builtPlant), "position controller tuning session");
            draft = PanelsDraftPort.INSTANCE;
        } catch (RuntimeException primaryFailure) {
            final PositionPlant rollbackPlant = builtPlant;
            throw CleanupActions.attemptAllAfterFailure(
                    primaryFailure,
                    () -> stopIfConstructed(rollbackPlant));
        }
    }

    private void beginAction() {
        if (cleanupClaimed) {
            return;
        }
        if (!started) {
            statusMessage = "Press FTC START before authorizing reference or motion";
            return;
        }
        if (!referenceAuthorized) {
            if (referenceTaskFactory == null) {
                statusMessage = "WAIT: preparing the Plant's assume-current reference and hold";
            } else {
                startReferenceAttempt();
            }
            return;
        }
        if (!holdPrepared) {
            statusMessage = "WAIT: current-position hold has not been prepared";
            return;
        }
        if (!readyForNextLeg()) {
            statusMessage = "REJECTED: one A authorizes one leg; wait until settled or press B";
            return;
        }
        Map<String, Double> firstSample = draft.read();
        pendingCapture = new PendingCapture(firstSample, clock.cycle(), clock.nowSec());
        statusMessage = "CAPTURING: waiting for the complete active draft to stay unchanged";
    }

    private void startReferenceAttempt() {
        if (referenceTask != null) {
            statusMessage = "REFERENCE ACTIVE: wait or press B to cancel this attempt";
            return;
        }
        Task fresh = Objects.requireNonNull(
                referenceTaskFactory.apply(ctx.hw, plant),
                "referenceTaskFactory returned null");
        referenceAuthorized = false;
        referenceTask = fresh;
        try {
            fresh.start(clock);
        } catch (RuntimeException failure) {
            throw terminateAfterFailure("starting the fresh reference Task", failure);
        }
        statusMessage = "REFERENCE ACTIVE: " + fresh.getDebugName();
    }

    private void processReferenceTask() {
        if (referenceTask == null || cleanupClaimed) {
            return;
        }
        Task active = referenceTask;
        if (!active.isComplete()) {
            active.update(clock);
        }
        if (!active.isComplete()) {
            statusMessage = "REFERENCE ACTIVE: " + active.getDebugName();
            return;
        }
        referenceTask = null;
        TaskOutcome outcome = active.getOutcome();
        if (outcome != TaskOutcome.SUCCESS) {
            referenceAuthorized = false;
            statusMessage = "REFERENCE ENDED: outcome=" + outcome
                    + "; press A to create a fresh retry";
            return;
        }
        if (!plant.isReferenced()) {
            throw new IllegalStateException(
                    "Reference Task reported SUCCESS but the PositionPlant is not referenced");
        }
        referenceAuthorized = true;
        validatePhysicalRange();
        statusMessage = "REFERENCE ESTABLISHED: preparing current-position hold; press A again "
                + "after the hold is proven";
    }

    private void prepareHoldIfReady() {
        if (!started || cleanupClaimed || holdPrepared || referenceTask != null
                || (!referenceAuthorized && referenceTaskFactory != null)) {
            return;
        }
        double current = controller.preparePositionHold(clock);
        if (!Double.isFinite(current)) {
            throw new IllegalStateException(
                    "Position hold preparation did not return a finite current measurement");
        }
        if (!plant.isReferenced()) {
            throw new IllegalStateException(
                    "Position hold preparation returned without establishing a physical reference; "
                            + "use the overload with a fresh referenceTaskFactory");
        }
        referenceAuthorized = true;
        validatePhysicalRange();
        if (!allowedPhysicalTargetRange.contains(current)) {
            throw new IllegalStateException(
                    "Position hold preparation measured " + current
                            + " outside the allowed physical experiment envelope "
                            + allowedPhysicalTargetRange
                            + "; place/reference the mechanism inside the envelope before tuning");
        }
        measurement = current;
        measurementAvailable = true;
        holdPrepared = true;
        statusMessage = "HOLD PREPARED: wait for Plant atTarget(" + current
                + "), then verify the draft and press A";
    }

    private boolean readyForNextLeg() {
        if (!measurementAvailable || !plant.atTarget(plant.commandTarget().get())) {
            return false;
        }
        return activeSegment == null || activeSegment.metrics.isSettled();
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
        String schemaError = FtcVelocityControlPanelsTester.validateDraftSchema(
                captured, controllerSchema, positionFields());
        if (schemaError != null) {
            statusMessage = "REJECTED: " + schemaError;
            return;
        }
        Candidate candidate = readCandidate(captured, nextEndpointA);
        String validationError = validateCandidate(candidate);
        if (validationError != null) {
            statusMessage = "REJECTED: " + validationError;
            return;
        }
        acceptCandidate(candidate);
    }

    private Candidate readCandidate(Map<String, Double> captured, boolean selectEndpointA) {
        return new Candidate(
                FtcPanelsTuners.readControllerFields(captured, controllerSchema),
                FtcPanelsTuners.readExperimentField(captured, FIELD_ENDPOINT_A),
                FtcPanelsTuners.readExperimentField(captured, FIELD_ENDPOINT_B),
                FtcPanelsTuners.readExperimentField(captured, FIELD_AUTO_HOLD_SEC),
                selectEndpointA);
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
        if (!allowedPhysicalTargetRange.contains(candidate.endpointA)) {
            return FIELD_ENDPOINT_A + " must be finite and inside " + allowedPhysicalTargetRange;
        }
        if (!allowedPhysicalTargetRange.contains(candidate.endpointB)) {
            return FIELD_ENDPOINT_B + " must be finite and inside " + allowedPhysicalTargetRange;
        }
        if (!Double.isFinite(candidate.autoHoldAfterSec) || candidate.autoHoldAfterSec < 0.0) {
            return FIELD_AUTO_HOLD_SEC + " must be finite and >= 0; 0 disables automatic hold";
        }
        ScalarRange liveRange = plant.targetRange();
        if (!liveRange.contains(candidate.endpointA) || !liveRange.contains(candidate.endpointB)) {
            return "both exact endpoints must remain inside the referenced Plant range " + liveRange;
        }
        return null;
    }

    private void acceptCandidate(Candidate candidate) {
        if (!readyForNextLeg()) {
            statusMessage = "REJECTED: controller changes and new legs require settled hold";
            return;
        }
        ControlTuningModel.Parameters comparisonController = lastAcceptedCandidate == null
                ? controllerSchema : lastAcceptedCandidate.controller;
        boolean controllerChanged = !candidate.controller.sameValues(comparisonController);
        boolean targetChanged = lastAcceptedCandidate == null
                || !candidate.sameSelectedTarget(lastAcceptedCandidate);
        ControlExperimentHistory.Transition transition = transition(
                controllerChanged, targetChanged);

        boolean targetNeedsCommit = targetChanged
                || activeSegment == null
                || !ControlTuningModel.sameDouble(
                        plant.commandTarget().get(), candidate.selectedTarget());
        double startingMeasurement = measurement;
        if (activeSegment != null) {
            captureEndingSnapshotOnce();
        }

        if (controllerChanged) {
            try {
                controller.apply(candidate.controller, clock);
            } catch (RuntimeException failure) {
                throw terminateAfterFailure("applying the complete position-controller candidate",
                        failure);
            }
            try {
                lastReadbacks = checkedReadbacks(controller.readbacks(), candidate.controller);
            } catch (RuntimeException failure) {
                throw terminateAfterFailure("reading back the position-controller candidate",
                        failure);
            }
        }

        if (targetNeedsCommit) {
            try {
                plant.commandTarget().set(candidate.selectedTarget());
            } catch (RuntimeException failure) {
                throw terminateAfterFailure(
                        "committing the selected exact position endpoint", failure);
            }
        }
        endActiveSegment("NEXT_ACCEPTED_LEG");
        lastAcceptedCandidate = candidate;
        nextEndpointA = !candidate.selectsEndpointA;
        double nowSec = clock.nowSec();
        activeSegment = new ActiveSegment(
                nextSegmentId++,
                transition,
                candidate,
                lastReadbacks,
                nowSec,
                startingMeasurement);
        statusMessage = transition + " ACCEPTED: session " + history.sessionId()
                + ", segment " + activeSegment.id + ", leg to endpoint "
                + (candidate.selectsEndpointA ? "A" : "B");
    }

    private void processAutomaticHold() {
        if (activeSegment == null || cleanupClaimed) {
            return;
        }
        double durationSec = activeSegment.candidate.autoHoldAfterSec;
        if (durationSec > 0.0
                && activeSegment.realizedByPlantUpdate
                && clock.nowSec() - activeSegment.startSec >= durationSec) {
            queueHold("AUTO_HOLD_TIMEOUT");
        }
    }

    private void queueHold(String reason) {
        if (!started || cleanupClaimed) {
            return;
        }
        pendingCapture = null;
        if (referenceTask != null) {
            Task cancelling = referenceTask;
            try {
                cancelling.cancel();
            } catch (RuntimeException failure) {
                throw terminateAfterFailure("cancelling the active reference Task", failure);
            }
            referenceTask = null;
            statusMessage = "REFERENCE CANCELLED: " + reason
                    + "; A creates a fresh reference Task";
            return;
        }
        if (!holdPrepared) {
            statusMessage = "HOLD UNAVAILABLE: the position coordinate is not prepared";
            return;
        }
        if (pendingHoldReason == null) {
            pendingHoldReason = reason;
            statusMessage = "HOLD REQUESTED: sampling current position before normal output";
        }
    }

    private void processPendingHold() {
        if (pendingHoldReason == null || cleanupClaimed) {
            return;
        }
        String reason = pendingHoldReason;
        pendingHoldReason = null;
        boolean retryInterruptedLeg = activeSegment != null
                && !activeSegment.metrics.isSettled();
        boolean interruptedEndpointA = retryInterruptedLeg
                && activeSegment.candidate.selectsEndpointA;
        ControlTuningModel.PositionRecoveryHold hold;
        if (activeSegment != null) {
            captureEndingSnapshotOnce();
        }
        try {
            hold = Objects.requireNonNull(
                    controller.preparePositionRecoveryHold(allowedPhysicalTargetRange, clock),
                    "position recovery hold");
        } catch (RuntimeException failure) {
            throw terminateAfterFailure(
                    "sampling and staging an envelope-safe position hold", failure);
        }
        measurement = hold.measurement;
        measurementAvailable = true;
        endActiveSegment(reason);
        if (retryInterruptedLeg) {
            nextEndpointA = interruptedEndpointA;
        }
        statusMessage = (hold.clamped
                ? "RECOVERING TO SAFE BOUNDARY: " : "HOLDING CURRENT: ")
                + reason + " at target " + hold.holdTarget
                + " from same-cycle measurement " + hold.measurement;
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
                plant.atTarget(activeSegment.candidate.selectedTarget()),
                evidence);
    }

    private void endActiveSegment(String reason) {
        if (activeSegment == null) {
            return;
        }
        ActiveSegment ending = activeSegment;
        captureEndingSnapshotOnce();
        PlantSnapshot finalSnapshot = pendingEndingSnapshot;
        ending.metrics.finish(clock.nowSec());
        Map<String, Double> metrics = new LinkedHashMap<String, Double>(ending.metrics.snapshot());
        addFinalPlantFacts(metrics, finalSnapshot);
        history.add(new ControlExperimentHistory.Record(
                history.sessionId(),
                ending.id,
                "POSITION",
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
        pendingEndingSnapshot = null;
        pendingEndingSnapshotAttempted = false;
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
        pendingHoldReason = null;
        final Task cancelling = referenceTask;
        referenceTask = null;
        if (activeSegment != null) {
            captureEndingSnapshotOnce();
        }
        CleanupActions.attemptAll(
                () -> stopIfConstructed(plant),
                () -> cancelIfActive(cancelling),
                () -> restoreIfConstructed(controller),
                () -> endActiveSegment(reason));
    }

    private void restoreIfConstructed(ControlTuningModel.Session session) {
        if (session != null) {
            session.restoreInitial(clock);
        }
    }

    private void validatePhysicalRange() {
        ScalarRange liveRange = plant.targetRange();
        if (liveRange == null || !liveRange.valid
                || !liveRange.contains(allowedPhysicalTargetRange.minValue)
                || !liveRange.contains(allowedPhysicalTargetRange.maxValue)) {
            throw new IllegalArgumentException(
                    "Position allowed physical target range " + allowedPhysicalTargetRange
                            + " must lie inside the referenced Plant range; Plant range="
                            + liveRange);
        }
        ScalarRange claimedRange = controller.plantTargetRange();
        if (claimedRange != null && claimedRange.valid
                && (!claimedRange.contains(allowedPhysicalTargetRange.minValue)
                || !claimedRange.contains(allowedPhysicalTargetRange.maxValue))) {
            throw new IllegalArgumentException(
                    "Position allowed physical target range " + allowedPhysicalTargetRange
                            + " lies outside the claimed controller Plant range " + claimedRange);
        }
    }

    private void renderTelemetry() {
        if (ctx == null) {
            return;
        }
        Map<String, Double> browserDraft = draft == null
                ? Collections.<String, Double>emptyMap() : draft.read();
        telemHeader(testerName);
        ctx.telemetry.addLine("A: one reference attempt or one alternating exact leg");
        ctx.telemetry.addLine("B: cancel reference / measured hold | BACK: stop + restore");
        ctx.telemetry.addData("Status", statusMessage);
        ctx.telemetry.addData("Session ID", sessionId());
        ctx.telemetry.addData("Controller topology",
                controller == null ? "UNCLAIMED" : controller.topology());
        ctx.telemetry.addData("Reference", plant == null ? "UNAVAILABLE" : plant.referenceStatus());
        ctx.telemetry.addData("Position periodicity",
                plant == null ? "UNAVAILABLE" : plant.periodicity());
        if (plant != null && plant.periodicity() == PositionPlant.Periodicity.PERIODIC) {
            ctx.telemetry.addData("Position period", plant.period());
            ctx.telemetry.addLine("Endpoints are exact unwrapped coordinates; no modulo reselection.");
        }
        ctx.telemetry.addData("Draft", draftStatus(browserDraft));
        ctx.telemetry.addData("Position feedback",
                measurementAvailable ? measurement : "UNAVAILABLE");
        ctx.telemetry.addData("Next leg", nextEndpointA ? "ENDPOINT_A" : "ENDPOINT_B");
        ctx.telemetry.addData("History records", historyRecords().size());

        Map<String, Double> metrics = Collections.emptyMap();
        if (activeSegment == null) {
            ctx.telemetry.addData("Current segment", "NONE (holding/reference/idle)");
        } else {
            ctx.telemetry.addData("Current segment",
                    activeSegment.id + " / " + activeSegment.transition);
            ctx.telemetry.addData("Captured request", activeSegment.candidate.compact());
            ctx.telemetry.addData("Controller readbacks", compactReadbacks(activeSegment.readbacks));
            metrics = activeSegment.metrics.snapshot();
            ctx.telemetry.addData("Response metrics", compactMetrics(metrics));
            ControlTuningTelemetry.addEvidence(
                    ctx.telemetry,
                    "tune.positionControl.controller.",
                    activeSegment.metrics.controllerNumericEvidence(),
                    activeSegment.metrics.evidence());
        }
        renderHistoryRows();

        double target = activeSegment == null
                ? (plant != null && plant.hasCommandTarget() ? plant.commandTarget().get() : 0.0)
                : activeSegment.candidate.selectedTarget();
        ctx.telemetry.addData(GRAPH_SEGMENT_ID,
                activeSegment == null ? 0.0 : activeSegment.id);
        ControlTuningTelemetry.addOptionalNumber(ctx.telemetry, GRAPH_TARGET, target);
        ControlTuningTelemetry.addOptionalNumber(ctx.telemetry, GRAPH_MEASUREMENT,
                measurementAvailable ? measurement : null);
        ControlTuningTelemetry.addOptionalNumber(ctx.telemetry, GRAPH_ERROR,
                measurementAvailable && Double.isFinite(target) ? target - measurement : null);
        ControlTuningTelemetry.addOptionalNumber(ctx.telemetry, METRIC_FIRST_AT_TARGET_SEC,
                metrics.get("firstAtTargetSec"));
        ControlTuningTelemetry.addOptionalNumber(ctx.telemetry, METRIC_SETTLING_SEC,
                metrics.get("settlingSec"));
        ControlTuningTelemetry.addOptionalNumber(ctx.telemetry, METRIC_OVERSHOOT,
                metrics.get("overshoot"));
        ControlTuningTelemetry.addOptionalNumber(ctx.telemetry, METRIC_PEAK_RATE,
                metrics.get("peakMeasuredRate"));
        ControlTuningTelemetry.addOptionalNumber(ctx.telemetry, METRIC_HOLD_ERROR,
                metrics.get("holdError"));
        ControlTuningTelemetry.addOptionalNumber(ctx.telemetry, METRIC_HOLD_DRIFT,
                metrics.get("holdDrift"));
        ControlTuningTelemetry.addOptionalNumber(ctx.telemetry, METRIC_OUTPUT_LIMITED_SEC,
                metrics.get("outputLimitedDurationSec"));
        ControlTuningTelemetry.addOptionalNumber(
                ctx.telemetry, METRIC_DISTURBANCE_DISPLACEMENT,
                metrics.get("disturbanceDisplacement"));
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
                            + " / endpoint=" + record.experimentRequest.get(FIELD_SELECTED_ENDPOINT)
                            + " / target=" + record.experimentRequest.get(FIELD_SELECTED_TARGET)
                            + " / settle=" + record.metrics.get("settlingSec"));
            ControlTuningTelemetry.addEvidence(
                    ctx.telemetry,
                    "tune.positionControl.history." + record.segmentId + ".controller.",
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
                : FtcVelocityControlPanelsTester.validateDraftSchema(
                        browserDraft, controllerSchema, positionFields());
        if (schemaError != null) {
            return "INVALID: " + schemaError;
        }
        Candidate browser = readCandidate(browserDraft, nextEndpointA);
        if (lastAcceptedCandidate != null && browser.sameFullRequest(lastAcceptedCandidate)) {
            return "UNCHANGED REPEAT — A STILL AUTHORIZES ONE LEG";
        }
        if (lastAcceptedCandidate != null
                && browser.controller.sameValues(lastAcceptedCandidate.controller)
                && browser.sameSelectedTarget(lastAcceptedCandidate)) {
            return "ANCILLARY CHANGE — A STARTS A REPEAT WITHOUT REWRITING TARGET";
        }
        return "READY / NOT ACCEPTED";
    }

    private static List<String> positionFields() {
        List<String> fields = new ArrayList<String>(3);
        fields.add(FIELD_ENDPOINT_A);
        fields.add(FIELD_ENDPOINT_B);
        fields.add(FIELD_AUTO_HOLD_SEC);
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

    private static void requirePositionPlant(PositionPlant plant) {
        if (plant == null) {
            throw new IllegalStateException("position plantFactory returned null");
        }
        if (!plant.hasCommandTarget()) {
            throw new IllegalArgumentException(
                    "Position control tuning requires one exact graph-owned command target");
        }
        if (!plant.hasFeedback()) {
            throw new IllegalArgumentException("Position control tuning requires feedback");
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
        return "travel=" + metrics.get("travel")
                + ", firstAtTarget=" + metrics.get("firstAtTargetSec")
                + ", settle=" + metrics.get("settlingSec")
                + ", overshoot=" + metrics.get("overshoot")
                + ", peakRate=" + metrics.get("peakMeasuredRate")
                + ", holdDrift=" + metrics.get("holdDrift")
                + ", recovery=" + metrics.get("disturbanceRecoverySec");
    }

    private PlantSnapshot capturePlantSnapshot() {
        try {
            return plant == null ? null : plant.snapshot();
        } catch (RuntimeException ignored) {
            return null;
        }
    }

    /** Preserve one boundary attempt even when capture fails, so cleanup cannot recapture later. */
    private void captureEndingSnapshotOnce() {
        if (pendingEndingSnapshotAttempted) {
            return;
        }
        pendingEndingSnapshotAttempted = true;
        pendingEndingSnapshot = capturePlantSnapshot();
    }

    private static void addFinalPlantFacts(
            Map<String, Double> destination,
            PlantSnapshot snapshot) {
        destination.put(
                "finalRequestedTarget",
                snapshot == null ? Double.NaN : snapshot.requestedTarget());
        destination.put(
                "finalAppliedTarget",
                snapshot == null ? Double.NaN : snapshot.appliedTarget());
        destination.put(
                "finalMeasurement",
                snapshot == null ? Double.NaN : snapshot.measurement());
    }

    private static String describe(RuntimeException failure) {
        String message = failure.getMessage();
        return failure.getClass().getSimpleName()
                + (message == null || message.trim().isEmpty() ? "" : ": " + message);
    }

    private static void cancelIfActive(Task task) {
        if (task != null && !task.isComplete()) {
            task.cancel();
        }
    }

    private static void stopIfConstructed(Plant plant) {
        if (plant != null) {
            plant.stop();
        }
    }
}
