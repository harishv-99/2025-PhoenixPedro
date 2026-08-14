package edu.ftcphoenix.fw.integrations.panels;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BiFunction;
import java.util.function.Function;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTargetStatus;
import edu.ftcphoenix.fw.actuation.Plants;
import edu.ftcphoenix.fw.actuation.PositionPlant;
import edu.ftcphoenix.fw.actuation.ScalarRange;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.source.ScalarTarget;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.Task;
import edu.ftcphoenix.fw.task.TaskOutcome;
import edu.ftcphoenix.fw.tools.tester.TeleOpTester;
import edu.ftcphoenix.fw.tools.tester.TesterContext;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Workflow-level regression coverage with deterministic controller-neutral fakes. */
public final class FtcControlPanelsTesterTest {

    @Test
    public void publicFacadeHasOnlyTheApprovedControlWorkflowsAndDraftTransport()
            throws Exception {
        Method velocity = FtcPanelsTuners.class.getDeclaredMethod(
                "velocityControl", String.class, ScalarRange.class, Function.class);
        Method position = FtcPanelsTuners.class.getDeclaredMethod(
                "positionControl", String.class, ScalarRange.class, Function.class);
        Method referencedPosition = FtcPanelsTuners.class.getDeclaredMethod(
                "positionControl", String.class, ScalarRange.class,
                Function.class, BiFunction.class);
        assertTrue(Modifier.isPublic(velocity.getModifiers()));
        assertTrue(Modifier.isStatic(velocity.getModifiers()));
        assertTrue(Modifier.isPublic(position.getModifiers()));
        assertTrue(Modifier.isPublic(referencedPosition.getModifiers()));

        for (Method method : FtcPanelsTuners.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                assertTrue("unexpected public facade method " + method,
                        method.equals(velocity)
                                || method.equals(position)
                                || method.equals(referencedPosition));
            }
            assertFalse("removed velocityPidf alias must not remain",
                    method.getName().equals("velocityPidf"));
        }

        Field[] fields = FtcPanelsTuners.class.getFields();
        assertEquals(1, fields.length);
        assertEquals("draft", fields[0].getName());
        assertEquals(Map.class, fields[0].getType());
        assertTrue(Modifier.isStatic(fields[0].getModifiers()));
        assertTrue(fields[0].getGenericType().getTypeName().contains("java.lang.String"));
        assertTrue(fields[0].getGenericType().getTypeName().contains("java.lang.Double"));
    }

    @Test
    public void facadeAcceptsSignedVelocityRangeAndDefersAcquisition() {
        final boolean[] acquired = {false};
        TeleOpTester velocity = FtcPanelsTuners.velocityControl(
                "Signed velocity",
                ScalarRange.bounded(-100.0, 80.0),
                ignored -> {
                    acquired[0] = true;
                    return null;
                });
        TeleOpTester position = FtcPanelsTuners.positionControl(
                "Position",
                ScalarRange.bounded(-20.0, 20.0),
                ignored -> {
                    acquired[0] = true;
                    return null;
                });

        assertEquals("Signed velocity", velocity.name());
        assertEquals("Position", position.name());
        assertFalse(acquired[0]);

        try {
            FtcPanelsTuners.velocityControl(
                    "bad", ScalarRange.unbounded(), ignored -> null);
            fail("expected bounded-range rejection");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("finite bounded"));
        }
    }

    @Test
    public void activeDraftReplacementPreservesOrderAndDropsStaleTopologyKeys() {
        AtomicInteger refreshes = new AtomicInteger();
        ControlTuningModel.Parameters first = parameters("feedback.kP", 1.0);
        LinkedHashMap<String, Double> velocity = new LinkedHashMap<String, Double>();
        velocity.put("targetVelocity", 20.0);
        velocity.put("autoStopAfterSec", 2.0);
        FtcPanelsTuners.seedActiveDraft(first, velocity, refreshes::incrementAndGet);
        assertEquals(Arrays.asList(
                        "controller.feedback.kP",
                        "experiment.targetVelocity",
                        "experiment.autoStopAfterSec"),
                new ArrayList<String>(FtcPanelsTuners.readActiveDraft().keySet()));

        LinkedHashMap<String, Double> positionParameters = new LinkedHashMap<String, Double>();
        positionParameters.put("outerPosition.kP", 3.0);
        positionParameters.put("innerVelocity.kP", 4.0);
        LinkedHashMap<String, Double> position = new LinkedHashMap<String, Double>();
        position.put("endpointA", 0.0);
        position.put("endpointB", 10.0);
        position.put("autoHoldAfterSec", 3.0);
        FtcPanelsTuners.seedActiveDraft(
                new ControlTuningModel.Parameters(positionParameters), position,
                refreshes::incrementAndGet);

        Map<String, Double> active = FtcPanelsTuners.readActiveDraft();
        assertEquals(Arrays.asList(
                        "controller.outerPosition.kP",
                        "controller.innerVelocity.kP",
                        "experiment.endpointA",
                        "experiment.endpointB",
                        "experiment.autoHoldAfterSec"),
                new ArrayList<String>(active.keySet()));
        assertFalse(active.containsKey("controller.feedback.kP"));
        assertFalse(active.containsKey("experiment.targetVelocity"));
        assertEquals(2, refreshes.get());
    }

    @Test
    public void draftCaptureRequiresExactOrderedSchemaAndFiniteCompleteValues() {
        ControlTuningModel.Parameters schema = parameters("feedback.kP", 1.0);
        List<String> experiment = Arrays.asList("targetVelocity", "autoStopAfterSec");
        LinkedHashMap<String, Double> valid = new LinkedHashMap<String, Double>();
        valid.put("controller.feedback.kP", 1.0);
        valid.put("experiment.targetVelocity", 20.0);
        valid.put("experiment.autoStopAfterSec", 2.0);
        assertNull(FtcVelocityControlPanelsTester.validateDraftSchema(
                valid, schema, experiment));

        LinkedHashMap<String, Double> extra = new LinkedHashMap<String, Double>(valid);
        extra.put("controller.stale.kD", 0.0);
        assertTrue(FtcVelocityControlPanelsTester.validateDraftSchema(
                extra, schema, experiment).contains("schema changed"));

        LinkedHashMap<String, Double> reordered = new LinkedHashMap<String, Double>();
        reordered.put("experiment.targetVelocity", 20.0);
        reordered.put("controller.feedback.kP", 1.0);
        reordered.put("experiment.autoStopAfterSec", 2.0);
        assertTrue(FtcVelocityControlPanelsTester.validateDraftSchema(
                reordered, schema, experiment).contains("schema changed"));

        LinkedHashMap<String, Double> nonfinite = new LinkedHashMap<String, Double>(valid);
        nonfinite.put("experiment.targetVelocity", Double.NaN);
        assertTrue(FtcVelocityControlPanelsTester.validateDraftSchema(
                nonfinite, schema, experiment).contains("must be finite"));
    }

    @Test
    public void firstVelocityAAndPostBRepeatWaitForTruthfulZero() {
        VelocityFixture fixture = new VelocityFixture(
                ControlTuningModel.ReconfigurationPolicy.HOT);
        fixture.plant.measurement = 40.0;
        fixture.start();
        fixture.draft.values = velocityDraft(1.0, 50.0, 0.0);

        fixture.pressA(0.10);
        assertEquals("first A must remain at zero", 0.0, fixture.plant.command.get(), 0.0);
        assertTrue(fixture.telemetry.value("Status").contains("COLD START WAIT"));
        fixture.plant.measurement = 0.0;
        fixture.tick(0.35, false, false, false);
        assertEquals(50.0, fixture.plant.command.get(), 0.0);

        fixture.tick(0.40, false, false, false);
        fixture.plant.measurement = 50.0;
        fixture.pressB(0.45);
        assertEquals(0.0, fixture.plant.command.get(), 0.0);
        fixture.pressA(0.55);
        assertEquals("post-B repeat must still wait at zero",
                0.0, fixture.plant.command.get(), 0.0);
        fixture.plant.measurement = 0.0;
        fixture.tick(0.80, false, false, false);
        assertEquals(50.0, fixture.plant.command.get(), 0.0);
    }

    @Test
    public void tinyPositiveVelocityTimeoutCannotHideTheFirstPlantRealization() {
        VelocityFixture fixture = new VelocityFixture(
                ControlTuningModel.ReconfigurationPolicy.HOT);
        fixture.plant.measurement = 0.0;
        fixture.start();
        fixture.draft.values = velocityDraft(1.0, 25.0, 0.01);

        fixture.pressA(0.10);
        assertEquals("cold acceptance stages after this cycle's Plant update",
                0.0, fixture.plant.requested, 0.0);
        fixture.tick(0.50, false, false, false);
        assertEquals("the positive-duration request must reach one Plant update",
                25.0, fixture.plant.requested, 0.0);
        assertTrue(fixture.tester.historyRecords().isEmpty());

        fixture.tick(0.52, false, false, false);
        assertEquals(0.0, fixture.plant.requested, 0.0);
        assertEquals("AUTO_STOP_TIMEOUT",
                fixture.tester.historyRecords().get(0).terminationReason);
    }

    @Test
    public void velocityDraftStartsAtNearestAllowedZeroAndPlantMustProvideActiveZero() {
        List<String> events = new ArrayList<String>();
        FakePlant plant = new FakePlant(events, ScalarRange.bounded(-100.0, 100.0));
        FakeSession session = new FakeSession(
                plant.range, ControlTuningModel.ReconfigurationPolicy.HOT, events);
        VelocityDraft draft = new VelocityDraft();
        FtcVelocityControlPanelsTester tester = new FtcVelocityControlPanelsTester(
                "Positive velocity",
                ScalarRange.bounded(10.0, 80.0),
                plant,
                session,
                draft);
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        tester.init(context(new Gamepad(), new RecordingTelemetry(), clock));
        assertEquals(10.0, draft.values.get("experiment.targetVelocity"), 0.0);

        FakePlant noZeroPlant = new FakePlant(
                new ArrayList<String>(), ScalarRange.bounded(10.0, 100.0));
        FakeSession noZeroSession = new FakeSession(
                noZeroPlant.range,
                ControlTuningModel.ReconfigurationPolicy.HOT,
                noZeroPlant.events);
        FtcVelocityControlPanelsTester invalid = new FtcVelocityControlPanelsTester(
                "No zero",
                ScalarRange.bounded(10.0, 80.0),
                noZeroPlant,
                noZeroSession,
                new VelocityDraft());
        try {
            invalid.init(context(new Gamepad(), new RecordingTelemetry(), clock));
            fail("expected Plant zero-range rejection");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage(),
                    expected.getMessage().contains("contains the required zero request"));
        }
        assertEquals(1, noZeroPlant.stopCount);
        assertEquals(1, noZeroSession.restoreCount);
    }

    @Test
    public void velocityTargetControllerAndRepeatTransitionsChangeOnlyTheirOwners() {
        VelocityFixture fixture = new VelocityFixture(
                ControlTuningModel.ReconfigurationPolicy.HOT);
        fixture.startAtZeroWith(1.0, 20.0);
        fixture.session.applyCount = 0;
        fixture.plant.command.setCount = 0;

        fixture.draft.values = velocityDraft(1.0, 30.0, 0.0);
        fixture.pressA(0.50);
        assertEquals(0, fixture.session.applyCount);
        assertEquals(1, fixture.plant.command.setCount);

        fixture.plant.command.setCount = 0;
        fixture.draft.values = velocityDraft(2.0, 30.0, 0.0);
        fixture.pressA(0.80);
        assertEquals(1, fixture.session.applyCount);
        assertEquals("controller-only A retains the running target",
                0, fixture.plant.command.setCount);

        fixture.session.applyCount = 0;
        fixture.pressA(1.10);
        assertEquals(0, fixture.session.applyCount);
        assertEquals(0, fixture.plant.command.setCount);

        fixture.session.applyCount = 0;
        fixture.plant.command.setCount = 0;
        fixture.draft.values = velocityDraft(2.0, 30.0, 4.0);
        fixture.pressA(1.35);
        assertEquals("timer-only repeat must not apply the controller",
                0, fixture.session.applyCount);
        assertEquals("timer-only repeat must not rewrite the running target",
                0, fixture.plant.command.setCount);
        fixture.pressB(1.65);

        List<ControlExperimentHistory.Record> records = fixture.tester.historyRecords();
        assertEquals(5, records.size());
        assertEquals(ControlExperimentHistory.Transition.TARGET_CHANGE,
                records.get(1).transition);
        assertEquals("the prior segment must snapshot its own final request",
                20.0, records.get(0).metrics.get("finalRequestedTarget"), 0.0);
        assertEquals(30.0, records.get(1).metrics.get("finalRequestedTarget"), 0.0);
        assertEquals(ControlExperimentHistory.Transition.CONTROLLER_CHANGE,
                records.get(2).transition);
        assertEquals(ControlExperimentHistory.Transition.REPEAT,
                records.get(3).transition);
        assertEquals(ControlExperimentHistory.Transition.REPEAT,
                records.get(4).transition);
        assertEquals(2.0,
                records.get(3).controllerCandidate.value("kP"), 0.0);
        assertEquals(30.0,
                records.get(3).experimentRequest.get("targetVelocity"), 0.0);
        assertEquals(4.0,
                records.get(4).experimentRequest.get("autoStopAfterSec"), 0.0);
    }

    @Test
    public void velocityApplyFailureAfterMutationIsTerminalNotASafeReject() {
        VelocityFixture fixture = new VelocityFixture(
                ControlTuningModel.ReconfigurationPolicy.HOT);
        fixture.startAtZeroWith(1.0, 20.0);
        fixture.session.applyFailureAfterMutation =
                new IllegalArgumentException("setter failed after first mutation");
        fixture.draft.values = velocityDraft(2.0, 20.0, 0.0);

        try {
            fixture.pressA(0.50);
            fail("expected terminal apply failure");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("state may be uncertain"));
        }

        assertEquals(1, fixture.plant.stopCount);
        assertEquals(1, fixture.session.restoreCount);
        assertEquals(1, fixture.session.applyCount);
        assertEquals(1, fixture.tester.historyRecords().size());
        assertEquals(20.0, fixture.tester.historyRecords().get(0)
                .metrics.get("finalRequestedTarget"), 0.0);
        assertEquals("TERMINAL_FAILURE",
                fixture.tester.historyRecords().get(0).terminationReason);
    }

    @Test
    public void rejectedTypedCandidateCannotEndOrZeroAnActiveGroupedExperiment() {
        VelocityFixture fixture = new VelocityFixture(
                ControlTuningModel.ReconfigurationPolicy.ZERO_AND_SETTLE);
        fixture.startAtZeroWith(1.0, 20.0);
        fixture.session.maximumCandidate = 10.0;
        fixture.session.applyCount = 0;
        fixture.plant.command.setCount = 0;
        fixture.draft.values = velocityDraft(20.0, 20.0, 0.0);

        fixture.pressA(0.50);

        assertTrue(fixture.telemetry.value("Status").contains("REJECTED"));
        assertEquals(20.0, fixture.plant.command.get(), 0.0);
        assertEquals(0, fixture.plant.command.setCount);
        assertEquals(0, fixture.session.applyCount);
        assertTrue(fixture.tester.historyRecords().isEmpty());
    }

    @Test
    public void groupedVelocityControllerChangeWaitsForPerMemberZeroEvidence() {
        VelocityFixture fixture = new VelocityFixture(
                ControlTuningModel.ReconfigurationPolicy.ZERO_AND_SETTLE);
        fixture.startAtZeroWith(1.0, 20.0);
        fixture.session.applyCount = 0;
        fixture.plant.measurement = 20.0;
        fixture.draft.values = velocityDraft(2.0, 20.0, 0.0);

        fixture.pressA(0.50);
        assertEquals(0.0, fixture.plant.command.get(), 0.0);
        assertEquals(0, fixture.session.applyCount);
        fixture.plant.measurement = 0.0;
        fixture.session.ready = false;
        fixture.tick(0.80, false, false, false);
        assertEquals(0, fixture.session.applyCount);
        fixture.session.ready = true;
        fixture.tick(0.90, false, false, false);
        assertEquals(1, fixture.session.applyCount);
        assertEquals(20.0, fixture.plant.command.get(), 0.0);
    }

    @Test
    public void groupedMemberEvidenceRemainsVisibleWhenAggregateFeedbackIsNonfinite() {
        VelocityFixture fixture = new VelocityFixture(
                ControlTuningModel.ReconfigurationPolicy.ZERO_AND_SETTLE);
        fixture.startAtZeroWith(1.0, 20.0);
        LinkedHashMap<String, Double> numeric = new LinkedHashMap<String, Double>();
        numeric.put("member.1.nativeCommandedTarget", 20.0);
        numeric.put("member.1.nativeMeasurement", 21.0);
        numeric.put("member.1.nativeError", -1.0);
        numeric.put("member.2.nativeCommandedTarget", -20.0);
        numeric.put("member.2.nativeMeasurement", Double.NaN);
        numeric.put("member.2.nativeError", Double.NaN);
        LinkedHashMap<String, String> text = new LinkedHashMap<String, String>();
        text.put("member.1.motorName", "left");
        text.put("member.2.motorName", "right");
        fixture.session.publishedEvidence = new ControlTuningModel.Evidence(
                numeric, text, false, false);
        fixture.plant.measurement = Double.NaN;

        fixture.tick(0.50, false, false, false);

        assertEquals("left", fixture.telemetry.value(
                "tune.velocityControl.controller.member.1.motorName"));
        assertEquals("20.0", fixture.telemetry.value(
                "tune.velocityControl.controller.member.1.nativeCommandedTarget"));
        assertEquals("21.0", fixture.telemetry.value(
                "tune.velocityControl.controller.member.1.nativeMeasurement"));
        assertEquals("-1.0", fixture.telemetry.value(
                "tune.velocityControl.controller.member.1.nativeError"));
        assertEquals("right", fixture.telemetry.value(
                "tune.velocityControl.controller.member.2.motorName"));
        assertEquals("0.0", fixture.telemetry.value(
                "tune.velocityControl.controller.member.2.nativeMeasurement.available"));

        fixture.pressB(0.60);
        ControlExperimentHistory.Record record = fixture.tester.historyRecords().get(0);
        assertTrue(Double.isNaN(record.metrics.get(
                "controller.member.2.nativeMeasurement")));
        assertEquals("right", record.evidence.get("member.2.motorName"));
        assertEquals("left", fixture.telemetry.value(
                "tune.velocityControl.history.1.controller.member.1.motorName"));
        assertEquals("-1.0", fixture.telemetry.value(
                "tune.velocityControl.history.1.controller.member.1.nativeError"));
    }

    @Test
    public void metricsSeparateAtTargetSettlingFromPostSettledDisturbanceRecovery() {
        ControlResponseMetrics.Velocity metrics =
                new ControlResponseMetrics.Velocity(10.0, 0.0, 0.0);
        metrics.update(0.10, 9.8, true, ControlTuningModel.Evidence.empty());
        metrics.update(0.31, 10.0, true, ControlTuningModel.Evidence.empty());
        metrics.update(0.40, 7.0, false, ControlTuningModel.Evidence.empty());
        metrics.update(0.50, 9.9, true, ControlTuningModel.Evidence.empty());
        metrics.update(0.71, 10.0, true, ControlTuningModel.Evidence.empty());

        Map<String, Double> snapshot = metrics.snapshot();
        assertEquals(0.10, snapshot.get("firstAtTargetSec"), 1e-9);
        assertEquals(0.30, snapshot.get("settlingSec"), 1e-9);
        assertEquals(3.0, snapshot.get("disturbancePeakAbsError"), 0.0);
        assertEquals(0.30, snapshot.get("disturbanceRecoverySec"), 1e-9);
        assertEquals(3.0, snapshot.get("directionalDroop"), 0.0);
    }

    @Test
    public void metricsDoNotCountStartupAsDroopOrPreSegmentTimeAsOutputLimiting() {
        ControlTuningModel.Evidence limited = new ControlTuningModel.Evidence(
                Collections.<String, Double>emptyMap(),
                Collections.<String, String>emptyMap(),
                true,
                true);
        ControlResponseMetrics.Velocity metrics =
                new ControlResponseMetrics.Velocity(10.0, 5.0, 0.0);
        metrics.update(5.0, 0.0, false, limited);
        metrics.update(5.1, 9.8, true, limited);
        metrics.update(5.31, 10.0, true, limited);

        Map<String, Double> snapshot = metrics.snapshot();
        assertEquals(0.0, snapshot.get("directionalDroop"), 0.0);
        assertEquals(0.31, snapshot.get("outputLimitedDurationSec"), 1e-9);
    }

    @Test
    public void outputLimitedDurationBelongsToThePreviouslyHeldOutputSample() {
        ControlTuningModel.Evidence limited = new ControlTuningModel.Evidence(
                Collections.<String, Double>emptyMap(),
                Collections.<String, String>emptyMap(), true, true);
        ControlTuningModel.Evidence unlimited = new ControlTuningModel.Evidence(
                Collections.<String, Double>emptyMap(),
                Collections.<String, String>emptyMap(), true, false);
        ControlResponseMetrics.Velocity metrics =
                new ControlResponseMetrics.Velocity(10.0, 0.0, 10.0);

        metrics.update(0.0, 10.0, true, limited);
        metrics.update(1.0, 10.0, true, unlimited);
        assertEquals("limited-to-unlimited interval belongs to the prior limited output",
                1.0, metrics.snapshot().get("outputLimitedDurationSec"), 0.0);
        metrics.update(2.0, 10.0, true, limited);
        assertEquals("unlimited-to-limited interval must not be back-charged",
                1.0, metrics.snapshot().get("outputLimitedDurationSec"), 0.0);
        metrics.update(3.0, 10.0, true, limited);
        assertEquals(2.0, metrics.snapshot().get("outputLimitedDurationSec"), 0.0);

        ControlResponseMetrics.Velocity endingLimited =
                new ControlResponseMetrics.Velocity(10.0, 5.0, 10.0);
        endingLimited.update(5.0, 10.0, true, limited);
        endingLimited.finish(5.75);
        assertEquals("segment end owns the final held-limited interval",
                0.75, endingLimited.snapshot().get("outputLimitedDurationSec"), 0.0);
        endingLimited.finish(6.0);
        assertEquals("finish must be idempotent",
                0.75, endingLimited.snapshot().get("outputLimitedDurationSec"), 0.0);
    }

    @Test
    public void zeroVelocityTargetLeavesDirectionalMetricsUnavailable() {
        ControlResponseMetrics.Velocity metrics =
                new ControlResponseMetrics.Velocity(0.0, 0.0, 5.0);
        metrics.update(0.10, 1.0, false, ControlTuningModel.Evidence.empty());

        assertTrue(Double.isNaN(metrics.snapshot().get("directionalDroop")));
        assertTrue(Double.isNaN(metrics.snapshot().get("overshoot")));
    }

    @Test
    public void positionDisturbanceDisplacementUsesRetainedSettledPosition() {
        ControlResponseMetrics.Position metrics =
                new ControlResponseMetrics.Position(10.0, 0.0, 0.0);
        metrics.update(0.10, 9.5, true, ControlTuningModel.Evidence.empty());
        metrics.update(0.31, 9.6, true, ControlTuningModel.Evidence.empty());
        metrics.update(0.40, 8.0, false, ControlTuningModel.Evidence.empty());

        Map<String, Double> snapshot = metrics.snapshot();
        assertEquals(2.0, snapshot.get("disturbancePeakAbsError"), 0.0);
        assertEquals(1.6, snapshot.get("disturbanceDisplacement"), 1e-9);
    }

    @Test
    public void positionDisturbanceDisplacementResetsForEachRecoveredEvent() {
        ControlResponseMetrics.Position metrics =
                new ControlResponseMetrics.Position(10.0, 0.0, 0.0);
        metrics.update(0.10, 10.0, true, ControlTuningModel.Evidence.empty());
        metrics.update(0.31, 10.0, true, ControlTuningModel.Evidence.empty());
        metrics.update(0.40, 7.0, false, ControlTuningModel.Evidence.empty());
        metrics.update(0.50, 10.0, true, ControlTuningModel.Evidence.empty());
        metrics.update(0.71, 10.0, true, ControlTuningModel.Evidence.empty());
        metrics.update(0.80, 9.0, false, ControlTuningModel.Evidence.empty());

        Map<String, Double> snapshot = metrics.snapshot();
        assertEquals(1.0, snapshot.get("disturbancePeakAbsError"), 0.0);
        assertEquals(1.0, snapshot.get("disturbanceDisplacement"), 0.0);
    }

    @Test
    public void positionHoldErrorUsesTargetMinusMeasurementSign() {
        ControlResponseMetrics.Position metrics =
                new ControlResponseMetrics.Position(10.0, 0.0, 0.0);
        metrics.update(0.10, 10.0, true, ControlTuningModel.Evidence.empty());
        metrics.update(0.31, 10.0, true, ControlTuningModel.Evidence.empty());
        metrics.update(0.40, 11.0, true, ControlTuningModel.Evidence.empty());

        assertEquals(-1.0, metrics.snapshot().get("holdError"), 0.0);
    }

    @Test
    public void optionalGraphZeroRequiresItsCompanionAvailabilityFact() {
        RecordingTelemetry telemetry = new RecordingTelemetry();
        ControlTuningTelemetry.addOptionalNumber(
                telemetry.proxy, "firstAtTargetSec", Double.NaN);
        assertEquals("0.0", telemetry.value("firstAtTargetSec"));
        assertEquals("0.0", telemetry.value("firstAtTargetSec.available"));

        ControlTuningTelemetry.addOptionalNumber(
                telemetry.proxy, "firstAtTargetSec", 0.0);
        assertEquals("0.0", telemetry.value("firstAtTargetSec"));
        assertEquals("1.0", telemetry.value("firstAtTargetSec.available"));
    }

    @Test
    public void positionAlternatesConcreteLegsAndControllerChangeIsBoth() {
        PositionFixture fixture = new PositionFixture(true, null);
        fixture.startHeldAt(50.0);
        fixture.draft.values = positionDraft(1.0, 0.0, 100.0, 0.0);
        fixture.pressA(0.20);
        assertEquals(0.0, fixture.plant.command.get(), 0.0);

        fixture.plant.measurement = 0.0;
        fixture.tick(0.50, false, false, false);
        fixture.tick(0.72, false, false, false);
        fixture.draft.values = positionDraft(2.0, 0.0, 100.0, 0.0);
        fixture.pressA(0.80);
        assertEquals(100.0, fixture.plant.command.get(), 0.0);
        assertEquals(1, fixture.session.applyCount);
        fixture.pressB(1.05);

        List<ControlExperimentHistory.Record> records = fixture.tester.historyRecords();
        assertEquals(2, records.size());
        assertEquals(ControlExperimentHistory.Transition.TARGET_CHANGE,
                records.get(0).transition);
        assertEquals(ControlExperimentHistory.Transition.CONTROLLER_AND_TARGET_CHANGE,
                records.get(1).transition);
        assertEquals(0.0, records.get(0).metrics.get("finalRequestedTarget"), 0.0);
        assertEquals(100.0, records.get(1).experimentRequest.get("selectedTarget"), 0.0);
        assertEquals(100.0, records.get(1).metrics.get("travel"), 0.0);
    }

    @Test
    public void positionApplyFailureAfterMutationIsTerminalNotASafeReject() {
        PositionFixture fixture = new PositionFixture(true, null);
        fixture.startHeldAt(50.0);
        fixture.draft.values = positionDraft(1.0, 0.0, 100.0, 0.0);
        fixture.pressA(0.20);
        fixture.plant.measurement = 0.0;
        fixture.tick(0.50, false, false, false);
        fixture.tick(0.72, false, false, false);
        fixture.session.applyFailureAfterMutation =
                new IllegalArgumentException("position setter failed after mutation");
        fixture.draft.values = positionDraft(2.0, 0.0, 100.0, 0.0);

        try {
            fixture.pressA(0.80);
            fail("expected terminal apply failure");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("state may be uncertain"));
        }

        assertEquals(1, fixture.plant.stopCount);
        assertEquals(1, fixture.session.restoreCount);
        assertEquals(1, fixture.session.applyCount);
        assertEquals(1, fixture.tester.historyRecords().size());
        assertEquals(0.0, fixture.tester.historyRecords().get(0)
                .metrics.get("finalRequestedTarget"), 0.0);
        assertEquals("TERMINAL_FAILURE",
                fixture.tester.historyRecords().get(0).terminationReason);
    }

    @Test
    public void positionControllerEvidenceSurvivesNonfiniteAggregateMeasurement() {
        PositionFixture fixture = new PositionFixture(true, null);
        fixture.startHeldAt(50.0);
        fixture.draft.values = positionDraft(1.0, 0.0, 100.0, 0.0);
        fixture.pressA(0.20);
        LinkedHashMap<String, Double> numeric = new LinkedHashMap<String, Double>();
        numeric.put("member.1.nativeMeasurement", 51.0);
        LinkedHashMap<String, String> text = new LinkedHashMap<String, String>();
        text.put("member.1.motorName", "arm");
        fixture.session.publishedEvidence = new ControlTuningModel.Evidence(
                numeric, text, false, false);
        fixture.plant.measurement = Double.NaN;

        fixture.tick(0.50, false, false, false);

        assertEquals("arm", fixture.telemetry.value(
                "tune.positionControl.controller.member.1.motorName"));
        assertEquals("51.0", fixture.telemetry.value(
                "tune.positionControl.controller.member.1.nativeMeasurement"));
    }

    @Test
    public void zeroTravelPositionLeavesDirectionAndOvershootUninvented() {
        ControlResponseMetrics.Position metrics =
                new ControlResponseMetrics.Position(10.0, 0.0, 10.0);
        metrics.update(0.10, 10.0, true, ControlTuningModel.Evidence.empty());

        assertEquals(0.0, metrics.snapshot().get("direction"), 0.0);
        assertTrue(Double.isNaN(metrics.snapshot().get("overshoot")));
    }

    @Test
    public void positionAncillaryEditsRepeatTheConcreteTargetWithoutRewritingIt() {
        PositionFixture fixture = new PositionFixture(true, null);
        fixture.startHeldAt(50.0);
        fixture.draft.values = positionDraft(1.0, 0.0, 100.0, 0.0);
        fixture.pressA(0.20);
        fixture.plant.measurement = 0.0;
        fixture.tick(0.50, false, false, false);
        fixture.tick(0.72, false, false, false);

        fixture.session.applyCount = 0;
        fixture.plant.command.setCount = 0;
        fixture.draft.values = positionDraft(1.0, 20.0, 0.0, 4.0);
        fixture.pressA(0.80);
        assertEquals(0, fixture.session.applyCount);
        assertEquals(0, fixture.plant.command.setCount);
        fixture.pressB(1.10);

        List<ControlExperimentHistory.Record> records = fixture.tester.historyRecords();
        assertEquals(2, records.size());
        assertEquals(ControlExperimentHistory.Transition.REPEAT,
                records.get(1).transition);
        assertEquals(20.0, records.get(1).experimentRequest.get("endpointA"), 0.0);
        assertEquals(0.0, records.get(1).experimentRequest.get("endpointB"), 0.0);
        assertEquals(4.0, records.get(1).experimentRequest.get("autoHoldAfterSec"), 0.0);
        assertEquals(2.0, records.get(1).experimentRequest.get("selectedEndpoint"), 0.0);
    }

    @Test
    public void tinyPositivePositionTimeoutStillReachesPlantBeforeRecoveryHold() {
        PositionFixture fixture = new PositionFixture(true, null);
        fixture.startHeldAt(50.0);
        fixture.draft.values = positionDraft(1.0, 0.0, 100.0, 0.01);

        fixture.pressA(0.20);
        int realizedUpdate = fixture.events.lastIndexOf("plant.update");
        assertEquals(0.0, fixture.plant.requested, 0.0);
        fixture.tick(0.60, false, false, false);

        int recovery = fixture.events.indexOf("session.recoveryHold");
        assertTrue(fixture.events.toString(), realizedUpdate >= 0);
        assertTrue(fixture.events.toString(), recovery > realizedUpdate);
        assertEquals("AUTO_HOLD_TIMEOUT",
                fixture.tester.historyRecords().get(0).terminationReason);
    }

    @Test
    public void positionRejectsSecondLegMidMotionAndBRecoversToEnvelopeBoundary() {
        PositionFixture fixture = new PositionFixture(true, null);
        fixture.startHeldAt(50.0);
        fixture.draft.values = positionDraft(1.0, 0.0, 100.0, 0.0);
        fixture.pressA(0.20);
        fixture.draft.values = positionDraft(2.0, 0.0, 100.0, 0.0);
        fixture.pressA(0.45);
        assertEquals(0, fixture.session.applyCount);
        assertEquals(0.0, fixture.plant.command.get(), 0.0);
        assertTrue(fixture.telemetry.value("Status").contains("one A authorizes one leg"));

        fixture.plant.measurement = -4.0;
        fixture.pressB(0.70);
        assertEquals(0.0, fixture.plant.command.get(), 0.0);
        assertEquals(1, fixture.session.recoveryCount);
        assertTrue(fixture.events.toString(),
                fixture.events.indexOf("session.recoveryHold")
                        < fixture.events.lastIndexOf("plant.update"));
        assertTrue(fixture.telemetry.value("Status").contains("RECOVERING TO SAFE BOUNDARY"));
    }

    @Test
    public void nonfinitePositionHoldFailsTerminallyAndRestores() {
        PositionFixture fixture = new PositionFixture(true, null);
        fixture.startHeldAt(50.0);
        fixture.plant.measurement = Double.NaN;
        fixture.tick(0.20, false, false, false);
        try {
            fixture.pressB(0.30);
            fail("expected terminal hold failure");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("state may be uncertain"));
        }
        assertEquals(1, fixture.plant.stopCount);
        assertEquals(1, fixture.session.restoreCount);
    }

    @Test
    public void referenceBUsesFreshTaskAndSuccessfulTaskPrecedesHoldAndPlantUpdate() {
        List<String> events = new ArrayList<String>();
        AtomicInteger taskCount = new AtomicInteger();
        AtomicInteger cancelCount = new AtomicInteger();
        final FakePositionPlant[] plantRef = new FakePositionPlant[1];
        BiFunction<HardwareMap, PositionPlant, Task> factory = (hardwareMap, plant) -> {
            int id = taskCount.incrementAndGet();
            return new ReferenceTask(
                    id,
                    events,
                    cancelCount,
                    (FakePositionPlant) plant,
                    id == 1 ? 100 : 2);
        };
        PositionFixture fixture = new PositionFixture(false, factory, events);
        plantRef[0] = fixture.plant;
        fixture.start();
        fixture.tick(0.01, false, false, false);

        fixture.tick(0.10, true, false, false);
        assertEquals(events.toString(), 1, taskCount.get());
        fixture.tick(0.20, false, true, false);
        assertEquals(events.toString(), 1, cancelCount.get());
        fixture.tick(0.30, true, false, false);
        fixture.tick(0.40, false, false, false);

        assertEquals(2, taskCount.get());
        assertTrue(fixture.plant.referenced);
        assertBefore(events, "task.2.update.2", "session.prepareHold");
        assertBefore(events, "session.prepareHold", "plant.update");
        assertTrue(fixture.telemetry.value("Status").contains("HOLD PREPARED"));
    }

    @Test
    public void nonSuccessfulReferenceCannotAuthorizeHoldEvenIfItEstablishedCoordinates() {
        List<String> events = new ArrayList<String>();
        AtomicInteger taskCount = new AtomicInteger();
        final ScriptedReferenceTask[] first = new ScriptedReferenceTask[1];
        BiFunction<HardwareMap, PositionPlant, Task> factory = (hardwareMap, plant) -> {
            int id = taskCount.incrementAndGet();
            ScriptedReferenceTask task = new ScriptedReferenceTask(
                    id, events, (FakePositionPlant) plant);
            if (id == 1) {
                task.establishOnUpdate = true;
                task.updateOutcome = TaskOutcome.TIMEOUT;
                first[0] = task;
            }
            return task;
        };
        PositionFixture fixture = new PositionFixture(false, factory, events);
        fixture.start();
        fixture.tick(0.01, false, false, false);
        fixture.tick(0.10, true, false, false);
        fixture.tick(0.20, false, false, false);

        assertTrue(fixture.plant.referenced);
        assertEquals(TaskOutcome.TIMEOUT, first[0].outcome);
        assertFalse(events.contains("session.prepareHold"));
        fixture.tick(0.30, true, false, false);
        assertEquals("A must create a fresh Task after non-success", 2, taskCount.get());
        assertFalse(events.contains("session.prepareHold"));
    }

    @Test
    public void cancelledReferenceCannotAuthorizeHoldFromItsIncidentalCoordinate() {
        List<String> events = new ArrayList<String>();
        AtomicInteger taskCount = new AtomicInteger();
        BiFunction<HardwareMap, PositionPlant, Task> factory = (hardwareMap, plant) -> {
            ScriptedReferenceTask task = new ScriptedReferenceTask(
                    taskCount.incrementAndGet(), events, (FakePositionPlant) plant);
            task.establishOnUpdate = true;
            return task;
        };
        PositionFixture fixture = new PositionFixture(false, factory, events);
        fixture.start();
        fixture.tick(0.01, false, false, false);
        fixture.tick(0.10, true, false, false);
        fixture.tick(0.20, false, false, false);
        fixture.tick(0.30, false, true, false);
        assertTrue(fixture.plant.referenced);
        assertFalse(events.contains("session.prepareHold"));

        fixture.tick(0.40, false, false, false);
        fixture.tick(0.50, true, false, false);
        assertEquals("A must create a fresh Task after cancellation", 2, taskCount.get());
    }

    @Test
    public void referenceStartFailureCancelsClaimedTaskAndRunsTerminalCleanup() {
        List<String> events = new ArrayList<String>();
        final ScriptedReferenceTask[] taskRef = new ScriptedReferenceTask[1];
        BiFunction<HardwareMap, PositionPlant, Task> factory = (hardwareMap, plant) -> {
            ScriptedReferenceTask task = new ScriptedReferenceTask(
                    1, events, (FakePositionPlant) plant);
            task.throwOnStart = true;
            taskRef[0] = task;
            return task;
        };
        PositionFixture fixture = new PositionFixture(false, factory, events);
        fixture.start();
        fixture.tick(0.01, false, false, false);
        try {
            fixture.tick(0.10, true, false, false);
            fail("expected terminal start failure");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("state may be uncertain"));
        }

        assertEquals(1, taskRef[0].cancelAttempts);
        assertEquals(1, fixture.plant.stopCount);
        assertEquals(1, fixture.session.restoreCount);
        assertBefore(events, "plant.stop", "task.1.cancel.1");
    }

    @Test
    public void referenceCancelFailureIsRetriedDuringStopFirstCleanup() {
        List<String> events = new ArrayList<String>();
        final ScriptedReferenceTask[] taskRef = new ScriptedReferenceTask[1];
        BiFunction<HardwareMap, PositionPlant, Task> factory = (hardwareMap, plant) -> {
            ScriptedReferenceTask task = new ScriptedReferenceTask(
                    1, events, (FakePositionPlant) plant);
            task.cancelFailuresRemaining = 1;
            taskRef[0] = task;
            return task;
        };
        PositionFixture fixture = new PositionFixture(false, factory, events);
        fixture.start();
        fixture.tick(0.01, false, false, false);
        fixture.tick(0.10, true, false, false);
        fixture.tick(0.20, false, false, false);
        try {
            fixture.tick(0.30, false, true, false);
            fail("expected terminal cancel failure");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("state may be uncertain"));
        }

        assertEquals(2, taskRef[0].cancelAttempts);
        assertEquals(1, fixture.plant.stopCount);
        assertEquals(1, fixture.session.restoreCount);
        assertBefore(events, "plant.stop", "task.1.cancel.2");
    }

    @Test
    public void threeArgPositionWorkflowPreparesRealAssumeCurrentReferenceAtStart() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        PositionPlant plant = Plants.fromOutputs()
                .regulatedPosition(output, clock -> 123.0)
                .nonPeriodic()
                .bounded(0.0, 10.0)
                .scaleToNative(2.0)
                .assumeCurrentPositionIs(7.0)
                .positionTolerance(0.1)
                .setpointFromAppliedTarget()
                .feedbackFromPid(0.2)
                .targetFromNewCommand(1.0)
                .build();
        ControlTuningModel.Session session = ControlTuningAdapters.claimPosition(plant);
        TopologyPositionDraft draft = new TopologyPositionDraft();
        RecordingTelemetry telemetry = new RecordingTelemetry();
        Gamepad gamepad = new Gamepad();
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        FtcPositionControlPanelsTester tester = new FtcPositionControlPanelsTester(
                "Assume current",
                ScalarRange.bounded(0.0, 10.0),
                plant,
                session,
                draft,
                null);

        tester.init(context(gamepad, telemetry, clock));
        assertFalse(plant.isReferenced());
        tester.start();
        clock.update(0.01);
        tester.loop(clock.dtSec());

        assertTrue(plant.isReferenced());
        assertEquals(7.0, plant.commandTarget().get(), 0.0);
        assertEquals(1, output.writes);
        assertTrue(telemetry.value("Status").contains("HOLD PREPARED"));
    }

    @Test
    public void threeArgPositionWorkflowRejectsRealNeedsReferenceAtFirstStartLoop() {
        RecordingPowerOutput output = new RecordingPowerOutput();
        PositionPlant plant = Plants.fromOutputs()
                .regulatedPosition(output, clock -> 123.0)
                .nonPeriodic()
                .bounded(0.0, 10.0)
                .scaleToNative(2.0)
                .needsReference("home required")
                .positionTolerance(0.1)
                .setpointFromAppliedTarget()
                .feedbackFromPid(0.2)
                .targetFromNewCommand(1.0)
                .build();
        ControlTuningModel.Session session = ControlTuningAdapters.claimPosition(plant);
        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        FtcPositionControlPanelsTester tester = new FtcPositionControlPanelsTester(
                "Needs reference",
                ScalarRange.bounded(0.0, 10.0),
                plant,
                session,
                new TopologyPositionDraft(),
                null);
        tester.init(context(new Gamepad(), new RecordingTelemetry(), clock));
        tester.start();
        clock.update(0.01);
        try {
            tester.loop(clock.dtSec());
            fail("expected physical-reference rejection");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage(),
                    expected.getMessage().contains("physical reference"));
        }
        assertEquals(1, output.writes);
    }

    @Test
    public void shortSessionIdsAreHumanTranscribableAndUnique() {
        ControlExperimentHistory first = new ControlExperimentHistory();
        ControlExperimentHistory second = new ControlExperimentHistory();
        assertTrue(first.sessionId().matches("[0-9a-z]+-[0-9a-z]+"));
        assertTrue(first.sessionId().length() < 24);
        assertNotEquals(first.sessionId(), second.sessionId());
    }

    @Test
    public void historyRetainsCompleteImmutableTypedSnapshots() {
        VelocityFixture fixture = new VelocityFixture(
                ControlTuningModel.ReconfigurationPolicy.HOT);
        fixture.startAtZeroWith(1.0, 20.0);
        fixture.plant.measurement = 20.0;
        fixture.pressB(0.60);

        List<ControlExperimentHistory.Record> records = fixture.tester.historyRecords();
        assertEquals(1, records.size());
        ControlExperimentHistory.Record record = records.get(0);
        assertEquals(fixture.tester.sessionId(), record.sessionId);
        assertEquals(1L, record.segmentId);
        assertEquals("VELOCITY", record.domain);
        assertEquals(ControlExperimentHistory.Transition.TARGET_CHANGE, record.transition);
        assertEquals(1.0, record.controllerCandidate.value("kP"), 0.0);
        assertEquals(1, record.controllerReadbacks.size());
        assertEquals(20.0, record.experimentRequest.get("targetVelocity"), 0.0);
        assertTrue(record.metrics.containsKey("firstAtTargetSec"));
        assertTrue(record.evidence.containsKey("outputLimiting"));
        assertTrue(record.endedAtSec >= record.startedAtSec);
        assertEquals("B_PRESSED", record.terminationReason);

        assertUnsupported(() -> records.clear());
        assertUnsupported(() -> record.experimentRequest.put("stale", 1.0));
        assertUnsupported(() -> record.metrics.clear());
        assertUnsupported(() -> record.controllerCandidate.values().put("kP", 2.0));
        assertUnsupported(() -> record.controllerReadbacks.clear());
    }

    private static ControlTuningModel.Parameters parameters(String name, double value) {
        LinkedHashMap<String, Double> values = new LinkedHashMap<String, Double>();
        values.put(name, value);
        return new ControlTuningModel.Parameters(values);
    }

    private static Map<String, Double> velocityDraft(double p, double target, double stopSec) {
        LinkedHashMap<String, Double> values = new LinkedHashMap<String, Double>();
        values.put("controller.kP", p);
        values.put("experiment.targetVelocity", target);
        values.put("experiment.autoStopAfterSec", stopSec);
        return values;
    }

    private static Map<String, Double> positionDraft(
            double p, double a, double b, double holdSec) {
        LinkedHashMap<String, Double> values = new LinkedHashMap<String, Double>();
        values.put("controller.kP", p);
        values.put("experiment.endpointA", a);
        values.put("experiment.endpointB", b);
        values.put("experiment.autoHoldAfterSec", holdSec);
        return values;
    }

    private static void assertBefore(List<String> events, String first, String second) {
        int firstIndex = events.indexOf(first);
        assertTrue(first + " missing from " + events, firstIndex >= 0);
        int relativeSecond = events.subList(firstIndex + 1, events.size()).indexOf(second);
        int secondIndex = relativeSecond < 0 ? -1 : firstIndex + 1 + relativeSecond;
        assertTrue(second + " missing from " + events, secondIndex >= 0);
        assertTrue(first + " must precede " + second + " in " + events,
                firstIndex < secondIndex);
    }

    private static void assertUnsupported(Runnable mutation) {
        try {
            mutation.run();
            fail("expected immutable snapshot");
        } catch (UnsupportedOperationException expected) {
            // expected
        }
    }

    private static final class VelocityFixture {
        final List<String> events = new ArrayList<String>();
        final FakePlant plant = new FakePlant(events, ScalarRange.bounded(-100.0, 100.0));
        final FakeSession session;
        final VelocityDraft draft = new VelocityDraft();
        final RecordingTelemetry telemetry = new RecordingTelemetry();
        final Gamepad gamepad = new Gamepad();
        final LoopClock clock = new LoopClock();
        final FtcVelocityControlPanelsTester tester;

        VelocityFixture(ControlTuningModel.ReconfigurationPolicy policy) {
            session = new FakeSession(plant.range, policy, events);
            tester = new FtcVelocityControlPanelsTester(
                    "Velocity", ScalarRange.bounded(-80.0, 80.0), plant, session, draft);
            clock.reset(0.0);
            tester.init(context(gamepad, telemetry, clock));
        }

        void start() {
            tester.start();
            tick(0.01, false, false, false);
        }

        void startAtZeroWith(double p, double target) {
            plant.measurement = 0.0;
            start();
            draft.values = velocityDraft(p, target, 0.0);
            pressA(0.10);
            tick(0.35, false, false, false);
            tick(0.40, false, false, false);
        }

        void pressA(double timeSec) {
            tick(timeSec, true, false, false);
            tick(timeSec + 0.02, false, false, false);
            tick(timeSec + 0.14, false, false, false);
        }

        void pressB(double timeSec) {
            tick(timeSec, false, true, false);
            tick(timeSec + 0.02, false, false, false);
        }

        void tick(double timeSec, boolean a, boolean b, boolean back) {
            gamepad.a = gamepad.cross = a;
            gamepad.b = gamepad.circle = b;
            gamepad.back = gamepad.share = back;
            clock.update(timeSec);
            tester.loop(clock.dtSec());
        }
    }

    private static final class PositionFixture {
        final List<String> events;
        final FakePositionPlant plant;
        final FakeSession session;
        final PositionDraft draft = new PositionDraft();
        final RecordingTelemetry telemetry = new RecordingTelemetry();
        final Gamepad gamepad = new Gamepad();
        final LoopClock clock = new LoopClock();
        final FtcPositionControlPanelsTester tester;

        PositionFixture(boolean referenced,
                        BiFunction<HardwareMap, PositionPlant, Task> referenceFactory) {
            this(referenced, referenceFactory, new ArrayList<String>());
        }

        PositionFixture(boolean referenced,
                        BiFunction<HardwareMap, PositionPlant, Task> referenceFactory,
                        List<String> events) {
            this.events = events;
            plant = new FakePositionPlant(events, referenced);
            session = new FakeSession(ScalarRange.bounded(0.0, 100.0),
                    ControlTuningModel.ReconfigurationPolicy.HOT, events);
            session.positionPlant = plant;
            tester = new FtcPositionControlPanelsTester(
                    "Position",
                    ScalarRange.bounded(0.0, 100.0),
                    plant,
                    session,
                    draft,
                    referenceFactory);
            clock.reset(0.0);
            tester.init(context(gamepad, telemetry, clock));
        }

        void start() {
            tester.start();
        }

        void startHeldAt(double position) {
            plant.measurement = position;
            session.holdMeasurement = position;
            start();
            tick(0.01, false, false, false);
            tick(0.05, false, false, false);
        }

        void pressA(double timeSec) {
            tick(timeSec, true, false, false);
            tick(timeSec + 0.02, false, false, false);
            tick(timeSec + 0.14, false, false, false);
        }

        void pressB(double timeSec) {
            tick(timeSec, false, true, false);
            tick(timeSec + 0.02, false, false, false);
        }

        void tick(double timeSec, boolean a, boolean b, boolean back) {
            gamepad.a = gamepad.cross = a;
            gamepad.b = gamepad.circle = b;
            gamepad.back = gamepad.share = back;
            clock.update(timeSec);
            tester.loop(clock.dtSec());
        }
    }

    private static TesterContext context(
            Gamepad gamepad, RecordingTelemetry telemetry, LoopClock clock) {
        return new TesterContext(
                new HardwareMap(null, null),
                telemetry.proxy,
                gamepad,
                new Gamepad(),
                clock);
    }

    private static final class VelocityDraft
            implements FtcVelocityControlPanelsTester.DraftPort {
        Map<String, Double> values = velocityDraft(1.0, 0.0, 5.0);

        @Override public Map<String, Double> read() {
            return new LinkedHashMap<String, Double>(values);
        }

        @Override public void seed(ControlTuningModel.Parameters controller,
                                   Map<String, Double> experiment) {
            values = new LinkedHashMap<String, Double>();
            values.put("controller.kP", controller.value("kP"));
            values.put("experiment.targetVelocity", experiment.get("targetVelocity"));
            values.put("experiment.autoStopAfterSec", experiment.get("autoStopAfterSec"));
        }
    }

    private static final class PositionDraft
            implements FtcPositionControlPanelsTester.DraftPort {
        Map<String, Double> values = positionDraft(1.0, 0.0, 100.0, 5.0);

        @Override public Map<String, Double> read() {
            return new LinkedHashMap<String, Double>(values);
        }

        @Override public void seed(ControlTuningModel.Parameters controller,
                                   Map<String, Double> experiment) {
            values = new LinkedHashMap<String, Double>();
            values.put("controller.kP", controller.value("kP"));
            values.put("experiment.endpointA", experiment.get("endpointA"));
            values.put("experiment.endpointB", experiment.get("endpointB"));
            values.put("experiment.autoHoldAfterSec", experiment.get("autoHoldAfterSec"));
        }
    }

    private static final class TopologyPositionDraft
            implements FtcPositionControlPanelsTester.DraftPort {
        Map<String, Double> values = Collections.emptyMap();

        @Override public Map<String, Double> read() {
            return new LinkedHashMap<String, Double>(values);
        }

        @Override public void seed(ControlTuningModel.Parameters controller,
                                   Map<String, Double> experiment) {
            LinkedHashMap<String, Double> seeded = new LinkedHashMap<String, Double>();
            for (Map.Entry<String, Double> entry : controller.values().entrySet()) {
                seeded.put("controller." + entry.getKey(), entry.getValue());
            }
            seeded.put("experiment.endpointA", experiment.get("endpointA"));
            seeded.put("experiment.endpointB", experiment.get("endpointB"));
            seeded.put("experiment.autoHoldAfterSec", experiment.get("autoHoldAfterSec"));
            values = seeded;
        }
    }

    private static final class FakeSession implements ControlTuningModel.Session {
        final ScalarRange range;
        final ControlTuningModel.ReconfigurationPolicy policy;
        final List<String> events;
        final ControlTuningModel.Parameters initial = parameters("kP", 1.0);
        ControlTuningModel.Parameters applied = initial;
        FakePositionPlant positionPlant;
        double holdMeasurement = 50.0;
        double maximumCandidate = Double.POSITIVE_INFINITY;
        boolean ready = true;
        ControlTuningModel.Evidence publishedEvidence = ControlTuningModel.Evidence.empty();
        RuntimeException applyFailureAfterMutation;
        int applyCount;
        int restoreCount;
        int recoveryCount;

        FakeSession(ScalarRange range,
                    ControlTuningModel.ReconfigurationPolicy policy,
                    List<String> events) {
            this.range = range;
            this.policy = policy;
            this.events = events;
        }

        @Override public String topology() { return "FAKE"; }
        @Override public ScalarRange plantTargetRange() { return range; }
        @Override public ControlTuningModel.Parameters initialCandidate() { return initial; }
        @Override public List<ControlTuningModel.Readback> readbacks() {
            return Collections.singletonList(new ControlTuningModel.Readback("fake", applied));
        }
        @Override public void validate(ControlTuningModel.Parameters candidate) {
            if (candidate.value("kP") > maximumCandidate) {
                throw new IllegalArgumentException("candidate exceeds typed controller domain");
            }
        }
        @Override public void apply(ControlTuningModel.Parameters candidate, LoopClock clock) {
            events.add("session.apply");
            applyCount++;
            applied = candidate;
            if (applyFailureAfterMutation != null) {
                throw applyFailureAfterMutation;
            }
        }
        @Override public void restoreInitial(LoopClock clock) {
            events.add("session.restore");
            restoreCount++;
            applied = initial;
        }
        @Override public ControlTuningModel.ReconfigurationPolicy reconfigurationPolicy() {
            return policy;
        }
        @Override public boolean readyForReconfiguration(LoopClock clock) { return ready; }
        @Override public ControlTuningModel.Evidence evidence(LoopClock clock) {
            return publishedEvidence;
        }
        @Override public double preparePositionHold(LoopClock clock) {
            events.add("session.prepareHold");
            positionPlant.command.set(holdMeasurement);
            return holdMeasurement;
        }
        @Override public ControlTuningModel.PositionRecoveryHold preparePositionRecoveryHold(
                ScalarRange allowedPhysicalRange,
                LoopClock clock) {
            events.add("session.recoveryHold");
            double sampled = positionPlant.measurement;
            double target = allowedPhysicalRange.clamp(sampled);
            if (!Double.isFinite(target)) {
                throw new IllegalStateException("same-cycle position measurement is unavailable");
            }
            recoveryCount++;
            positionPlant.command.set(target);
            return new ControlTuningModel.PositionRecoveryHold(
                    sampled,
                    target,
                    !ControlTuningModel.sameDouble(sampled, target));
        }
    }

    private static class FakePlant implements Plant {
        final List<String> events;
        final RecordingTarget command;
        final ScalarRange range;
        double requested;
        double applied;
        double measurement;
        int stopCount;

        FakePlant(List<String> events, ScalarRange range) {
            this.events = events;
            this.range = range;
            command = new RecordingTarget(events);
        }

        @Override public void update(LoopClock clock) {
            requested = command.get();
            applied = requested;
            events.add("plant.update");
        }
        @Override public double getRequestedTarget() { return requested; }
        @Override public double getAppliedTarget() { return applied; }
        @Override public PlantTargetStatus getTargetStatus() { return PlantTargetStatus.ACCEPTED; }
        @Override public boolean hasFeedback() { return true; }
        @Override public double getMeasurement() { return measurement; }
        @Override public boolean atTarget() { return atTarget(requested); }
        @Override public boolean atTarget(double target) {
            return Double.isFinite(measurement)
                    && requested == target
                    && Math.abs(measurement - target) <= 0.5;
        }
        @Override public boolean hasCommandTarget() { return true; }
        @Override public ScalarTarget commandTarget() { return command; }
        @Override public void stop() {
            events.add("plant.stop");
            stopCount++;
        }
    }

    private static final class FakePositionPlant extends FakePlant implements PositionPlant {
        boolean referenced;

        FakePositionPlant(List<String> events, boolean referenced) {
            super(events, ScalarRange.bounded(0.0, 100.0));
            this.referenced = referenced;
        }

        @Override public Periodicity periodicity() { return Periodicity.NON_PERIODIC; }
        @Override public double period() { return Double.NaN; }
        @Override public ScalarRange targetRange() {
            return referenced ? range : ScalarRange.invalid("reference required");
        }
        @Override public boolean isReferenced() { return referenced; }
        @Override public String referenceStatus() {
            return referenced ? "REFERENCED" : "REFERENCE REQUIRED";
        }
        @Override public void establishReferenceAt(double plantPosition) { referenced = true; }
    }

    private static final class RecordingTarget implements ScalarTarget {
        final List<String> events;
        double value;
        int setCount;

        RecordingTarget(List<String> events) { this.events = events; }
        @Override public void set(double value) {
            this.value = value;
            setCount++;
            events.add("target=" + value);
        }
        @Override public double get() { return value; }
        @Override public void reset() { value = 0.0; }
    }

    private static final class RecordingPowerOutput implements PowerOutput {
        double power = Double.NaN;
        int writes;

        @Override public void setPower(double power) {
            this.power = power;
            writes++;
        }

        @Override public double getCommandedPower() {
            return power;
        }
    }

    private static final class ReferenceTask implements Task {
        final int id;
        final List<String> events;
        final AtomicInteger cancellations;
        final FakePositionPlant plant;
        final int updatesToComplete;
        int updates;
        boolean started;
        boolean complete;
        TaskOutcome outcome = TaskOutcome.NOT_DONE;

        ReferenceTask(int id,
                      List<String> events,
                      AtomicInteger cancellations,
                      FakePositionPlant plant,
                      int updatesToComplete) {
            this.id = id;
            this.events = events;
            this.cancellations = cancellations;
            this.plant = plant;
            this.updatesToComplete = updatesToComplete;
        }

        @Override public void start(LoopClock clock) {
            if (started) throw new IllegalStateException("already started");
            started = true;
            events.add("task." + id + ".start");
        }
        @Override public void update(LoopClock clock) {
            updates++;
            events.add("task." + id + ".update." + updates);
            if (updates >= updatesToComplete) {
                plant.referenced = true;
                plant.measurement = 50.0;
                complete = true;
                outcome = TaskOutcome.SUCCESS;
            }
        }
        @Override public void cancel() {
            if (!started || complete) return;
            cancellations.incrementAndGet();
            complete = true;
            outcome = TaskOutcome.CANCELLED;
            events.add("task." + id + ".cancel");
        }
        @Override public boolean isComplete() { return complete; }
        @Override public TaskOutcome getOutcome() { return outcome; }
    }

    private static final class ScriptedReferenceTask implements Task {
        final int id;
        final List<String> events;
        final FakePositionPlant plant;
        boolean establishOnUpdate;
        boolean throwOnStart;
        int cancelFailuresRemaining;
        int cancelAttempts;
        boolean started;
        boolean complete;
        TaskOutcome updateOutcome = TaskOutcome.NOT_DONE;
        TaskOutcome outcome = TaskOutcome.NOT_DONE;

        ScriptedReferenceTask(int id, List<String> events, FakePositionPlant plant) {
            this.id = id;
            this.events = events;
            this.plant = plant;
        }

        @Override public void start(LoopClock clock) {
            started = true;
            events.add("task." + id + ".start");
            if (throwOnStart) {
                throw new IllegalStateException("start failed after side effect");
            }
        }

        @Override public void update(LoopClock clock) {
            events.add("task." + id + ".update");
            if (establishOnUpdate) {
                plant.referenced = true;
            }
            if (updateOutcome != TaskOutcome.NOT_DONE) {
                complete = true;
                outcome = updateOutcome;
            }
        }

        @Override public void cancel() {
            cancelAttempts++;
            events.add("task." + id + ".cancel." + cancelAttempts);
            if (cancelFailuresRemaining > 0) {
                cancelFailuresRemaining--;
                throw new IllegalStateException("cancel failed after side effect");
            }
            complete = true;
            outcome = TaskOutcome.CANCELLED;
        }

        @Override public boolean isComplete() { return complete; }
        @Override public TaskOutcome getOutcome() { return outcome; }
    }

    private static final class RecordingTelemetry {
        final Map<String, String> data = new LinkedHashMap<String, String>();
        final Telemetry proxy = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (instance, method, args) -> {
                    String name = method.getName();
                    if (method.getDeclaringClass() == Object.class) {
                        if ("toString".equals(name)) return "RecordingTelemetry";
                        if ("hashCode".equals(name)) return System.identityHashCode(instance);
                        if ("equals".equals(name)) return instance == args[0];
                    }
                    if ("clearAll".equals(name)) {
                        data.clear();
                        return null;
                    }
                    if ("addData".equals(name)) {
                        data.put(String.valueOf(args[0]), String.valueOf(args[1]));
                        return null;
                    }
                    Class<?> returnType = method.getReturnType();
                    if (returnType == boolean.class) return true;
                    if (returnType == int.class) return 0;
                    return null;
                });

        String value(String key) { return data.get(key); }
    }
}
