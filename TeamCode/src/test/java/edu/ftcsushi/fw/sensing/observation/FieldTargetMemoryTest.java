package edu.ftcsushi.fw.sensing.observation;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.core.time.LoopTimestamp;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import static org.junit.Assert.*;

/** Real capture-time history and immutable observations exercise bounded memory, not ball identity. */
public final class FieldTargetMemoryTest {
    private static final double EPS = 1.0e-9;

    @Test public void constructionStagesAreIndependentValidatedAndNeverPoll() {
        Input input = new Input();
        FieldTargetMemory.RetentionStep start = FieldTargetMemory.fromFieldObjects(input);
        FieldTargetMemory.MatchingStep oneSecond = start.retainingForSec(1);
        FieldTargetMemory.CapacityStep near = oneSecond.matchingWithinInches(2);
        FieldTargetMemory first = near.maxEntries(2);
        FieldTargetMemory second = near.maxEntries(3);
        FieldTargetMemory third = start.retainingForSec(0).matchingWithinInches(0).maxEntries(1);
        assertNotSame(first, second);
        assertEquals(1, first.source().retentionSec(), 0);
        assertEquals(0, third.source().retentionSec(), 0);
        assertSame(first.source(), first.source());
        assertEquals(FieldTargetMemory.InputDecision.NOT_UPDATED, first.snapshot().inputDecision());
        assertEquals(0, input.reads);
        expectIllegal(() -> start.retainingForSec(-1), "retentionSec");
        expectIllegal(() -> start.retainingForSec(Double.NaN), "retentionSec");
        expectIllegal(() -> start.retainingForSec(Double.POSITIVE_INFINITY), "retentionSec");
        expectIllegal(() -> oneSecond.matchingWithinInches(-1), "matchRadiusInches");
        expectIllegal(() -> oneSecond.matchingWithinInches(Double.NaN), "matchRadiusInches");
        expectIllegal(() -> oneSecond.matchingWithinInches(Double.POSITIVE_INFINITY), "matchRadiusInches");
        expectIllegal(() -> near.maxEntries(0), "maxEntries");
        expectIllegal(() -> near.maxEntries(TargetObservations2d.MAX_OBSERVATIONS + 1), "maxEntries");
        assertEquals(0, input.reads);
    }

    @Test public void stationaryFieldClusterSurvivesTranslationRotationAndPartialVisibility() {
        Fixture f = new Fixture(1, 0.1, 8);
        f.publishPose(0, 0, 0);
        f.input.value = f.fieldFrame(10, 0, 20, 0, 30, 0);
        f.update();
        FieldTargetMemory.Snapshot first = f.memory.snapshot();
        FieldTargetMemory.Entry unseen = at(first, 30, 0);
        FieldTargetMemory.Entry oldTen = at(first, 10, 0);
        f.next(0.5);
        f.publishPose(10, -10, Math.PI / 2);
        // Same field points at (10, 0) and (20, 0), authored independently in the rotated robot frame.
        f.input.value = f.fieldFrame(10, 0, 10, -10);
        f.update();
        assertEquals(3, f.memory.snapshot().entries().size());
        assertEquals(2, f.memory.snapshot().refreshedCount());
        assertSame(oldTen.key(), at(f.memory.snapshot(), 10, 0).key());
        assertSame(unseen, at(f.memory.snapshot(), 30, 0));
        assertEquals(0.5, unseen.lastSighting().ageSec(f.clock()), EPS);
        assertEquals(0, at(f.memory.snapshot(), 10, 0).lastSighting().ageSec(f.clock()), EPS);
        assertFalse(at(f.memory.snapshot(), 10, 0).lastSighting().hasQuality());
        assertTrue(at(f.memory.snapshot(), 10, 0).lastSighting().fieldLookup().isAvailable());
        f.next(0.625);
        f.input.value = TargetObservations2d.unavailable("camera obscured");
        f.update();
        assertEquals(2, f.memory.snapshot().entries().size());
        assertEquals(1, f.memory.snapshot().expiredCount());
        assertFalse(unseen.isUsable(f.clock()));
        assertEquals(30, unseen.lastSighting().fieldXInches, 0);
        assertEquals(3, first.entries().size());
    }

    @Test public void uniqueNearbyObservationRefreshesKeyButNeverMutatesPriorSnapshot() {
        Fixture f = new Fixture(1, 2, 8);
        f.observe(10, 0);
        FieldTargetMemory.Entry old = f.memory.snapshot().entries().get(0);
        f.next(0.25);
        f.observe(12, 0);
        FieldTargetMemory.Entry refreshed = f.memory.snapshot().entries().get(0);
        assertSame(old.key(), refreshed.key());
        assertNotSame(old, refreshed);
        assertEquals(10, old.lastSighting().fieldXInches, 0);
        assertEquals(12, refreshed.lastSighting().fieldXInches, 0);
        assertEquals(0.25, old.lastSighting().ageSec(f.clock()), 0);
        assertEquals(0, refreshed.lastSighting().ageSec(f.clock()), 0);
        assertTrue(old.isUsable(f.clock()));
        f.next(1);
        assertFalse(old.isUsable(f.clock()));
        assertTrue(refreshed.isUsable(f.clock()));
    }

    @Test public void oneOldToTwoNewOrTwoOldToOneNewNeverGreedilyRefreshes() {
        Fixture f = new Fixture(1, 3, 8);
        f.observe(10, 0, 30, 0, 34, 0);
        List<FieldTargetMemory.Entry> original = f.memory.snapshot().entries();
        f.next(0.25);
        f.observe(9, 0, 11, 0, 32, 0);
        assertEquals(original, f.memory.snapshot().entries());
        assertEquals(3, f.memory.snapshot().ambiguousCandidateCount());
        assertEquals(0, f.memory.snapshot().refreshedCount());
        assertEquals(0, f.memory.snapshot().createdCount());
    }

    @Test public void duplicateGeometryRemainsABlockerForAnotherNearbyCandidate() {
        Fixture f = new Fixture(1, 2, 8);
        f.observe(10, 0);
        FieldTargetMemory.Entry old = f.memory.snapshot().entries().get(0);
        f.next(0.25);
        f.observe(10, 0, 10, 0, 11, 0);
        assertEquals(Collections.singletonList(old), f.memory.snapshot().entries());
        assertEquals(3, f.memory.snapshot().ambiguousCandidateCount());
        assertEquals(3, f.memory.snapshot().eligibleCandidateCount());
    }

    @Test public void duplicateBirthsAreRejectedButNearbyDistinctClusterPointsRemainSeparate() {
        Fixture f = new Fixture(1, 10, 8);
        f.observe(0.0, 0.0, -0.0, -0.0, 10, 0, 11, 0);
        assertEquals(2, f.memory.snapshot().entries().size());
        assertEquals(2, f.memory.snapshot().ambiguousCandidateCount());
        assertEquals(2, f.memory.snapshot().createdCount());
        assertNotNull(at(f.memory.snapshot(), 10, 0));
        assertNotNull(at(f.memory.snapshot(), 11, 0));
    }

    @Test public void canonicalBirthAndEqualTimeEvictionDoNotDependOnDetectionOrder() {
        Fixture forward = new Fixture(1, 1, 2);
        Fixture reversed = new Fixture(1, 1, 2);
        forward.observe(1, 0, 2, 0, 3, 0);
        reversed.observe(3, 0, 1, 0, 2, 0);
        for (int i = 0; i < 2; i++) {
            FieldTargetMemory.Entry a = forward.memory.snapshot().entries().get(i);
            FieldTargetMemory.Entry b = reversed.memory.snapshot().entries().get(i);
            assertEquals(i + 2, a.lastSighting().fieldXInches, 0);
            assertEquals(a.lastSighting().fieldXInches, b.lastSighting().fieldXInches, 0);
            assertEquals(i + 1, a.key().sequence());
            assertEquals(a.key().sequence(), b.key().sequence());
            assertNotSame(a.key(), b.key());
        }
        assertEquals(1, forward.memory.snapshot().evictedCount());
    }

    @Test public void evictionRevokesOldestSightingRatherThanOldestRefreshedKey() {
        Fixture f = new Fixture(2, 1, 2);
        f.observe(10, 0, 20, 0);
        FieldTargetMemory.Entry ten = at(f.memory.snapshot(), 10, 0);
        FieldTargetMemory.Entry twenty = at(f.memory.snapshot(), 20, 0);
        f.next(0.25);
        f.observe(10, 0, 30, 0);
        assertTrue(ten.isUsable(f.clock()));
        assertFalse(twenty.isUsable(f.clock()));
        assertEquals(2, f.memory.snapshot().entries().size());
        assertEquals(1, f.memory.snapshot().evictedCount());
        assertSame(ten.key(), at(f.memory.snapshot(), 10, 0).key());
    }

    @Test public void zeroAndInclusiveRetentionAndRadiusAreExplicit() {
        Fixture f = new Fixture(0.25, 0, 8);
        f.observe(10, 0);
        FieldTargetMemory.Entry old = f.memory.snapshot().entries().get(0);
        f.next(0.25);
        f.input.value = TargetObservations2d.unavailable("no image");
        f.update();
        assertTrue(old.isUsable(f.clock()));
        assertEquals(1, f.memory.snapshot().entries().size());
        f.next(0.125);
        assertFalse(old.isUsable(f.clock()));
        f.update();
        assertTrue(f.memory.snapshot().entries().isEmpty());

        Fixture currentOnly = new Fixture(0, 0, 8);
        currentOnly.observe(10, 0);
        assertTrue(currentOnly.memory.snapshot().entries().get(0).isUsable(currentOnly.clock()));
        currentOnly.next(0);
        currentOnly.observe(12, 0); // Same capture time: not a new image.
        assertEquals(10, currentOnly.memory.snapshot().entries().get(0).lastSighting().fieldXInches, 0);
        currentOnly.next(0.125);
        currentOnly.observe(12, 0);
        assertEquals(1, currentOnly.memory.snapshot().expiredCount());
        assertEquals(12, currentOnly.memory.snapshot().entries().get(0).lastSighting().fieldXInches, 0);
    }

    @Test public void equalTimeNewObjectsAndOutOfOrderFramesCannotRefresh() {
        Fixture f = new Fixture(2, 5, 8);
        f.observe(10, 0);
        FieldTargetMemory.Entry first = f.memory.snapshot().entries().get(0);
        TargetObservations2d oldFrame = f.input.value;
        f.next(0);
        f.observe(11, 0); // Different timestamp object at the exact same clock time.
        assertSame(first, f.memory.snapshot().entries().get(0));
        assertEquals(FieldTargetMemory.InputDecision.REPEATED_OR_OUT_OF_ORDER,
                f.memory.snapshot().inputDecision());
        f.next(0.25);
        f.observe(12, 0);
        FieldTargetMemory.Entry newer = f.memory.snapshot().entries().get(0);
        f.next(0.25);
        f.input.value = oldFrame;
        f.update();
        assertSame(newer, f.memory.snapshot().entries().get(0));
        assertEquals(0.25, newer.lastSighting().ageSec(f.clock()), 0);
    }

    @Test public void unavailableAndObservedEmptyAreDifferentAndNeitherRemovesUnexpiredLocations() {
        Fixture f = new Fixture(1, 1, 8);
        f.observe(10, 0);
        FieldTargetMemory.Entry first = f.memory.snapshot().entries().get(0);
        f.next(0.25);
        f.input.value = TargetObservations2d.unavailable("pipeline changing");
        f.update();
        assertEquals(FieldTargetMemory.InputDecision.UNAVAILABLE, f.memory.snapshot().inputDecision());
        assertEquals("pipeline changing", f.memory.snapshot().inputReason());
        assertSame(first, f.memory.snapshot().entries().get(0));
        f.next(0.25);
        f.observe();
        assertEquals(FieldTargetMemory.InputDecision.OBSERVED_EMPTY, f.memory.snapshot().inputDecision());
        assertSame(first, f.memory.snapshot().entries().get(0));
    }

    @Test public void initiallyUnprojectableCaptureIsConsumedNotReinterpretedLater() {
        Fixture f = new Fixture(1, 1, 8);
        LoopTimestamp timestamp = f.clock().nowTimestamp();
        TargetObservations2d raw = rawFrame(timestamp, 10, 0);
        f.input.value = raw;
        f.update();
        assertTrue(f.memory.snapshot().entries().isEmpty());
        assertEquals(1, f.memory.snapshot().ineligibleCandidateCount());
        assertTrue(f.memory.snapshot().inputReason().contains("field projection not requested"));
        f.publishPose(0, 0, 0);
        f.input.value = f.project(raw);
        assertTrue(f.input.value.observations().get(0).hasFieldPosition());
        f.next(0.125);
        f.update();
        assertEquals(FieldTargetMemory.InputDecision.REPEATED_OR_OUT_OF_ORDER,
                f.memory.snapshot().inputDecision());
        assertTrue(f.memory.snapshot().entries().isEmpty());
        f.next(0.125);
        f.observe(10, 0);
        assertEquals(1, f.memory.snapshot().entries().size());
    }

    @Test public void noPositionIdentifiedAndFailedLookupMembersHaveTruthfulCounts() {
        Fixture f = new Fixture(1, 1, 8);
        f.publishPose(0, 0, 0);
        LoopTimestamp timestamp = f.clock().nowTimestamp();
        TargetObservation2d identified = TargetObservation2d.ofRobotRelativePosition(3, 10, 0,
                Double.NaN, timestamp);
        TargetObservation2d bearing = TargetObservation2d.ofRobotRelativeBearing(0, Double.NaN, timestamp);
        TargetObservation2d positioned = TargetObservation2d.ofRobotRelativePosition(20, 0, Double.NaN, timestamp);
        f.input.value = f.project(TargetObservations2d.fromFrame(timestamp,
                Arrays.asList(identified, bearing, positioned)));
        f.update();
        assertEquals(3, f.memory.snapshot().inputCandidateCount());
        assertEquals(1, f.memory.snapshot().eligibleCandidateCount());
        assertEquals(2, f.memory.snapshot().ineligibleCandidateCount());
        assertTrue(f.memory.snapshot().inputReason().contains("identified targets"));
        assertEquals(20, f.memory.snapshot().entries().get(0).lastSighting().fieldXInches, 0);
    }

    @Test public void newerButAlreadyExpiredCaptureAdvancesWatermarkWithoutMatchingOrBirth() {
        Fixture f = new Fixture(0.25, 2, 8);
        f.observe(10, 0);
        f.next(0.125);
        f.publishPose(0, 0, 0);
        TargetObservations2d delayed = f.fieldFrame(11, 0, 30, 0);
        f.next(0.5);
        f.input.value = delayed;
        f.update();
        assertEquals(FieldTargetMemory.InputDecision.CAPTURE_EXPIRED, f.memory.snapshot().inputDecision());
        assertEquals(0, f.memory.snapshot().createdCount());
        assertEquals(0, f.memory.snapshot().refreshedCount());
        assertTrue(f.memory.snapshot().entries().isEmpty());
        f.next(0);
        f.update();
        assertEquals(FieldTargetMemory.InputDecision.REPEATED_OR_OUT_OF_ORDER,
                f.memory.snapshot().inputDecision());
    }

    @Test public void foreignAndFutureCapturesDoNotAdvanceWatermark() {
        Fixture f = new Fixture(10, 1, 8);
        f.observe(10, 0);
        f.next(1);
        LoopTimestamp foreign = new ManualLoopClock(1).clock().nowTimestamp();
        f.input.value = rawFrame(foreign, 20, 0);
        f.update();
        assertEquals(FieldTargetMemory.InputDecision.INVALID_TIMESTAMP, f.memory.snapshot().inputDecision());
        f.clock().update(10);
        LoopTimestamp future = f.clock().nowTimestamp();
        f.clock().update(2); // Deliberately malformed host timing, not a reset epoch.
        f.input.value = rawFrame(future, 20, 0);
        f.update();
        assertEquals(FieldTargetMemory.InputDecision.INVALID_TIMESTAMP, f.memory.snapshot().inputDecision());
        f.clock().update(3);
        f.publishPose(0, 0, 0);
        f.input.value = f.fieldFrame(30, 0);
        f.update();
        assertEquals(FieldTargetMemory.InputDecision.PROCESSED, f.memory.snapshot().inputDecision());
        assertEquals(2, f.memory.snapshot().entries().size());
    }

    @Test public void evenSubmicrosecondFutureCaptureCannotAdvanceTheWatermark() {
        Fixture f = new Fixture(1, 1, 8);
        f.observe(10, 0);
        f.clock().update(0.25);
        f.publishPose(0, 0, 0);
        f.input.value = f.fieldFrame(20, 0);
        f.clock().update(0.25 - 0.0000005);
        assertTrue(f.input.value.timestamp().isFresh(f.clock(), 1)); // Timestamp comparison tolerance.
        f.update();
        assertEquals(FieldTargetMemory.InputDecision.INVALID_TIMESTAMP, f.memory.snapshot().inputDecision());
        assertEquals(1, f.memory.snapshot().entries().size());
        f.clock().update(0.25);
        f.update();
        assertEquals(FieldTargetMemory.InputDecision.PROCESSED, f.memory.snapshot().inputDecision());
        assertEquals(2, f.memory.snapshot().entries().size());
    }

    @Test public void repeatedUpdateViewAndSnapshotsArePassiveAndExpiryDoesNotPoll() {
        Fixture f = new Fixture(0.25, 1, 8);
        assertSame(f.memory.snapshot(), f.memory.source().get(f.clock()));
        assertEquals(0, f.input.reads);
        f.observe(10, 0);
        FieldTargetMemory.Snapshot first = f.memory.snapshot();
        f.update();
        assertSame(first, f.memory.snapshot());
        assertSame(first, f.memory.source().get(f.clock()));
        f.memory.source().reset();
        assertEquals(0, f.input.resets);
        assertEquals(1, f.input.reads);
        f.next(0.5);
        assertSame(first, f.memory.source().get(f.clock()));
        assertFalse(first.entries().get(0).isUsable(f.clock()));
        assertEquals(1, f.input.reads);
        try { first.entries().clear(); fail("immutable list"); }
        catch (UnsupportedOperationException expected) { }
    }

    @Test public void failedUpdateRetainsPublicationSuspendsEvidenceAndCannotReplaySameCycle() {
        Fixture f = new Fixture(1, 1, 8);
        f.observe(10, 0);
        FieldTargetMemory.Snapshot first = f.memory.snapshot();
        FieldTargetMemory.Entry entry = first.entries().get(0);
        f.next(0.25);
        RuntimeException failure = new IllegalStateException("camera failed");
        f.input.action = () -> { throw failure; };
        assertSame(failure, captureFailure(f::update));
        assertSame(first, f.memory.snapshot());
        assertFalse(entry.isUsable(f.clock()));
        assertSame(failure, captureFailure(f::update));
        assertSame(failure, captureFailure(() -> f.memory.source().get(f.clock())));
        assertEquals(2, f.input.reads);
        f.input.action = null;
        f.next(0.25);
        f.update();
        assertTrue(entry.isUsable(f.clock()));
        assertSame(entry, f.memory.source().get(f.clock()).entries().get(0));
        assertEquals(3, f.input.reads);
    }

    @Test public void failureDoesNotCommitExpiryOrConsumeThePendingCapture() {
        Fixture f = new Fixture(0.25, 1, 8);
        f.observe(10, 0);
        FieldTargetMemory.Snapshot first = f.memory.snapshot();
        f.next(0.5);
        f.publishPose(0, 0, 0);
        f.input.value = f.fieldFrame(20, 0);
        RuntimeException failure = new IllegalStateException("before return");
        f.input.action = () -> { throw failure; };
        assertSame(failure, captureFailure(f::update));
        assertSame(first, f.memory.snapshot());
        f.input.action = null;
        f.next(0);
        f.update();
        assertEquals(1, f.memory.snapshot().expiredCount());
        assertEquals(1, f.memory.snapshot().createdCount());
        assertEquals(20, f.memory.snapshot().entries().get(0).lastSighting().fieldXInches, 0);
    }

    @Test public void caughtUpdateReentryStillFailsOuterUpdateAndNextCycleCanRecover() {
        assertCaughtReentry(false);
    }

    @Test public void caughtResetReentryStillFailsOuterUpdateAndDoesNotInvalidatePartially() {
        assertCaughtReentry(true);
    }

    @Test public void resetRevokesOldKeysFencesCapturesAndPreservesClaimedCycle() {
        Fixture f = new Fixture(1, 1, 8);
        f.observe(10, 0);
        FieldTargetMemory.Entry old = f.memory.snapshot().entries().get(0);
        f.memory.reset(f.clock());
        assertFalse(old.isUsable(f.clock()));
        assertTrue(f.memory.snapshot().entries().isEmpty());
        f.update();
        assertEquals(1, f.input.reads);
        assertEquals(FieldTargetMemory.InputDecision.RESET, f.memory.snapshot().inputDecision());
        f.next(0);
        f.update();
        assertEquals(FieldTargetMemory.InputDecision.RESET_FENCE, f.memory.snapshot().inputDecision());
        f.next(0.125);
        f.observe(10, 0);
        FieldTargetMemory.Entry current = f.memory.snapshot().entries().get(0);
        assertNotSame(old.key(), current.key());
        assertFalse(old.isUsable(f.clock()));
        assertTrue(current.isUsable(f.clock()));
        assertEquals(0, f.input.resets);
    }

    @Test public void resetBeforeFirstUpdateBindsClockAndRejectsCapturesAtBoundary() {
        Fixture f = new Fixture(1, 1, 8);
        f.memory.reset(f.clock());
        f.observe(10, 0);
        assertTrue(f.memory.snapshot().entries().isEmpty());
        assertEquals(FieldTargetMemory.InputDecision.RESET_FENCE, f.memory.snapshot().inputDecision());
        expectIllegal(() -> f.memory.reset(new ManualLoopClock().clock()), "stable LoopClock");
        expectIllegal(() -> f.memory.update(new ManualLoopClock().clock()), "stable LoopClock");
        f.next(0.125);
        f.observe(10, 0);
        assertEquals(1, f.memory.snapshot().entries().size());
    }

    @Test public void resetAfterFailureCannotReleaseSameCycleFailureOrRepoll() {
        Fixture f = new Fixture(1, 1, 8);
        RuntimeException failure = new IllegalStateException("poll failure");
        f.input.action = () -> { throw failure; };
        assertSame(failure, captureFailure(f::update));
        f.memory.reset(f.clock());
        f.input.action = null;
        assertSame(failure, captureFailure(f::update));
        assertSame(failure, captureFailure(() -> f.memory.source().get(f.clock())));
        assertEquals(1, f.input.reads);
        f.next(0.125);
        f.observe(10, 0);
        assertEquals(1, f.memory.source().get(f.clock()).entries().size());
    }

    @Test public void clockResetInvalidatesImmediatelyAndNextUpdateFencesTheNewEpoch() {
        Fixture f = new Fixture(1, 1, 8);
        f.observe(10, 0);
        FieldTargetMemory.Entry old = f.memory.snapshot().entries().get(0);
        f.clock().reset(0);
        assertFalse(old.isUsable(f.clock()));
        f.update();
        assertTrue(f.memory.snapshot().entries().isEmpty());
        f.clock().update(0);
        f.publishPose(0, 0, 0);
        f.input.value = f.fieldFrame(20, 0);
        f.update();
        assertEquals(FieldTargetMemory.InputDecision.RESET_FENCE, f.memory.snapshot().inputDecision());
        f.clock().update(0.125);
        f.publishPose(0, 0, 0);
        f.input.value = f.fieldFrame(20, 0);
        f.update();
        assertEquals(1, f.memory.snapshot().entries().size());
        assertFalse(old.isUsable(f.clock()));
    }

    @Test public void callbackStopIsTerminalWithoutPublishingPreparedWork() {
        Fixture f = new Fixture(1, 1, 8);
        f.observe(10, 0);
        FieldTargetMemory.Entry old = f.memory.snapshot().entries().get(0);
        f.next(0.125);
        f.input.action = f.memory::stop;
        f.update();
        assertTrue(f.memory.snapshot().stopped());
        assertTrue(f.memory.snapshot().entries().isEmpty());
        assertFalse(old.isUsable(f.clock()));
        assertSame(f.memory.snapshot(), f.memory.source().get(f.clock()));
        f.memory.stop();
        assertTrue(captureFailure(f::update).getMessage().contains("stopped"));
        assertTrue(captureFailure(() -> f.memory.reset(f.clock())).getMessage().contains("stopped"));
        assertEquals(2, f.input.reads);
        assertEquals(0, f.input.resets);
    }

    @Test public void callbackClockAdvanceOrResetFailsWithoutPublishingPreparedWork() {
        for (boolean reset : new boolean[] {false, true}) {
            Fixture f = new Fixture(1, 1, 8);
            f.observe(10, 0);
            FieldTargetMemory.Snapshot first = f.memory.snapshot();
            f.next(0.125);
            f.input.action = () -> {
                if (reset) f.clock().reset(0);
                else f.clock().update(0.25);
            };
            RuntimeException failure = captureFailure(f::update);
            assertTrue(failure.getMessage().contains("shared LoopClock"));
            assertSame(first, f.memory.snapshot());
            assertFalse(first.entries().get(0).isUsable(f.clock()));
            assertSame(failure, captureFailure(() -> f.memory.source().get(f.clock())));
            f.input.action = null;
            f.clock().update(0.375);
            f.input.value = TargetObservations2d.unavailable("no new image");
            f.update();
            assertEquals(reset ? 0 : 1, f.memory.source().get(f.clock()).entries().size());
        }
    }

    @Test public void callbackStopAfterCaughtReentryStillPublishesOnlyTerminalEmptyEvidence() {
        Fixture f = new Fixture(1, 1, 8);
        f.observe(10, 0);
        FieldTargetMemory.Entry old = f.memory.snapshot().entries().get(0);
        f.next(0.125);
        f.input.action = () -> {
            assertTrue(captureFailure(f::update).getMessage().contains("reentry"));
            f.memory.stop();
        };
        f.update();
        assertTrue(f.memory.source().get(f.clock()).stopped());
        assertTrue(f.memory.snapshot().entries().isEmpty());
        assertFalse(old.isUsable(f.clock()));
        assertEquals(2, f.input.reads);
    }

    @Test public void stopBeforeStartNeedsNoClockAndCannotBeRevived() {
        Fixture f = new Fixture(1, 1, 8);
        f.memory.stop();
        f.memory.stop();
        assertTrue(f.memory.source().get(f.clock()).stopped());
        assertFalse(f.memory.snapshot().timestamp().isAvailable());
        assertTrue(captureFailure(f::update).getMessage().contains("stopped"));
        assertTrue(captureFailure(() -> f.memory.reset(f.clock())).getMessage().contains("stopped"));
        assertEquals(0, f.input.reads);
    }

    @Test public void finiteExtremePositionsAndMaximumFrameRemainBounded() {
        Fixture extremes = new Fixture(Double.MAX_VALUE, Double.MAX_VALUE, 2);
        extremes.observe(-Double.MAX_VALUE, 0, Double.MAX_VALUE, 0);
        assertEquals(2, extremes.memory.snapshot().entries().size());
        extremes.next(0.125);
        extremes.observe(Double.MAX_VALUE, 0);
        assertEquals(1, extremes.memory.snapshot().refreshedCount());
        assertEquals(2, extremes.memory.snapshot().entries().size());

        Fixture maximum = new Fixture(1, 0, 256);
        double[] positions = new double[TargetObservations2d.MAX_OBSERVATIONS * 2];
        for (int i = 0; i < positions.length; i += 2) positions[i] = i;
        maximum.observe(positions);
        assertEquals(256, maximum.memory.snapshot().entries().size());
        maximum.next(0.125);
        maximum.observe(positions);
        assertEquals(256, maximum.memory.snapshot().refreshedCount());
        assertEquals(256, maximum.memory.snapshot().entries().size());
    }

    private static void assertCaughtReentry(boolean reset) {
        Fixture f = new Fixture(1, 1, 8);
        f.observe(10, 0);
        FieldTargetMemory.Snapshot first = f.memory.snapshot();
        f.next(0.25);
        RuntimeException[] inner = new RuntimeException[1];
        f.input.action = () -> inner[0] = captureFailure(() -> {
            if (reset) f.memory.reset(f.clock()); else f.update();
        });
        RuntimeException outer = captureFailure(f::update);
        assertSame(inner[0], outer);
        assertTrue(outer.getMessage().contains("reentry"));
        assertSame(first, f.memory.snapshot());
        assertFalse(first.entries().get(0).isUsable(f.clock()));
        f.input.action = null;
        f.next(0.125);
        f.update();
        assertTrue(first.entries().get(0).isUsable(f.clock()));
        assertEquals(3, f.input.reads);
    }

    private static FieldTargetMemory.Entry at(FieldTargetMemory.Snapshot snapshot, double x, double y) {
        for (FieldTargetMemory.Entry entry : snapshot.entries()) {
            TargetObservation2d observation = entry.lastSighting();
            if (Math.abs(observation.fieldXInches - x) < EPS
                    && Math.abs(observation.fieldYInches - y) < EPS) return entry;
        }
        throw new AssertionError("missing field point (" + x + ", " + y + ")");
    }

    private static TargetObservations2d rawFrame(LoopTimestamp timestamp, double... xy) {
        List<TargetObservation2d> observations = new ArrayList<>();
        for (int i = 0; i < xy.length; i += 2) {
            observations.add(TargetObservation2d.ofRobotRelativePosition(xy[i], xy[i + 1], Double.NaN, timestamp));
        }
        return TargetObservations2d.fromFrame(timestamp, observations);
    }

    private static RuntimeException captureFailure(Runnable operation) {
        try { operation.run(); }
        catch (RuntimeException failure) { return failure; }
        throw new AssertionError("expected RuntimeException");
    }

    private static void expectIllegal(Runnable operation, String message) {
        RuntimeException failure = captureFailure(operation);
        assertTrue(failure instanceof IllegalArgumentException);
        assertTrue(failure.getMessage(), failure.getMessage().contains(message));
    }

    /** Clock-aware borrowed-input fake counts polling/reset and permits adversarial callbacks. */
    private static final class Input implements Source<TargetObservations2d> {
        TargetObservations2d value = TargetObservations2d.unavailable("no image yet");
        Runnable action;
        int reads;
        int resets;
        @Override public TargetObservations2d get(LoopClock clock) {
            reads++;
            if (action != null) action.run();
            return value;
        }
        @Override public void reset() { resets++; }
    }

    /** Reuses production pose history/projection with authored cached poses and image positions. */
    private static final class Fixture {
        final ObservationSourcesTest.Fixture poses = new ObservationSourcesTest.Fixture();
        final Input input = new Input();
        final FieldTargetMemory memory;
        Fixture(double retention, double radius, int capacity) {
            memory = FieldTargetMemory.fromFieldObjects(input).retainingForSec(retention)
                    .matchingWithinInches(radius).maxEntries(capacity);
        }
        LoopClock clock() { return poses.time.clock(); }
        void next(double sec) { poses.time.nextCycle(sec); }
        void publishPose(double x, double y, double heading) { poses.publish(x, y, heading); }
        TargetObservations2d fieldFrame(double... xy) { return project(rawFrame(clock().nowTimestamp(), xy)); }
        TargetObservations2d project(TargetObservations2d raw) {
            return ObservationSources.inField(Source.constant(raw), poses.history.lookupSource()).get(clock());
        }
        void observe(double... xy) {
            publishPose(0, 0, 0);
            input.value = fieldFrame(xy);
            update();
        }
        void update() { memory.update(clock()); }
    }
}
