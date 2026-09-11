package edu.ftcsushi.robots.examples.visionpickup;

import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Pose2d;
import edu.ftcsushi.fw.core.source.Source;
import edu.ftcsushi.fw.sensing.observation.FieldTargetMemory;
import edu.ftcsushi.fw.sensing.observation.FieldTargetSelectionPolicies;
import edu.ftcsushi.fw.sensing.observation.FieldTargetSelectionSource;
import edu.ftcsushi.fw.sensing.observation.ObservationSources;
import edu.ftcsushi.fw.sensing.observation.TargetObservations2d;
import edu.ftcsushi.fw.sensing.observation.TargetSelections;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;

import static org.junit.Assert.*;

/** Memory is real and remains usable; the existing pickup still requires a fresh whole image. */
public final class VisionPickupMemoryBoundaryTest {
    @Test public void rememberedLocationCannotReplaceAnEmptyNewerPickupRecheck() {
        VisionPickupTestRig rig = rig();
        FieldTargetMemory memory = memory(rig);
        FieldTargetSelectionSource selected = selection(memory);
        memory.update(rig.clock());
        Task task = rig.pickup.createPickupTask(clock -> true);
        task.start(rig.clock());
        assertEquals(VisionPickup.Phase.RECHECK, rig.pickup.status().phase);

        rig.step(0.05, new Pose2d(2, 0, 0)); // New, genuinely empty image; remembered ball remains.
        memory.update(rig.clock());
        rig.pickup.update(rig.clock());
        task.update(rig.clock());
        assertTrue(selected.get(rig.clock()).isUsable(rig.clock()));
        assertEquals(TaskOutcome.CANCELLED, task.getOutcome());
        assertTrue(rig.pickup.status().reason.contains("target lost at final recheck"));
        assertTrue(rig.intakeRequests.isEmpty());
        assertEquals(0, rig.drive().axial, 0);
    }

    @Test public void cachedImageAndUsableMemoryStillTimeOutWithoutANewerRecheck() {
        VisionPickupTestRig rig = rig();
        FieldTargetMemory memory = memory(rig);
        FieldTargetSelectionSource selected = selection(memory);
        memory.update(rig.clock());
        TargetObservations2d original = rig.raw;
        Task task = rig.pickup.createPickupTask(clock -> true);
        task.start(rig.clock());
        rig.step(0.05, new Pose2d(2, 0, 0));
        rig.raw = original;
        memory.update(rig.clock());
        rig.pickup.update(rig.clock());
        task.update(rig.clock());
        assertEquals(VisionPickup.Phase.RECHECK, rig.pickup.status().phase);
        assertEquals(TaskOutcome.NOT_DONE, task.getOutcome());
        assertTrue(selected.get(rig.clock()).isUsable(rig.clock()));

        rig.step(0.30, new Pose2d(2, 0, 0));
        rig.raw = original;
        memory.update(rig.clock());
        rig.pickup.update(rig.clock());
        task.update(rig.clock());
        assertTrue(selected.get(rig.clock()).isUsable(rig.clock()));
        assertEquals(TaskOutcome.TIMEOUT, task.getOutcome());
        assertTrue(rig.intakeRequests.isEmpty());
        assertEquals(0, rig.drive().axial, 0);
    }

    private static VisionPickupTestRig rig() {
        return new VisionPickupTestRig(VisionPickupTestRig.configured(), new Pose2d(2, 0, 0), 10, 0);
    }

    private static FieldTargetMemory memory(VisionPickupTestRig rig) {
        return FieldTargetMemory.fromFieldObjects(ObservationSources.inField(
                Source.of(clock -> rig.raw), rig.history.lookupSource()))
                .retainingForSec(1).matchingWithinInches(1).maxEntries(4);
    }

    private static FieldTargetSelectionSource selection(FieldTargetMemory memory) {
        return TargetSelections.fromRecentFieldLocations(memory.source())
                .choose(FieldTargetSelectionPolicies.nearFieldPoint(10, 0, 5));
    }
}
