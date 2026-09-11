package edu.ftcsushi.robots.examples.tagalignment;

import java.util.Objects;

import edu.ftcsushi.fw.drive.guidance.DriveGuidance;
import edu.ftcsushi.fw.drive.guidance.DriveGuidancePlan;
import edu.ftcsushi.fw.drive.guidance.DriveGuidanceSpec;
import edu.ftcsushi.fw.sensing.vision.CameraMountConfig;
import edu.ftcsushi.fw.sensing.vision.apriltag.AprilTagSensor;
import edu.ftcsushi.fw.spatial.ReferenceFrame2d;
import edu.ftcsushi.fw.spatial.References;

/** This example's shared, mode-neutral approach geometry; no field pose or field layout is used. */
public final class TagAlignment {
    private TagAlignment() {}

    /**
     * Builds an immutable plan from the draft. The source and fixed mount are borrowed; each
     * {@code plan.overlay()}, {@code plan.query()}, or {@code plan.task(...)} owns fresh runtime state.
     */
    public static DriveGuidancePlan plan(TagAlignmentProfile profile,
                                         AprilTagSensor tags,
                                         CameraMountConfig mount) {
        Objects.requireNonNull(profile, "profile");
        ReferenceFrame2d approach = References.relativeToTagFrame(profile.tagId,
                profile.tagForwardInches, profile.tagLeftInches, profile.tagHeadingRad);
        return DriveGuidance.plan()
                .translateTo().point(References.framePoint(approach))
                .andFaceTo().frameHeading(approach)
                .solveWith().relativeAprilTags(tags, mount)
                .maxAgeSec(profile.maxTagAgeSec)
                .onLoss(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
                .doneRelativeAprilTags()
                .driveTuning().use(profile.tuning).doneDriveTuning()
                .build();
    }
}
