# Align to the tag you actually see

This example teaches one idea: approach a point attached to a visible AprilTag without estimating
the robot's position on the field. A camera observation tells us where the tag is relative to the
camera. The configured camera mount converts that measurement to the robot's frame. A *frame* is
an origin plus directions: here, the tag's origin and directions let us describe the same approach
point even if the tagged fixture is placed somewhere else.

Complete source starts with [TagAlignment.java](TagAlignment.java). One plan describes both the approach point and
the desired heading:

```java
ReferenceFrame2d approach = References.relativeToTagFrame(
        profile.tagId, profile.tagForwardInches, profile.tagLeftInches, profile.tagHeadingRad);

DriveGuidancePlan plan = DriveGuidance.plan()
        .translateTo().point(References.framePoint(approach))
        .andFaceTo().frameHeading(approach)
        .solveWith().relativeAprilTags(tags, mount)
        .maxAgeSec(profile.maxTagAgeSec)
        .onLoss(DriveGuidanceSpec.LossPolicy.PASS_THROUGH)
        .doneRelativeAprilTags()
        .driveTuning().use(profile.tuning).doneDriveTuning()
        .build();
```

The example targets the robot center. In the tag frame, positive X points outward from the tag
face; the draft asks for a point 18 inches outward and zero inches to the left. A heading of
`Math.PI` radians (180 degrees) points back toward the tag. Check these directions with your
actual mounting and printed tag before enabling motion. Distances use inches and angles use
radians; robot commands use +X forward, +Y left, and counter-clockwise positive turning.

There is no `AbsolutePoseEstimator`, fixed field layout, or switch to another pose source. Detector
metadata is still necessary: the camera must know the tag ID and physical printed size to measure
distance. The SDK's default detector library is selected in this draft; configure a custom
`camera.aprilTags.tagLibrary` when the chosen tag differs. Detector metadata is not field-position
knowledge.

## Complete source manifest

- [TagAlignment.java](TagAlignment.java): the shared plan factory.
- [TagAlignmentProfile.java](TagAlignmentProfile.java): independent hardware and behavior data.
- [TagAlignmentCamera.java](TagAlignmentCamera.java): the managed camera lifecycle owner.
- [TagAlignmentControls.java](TagAlignmentControls.java): held-button mapping and sampled status.
- [TagAlignmentTeleOp.java](TagAlignmentTeleOp.java): managed TeleOp composition.
- [TagAlignmentAuto.java](TagAlignmentAuto.java): managed one-Task Auto composition.
- [TagAlignmentTest.java](../../../../../../../test/java/edu/ftcsushi/robots/examples/tagalignment/TagAlignmentTest.java): portable software evidence.

## Try it safely

Both OpModes are disabled and [TagAlignmentProfile.java](TagAlignmentProfile.java) starts with
`allowMotion = false`. The profile supplies independent motor names/directions, camera mount,
tag ID and offset, command caps, and timeouts. Its numbers are illustrative, not safe or accurate
for every robot. Review all of them, confirm tag size and approach clearance, test wheel directions
with the robot safely supported, and test tag geometry without motion first. Only then set
`allowMotion = true`, remove the desired OpMode's `@Disabled`, and perform a supervised low-speed
run with FTC STOP available. The camera needs a usable frame; START does not guarantee readiness.

| Driver Station program | Behavior |
| --- | --- |
| `FW Direct Tag Align` | Sticks drive normally. Holding the left bumper assists position and heading. Release restores manual control. |
| `FW Direct Tag Align Auto` | START begins one fresh, bounded approach Task. It finishes with a reported outcome and performs no further action. |

TeleOp composes `manual.overlayWhen(hold, plan.overlay(), DriveOverlayMask.ALL)`. An *overlay*
replaces selected parts of the driver's command while enabled. `PASS_THROUGH` means a channel
without usable evidence keeps its manual command. Fresh evidence resumes assistance while the
bumper remains held. Release if you do not want it to resume. The displayed status comes from the
actual sampled overlay; formatting it does not read the camera again. “Assisting” does not claim
arrival, and this small example does not provide vibration cues.

Auto uses `program.rootTask(plan.task(drive.sink, profile.auto))`. A *Task* is work updated a little
each loop. Here, missing any requested translation or heading evidence immediately requests zero
drive. The Task can resume while its bounded loss interval remains open; a longer loss reports
`TIMEOUT`, never `SUCCESS`. `SUCCESS` requires both errors inside their configured tolerances.
FTC STOP cancels active work and requests zero. A new action must use a new `plan.task(...)`;
completed or cancelled Task instances are single-use. A returned stop call is a software request,
not proof that hardware has physically stopped.

## Who owns the work?

`FtcRobotOpMode` owns the only loop clock and orders Services, Bindings, Tasks, Outputs/Drive, and
Presenters. The registered camera service owns and closes one `FtcWebcamVisionLane`; its AprilTag
view is borrowed by guidance. Camera frames are sampled lazily by that shared view, so the service
does not add another camera poll or clock. Registering it before building later pieces ensures
managed cleanup can close it if later configuration fails.

TeleOp registers one drive source and sink with `program.drive(...)`. Auto instead registers a
small drive lifecycle service and gives its sink to the one Task: there is no manual or zero-idle
output writing over the Task. The direct mecanum sink requires no follower heartbeat. The
presenters only format cached control decisions or the Task's retained outcome.

## What this does not solve

Direct tag-relative alignment follows the actual tagged fixture when the tag and target move
together. It does not correct a wrong camera mount, wrong tag size, an inaccurate tag-to-target
offset, delayed images, wheel slip, collision clearance, or obstacle avoidance. It does not
remember an occluded fixture, infer field position, or choose among multiple tags. Keep the tag
visible through the approach or handle loss explicitly. Use field-pose guidance for field-fixed
destinations; do not silently switch coordinate authorities during this local approach.

The portable tests author camera frames and exercise the real plan, overlay, and Task. They cover
mount geometry, draft snapshots, manual release, loss/reacquisition, timeout, cancellation, and
fresh Task creation. They do not establish physical camera accuracy, braking distance, or tuning.
