package edu.ftcphoenix.fw.drive.guidance;

import java.util.ArrayList;
import java.util.Objects;

import edu.ftcphoenix.fw.drive.DriveOverlay;
import edu.ftcphoenix.fw.drive.DriveOverlayMask;
import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.fw.localization.PoseEstimator;
import edu.ftcphoenix.fw.sensing.observation.ObservationSource2d;

/**
 * Builder + helpers for creating {@link DriveGuidancePlan}s.
 *
 * <p>This replaces older tag-specific drive assist helpers (e.g. TagAim) with a single,
 * composable “guidance overlay” abstraction:</p>
 * <ul>
 *   <li>You describe a target (field point, tag-relative point, robot-relative offset, or field heading).</li>
 *   <li>You describe what to do (translate, aim, or both).</li>
 *   <li>You pick feedback sources (observation, field pose, or both).</li>
 *   <li>You apply the resulting overlay to a base {@link edu.ftcphoenix.fw.drive.DriveSource} using
 *       {@link edu.ftcphoenix.fw.drive.DriveSource#overlayWhen}.</li>
 * </ul>
 */
public final class DriveGuidance {

    private DriveGuidance() {
        // static utility
    }

    /**
     * Start building a {@link DriveGuidancePlan}.
     */
    public static PlanBuilder0 plan() {
        return new Builder0(new State());
    }

    /**
     * Create a pose-lock overlay that holds the current field pose.
     */
    public static DriveOverlay poseLock(PoseEstimator poseEstimator) {
        return poseLock(poseEstimator, DriveGuidancePlan.Tuning.defaults());
    }

    /**
     * Create a pose-lock overlay with custom tuning.
     */
    public static DriveOverlay poseLock(PoseEstimator poseEstimator, DriveGuidancePlan.Tuning tuning) {
        return new PoseLockOverlay(poseEstimator, tuning);
    }

    // ------------------------------------------------------------------------
    // Plan builder staging
    // ------------------------------------------------------------------------

    /**
     * Common methods shared by all plan builder stages.
     *
     * <p><b>Student mental model:</b> you first tell DriveGuidance what you want to do
     * (move and/or aim). Then you wire up feedback (vision and/or odometry). Finally you
     * call {@link #build()} to get an immutable {@link DriveGuidancePlan}.</p>
     *
     * <p>Most plans follow this shape:</p>
     * <pre>{@code
     * DriveGuidancePlan plan = DriveGuidance.plan()
     *     .aimTo()...doneAimTo()
     *     .translateTo()...doneTranslateTo()   // optional
     *     .feedback()...doneFeedback()
     *     .build();
     * }</pre>
     */
    public interface PlanBuilderCommon<SELF> {

        /**
         * Configure how DriveGuidance knows where the robot and/or the target is.
         *
         * <p>You must configure at least one feedback source:</p>
         * <ul>
         *   <li><b>Vision / observations</b> via {@link FeedbackBuilder#observation(ObservationSource2d)}.
         *       This works even with no odometry.</li>
         *   <li><b>Field pose / odometry</b> via {@link FeedbackBuilder#fieldPose(PoseEstimator)}.</li>
         *   <li><b>Both</b> for adaptive “smart” behavior (use vision when available, fall back to odometry).</li>
         * </ul>
         *
         * <p>When you finish configuring feedback, call {@link FeedbackBuilder#doneFeedback()} to return
         * to the main plan builder.</p>
         *
         * <h3>Example: AprilTag vision feedback</h3>
         * <pre>{@code
         * ObservationSource2d obs = ObservationSources.aprilTag(tagTarget, cameraMount);
         *
         * DriveGuidancePlan plan = DriveGuidance.plan()
         *     .aimTo().tagCenter().doneAimTo()
         *     .feedback()
         *         .observation(obs, 0.25, 0.0)   // ignore obs older than 0.25s
         *         .lossPolicy(DriveGuidancePlan.LossPolicy.PASS_THROUGH)
         *         .doneFeedback()
         *     .build();
         * }</pre>
         *
         * @return a nested builder for selecting feedback sources
         */
        FeedbackBuilder<SELF> feedback();

        /**
         * Choose which point(s) on the robot DriveGuidance should control.
         *
         * <p>By default, DriveGuidance controls the <b>robot center</b> for both moving and aiming.
         * In many games you actually care about an off-center mechanism (shooter, intake tip,
         * bucket, etc.).</p>
         *
         * <p>Use {@link ControlFrames#robotCenter()} for the common case.
         * Use {@link ControlFrames#withAimFrame(edu.ftcphoenix.fw.core.geometry.Pose2d)} / {@link ControlFrames#withTranslationFrame(edu.ftcphoenix.fw.core.geometry.Pose2d)}
         * when you need an offset.</p>
         *
         * @param frames robot → controlled-frame transforms
         * @return this builder stage for chaining
         */
        SELF controlFrames(ControlFrames frames);

        /**
         * Adjust how strongly DriveGuidance corrects errors.
         *
         * <p>This is the method students most often tweak when an assist feels “off”.</p>
         *
         * <ul>
         *   <li>If the assist is <b>too weak</b>: increase {@code kPTranslate} and/or {@code kPAim}.</li>
         *   <li>If it is <b>too twitchy</b>: decrease the gains.</li>
         *   <li>If it moves/turns <b>too fast</b>: lower {@code maxTranslateCmd} / {@code maxOmegaCmd}.</li>
         *   <li>If it keeps “hunting” near the target: increase {@code aimDeadbandRad}.</li>
         * </ul>
         *
         * <p><b>Tip:</b> start with {@link DriveGuidancePlan.Tuning#defaults()} and change <em>one</em>
         * value at a time.</p>
         *
         * <h3>Example: make aiming gentler</h3>
         * <pre>{@code
         * DriveGuidancePlan.Tuning t = DriveGuidancePlan.Tuning.defaults()
         *     .withAimKp(1.5);          // lower kPAim = gentler turning
         *
         * DriveGuidancePlan plan = DriveGuidance.plan()
         *     .aimTo().tagCenter().doneAimTo()
         *     .tuning(t)
         *     .feedback()...doneFeedback()
         *     .build();
         * }</pre>
         *
         * @param tuning controller tuning values
         * @return this builder stage for chaining
         */
        SELF tuning(DriveGuidancePlan.Tuning tuning);

        /**
         * Finish the builder and create an immutable {@link DriveGuidancePlan}.
         *
         * <p>After you have a plan, you can apply it as an overlay in TeleOp. For example,
         * to override only turning (omega) while a button is held:</p>
         *
         * <pre>{@code
         * DriveSource drive = baseDrive.overlayWhen(
         *     gamepads.p2().leftBumper()::isHeld,
         *     plan.overlay(),
         *     DriveOverlayMask.OMEGA_ONLY);
         * }</pre>
         *
         * @return an immutable plan (safe to reuse across loops)
         */
        DriveGuidancePlan build();
    }

    /**
     * Initial stage: you may configure translation (move) and/or aim (turn).
     */
    public interface PlanBuilder0 extends PlanBuilderCommon<PlanBuilder0> {

        /**
         * Configure a translation target: “move the robot to a point”.
         *
         * <p>You can call this at most once per plan. After choosing a target, call
         * {@link TranslateToBuilder#doneTranslateTo()} to return to the main plan builder.</p>
         */
        TranslateToBuilder<PlanBuilder1> translateTo();

        /**
         * Configure an aim target: “turn the robot to point toward something”.
         *
         * <p>You can call this at most once per plan. After choosing a target, call
         * {@link AimToBuilder#doneAimTo()} to return to the main plan builder.</p>
         */
        AimToBuilder<PlanBuilder2> aimTo();
    }

    /**
     * Stage after {@code translateTo()} has been configured (you can still add aim).
     */
    public interface PlanBuilder1 extends PlanBuilderCommon<PlanBuilder1> {

        /**
         * Configure an aim target (turn).
         */
        AimToBuilder<PlanBuilder3> aimTo();
    }

    /**
     * Stage after {@code aimTo()} has been configured (you can still add translation).
     */
    public interface PlanBuilder2 extends PlanBuilderCommon<PlanBuilder2> {

        /**
         * Configure a translation target (move).
         */
        TranslateToBuilder<PlanBuilder3> translateTo();
    }

    /**
     * Stage after both translation and aim targets have been configured.
     */
    public interface PlanBuilder3 extends PlanBuilderCommon<PlanBuilder3> {
        // No additional target methods.
    }

    // ------------------------------------------------------------------------
    // Nested builders
    // ------------------------------------------------------------------------

    /**
     * Nested builder for choosing a translation target.
     *
     * <p><b>Units:</b> all distances are in inches.</p>
     */
    public interface TranslateToBuilder<RETURN> {

        /**
         * Move toward a point in <b>field coordinates</b>.
         *
         * <p>Use this when you have a good {@link PoseEstimator} (odometry) and want to drive
         * to a known spot on the field.</p>
         *
         * @param xInches field X (forward) in inches
         * @param yInches field Y (left) in inches
         */
        TranslateToBuilder<RETURN> fieldPointInches(double xInches, double yInches);

        /**
         * Move toward a point that is defined relative to a specific AprilTag.
         *
         * <p>Coordinates are in the tag frame: +X is forward away from the tag, +Y is left.</p>
         *
         * <p>If you use field-pose feedback, you must also provide a {@link TagLayout} so the
         * tag can be located on the field.</p>
         *
         * @param tagId         the fixed tag ID
         * @param forwardInches forward offset from the tag (inches)
         * @param leftInches    left offset from the tag (inches)
         */
        TranslateToBuilder<RETURN> tagRelativePointInches(int tagId, double forwardInches, double leftInches);

        /**
         * Move relative to “whichever tag is currently observed”.
         *
         * <p>This is useful for vision-first TeleOp assist when you don’t care which scoring tag
         * you see — you just want to align to the one in view.</p>
         *
         * @param forwardInches forward offset from the observed tag (inches)
         * @param leftInches    left offset from the observed tag (inches)
         */
        TranslateToBuilder<RETURN> tagRelativePointInches(double forwardInches, double leftInches);

        /**
         * Move by a fixed offset defined in the <b>robot/translation frame</b> when the overlay is enabled.
         *
         * <p>This is a "nudge" style target: each time the guidance overlay becomes active,
         * it snapshots the current field pose and treats that pose as the origin for the requested
         * offset.</p>
         *
         * <p>Example: drive forward 6 inches from wherever you are:</p>
         * <pre>{@code
         * DriveGuidancePlan plan = DriveGuidance.plan()
         *     .translateTo().robotRelativePointInches(6, 0).doneTranslateTo()
         *     .feedback().fieldPose(poseEstimator).doneFeedback()
         *     .build();
         * }</pre>
         *
         * <p><b>Requires:</b> field-pose feedback. Observation-only feedback cannot measure robot
         * displacement.</p>
         *
         * @param forwardInches forward offset (+X) in inches
         * @param leftInches    left offset (+Y) in inches
         */
        TranslateToBuilder<RETURN> robotRelativePointInches(double forwardInches, double leftInches);

        /**
         * Finish translation configuration and return to the main plan builder.
         */
        RETURN doneTranslateTo();
    }

    /**
     * Nested builder for choosing an aim target.
     *
     * <p><b>Units:</b> all distances are in inches, angles are in radians.</p>
     */
    public interface AimToBuilder<RETURN> {

        /**
         * Turn so the controlled frame “looks at” a point in <b>field coordinates</b>.
         *
         * @param xInches field X (forward) in inches
         * @param yInches field Y (left) in inches
         */
        AimToBuilder<RETURN> fieldPointInches(double xInches, double yInches);

        /**
         * Turn so the controlled aim frame reaches an <b>absolute field heading</b>.
         *
         * <p>This is the "turn-to-heading" sibling of {@link #fieldPointInches(double, double)}.
         * Instead of aiming at a point, DriveGuidance will rotate until the aim frame's heading
         * matches the requested field heading.</p>
         *
         * <p><b>Requires:</b> field-pose feedback (a heading estimate). Observation-only feedback
         * cannot solve an absolute field heading.</p>
         *
         * @param fieldHeadingRad desired heading in the field frame (radians)
         */
        AimToBuilder<RETURN> fieldHeadingRad(double fieldHeadingRad);

        /**
         * Convenience overload: specify the desired field heading in degrees.
         */
        AimToBuilder<RETURN> fieldHeadingDeg(double fieldHeadingDeg);

        /**
         * Turn so the aim frame reaches an <b>absolute field heading</b> derived from a tag's orientation.
         *
         * <p>This is useful when you want to align to a wall/apriltag plane rather than
         * “look at” the tag center (bearing).</p>
         *
         * <p><b>Requires:</b> field-pose feedback with a {@link TagLayout}.</p>
         */
        AimToBuilder<RETURN> tagHeadingRad(int tagId, double headingOffsetRad);

        /**
         * Convenience: face the tag (i.e., tag heading + π) plus an additional offset.
         */
        AimToBuilder<RETURN> tagFaceTagRad(int tagId, double headingOffsetRad);

        /**
         * Turn so the robot “looks at” a point defined in a specific tag's coordinate frame.
         *
         * <p>This is how you do things like “aim 4 inches left of tag 5”.</p>
         */
        AimToBuilder<RETURN> tagRelativePointInches(int tagId, double forwardInches, double leftInches);

        /**
         * Turn so the robot “looks at” a point in the currently observed tag's frame.
         */
        AimToBuilder<RETURN> tagRelativePointInches(double forwardInches, double leftInches);

        /**
         * Convenience: aim at the center of a specific tag.
         */
        AimToBuilder<RETURN> tagCenter(int tagId);

        /**
         * Convenience: aim at the center of whichever tag is currently observed.
         */
        AimToBuilder<RETURN> tagCenter();

        /**
         * Finish aim configuration and return to the main plan builder.
         */
        RETURN doneAimTo();
    }

    /**
     * Nested builder for feedback configuration.
     *
     * <p>This is where you choose whether guidance uses vision, odometry, or both.</p>
     */
    public interface FeedbackBuilder<RETURN> {

        /**
         * Use observation-only feedback (vision, object detection, etc.) with default gating.
         *
         * <p>Use this when you do <b>not</b> have reliable odometry, or you want the assist to work
         * purely off what the camera sees.</p>
         *
         * @param observation a source of robot-relative observations (called once per loop)
         */
        FeedbackBuilder<RETURN> observation(ObservationSource2d observation);

        /**
         * Use observation-only feedback with explicit gating.
         *
         * @param observation observation source
         * @param maxAgeSec   ignore observations older than this many seconds (typical: 0.2–0.5)
         * @param minQuality  ignore observations with quality below this (0 accepts all)
         */
        FeedbackBuilder<RETURN> observation(ObservationSource2d observation, double maxAgeSec, double minQuality);

        /**
         * Use field-pose feedback (odometry / localization) with default gating.
         *
         * <p>This is the right choice when you have odometry wheels or a fused pose estimator.</p>
         */
        FeedbackBuilder<RETURN> fieldPose(PoseEstimator poseEstimator);

        /**
         * Use field-pose feedback and provide a {@link TagLayout}.
         *
         * <p>You need a {@link TagLayout} if you want to use any tag-relative targets
         * (because the system must know where the tag is on the field).</p>
         */
        FeedbackBuilder<RETURN> fieldPose(PoseEstimator poseEstimator, TagLayout tagLayout);

        /**
         * Use field-pose feedback with explicit gating.
         */
        FeedbackBuilder<RETURN> fieldPose(PoseEstimator poseEstimator, TagLayout tagLayout, double maxAgeSec, double minQuality);

        /**
         * Set (or replace) the tag layout used for tag-relative targets in field-pose mode.
         */
        FeedbackBuilder<RETURN> tagLayout(TagLayout tagLayout);

        /**
         * Configure adaptive “smart” selection gates.
         *
         * <p>This only matters when <em>both</em> observation and field pose are configured.
         * If you never call this, reasonable defaults are used.</p>
         *
         * @param enterRangeInches switch to observation when observed target range is <= this
         * @param exitRangeInches  switch back to field pose when observed target range is >= this
         * @param takeoverBlendSec blend time when switching sources (0 = instant)
         */
        FeedbackBuilder<RETURN> gates(double enterRangeInches, double exitRangeInches, double takeoverBlendSec);

        /**
         * Prefer observation for turning (omega) whenever it is valid.
         *
         * <p>This is usually what you want for AprilTag aiming: if the observation is valid, use the
         * camera bearing for crisp turning. Translation may still rely on field pose.</p>
         */
        FeedbackBuilder<RETURN> preferObservationForOmegaWhenValid(boolean prefer);

        /**
         * Set behavior when guidance cannot produce a valid command.
         *
         * <p><b>Recommendation for TeleOp:</b> use {@link DriveGuidancePlan.LossPolicy#PASS_THROUGH}
         * so the driver keeps control if vision/pose drops out.</p>
         */
        FeedbackBuilder<RETURN> lossPolicy(DriveGuidancePlan.LossPolicy lossPolicy);

        /**
         * Finish feedback configuration and return to the main plan builder.
         */
        RETURN doneFeedback();
    }

    // ------------------------------------------------------------------------
    // Implementation
    // ------------------------------------------------------------------------

    /**
     * Mutable builder state shared across staged builder instances.
     *
     * <p>The outer staged builder types ({@code PlanBuilder0..3}) are lightweight views
     * over this shared state so IntelliSense can prevent illegal call sequences
     * (like calling {@code translateTo()} twice).</p>
     */
    private static final class State {
        DriveGuidancePlan.TranslationTarget translationTarget;
        DriveGuidancePlan.AimTarget aimTarget;

        ControlFrames controlFrames = ControlFrames.robotCenter();
        DriveGuidancePlan.Tuning tuning = DriveGuidancePlan.Tuning.defaults();

        // Feedback
        ObservationSource2d observationSource;
        double obsMaxAgeSec = DriveGuidancePlan.Observation.DEFAULT_MAX_AGE_SEC;
        double obsMinQuality = DriveGuidancePlan.Observation.DEFAULT_MIN_QUALITY;

        PoseEstimator poseEstimator;
        TagLayout tagLayout;
        double poseMaxAgeSec = DriveGuidancePlan.FieldPose.DEFAULT_MAX_AGE_SEC;
        double poseMinQuality = DriveGuidancePlan.FieldPose.DEFAULT_MIN_QUALITY;

        DriveGuidancePlan.Gates gates;
        boolean preferObsOmega = true;
        DriveGuidancePlan.LossPolicy lossPolicy = DriveGuidancePlan.LossPolicy.PASS_THROUGH;
    }

    private static DriveGuidancePlan buildPlan(State s) {
        if (s.translationTarget == null && s.aimTarget == null) {
            throw new IllegalStateException("DriveGuidance plan needs translateTo() and/or aimTo() configured");
        }

        // Validate cross-feature compatibility up front so students get a clear error at build()
        // instead of a silent "no output" at runtime.
        validateCapabilitiesOrThrow(s);

        // Build feedback config.
        DriveGuidancePlan.Observation obs = null;
        if (s.observationSource != null) {
            obs = new DriveGuidancePlan.Observation(s.observationSource, s.obsMaxAgeSec, s.obsMinQuality);
        }

        DriveGuidancePlan.FieldPose fp = null;
        if (s.poseEstimator != null) {
            fp = new DriveGuidancePlan.FieldPose(s.poseEstimator, s.tagLayout, s.poseMaxAgeSec, s.poseMinQuality);
        }

        if (obs == null && fp == null) {
            throw new IllegalStateException("DriveGuidance plan needs feedback(): observation(...) and/or fieldPose(...)");
        }

        DriveGuidancePlan.Feedback fb = DriveGuidancePlan.Feedback.create(
                obs,
                fp,
                s.gates,
                s.preferObsOmega,
                s.lossPolicy
        );

        return new DriveGuidancePlan(
                s.translationTarget,
                s.aimTarget,
                s.controlFrames,
                s.tuning,
                fb
        );
    }

    /**
     * Validate that the configured targets and feedback are mutually compatible.
     *
     * <p>The builder is intentionally “student strict”: when a plan configuration can never
     * produce commands for the requested target(s) with the chosen feedback sources, Phoenix
     * throws at {@link PlanBuilderCommon#build()} with a message that explains what to change.</p>
     *
     * <p>This avoids silent runtime failures where an overlay simply returns
     * {@link DriveOverlayMask#NONE} forever.</p>
     */
    private static void validateCapabilitiesOrThrow(State s) {
        ArrayList<String> errors = new ArrayList<>();

        boolean hasObs = s.observationSource != null;
        boolean hasFieldPose = s.poseEstimator != null;

        if (!hasObs && !hasFieldPose) {
            errors.add("feedback() must configure observation(...) and/or fieldPose(...)");
        }

        // --- Basic numeric validations (helpful guardrails for students) ---
        if (hasObs) {
            if (!Double.isFinite(s.obsMaxAgeSec) || s.obsMaxAgeSec < 0.0) {
                errors.add("observation(...): maxAgeSec must be >= 0");
            }
            if (!Double.isFinite(s.obsMinQuality) || s.obsMinQuality < 0.0 || s.obsMinQuality > 1.0) {
                errors.add("observation(...): minQuality must be in [0, 1]");
            }
        }

        if (hasFieldPose) {
            if (!Double.isFinite(s.poseMaxAgeSec) || s.poseMaxAgeSec < 0.0) {
                errors.add("fieldPose(...): maxAgeSec must be >= 0");
            }
            if (!Double.isFinite(s.poseMinQuality) || s.poseMinQuality < 0.0 || s.poseMinQuality > 1.0) {
                errors.add("fieldPose(...): minQuality must be in [0, 1]");
            }
        }

        if (s.gates != null) {
            if (!Double.isFinite(s.gates.enterRangeInches) || s.gates.enterRangeInches < 0.0) {
                errors.add("gates(...): enterRangeInches must be >= 0");
            }
            if (!Double.isFinite(s.gates.exitRangeInches) || s.gates.exitRangeInches < 0.0) {
                errors.add("gates(...): exitRangeInches must be >= 0");
            }
            if (!Double.isFinite(s.gates.takeoverBlendSec) || s.gates.takeoverBlendSec < 0.0) {
                errors.add("gates(...): takeoverBlendSec must be >= 0");
            }
            if (Double.isFinite(s.gates.enterRangeInches)
                    && Double.isFinite(s.gates.exitRangeInches)
                    && s.gates.exitRangeInches < s.gates.enterRangeInches) {
                errors.add("gates(...): exitRangeInches must be >= enterRangeInches");
            }
        }

        // gates(...) is only meaningful in adaptive mode.
        if (s.gates != null && !(hasObs && hasFieldPose)) {
            errors.add("gates(...) is only used when both observation(...) and fieldPose(...) feedback are configured");
        }

        // --- Target/feedback capability checks ---
        // Field points require field pose feedback.
        if ((s.translationTarget instanceof DriveGuidancePlan.FieldPoint)
                || (s.aimTarget instanceof DriveGuidancePlan.FieldPoint)) {
            if (!hasFieldPose) {
                errors.add("field point targets require fieldPose(...) feedback");
            }
        }

        // Field heading requires field pose feedback.
        if (s.aimTarget instanceof DriveGuidancePlan.FieldHeading
                || s.aimTarget instanceof DriveGuidancePlan.TagHeading) {
            if (!hasFieldPose) {
                errors.add("field heading targets require fieldPose(...) feedback");
            }
        }

        // Robot-relative translation requires field pose feedback.
        if (s.translationTarget instanceof DriveGuidancePlan.RobotRelativePoint) {
            if (!hasFieldPose) {
                errors.add("robot-relative translation targets require fieldPose(...) feedback");
            }
        }

        // Tag-relative targets: certain combinations require observation and/or a tag layout.
        boolean usesTagTargets = (s.translationTarget instanceof DriveGuidancePlan.TagRelativePoint)
                || (s.aimTarget instanceof DriveGuidancePlan.TagRelativePoint)
                || (s.aimTarget instanceof DriveGuidancePlan.TagHeading);

        if (usesTagTargets && hasFieldPose) {
            if (s.tagLayout == null) {
                errors.add("tag-relative targets with fieldPose(...) require a TagLayout (call fieldPose(poseEstimator, tagLayout) or tagLayout(tagLayout))");
            }
        }

        // If any tag target uses “observed tag” (tagId = -1), we need an observation source
        // to supply the tag ID at least once.
        if (!hasObs) {
            if (isObservedTag(s.translationTarget) || isObservedTag(s.aimTarget)) {
                errors.add("tagRelativePointInches(forward,left) and tagCenter() (observed tag) require observation(...) feedback");
            }
        }

        // If a fixed tag ID is referenced and a layout is present, validate it exists.
        if (s.tagLayout != null) {
            checkTagIdExists(s.translationTarget, s.tagLayout, errors);
            checkTagIdExists(s.aimTarget, s.tagLayout, errors);
        }

        if (!errors.isEmpty()) {
            StringBuilder msg = new StringBuilder();
            msg.append("Invalid DriveGuidance plan:\n");
            for (int i = 0; i < errors.size(); i++) {
                msg.append(" - ").append(errors.get(i)).append('\n');
            }
            throw new IllegalStateException(msg.toString());
        }
    }

    private static boolean isObservedTag(Object target) {
        if (target instanceof DriveGuidancePlan.TagRelativePoint) {
            return ((DriveGuidancePlan.TagRelativePoint) target).tagId < 0;
        }
        if (target instanceof DriveGuidancePlan.TagHeading) {
            return ((DriveGuidancePlan.TagHeading) target).tagId < 0;
        }
        return false;
    }

    private static void checkTagIdExists(Object target, TagLayout layout, ArrayList<String> errors) {
        if (target instanceof DriveGuidancePlan.TagRelativePoint) {
            DriveGuidancePlan.TagRelativePoint tp = (DriveGuidancePlan.TagRelativePoint) target;
            if (tp.tagId >= 0 && !layout.has(tp.tagId)) {
                errors.add("TagLayout does not contain tag id=" + tp.tagId);
            }
            return;
        }
        if (target instanceof DriveGuidancePlan.TagHeading) {
            DriveGuidancePlan.TagHeading th = (DriveGuidancePlan.TagHeading) target;
            if (th.tagId >= 0 && !layout.has(th.tagId)) {
                errors.add("TagLayout does not contain tag id=" + th.tagId);
            }
            return;
        }
    }

    /**
     * Base builder that provides shared methods across plan stages.
     */
    private static abstract class BaseBuilder<SELF> {
        final State s;

        BaseBuilder(State s) {
            this.s = s;
        }

        @SuppressWarnings("unchecked")
        final SELF self() {
            return (SELF) this;
        }

        public final SELF controlFrames(ControlFrames frames) {
            s.controlFrames = Objects.requireNonNull(frames, "frames");
            return self();
        }

        public final SELF tuning(DriveGuidancePlan.Tuning tuning) {
            s.tuning = Objects.requireNonNull(tuning, "tuning");
            return self();
        }

        public final FeedbackBuilder<SELF> feedback() {
            return new FeedbackStep<>(s, self());
        }

        public final DriveGuidancePlan build() {
            return buildPlan(s);
        }
    }

    /**
     * Implementation of the initial plan builder stage.
     */
    private static final class Builder0 extends BaseBuilder<PlanBuilder0> implements PlanBuilder0 {
        Builder0(State s) {
            super(s);
        }

        @Override
        public TranslateToBuilder<PlanBuilder1> translateTo() {
            return new TranslateToStep<>(s, new Builder1(s));
        }

        @Override
        public AimToBuilder<PlanBuilder2> aimTo() {
            return new AimToStep<>(s, new Builder2(s));
        }
    }

    /**
     * Implementation of the plan builder stage after translation has been configured.
     */
    private static final class Builder1 extends BaseBuilder<PlanBuilder1> implements PlanBuilder1 {
        Builder1(State s) {
            super(s);
        }

        @Override
        public AimToBuilder<PlanBuilder3> aimTo() {
            return new AimToStep<>(s, new Builder3(s));
        }
    }

    /**
     * Implementation of the plan builder stage after aiming has been configured.
     */
    private static final class Builder2 extends BaseBuilder<PlanBuilder2> implements PlanBuilder2 {
        Builder2(State s) {
            super(s);
        }

        @Override
        public TranslateToBuilder<PlanBuilder3> translateTo() {
            return new TranslateToStep<>(s, new Builder3(s));
        }
    }

    /**
     * Implementation of the plan builder stage after both translation and aim have been configured.
     */
    private static final class Builder3 extends BaseBuilder<PlanBuilder3> implements PlanBuilder3 {
        Builder3(State s) {
            super(s);
        }
    }

    /**
     * Implementation of {@link TranslateToBuilder}.
     */
    private static final class TranslateToStep<RETURN> implements TranslateToBuilder<RETURN> {
        private final State s;
        private final RETURN ret;

        TranslateToStep(State s, RETURN ret) {
            this.s = s;
            this.ret = ret;
        }

        @Override
        public TranslateToBuilder<RETURN> fieldPointInches(double xInches, double yInches) {
            if (s.translationTarget != null) {
                throw new IllegalStateException("translateTo() target already configured; choose only one target method");
            }
            s.translationTarget = new DriveGuidancePlan.FieldPoint(xInches, yInches);
            return this;
        }

        @Override
        public TranslateToBuilder<RETURN> tagRelativePointInches(int tagId, double forwardInches, double leftInches) {
            if (s.translationTarget != null) {
                throw new IllegalStateException("translateTo() target already configured; choose only one target method");
            }
            s.translationTarget = new DriveGuidancePlan.TagRelativePoint(tagId, forwardInches, leftInches);
            return this;
        }

        @Override
        public TranslateToBuilder<RETURN> tagRelativePointInches(double forwardInches, double leftInches) {
            return tagRelativePointInches(-1, forwardInches, leftInches);
        }

        @Override
        public TranslateToBuilder<RETURN> robotRelativePointInches(double forwardInches, double leftInches) {
            if (s.translationTarget != null) {
                throw new IllegalStateException("translateTo() target already configured; choose only one target method");
            }
            s.translationTarget = new DriveGuidancePlan.RobotRelativePoint(forwardInches, leftInches);
            return this;
        }

        @Override
        public RETURN doneTranslateTo() {
            if (s.translationTarget == null) {
                throw new IllegalStateException("translateTo() requires a target before doneTranslateTo()");
            }
            return ret;
        }
    }

    /**
     * Implementation of {@link AimToBuilder}.
     */
    private static final class AimToStep<RETURN> implements AimToBuilder<RETURN> {
        private final State s;
        private final RETURN ret;

        AimToStep(State s, RETURN ret) {
            this.s = s;
            this.ret = ret;
        }

        @Override
        public AimToBuilder<RETURN> fieldPointInches(double xInches, double yInches) {
            if (s.aimTarget != null) {
                throw new IllegalStateException("aimTo() target already configured; choose only one target method");
            }
            s.aimTarget = new DriveGuidancePlan.FieldPoint(xInches, yInches);
            return this;
        }

        @Override
        public AimToBuilder<RETURN> fieldHeadingRad(double fieldHeadingRad) {
            if (s.aimTarget != null) {
                throw new IllegalStateException("aimTo() target already configured; choose only one target method");
            }
            s.aimTarget = new DriveGuidancePlan.FieldHeading(fieldHeadingRad);
            return this;
        }

        @Override
        public AimToBuilder<RETURN> fieldHeadingDeg(double fieldHeadingDeg) {
            return fieldHeadingRad(Math.toRadians(fieldHeadingDeg));
        }

        @Override
        public AimToBuilder<RETURN> tagHeadingRad(int tagId, double headingOffsetRad) {
            if (s.aimTarget != null) {
                throw new IllegalStateException("aimTo() target already configured; choose only one target method");
            }
            s.aimTarget = new DriveGuidancePlan.TagHeading(tagId, headingOffsetRad);
            return this;
        }

        @Override
        public AimToBuilder<RETURN> tagFaceTagRad(int tagId, double headingOffsetRad) {
            return tagHeadingRad(tagId, Math.PI + headingOffsetRad);
        }

        @Override
        public AimToBuilder<RETURN> tagRelativePointInches(int tagId, double forwardInches, double leftInches) {
            if (s.aimTarget != null) {
                throw new IllegalStateException("aimTo() target already configured; choose only one target method");
            }
            s.aimTarget = new DriveGuidancePlan.TagRelativePoint(tagId, forwardInches, leftInches);
            return this;
        }

        @Override
        public AimToBuilder<RETURN> tagRelativePointInches(double forwardInches, double leftInches) {
            return tagRelativePointInches(-1, forwardInches, leftInches);
        }

        @Override
        public AimToBuilder<RETURN> tagCenter(int tagId) {
            return tagRelativePointInches(tagId, 0.0, 0.0);
        }

        @Override
        public AimToBuilder<RETURN> tagCenter() {
            return tagRelativePointInches(-1, 0.0, 0.0);
        }

        @Override
        public RETURN doneAimTo() {
            if (s.aimTarget == null) {
                throw new IllegalStateException("aimTo() requires a target before doneAimTo()");
            }
            return ret;
        }
    }

    /**
     * Implementation of {@link FeedbackBuilder}.
     */
    private static final class FeedbackStep<RETURN> implements FeedbackBuilder<RETURN> {
        private final State s;
        private final RETURN ret;

        FeedbackStep(State s, RETURN ret) {
            this.s = s;
            this.ret = ret;
        }

        @Override
        public FeedbackBuilder<RETURN> observation(ObservationSource2d observation) {
            s.observationSource = Objects.requireNonNull(observation, "observation");
            s.obsMaxAgeSec = DriveGuidancePlan.Observation.DEFAULT_MAX_AGE_SEC;
            s.obsMinQuality = DriveGuidancePlan.Observation.DEFAULT_MIN_QUALITY;
            return this;
        }

        @Override
        public FeedbackBuilder<RETURN> observation(ObservationSource2d observation, double maxAgeSec, double minQuality) {
            s.observationSource = Objects.requireNonNull(observation, "observation");
            s.obsMaxAgeSec = maxAgeSec;
            s.obsMinQuality = minQuality;
            return this;
        }

        @Override
        public FeedbackBuilder<RETURN> fieldPose(PoseEstimator poseEstimator) {
            s.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator");
            s.poseMaxAgeSec = DriveGuidancePlan.FieldPose.DEFAULT_MAX_AGE_SEC;
            s.poseMinQuality = DriveGuidancePlan.FieldPose.DEFAULT_MIN_QUALITY;
            return this;
        }

        @Override
        public FeedbackBuilder<RETURN> fieldPose(PoseEstimator poseEstimator, TagLayout tagLayout) {
            s.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator");
            s.tagLayout = tagLayout;
            s.poseMaxAgeSec = DriveGuidancePlan.FieldPose.DEFAULT_MAX_AGE_SEC;
            s.poseMinQuality = DriveGuidancePlan.FieldPose.DEFAULT_MIN_QUALITY;
            return this;
        }

        @Override
        public FeedbackBuilder<RETURN> fieldPose(PoseEstimator poseEstimator,
                                                 TagLayout tagLayout,
                                                 double maxAgeSec,
                                                 double minQuality) {
            s.poseEstimator = Objects.requireNonNull(poseEstimator, "poseEstimator");
            s.tagLayout = tagLayout;
            s.poseMaxAgeSec = maxAgeSec;
            s.poseMinQuality = minQuality;
            return this;
        }

        @Override
        public FeedbackBuilder<RETURN> tagLayout(TagLayout tagLayout) {
            s.tagLayout = tagLayout;
            return this;
        }

        @Override
        public FeedbackBuilder<RETURN> gates(double enterRangeInches, double exitRangeInches, double takeoverBlendSec) {
            s.gates = new DriveGuidancePlan.Gates(enterRangeInches, exitRangeInches, takeoverBlendSec);
            return this;
        }

        @Override
        public FeedbackBuilder<RETURN> preferObservationForOmegaWhenValid(boolean prefer) {
            s.preferObsOmega = prefer;
            return this;
        }

        @Override
        public FeedbackBuilder<RETURN> lossPolicy(DriveGuidancePlan.LossPolicy lossPolicy) {
            s.lossPolicy = Objects.requireNonNull(lossPolicy, "lossPolicy");
            return this;
        }

        @Override
        public RETURN doneFeedback() {
            return ret;
        }
    }
}
