package edu.ftcphoenix.robots.phoenix;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

import edu.ftcphoenix.fw.field.TagLayout;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoSpec;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoStrategyId;
import edu.ftcphoenix.robots.phoenix.autonomous.pedro.PhoenixPedroPathFactory;

/**
 * Evaluates Phoenix-specific readiness rules without owning FTC UI or runtime lifecycle.
 *
 * <p>The framework and Pedro integration validate their own reusable construction contracts.
 * This utility owns the robot policy that those lower layers cannot know: which calibration
 * acknowledgements Phoenix requires, which alliance target belongs to the selected Auto, and
 * whether integration-only geometry may run from a particular Driver Station entry.</p>
 *
 * <p>Every factory returns an immutable, deterministically ordered {@link Result}. Mode clients
 * may render all issues during INIT and use {@link Result#requireAllowed(String)} at the arming
 * boundary. This class deliberately has no telemetry dependency and no public builder.</p>
 */
public final class PhoenixReadiness {

    /** Identifies the Driver Station purpose requesting a Pedro autonomous routine. */
    public enum AutoPurpose {
        /** A competition autonomous entry that must use match-ready geometry and calibration. */
        MATCH_AUTO,
        /** The explicitly named Pedro integration test entry. */
        PEDRO_INTEGRATION_TEST
    }

    /** Severity of one readiness finding. */
    public enum Severity {
        /** Visible caution that does not prevent the explicitly selected purpose from running. */
        WARNING,
        /** Unsafe or inconsistent configuration that must prevent the requested behavior. */
        BLOCKING
    }

    /**
     * One immutable readiness finding.
     *
     * <p>{@link #id()} is stable so tests and presenters can identify a rule without parsing its
     * student-facing text. The message explains the current problem; the remediation tells a
     * student what to change.</p>
     */
    public static final class Issue {
        private final String id;
        private final Severity severity;
        private final String message;
        private final String remediation;

        private Issue(String id,
                      Severity severity,
                      String message,
                      String remediation) {
            this.id = requireText(id, "id");
            this.severity = Objects.requireNonNull(severity, "severity");
            this.message = requireText(message, "message");
            this.remediation = requireText(remediation, "remediation");
        }

        /** Return the stable machine-readable rule id. */
        public String id() {
            return id;
        }

        /** Return whether this finding warns or blocks. */
        public Severity severity() {
            return severity;
        }

        /** Return the concise student-facing description of the current problem. */
        public String message() {
            return message;
        }

        /** Return the concrete student-facing next step. */
        public String remediation() {
            return remediation;
        }

        @Override
        public String toString() {
            return severity + "[" + id + "]: " + message + " Fix: " + remediation;
        }
    }

    /** Immutable ordered readiness report for one requested Phoenix behavior. */
    public static final class Result {
        private final List<Issue> issues;

        private Result(List<Issue> issues) {
            this.issues = Collections.unmodifiableList(new ArrayList<Issue>(issues));
        }

        /** Return all findings in stable policy order. */
        public List<Issue> issues() {
            return issues;
        }

        /** Return {@code true} when no blocking finding prevents the requested behavior. */
        public boolean isAllowed() {
            return firstBlockingIssueOrNull() == null;
        }

        /** Return whether the report contains at least one non-blocking warning. */
        public boolean hasWarnings() {
            for (Issue issue : issues) {
                if (issue.severity == Severity.WARNING) {
                    return true;
                }
            }
            return false;
        }

        /** Return the first blocking issue in policy order, or {@code null} when allowed. */
        public Issue firstBlockingIssueOrNull() {
            for (Issue issue : issues) {
                if (issue.severity == Severity.BLOCKING) {
                    return issue;
                }
            }
            return null;
        }

        /**
         * Fail with all blocking findings when this result cannot be armed.
         *
         * @param context human-facing owner/action name, such as {@code "Phoenix match Auto"}
         * @throws IllegalStateException when one or more blocking issues are present
         */
        public void requireAllowed(String context) {
            if (isAllowed()) {
                return;
            }

            String owner = context == null || context.trim().isEmpty()
                    ? "Phoenix behavior"
                    : context.trim();
            StringBuilder message = new StringBuilder(owner).append(" is not ready:");
            for (Issue issue : issues) {
                if (issue.severity != Severity.BLOCKING) {
                    continue;
                }
                message.append("\n - [")
                        .append(issue.id)
                        .append("] ")
                        .append(issue.message)
                        .append(" Fix: ")
                        .append(issue.remediation);
            }
            throw new IllegalStateException(message.toString());
        }

        @Override
        public String toString() {
            return "PhoenixReadiness.Result{allowed=" + isAllowed() + ", issues=" + issues + '}';
        }
    }

    private PhoenixReadiness() {
        // Factory utility.
    }

    /**
     * Evaluate whether localization-dependent TeleOp assists may be enabled.
     *
     * <p>Manual TeleOp remains a separate mode-client decision; this report applies only to pose
     * assists. Both Pinpoint axis direction and pod-offset acknowledgements are required.</p>
     *
     * @param profile Phoenix profile that would configure TeleOp
     * @return immutable readiness report
     */
    public static Result teleOpPoseAssists(PhoenixProfile profile) {
        Objects.requireNonNull(profile, "profile");
        List<Issue> issues = new ArrayList<Issue>();
        addCalibrationIssues(issues, profile, AutoPurpose.MATCH_AUTO);
        return new Result(issues);
    }

    /**
     * Evaluate the selected alliance's Auto target catalog and fixed-field facts.
     *
     * <p>The inactive alliance target is intentionally irrelevant. This check validates configured
     * field facts only; it never requires the target tag to be visible during INIT.</p>
     *
     * @param spec selected autonomous setup
     * @param profile profile snapshot to validate
     * @return immutable readiness report
     */
    public static Result autoProfile(PhoenixAutoSpec spec, PhoenixProfile profile) {
        Objects.requireNonNull(spec, "spec");
        Objects.requireNonNull(profile, "profile");
        List<Issue> issues = new ArrayList<Issue>();
        addAutoProfileIssues(issues, spec, profile);
        return new Result(issues);
    }

    /**
     * Evaluate the complete Phoenix-owned policy for one Pedro autonomous request.
     *
     * <p>Successful Pedro runtime construction remains the integration layer's responsibility.
     * This report combines only Phoenix-owned purpose, calibration, alliance-fact, and route-
     * maturity rules. Issues are returned in deterministic policy order.</p>
     *
     * @param spec selected autonomous setup
     * @param profile profile snapshot to validate
     * @param purpose Driver Station entry requesting the routine
     * @return immutable readiness report
     */
    public static Result pedroAuto(
            PhoenixAutoSpec spec,
            PhoenixProfile profile,
            AutoPurpose purpose
    ) {
        Objects.requireNonNull(spec, "spec");
        Objects.requireNonNull(profile, "profile");
        Objects.requireNonNull(purpose, "purpose");
        PhoenixPedroPathFactory.RouteAvailability routeAvailability =
                PhoenixPedroPathFactory.routeAvailabilityFor(spec);

        List<Issue> issues = new ArrayList<Issue>();
        addPurposeIssues(issues, spec, purpose);
        addCalibrationIssues(issues, profile, purpose);
        addAutoProfileIssues(issues, spec, profile);
        addRouteIssues(issues, routeAvailability, purpose);
        return new Result(issues);
    }

    private static void addPurposeIssues(List<Issue> issues,
                                         PhoenixAutoSpec spec,
                                         AutoPurpose purpose) {
        boolean testStrategy = spec.strategy == PhoenixAutoStrategyId.PEDRO_INTEGRATION_TEST;
        if (purpose == AutoPurpose.MATCH_AUTO && testStrategy) {
            issues.add(issue(
                    "auto.purpose_strategy_mismatch",
                    Severity.BLOCKING,
                    "The Pedro Integration Test strategy cannot run from a match Auto entry.",
                    "Choose a competition strategy, or use the explicitly named Phoenix Pedro Auto Test OpMode."
            ));
        } else if (purpose == AutoPurpose.PEDRO_INTEGRATION_TEST && !testStrategy) {
            issues.add(issue(
                    "auto.purpose_strategy_mismatch",
                    Severity.BLOCKING,
                    "The Pedro test entry may run only the Pedro Integration Test strategy.",
                    "Select PEDRO_INTEGRATION_TEST, or use a Phoenix match Auto entry for this strategy."
            ));
        }

        if (purpose == AutoPurpose.PEDRO_INTEGRATION_TEST) {
            issues.add(issue(
                    "auto.pedro_integration_test",
                    Severity.WARNING,
                    "TEST ONLY: this Pedro integration exercise is not a competition autonomous routine.",
                    "Use it only for supervised drivetrain/integration testing; select a match Auto entry for competition."
            ));
        }
    }

    private static void addCalibrationIssues(List<Issue> issues,
                                             PhoenixProfile profile,
                                             AutoPurpose purpose) {
        if (profile.calibration == null) {
            issues.add(issue(
                    "pose.calibration_config_missing",
                    Severity.BLOCKING,
                    "PhoenixProfile.calibration is missing.",
                    "Restore a PhoenixProfile.CalibrationConfig and complete its Pinpoint checks."
            ));
            return;
        }

        if (!profile.calibration.pinpointAxesVerified) {
            issues.add(issue(
                    "pose.pinpoint_axes_unverified",
                    Severity.BLOCKING,
                    "Pinpoint axis directions have not been verified for Phoenix coordinates.",
                    "Run 'Calib: Pinpoint Axis Check (Robot)', correct the pod directions, then set pinpointAxesVerified=true."
            ));
        }

        if (!profile.calibration.pinpointPodOffsetsCalibrated) {
            Severity severity = purpose == AutoPurpose.PEDRO_INTEGRATION_TEST
                    ? Severity.WARNING
                    : Severity.BLOCKING;
            issues.add(issue(
                    "pose.pinpoint_pod_offsets_uncalibrated",
                    severity,
                    "Pinpoint pod offsets have not been calibrated.",
                    "Run 'Calib: Pinpoint Pod Offsets (Robot)', copy the measured offsets, then set pinpointPodOffsetsCalibrated=true."
            ));
        }
    }

    private static void addAutoProfileIssues(List<Issue> issues,
                                             PhoenixAutoSpec spec,
                                             PhoenixProfile profile) {
        if (profile.auto == null) {
            issues.add(issue(
                    "auto.config_missing",
                    Severity.BLOCKING,
                    "PhoenixProfile.auto is missing, so the selected alliance target cannot be resolved.",
                    "Restore a PhoenixProfile.AutoConfig with red and blue alliance scoring tag ids."
            ));
            return;
        }

        int tagId = selectedAllianceTagId(spec, profile.auto);
        String tagField = selectedAllianceTagField(spec);

        if (tagId < 0) {
            issues.add(issue(
                    "auto.selected_target_id_invalid",
                    Severity.BLOCKING,
                    tagField + " must be a non-negative AprilTag id, but was " + tagId + ".",
                    "Set " + tagField + " to the selected alliance's fixed scoring AprilTag id."
            ));
            return;
        }

        if (profile.autoAim == null || profile.autoAim.scoringTargets == null) {
            issues.add(issue(
                    "auto.target_catalog_missing",
                    Severity.BLOCKING,
                    "PhoenixProfile.autoAim.scoringTargets is missing.",
                    "Restore the scoring-target catalog and add the selected alliance tag id " + tagId + "."
            ));
        } else {
            if (profile.autoAim.scoringTargets.get(tagId) == null) {
                issues.add(issue(
                        "auto.selected_target_missing",
                        Severity.BLOCKING,
                        "Selected " + spec.alliance.label() + " Auto tag id " + tagId
                                + " (" + tagField + ") is not in PhoenixProfile.autoAim.scoringTargets.",
                        "Add a scoring-target entry keyed by tag id " + tagId
                                + ", or correct " + tagField + "."
                ));
            }
        }

        TagLayout fixedLayout = profile.field == null ? null : profile.field.fixedAprilTagLayout;
        if (fixedLayout == null) {
            issues.add(issue(
                    "auto.fixed_tag_layout_missing",
                    Severity.BLOCKING,
                    "PhoenixProfile.field.fixedAprilTagLayout is missing.",
                    "Configure a fixed-field TagLayout containing the selected alliance scoring tag id "
                            + tagId + "."
            ));
        } else if (!fixedLayout.has(tagId)) {
            issues.add(issue(
                    "auto.selected_target_not_fixed",
                    Severity.BLOCKING,
                    "Selected " + spec.alliance.label() + " Auto tag id " + tagId
                            + " is not in PhoenixProfile.field.fixedAprilTagLayout.",
                    "Use the correct season/practice fixed-tag layout, or correct " + tagField + "."
            ));
        }
    }

    private static void addRouteIssues(
            List<Issue> issues,
            PhoenixPedroPathFactory.RouteAvailability routeAvailability,
            AutoPurpose purpose
    ) {
        if (routeAvailability.isMatchReady()) {
            return;
        }

        if (purpose == AutoPurpose.MATCH_AUTO) {
            issues.add(issue(
                    "auto.route_not_match_ready",
                    Severity.BLOCKING,
                    "The selected route is integration-only: " + routeAvailability.reason,
                    "Implement and validate match geometry for the exact Auto spec before arming match Auto."
            ));
        } else {
            issues.add(issue(
                    "auto.route_integration_only",
                    Severity.WARNING,
                    "TEST ROUTE: " + routeAvailability.reason,
                    "Keep the robot supervised and use this geometry only from the explicit Pedro integration test."
            ));
        }
    }

    private static int selectedAllianceTagId(PhoenixAutoSpec spec,
                                             PhoenixProfile.AutoConfig auto) {
        return spec.alliance == PhoenixAutoSpec.Alliance.RED
                ? auto.redAllianceScoringTagId
                : auto.blueAllianceScoringTagId;
    }

    private static String selectedAllianceTagField(PhoenixAutoSpec spec) {
        return spec.alliance == PhoenixAutoSpec.Alliance.RED
                ? "PhoenixProfile.auto.redAllianceScoringTagId"
                : "PhoenixProfile.auto.blueAllianceScoringTagId";
    }

    private static Issue issue(String id,
                               Severity severity,
                               String message,
                               String remediation) {
        return new Issue(id, severity, message, remediation);
    }

    private static String requireText(String value, String name) {
        String text = Objects.requireNonNull(value, name).trim();
        if (text.isEmpty()) {
            throw new IllegalArgumentException(name + " must not be blank");
        }
        return text;
    }
}
