package edu.ftcphoenix.robots.phoenix.autonomous;

import java.util.Objects;

/**
 * Immutable description of the autonomous setup chosen for Phoenix.
 *
 * <p>This object is deliberately robot-owned. The framework UI can help operators choose values, and
 * Pedro-specific code can turn those values into paths, but the meaning of alliance, start position,
 * partner plan, and strategy belongs to Phoenix.</p>
 */
public final class PhoenixAutoSpec {

    /**
     * Alliance color selected for the match.
     */
    public enum Alliance {
        RED("Red", "Use red scoring target policy."),
        BLUE("Blue", "Use blue scoring target policy.");

        private final String label;
        private final String help;

        Alliance(String label, String help) {
            this.label = label;
            this.help = help;
        }

        /**
         * Human-facing label.
         */
        public String label() {
            return label;
        }

        /**
         * One-line menu help.
         */
        public String help() {
            return help;
        }
    }

    /**
     * Starting location within the selected alliance side.
     */
    public enum StartPosition {
        AUDIENCE("Audience", "Start near the audience-side lane."),
        BACKSTAGE("Backstage", "Start near the backstage-side lane.");

        private final String label;
        private final String help;

        StartPosition(String label, String help) {
            this.label = label;
            this.help = help;
        }

        /**
         * Human-facing label.
         */
        public String label() {
            return label;
        }

        /**
         * One-line menu help.
         */
        public String help() {
            return help;
        }
    }

    /**
     * Match-to-match partner coordination assumption.
     */
    public enum PartnerPlan {
        NONE("None", "Do not reserve a lane for partner behavior."),
        PARTNER_TAKES_NEAR_LANE("Partner takes near lane", "Avoid the lane closest to Phoenix's start."),
        PARTNER_TAKES_CENTER_LANE("Partner takes center lane", "Avoid the center lane during cycling."),
        PARTNER_SCORES_FIRST("Partner scores first", "Delay or choose a path that does not race the partner."),
        PARTNER_PARKS_ONLY("Partner parks only", "Assume Phoenix owns scoring/cycle traffic.");

        private final String label;
        private final String help;

        PartnerPlan(String label, String help) {
            this.label = label;
            this.help = help;
        }

        /**
         * Human-facing label.
         */
        public String label() {
            return label;
        }

        /**
         * One-line menu help.
         */
        public String help() {
            return help;
        }
    }

    /**
     * Selected alliance color.
     */
    public final Alliance alliance;
    /**
     * Selected start position.
     */
    public final StartPosition startPosition;
    /**
     * Selected partner coordination assumption.
     */
    public final PartnerPlan partnerPlan;
    /**
     * Selected autonomous strategy id.
     */
    public final PhoenixAutoStrategyId strategy;

    private PhoenixAutoSpec(Builder b) {
        this.alliance = Objects.requireNonNull(b.alliance, "alliance");
        this.startPosition = Objects.requireNonNull(b.startPosition, "startPosition");
        this.partnerPlan = Objects.requireNonNull(b.partnerPlan, "partnerPlan");
        this.strategy = Objects.requireNonNull(b.strategy, "strategy");
    }

    /**
     * Start building a spec with safe defaults.
     */
    public static Builder builder() {
        return new Builder();
    }

    /**
     * Convenience factory for static safe audience-side entries.
     */
    public static PhoenixAutoSpec audienceSafe(Alliance alliance) {
        return builder()
                .alliance(alliance)
                .startPosition(StartPosition.AUDIENCE)
                .partnerPlan(PartnerPlan.NONE)
                .strategy(PhoenixAutoStrategyId.SAFE_PRELOAD)
                .build();
    }

    /**
     * Human-facing one-line description for telemetry.
     */
    public String summary() {
        return alliance.label() + " / "
                + startPosition.label() + " / "
                + partnerPlan.label() + " / "
                + strategy.label();
    }

    /**
     * Mutable builder used by selector UIs before a final immutable spec is confirmed.
     */
    public static final class Builder {
        private Alliance alliance = Alliance.RED;
        private StartPosition startPosition = StartPosition.AUDIENCE;
        private PartnerPlan partnerPlan = PartnerPlan.NONE;
        private PhoenixAutoStrategyId strategy = PhoenixAutoStrategyId.SAFE_PRELOAD;

        private Builder() {
        }

        /**
         * Set the selected alliance.
         */
        public Builder alliance(Alliance alliance) {
            this.alliance = Objects.requireNonNull(alliance, "alliance");
            return this;
        }

        /**
         * Set the selected start position.
         */
        public Builder startPosition(StartPosition startPosition) {
            this.startPosition = Objects.requireNonNull(startPosition, "startPosition");
            return this;
        }

        /**
         * Set the selected partner plan.
         */
        public Builder partnerPlan(PartnerPlan partnerPlan) {
            this.partnerPlan = Objects.requireNonNull(partnerPlan, "partnerPlan");
            return this;
        }

        /**
         * Set the selected strategy.
         */
        public Builder strategy(PhoenixAutoStrategyId strategy) {
            this.strategy = Objects.requireNonNull(strategy, "strategy");
            return this;
        }

        /**
         * Current builder alliance, useful for live menu summaries.
         */
        public Alliance alliance() {
            return alliance;
        }

        /**
         * Current builder start position, useful for live menu summaries.
         */
        public StartPosition startPosition() {
            return startPosition;
        }

        /**
         * Current builder partner plan, useful for live menu summaries.
         */
        public PartnerPlan partnerPlan() {
            return partnerPlan;
        }

        /**
         * Current builder strategy, useful for live menu summaries.
         */
        public PhoenixAutoStrategyId strategy() {
            return strategy;
        }

        /**
         * Build the immutable spec.
         */
        public PhoenixAutoSpec build() {
            return new PhoenixAutoSpec(this);
        }
    }
}
