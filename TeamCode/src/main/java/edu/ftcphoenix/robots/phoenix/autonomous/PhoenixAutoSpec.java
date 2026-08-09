package edu.ftcphoenix.robots.phoenix.autonomous;

import java.util.Objects;

import edu.ftcphoenix.robots.phoenix.PhoenixAlliance;

/**
 * Immutable description of the autonomous setup chosen for Phoenix.
 *
 * <p>This object is deliberately robot-owned. The framework UI can help operators choose values, and
 * Pedro-specific code can turn those values into paths, but the meaning of alliance, start
 * position, and strategy belongs to Phoenix.</p>
 */
public final class PhoenixAutoSpec {

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
     * Selected alliance color.
     */
    public final PhoenixAlliance alliance;
    /**
     * Selected start position.
     */
    public final StartPosition startPosition;
    /** Selected autonomous strategy id. */
    public final PhoenixAutoStrategyId strategy;

    private PhoenixAutoSpec(Builder b) {
        this.alliance = Objects.requireNonNull(b.alliance, "alliance");
        this.startPosition = Objects.requireNonNull(b.startPosition, "startPosition");
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
    public static PhoenixAutoSpec audienceSafe(PhoenixAlliance alliance) {
        return builder()
                .alliance(alliance)
                .startPosition(StartPosition.AUDIENCE)
                .strategy(PhoenixAutoStrategyId.SAFE_PRELOAD)
                .build();
    }

    /**
     * Human-facing one-line description for telemetry.
     */
    public String summary() {
        return alliance.label() + " / "
                + startPosition.label() + " / "
                + strategy.label();
    }

    /** Mutable builder used by selector UIs before the immutable spec is frozen at START. */
    public static final class Builder {
        private PhoenixAlliance alliance = PhoenixAlliance.RED;
        private StartPosition startPosition = StartPosition.AUDIENCE;
        private PhoenixAutoStrategyId strategy = PhoenixAutoStrategyId.SAFE_PRELOAD;

        private Builder() {
        }

        /**
         * Set the selected alliance.
         */
        public Builder alliance(PhoenixAlliance alliance) {
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
         * Set the selected strategy.
         */
        public Builder strategy(PhoenixAutoStrategyId strategy) {
            this.strategy = Objects.requireNonNull(strategy, "strategy");
            return this;
        }

        /**
         * Current builder alliance, useful for live menu summaries.
         */
        public PhoenixAlliance alliance() {
            return alliance;
        }

        /**
         * Current builder start position, useful for live menu summaries.
         */
        public StartPosition startPosition() {
            return startPosition;
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
