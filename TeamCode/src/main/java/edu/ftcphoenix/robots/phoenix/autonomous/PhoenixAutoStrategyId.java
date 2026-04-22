package edu.ftcphoenix.robots.phoenix.autonomous;

/**
 * Robot-owned identifiers for Phoenix autonomous strategies.
 *
 * <p>The enum names are stable code-facing ids. Driver Station text should use {@link #label()} and
 * {@link #help()} so labels can improve without breaking saved selections, static OpModes, or test
 * code that compares strategy ids.</p>
 */
public enum PhoenixAutoStrategyId {
    /**
     * Low-risk preload routine that returns to a safe placeholder start/park location.
     */
    SAFE_PRELOAD("Safe Preload", "Low-risk preload/aim/shoot routine.", "DEFAULT"),

    /**
     * Preload score followed by a placeholder park segment.
     */
    PRELOAD_AND_PARK("Preload + Park", "Score preload, then finish in a safe parked state.", "OK"),

    /**
     * Strategy slot for routes that coordinate lane ownership with the alliance partner.
     */
    PARTNER_AWARE_CYCLE("Partner-Aware Cycle", "Choose the cycle lane based on the partner plan.", "WARN"),

    /**
     * Explicit Pedro integration exercise kept separate from competition strategy names.
     */
    PEDRO_INTEGRATION_TEST("Pedro Integration Test", "Twelve-inch route/aim/shoot/return test path.", "TEST");

    private final String label;
    private final String help;
    private final String tag;

    PhoenixAutoStrategyId(String label, String help, String tag) {
        this.label = label;
        this.help = help;
        this.tag = tag;
    }

    /**
     * Human-facing label for telemetry menus and confirmation screens.
     */
    public String label() {
        return label;
    }

    /**
     * One-line explanation suitable for a highlighted menu row.
     */
    public String help() {
        return help;
    }

    /**
     * Compact status tag for menu rows, or null if no tag should be shown.
     */
    public String tag() {
        return tag;
    }
}
