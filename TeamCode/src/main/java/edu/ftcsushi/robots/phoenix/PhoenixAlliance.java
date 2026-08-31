package edu.ftcsushi.robots.phoenix;

/**
 * Match alliance selected for one Phoenix runtime.
 *
 * <p>This mode-neutral robot fact is shared by TeleOp and Auto. It identifies which alliance's
 * scoring AprilTag is eligible for targeting; Auto also uses it when choosing alliance-relative
 * route geometry.</p>
 */
public enum PhoenixAlliance {
    /** Red alliance match policy. */
    RED("Red", "Use red-alliance match policy, including its scoring AprilTag."),
    /** Blue alliance match policy. */
    BLUE("Blue", "Use blue-alliance match policy, including its scoring AprilTag.");

    private final String label;
    private final String help;

    PhoenixAlliance(String label, String help) {
        this.label = label;
        this.help = help;
    }

    /** Return the human-facing alliance label. */
    public String label() {
        return label;
    }

    /** Return the one-line operator help for selecting this alliance. */
    public String help() {
        return help;
    }
}
