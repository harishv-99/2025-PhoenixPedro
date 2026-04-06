package edu.ftcphoenix.fw.tools.tester.calibration;

/**
 * Small immutable snapshot describing whether a calibration step is considered complete.
 *
 * <p>This is intentionally tiny so tester menus can surface progress without forcing robot-side
 * code to invent a new status type for each guided walkthrough.</p>
 */
public final class CalibrationStatus {

    /**
     * Whether the calibration step is considered complete enough for the walkthrough to move on.
     */
    public final boolean complete;

    /**
     * Human-friendly one-line summary shown in tester-menu help text.
     */
    public final String summary;

    private CalibrationStatus(boolean complete, String summary) {
        this.complete = complete;
        this.summary = summary;
    }

    /**
     * Create a "done" status.
     */
    public static CalibrationStatus complete(String summary) {
        return new CalibrationStatus(true, summary);
    }

    /**
     * Create a "not done yet" status.
     */
    public static CalibrationStatus incomplete(String summary) {
        return new CalibrationStatus(false, summary);
    }

    /**
     * Menu tag used by calibration walkthroughs.
     */
    public String menuTag() {
        return complete ? "OK" : "TODO";
    }

    /**
     * Returns the summary, or an empty string if none was provided.
     */
    public String summaryOrEmpty() {
        return summary == null ? "" : summary;
    }

    @Override
    public String toString() {
        if (summary == null || summary.isEmpty()) {
            return menuTag();
        }
        return menuTag() + ": " + summary;
    }
}
