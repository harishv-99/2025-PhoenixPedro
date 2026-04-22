package edu.ftcphoenix.fw.ftc.ui;

/**
 * Optional rendering metadata shared by FTC telemetry UI screens.
 *
 * <p>The context is deliberately lightweight: it carries display hints such as breadcrumb path,
 * current level, wizard step, status, and controls. It does not own input handling or robot state.
 * Callers can create a context directly for a single screen, while {@link MenuNavigator} creates one
 * automatically for the active screen based on its stack.</p>
 */
public final class MenuRenderContext {

    private String breadcrumb;
    private int level = -1;
    private int stepIndex = -1;
    private int stepCount = -1;
    private String statusTag;
    private String statusMessage;
    private String controlsHint;
    private String footer;

    /** Create an empty render context. */
    public static MenuRenderContext root() {
        return new MenuRenderContext();
    }

    /** Human-facing path such as {@code Auto > Red > Audience > Strategy}. */
    public String breadcrumb() {
        return breadcrumb;
    }

    /** Set the human-facing path shown near the top of the screen. */
    public MenuRenderContext setBreadcrumb(String breadcrumb) {
        this.breadcrumb = cleanOrNull(breadcrumb);
        return this;
    }

    /** Nesting level, where {@code 0} usually means the root screen. A negative value hides it. */
    public int level() {
        return level;
    }

    /** Set the nesting level shown near the top of the screen. */
    public MenuRenderContext setLevel(int level) {
        this.level = level;
        return this;
    }

    /** One-based wizard step index, or a negative value when no step should be shown. */
    public int stepIndex() {
        return stepIndex;
    }

    /** Wizard step count, or a negative value when no step should be shown. */
    public int stepCount() {
        return stepCount;
    }

    /** Set a one-based wizard step display such as {@code Step 3 of 5}. */
    public MenuRenderContext setStep(int stepIndex, int stepCount) {
        this.stepIndex = stepIndex;
        this.stepCount = stepCount;
        return this;
    }

    /** Optional compact screen status tag such as {@code OK}, {@code WARN}, or {@code TODO}. */
    public String statusTag() {
        return statusTag;
    }

    /** Optional screen status message paired with {@link #statusTag()}. */
    public String statusMessage() {
        return statusMessage;
    }

    /** Set a screen-level status line. */
    public MenuRenderContext setStatus(String tag, String message) {
        this.statusTag = cleanOrNull(tag);
        this.statusMessage = cleanOrNull(message);
        return this;
    }

    /** Optional controls hint shown near the bottom of the screen. */
    public String controlsHint() {
        return controlsHint;
    }

    /** Set the controls hint shown near the bottom of the screen. */
    public MenuRenderContext setControlsHint(String controlsHint) {
        this.controlsHint = cleanOrNull(controlsHint);
        return this;
    }

    /** Optional final line shown after the screen contents. */
    public String footer() {
        return footer;
    }

    /** Set the optional final line shown after the screen contents. */
    public MenuRenderContext setFooter(String footer) {
        this.footer = cleanOrNull(footer);
        return this;
    }

    /** Return true if this context has a complete wizard-step display. */
    public boolean hasStep() {
        return stepIndex > 0 && stepCount > 0;
    }

    private static String cleanOrNull(String value) {
        if (value == null) return null;
        String trimmed = value.trim();
        return trimmed.isEmpty() ? null : trimmed;
    }
}
