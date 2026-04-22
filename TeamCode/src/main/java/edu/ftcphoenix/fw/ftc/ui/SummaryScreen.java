package edu.ftcphoenix.fw.ftc.ui;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Read-only FTC telemetry screen for showing a compact state summary.
 *
 * <p>{@link ConfirmationScreen} is for a review page that can trigger confirm/cancel callbacks.
 * {@code SummaryScreen} is the quieter sibling used after something has already happened: an
 * autonomous selector has been locked, a tester has finished setup, or a calibration flow wants to
 * show a stable status page. The screen renders labeled rows and optional status/help/warning text,
 * but it does not imply that pressing {@code A} will apply an action.</p>
 *
 * <p>The screen can consume back/home actions when it represents a locked state. That lets a caller
 * replace a setup wizard with a single immutable summary after the robot runtime has been built,
 * preventing the visible menu state from drifting away from the already-initialized robot state.</p>
 */
public final class SummaryScreen implements MenuScreen {
    /**
     * One immutable row in the summary.
     */
    public static final class Row {
        /**
         * Human-facing row label.
         */
        public final String label;
        /**
         * Human-facing row value.
         */
        public final String value;

        private Row(String label, String value) {
            this.label = cleanOrDefault(label, "Value");
            this.value = cleanOrDefault(value, "-");
        }
    }

    /**
     * Fluent builder for a read-only summary screen.
     */
    public static final class Builder {
        private final String title;
        private final List<Row> rows = new ArrayList<Row>();
        private String help;
        private String statusTag;
        private String statusMessage;
        private String warning;
        private String controlsLine;
        private boolean consumeBack;
        private boolean consumeHome;
        private Runnable onSelect;
        private Runnable onSecondary;

        private Builder(String title) {
            this.title = cleanOrDefault(title, "Summary");
        }

        /**
         * Add one labeled value row to the summary.
         */
        public Builder row(String label, Object value) {
            rows.add(new Row(label, value == null ? "-" : String.valueOf(value)));
            return this;
        }

        /**
         * Set optional help text shown above the rows.
         */
        public Builder help(String help) {
            this.help = cleanOrNull(help);
            return this;
        }

        /**
         * Set an optional compact status line such as {@code OK}, {@code WARN}, or {@code LOCKED}.
         */
        public Builder status(String tag, String message) {
            this.statusTag = cleanOrNull(tag);
            this.statusMessage = cleanOrNull(message);
            return this;
        }

        /**
         * Set an optional warning line shown below the rows.
         */
        public Builder warning(String warning) {
            this.warning = cleanOrNull(warning);
            return this;
        }

        /**
         * Set an optional controls reminder line for this screen.
         */
        public Builder controls(String controlsLine) {
            this.controlsLine = cleanOrNull(controlsLine);
            return this;
        }

        /**
         * Configure whether this screen consumes navigator back/cancel actions.
         */
        public Builder consumeBack(boolean consumeBack) {
            this.consumeBack = consumeBack;
            return this;
        }

        /**
         * Configure whether this screen consumes navigator home/root actions.
         */
        public Builder consumeHome(boolean consumeHome) {
            this.consumeHome = consumeHome;
            return this;
        }

        /**
         * Set an optional callback for {@code A}; omit it when select should be a no-op.
         */
        public Builder onSelect(Runnable onSelect) {
            this.onSelect = onSelect;
            return this;
        }

        /**
         * Set an optional callback for {@code X}; omit it when secondary should be a no-op.
         */
        public Builder onSecondary(Runnable onSecondary) {
            this.onSecondary = onSecondary;
            return this;
        }

        /**
         * Create the immutable summary screen.
         */
        public SummaryScreen build() {
            return new SummaryScreen(this);
        }
    }

    private final String title;
    private final List<Row> rows;
    private final String help;
    private final String statusTag;
    private final String statusMessage;
    private final String warning;
    private final String controlsLine;
    private final boolean consumeBack;
    private final boolean consumeHome;
    private final Runnable onSelect;
    private final Runnable onSecondary;

    private SummaryScreen(Builder b) {
        this.title = b.title;
        this.rows = Collections.unmodifiableList(new ArrayList<Row>(b.rows));
        this.help = b.help;
        this.statusTag = b.statusTag;
        this.statusMessage = b.statusMessage;
        this.warning = b.warning;
        this.controlsLine = b.controlsLine;
        this.consumeBack = b.consumeBack;
        this.consumeHome = b.consumeHome;
        this.onSelect = b.onSelect;
        this.onSecondary = b.onSecondary;
    }

    /**
     * Start building a read-only summary screen with the given title.
     */
    public static Builder builder(String title) {
        return new Builder(title);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String title() {
        return title;
    }

    /**
     * Immutable snapshot of summary rows.
     */
    public List<Row> rows() {
        return rows;
    }

    /**
     * Run the optional select callback, when one was supplied.
     */
    @Override
    public void select() {
        if (onSelect != null) onSelect.run();
    }

    /**
     * Run the optional secondary callback, when one was supplied.
     */
    @Override
    public void secondary() {
        if (onSecondary != null) onSecondary.run();
    }

    /**
     * Return whether this summary intentionally blocks navigator-managed back/cancel.
     */
    @Override
    public boolean onBack() {
        return consumeBack;
    }

    /**
     * Return whether this summary intentionally blocks navigator-managed home/root.
     */
    @Override
    public boolean onHome() {
        return consumeHome;
    }

    /**
     * Render the summary screen to telemetry. This method does not call {@code telemetry.update()}.
     */
    @Override
    public void render(Telemetry telemetry, MenuRenderContext context) {
        if (telemetry == null) return;
        MenuRenderContext ctx = context == null ? MenuRenderContext.root() : context;
        telemetry.addLine("=== " + title + " ===");
        if (ctx.breadcrumb() != null) telemetry.addLine("Path: " + ctx.breadcrumb());
        String navLine = navLine(ctx);
        if (navLine != null) telemetry.addLine(navLine);
        String status = statusLine(statusTag != null ? statusTag : ctx.statusTag(), statusMessage != null ? statusMessage : ctx.statusMessage());
        if (status != null) telemetry.addLine(status);
        if (help != null) telemetry.addLine(help);
        telemetry.addLine("");
        if (rows.isEmpty()) telemetry.addLine("(no summary rows)");
        else for (int i = 0; i < rows.size(); i++)
            telemetry.addLine(rows.get(i).label + ": " + rows.get(i).value);
        if (warning != null) {
            telemetry.addLine("");
            telemetry.addLine("Warning: " + warning);
        }
        if (controlsLine != null) {
            telemetry.addLine("");
            telemetry.addLine(controlsLine);
        }
        if (ctx.controlsHint() != null) telemetry.addLine("Controls: " + ctx.controlsHint());
        if (ctx.footer() != null) telemetry.addLine(ctx.footer());
    }

    private static String navLine(MenuRenderContext ctx) {
        boolean hasStep = ctx.hasStep();
        boolean hasLevel = ctx.level() >= 0;
        if (!hasStep && !hasLevel) return null;
        String out = "";
        if (hasStep) out += "Step " + ctx.stepIndex() + " of " + ctx.stepCount();
        if (hasLevel) {
            if (!out.isEmpty()) out += " | ";
            out += "Level: " + ctx.level();
        }
        return out;
    }

    private static String statusLine(String tag, String message) {
        if (tag == null && message == null) return null;
        if (tag != null && message != null) return "Status: [" + tag + "] " + message;
        if (tag != null) return "Status: [" + tag + "]";
        return "Status: " + message;
    }

    private static String cleanOrNull(String value) {
        if (value == null) return null;
        String trimmed = value.trim();
        return trimmed.isEmpty() ? null : trimmed;
    }

    private static String cleanOrDefault(String value, String fallback) {
        String clean = cleanOrNull(value);
        return clean == null ? fallback : clean;
    }
}
