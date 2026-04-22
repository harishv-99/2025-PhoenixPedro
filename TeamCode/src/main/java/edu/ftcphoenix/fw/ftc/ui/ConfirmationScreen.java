package edu.ftcphoenix.fw.ftc.ui;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * FTC telemetry screen for reviewing a short summary before confirming an action.
 *
 * <p>The screen is intentionally generic: it does not know whether the caller is confirming an
 * autonomous setup, a calibration reset, or a tester action. It simply renders labeled rows and runs
 * the supplied callbacks when the operator confirms, cancels, or resets through the high-level
 * {@link MenuScreen} actions dispatched by {@link MenuNavigator}.</p>
 *
 * <p>Typical usage in an INIT-phase autonomous selector:</p>
 *
 * <pre>{@code
 * ConfirmationScreen confirm = ConfirmationScreen.builder("Confirm Auto")
 *         .row("Alliance", "RED")
 *         .row("Start", "AUDIENCE")
 *         .row("Strategy", "Safe Preload")
 *         .warning("Verify this before pressing START.")
 *         .onConfirm(() -> selectedSpec = builder.build())
 *         .build();
 * }</pre>
 *
 * <p>By convention, {@link #select()} means confirm, {@link #onBack()} means cancel/back, and
 * {@link #onHome()} means reset/home. The navigator still owns whether back pops to the previous
 * screen or home returns to the root.</p>
 */
public final class ConfirmationScreen implements MenuScreen {

    /**
     * One immutable row in the confirmation summary.
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
     * Fluent builder for a confirmation screen.
     */
    public static final class Builder {
        private final String title;
        private final List<Row> rows = new ArrayList<Row>();
        private String help;
        private String statusTag;
        private String statusMessage;
        private String warning;
        private String confirmLabel = "Confirm";
        private String cancelLabel = "Back";
        private String resetLabel = "Home";
        private Runnable onConfirm;
        private Runnable onCancel;
        private Runnable onReset;

        private Builder(String title) {
            this.title = cleanOrDefault(title, "Confirm");
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
         * Set an optional compact status line such as {@code OK}, {@code WARN}, or {@code TODO}.
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
         * Set optional human-facing action labels used in the controls reminder.
         */
        public Builder actionLabels(String confirmLabel, String cancelLabel, String resetLabel) {
            this.confirmLabel = cleanOrDefault(confirmLabel, "Confirm");
            this.cancelLabel = cleanOrDefault(cancelLabel, "Back");
            this.resetLabel = cleanOrDefault(resetLabel, "Home");
            return this;
        }

        /**
         * Set the callback invoked when the operator confirms with {@code A}.
         */
        public Builder onConfirm(Runnable onConfirm) {
            this.onConfirm = onConfirm;
            return this;
        }

        /**
         * Set the callback invoked before navigator-managed back/cancel handling.
         */
        public Builder onCancel(Runnable onCancel) {
            this.onCancel = onCancel;
            return this;
        }

        /**
         * Set the callback invoked before navigator-managed home/reset handling.
         */
        public Builder onReset(Runnable onReset) {
            this.onReset = onReset;
            return this;
        }

        /**
         * Create the immutable confirmation screen.
         */
        public ConfirmationScreen build() {
            return new ConfirmationScreen(this);
        }
    }

    private final String title;
    private final List<Row> rows;
    private final String help;
    private final String statusTag;
    private final String statusMessage;
    private final String warning;
    private final String confirmLabel;
    private final String cancelLabel;
    private final String resetLabel;
    private final Runnable onConfirm;
    private final Runnable onCancel;
    private final Runnable onReset;

    private ConfirmationScreen(Builder b) {
        this.title = b.title;
        this.rows = Collections.unmodifiableList(new ArrayList<Row>(b.rows));
        this.help = b.help;
        this.statusTag = b.statusTag;
        this.statusMessage = b.statusMessage;
        this.warning = b.warning;
        this.confirmLabel = b.confirmLabel;
        this.cancelLabel = b.cancelLabel;
        this.resetLabel = b.resetLabel;
        this.onConfirm = b.onConfirm;
        this.onCancel = b.onCancel;
        this.onReset = b.onReset;
    }

    /**
     * Start building a confirmation screen with the given title.
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
     * Confirm the summarized action.
     */
    @Override
    public void select() {
        if (onConfirm != null) {
            onConfirm.run();
        }
    }

    /**
     * Run the optional cancel callback, then let the navigator pop by returning false.
     */
    @Override
    public boolean onBack() {
        if (onCancel != null) {
            onCancel.run();
        }
        return false;
    }

    /**
     * Run the optional reset callback, then let the navigator return home by returning false.
     */
    @Override
    public boolean onHome() {
        if (onReset != null) {
            onReset.run();
        }
        return false;
    }

    /**
     * Render the confirmation screen to telemetry. This method does not call {@code telemetry.update()}.
     */
    @Override
    public void render(Telemetry telemetry, MenuRenderContext context) {
        if (telemetry == null) return;
        MenuRenderContext ctx = context == null ? MenuRenderContext.root() : context;

        telemetry.addLine("=== " + title + " ===");

        if (ctx.breadcrumb() != null) {
            telemetry.addLine("Path: " + ctx.breadcrumb());
        }

        String navLine = navLine(ctx);
        if (navLine != null) {
            telemetry.addLine(navLine);
        }

        String status = statusLine(statusTag != null ? statusTag : ctx.statusTag(),
                statusMessage != null ? statusMessage : ctx.statusMessage());
        if (status != null) {
            telemetry.addLine(status);
        }

        if (help != null) {
            telemetry.addLine(help);
        }

        telemetry.addLine("");
        if (rows.isEmpty()) {
            telemetry.addLine("(nothing to confirm)");
        } else {
            for (int i = 0; i < rows.size(); i++) {
                Row row = rows.get(i);
                telemetry.addLine(row.label + ": " + row.value);
            }
        }

        if (warning != null) {
            telemetry.addLine("");
            telemetry.addLine("Warning: " + warning);
        }

        telemetry.addLine("");
        telemetry.addLine("A: " + confirmLabel + " | B/BACK: " + cancelLabel + " | Y: " + resetLabel);

        if (ctx.controlsHint() != null) {
            telemetry.addLine("Controls: " + ctx.controlsHint());
        }
        if (ctx.footer() != null) {
            telemetry.addLine(ctx.footer());
        }
    }

    private static String navLine(MenuRenderContext ctx) {
        boolean hasStep = ctx.hasStep();
        boolean hasLevel = ctx.level() >= 0;
        if (!hasStep && !hasLevel) return null;

        String out = "";
        if (hasStep) {
            out += "Step " + ctx.stepIndex() + " of " + ctx.stepCount();
        }
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
