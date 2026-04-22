package edu.ftcphoenix.fw.tools.tester.calibration;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.ftcphoenix.fw.tools.tester.TeleOpTester;
import edu.ftcphoenix.fw.tools.tester.TesterSuite;

/**
 * Builder for guided calibration walkthrough menus.
 *
 * <p>The resulting object is still just a normal {@link TesterSuite}; the builder simply codifies
 * a few opinionated rules that we want robot projects to reuse consistently:</p>
 * <ul>
 *   <li>show steps in a fixed recommended order</li>
 *   <li>surface compact {@code OK} / {@code TODO} status tags next to steps when possible</li>
 *   <li>default the selection to the first incomplete tracked step</li>
 *   <li>keep the robot-side glue small and declarative</li>
 * </ul>
 *
 * <p>Status is now passed to the shared framework menu as an item tag instead of being embedded in
 * the label text. That keeps step labels stable while still making completion state easy to scan on
 * the Driver Station.</p>
 */
public final class CalibrationWalkthroughBuilder {

    private static final class Step {
        final String label;
        final String help;
        final Supplier<CalibrationStatus> status;
        final Supplier<TeleOpTester> testerFactory;

        Step(String label,
             String help,
             Supplier<CalibrationStatus> status,
             Supplier<TeleOpTester> testerFactory) {
            this.label = label;
            this.help = help;
            this.status = status;
            this.testerFactory = testerFactory;
        }
    }

    private final String title;
    private String help = "Run steps in order. A: enter step | BACK: go back";
    private int maxVisibleItems = 8;
    private int fallbackSelectedIndex = 0;

    private final List<Step> steps = new ArrayList<Step>();

    /** Create a builder for a named walkthrough. */
    public CalibrationWalkthroughBuilder(String title) {
        this.title = (title == null || title.trim().isEmpty())
                ? "Calibration Walkthrough"
                : title.trim();
    }

    /** Set the suite-level help line. */
    public CalibrationWalkthroughBuilder setHelp(String help) {
        this.help = help;
        return this;
    }

    /** Set how many items the suite renders at once. */
    public CalibrationWalkthroughBuilder setMaxVisibleItems(int maxVisibleItems) {
        this.maxVisibleItems = maxVisibleItems;
        return this;
    }

    /** Set which step should be highlighted if all tracked steps already look complete. */
    public CalibrationWalkthroughBuilder setFallbackSelectedIndex(int fallbackSelectedIndex) {
        this.fallbackSelectedIndex = fallbackSelectedIndex;
        return this;
    }

    /**
     * Add an informational step that is always shown without an {@code OK} / {@code TODO} tag.
     *
     * @return the zero-based step index
     */
    public int addStep(String label,
                       String help,
                       Supplier<TeleOpTester> testerFactory) {
        steps.add(new Step(label, help, null, testerFactory));
        return steps.size() - 1;
    }

    /**
     * Add a tracked step whose status contributes to the initial selection and item tag.
     *
     * @return the zero-based step index
     */
    public int addStep(String label,
                       String help,
                       Supplier<CalibrationStatus> status,
                       Supplier<TeleOpTester> testerFactory) {
        steps.add(new Step(label, help, status, testerFactory));
        return steps.size() - 1;
    }

    /** Build the guided suite. */
    public TesterSuite build() {
        TesterSuite suite = new TesterSuite()
                .setTitle(title)
                .setHelp(help)
                .setMaxVisibleItems(maxVisibleItems);

        int selectedIndex = Math.max(0, fallbackSelectedIndex);
        boolean foundIncomplete = false;

        for (int i = 0; i < steps.size(); i++) {
            Step step = steps.get(i);

            CalibrationStatus status = (step.status == null) ? null : step.status.get();
            if (!foundIncomplete && status != null && !status.complete) {
                selectedIndex = i;
                foundIncomplete = true;
            }

            String label = numberedLabel(i, step.label);
            String itemHelp = combinedHelp(step.help, status);
            String tag = status == null ? null : status.menuTag();

            suite.add(label, itemHelp, tag, step.testerFactory);
        }

        suite.setSelectedIndex(selectedIndex);
        return suite;
    }

    private static String numberedLabel(int index, String label) {
        String base = (label == null) ? "Step" : label;
        return (index + 1) + ") " + base;
    }

    private static String combinedHelp(String help, CalibrationStatus status) {
        String base = (help == null) ? "" : help.trim();
        if (status == null || status.summaryOrEmpty().isEmpty()) {
            return base.isEmpty() ? null : base;
        }
        if (base.isEmpty()) {
            return "Status: " + status.summaryOrEmpty();
        }
        return base + " Status: " + status.summaryOrEmpty();
    }
}
