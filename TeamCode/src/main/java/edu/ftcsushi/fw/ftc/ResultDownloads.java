package edu.ftcsushi.fw.ftc;

/**
 * Host-provided publication of one frozen, non-secret text result for a laptop download.
 *
 * <p>Ordinary testers use {@code TesterContext.downloads}; they do not construct a server. Calls
 * belong to the tester's existing lifecycle thread. The transfer reads supplied text only: it
 * never polls hardware, advances a clock, commits telemetry, or records results on the controller's
 * filesystem. Browser requests cannot start a recording or change robot behavior.</p>
 *
 * <p>The FTC host retains at most one current result of 512 KiB encoded UTF-8. It invalidates the
 * publication on tester exit, host reset, failure, or STOP. Download before leaving the tester,
 * but never delay an emergency stop to save evidence. An already-admitted transfer may finish its
 * old immutable result after invalidation; successful publication does not prove laptop receipt.
 * The link is for the trusted local robot network, not encrypted authentication.</p>
 */
public interface ResultDownloads {

    /**
     * Replace the current attachment with already-frozen text, without interpreting its contents.
     *
     * <p>The standard host requires a 1–96 character ASCII filename starting with a letter or
     * digit and otherwise containing only letters, digits, dots, hyphens, and underscores. Names
     * containing {@code ..} or ending in a dot are rejected. Text must contain valid Unicode and
     * encode to at most 512 KiB of UTF-8. Invalid input is rejected before replacing a result.</p>
     *
     * @param filename safe laptop download name, not a controller path
     * @param frozenUtf8Text immutable prepared result text; must not be null
     * @return true when retained for download; false when the host or lifetime is unavailable
     * @throws IllegalArgumentException when an available standard host receives invalid input
     */
    boolean publish(String filename, String frozenUtf8Text);

    /**
     * Return the current result's read-only browser page, or null when no result is available.
     * The URL identifies that result; replacing it never makes an old URL serve new contents.
     *
     * @return current result URL, or null; no hardware or network request is performed
     */
    String url();

    /**
     * Invalidate this scope's published result. Repeated calls and calls from a revoked scope do
     * nothing. This does not stop hardware, end a trial, or retract an already-admitted response.
     */
    void clear();
}
