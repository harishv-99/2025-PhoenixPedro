package edu.ftcsushi.fw.tools.tester.calibration;

import org.junit.Test;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.ResultDownloads;
import edu.ftcsushi.fw.tools.tester.TesterContext;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

/** Maintainer-only optional transport failure probes; these do not model a browser or hardware. */
public final class CalibrationReportTest {
    @Test public void formattingFailureWithdrawsPriorCandidateWithoutAdvancingTheClock() {
        Downloads downloads = new Downloads();
        LoopClock clock = new LoopClock();
        clock.reset(12.0);
        TesterContext ctx = new TesterContext(null, null, null, null, clock, downloads);
        CalibrationReport report = new CalibrationReport();
        report.publish(ctx, "first.txt", () -> "old candidate");
        long cycle = clock.cycle();
        report.publish(ctx, "failed.txt", () -> { throw new IllegalStateException("format"); });
        assertNull(downloads.text);
        assertEquals(cycle, clock.cycle());
        assertEquals(12.0, clock.nowSec(), 0.0);
    }

    @Test public void reentrantClearDuringPublicationCannotResurrectTheCandidate() {
        Downloads downloads = new Downloads();
        TesterContext ctx = new TesterContext(null, null, null, null, new LoopClock(), downloads);
        CalibrationReport report = new CalibrationReport();
        downloads.beforePublish = () -> report.clear(ctx);
        report.publish(ctx, "old.txt", () -> "cleared candidate");
        assertNull(downloads.text);
    }

    /** Outside-world substitute used with each real calibration owner's existing fixture. */
    static final class Downloads implements ResultDownloads {
        String text;
        String filename;
        int publishes;
        int clears;
        boolean throwPublish;
        boolean throwClear;
        boolean throwUrl;
        boolean unavailable;
        Runnable beforePublish;

        @Override public boolean publish(String filename, String frozenUtf8Text) {
            publishes++;
            if (beforePublish != null) beforePublish.run();
            if (throwPublish) throw new IllegalStateException("download publication failed");
            if (unavailable) return false;
            this.filename = filename;
            this.text = frozenUtf8Text;
            return true;
        }

        @Override public String url() {
            if (throwUrl) throw new IllegalStateException("download URL failed");
            return text == null ? null : "http://fixture.invalid/result";
        }

        @Override public void clear() {
            clears++;
            if (throwClear) throw new IllegalStateException("download clear failed");
            text = null;
            filename = null;
        }
    }
}
