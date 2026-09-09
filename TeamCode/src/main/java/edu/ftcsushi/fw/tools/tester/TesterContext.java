package edu.ftcsushi.fw.tools.tester;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.ResultDownloads;

import java.util.Objects;

/**
 * Shared runtime context for Sushi testers.
 *
 * <p>This is intentionally small and FTC-centric: it provides access to the FTC SDK
 * objects that testers commonly need (hardware map, telemetry, and gamepads).</p>
 *
 * <h2>One loop, one heartbeat</h2>
 * <p>The {@link #clock} provided here is the single per-OpMode loop heartbeat.
 * Testers should treat it as read-only and should <b>not</b> create their own
 * independent clocks for per-cycle systems (like button edge tracking).</p>
 *
 * <p>Using a shared {@link LoopClock} enables per-cycle idempotency: if input systems
 * are updated multiple times in the same OpMode cycle, they can safely treat the
 * second call as a no-op using {@link LoopClock#cycle()}.</p>
 */
public final class TesterContext {

    private static final ResultDownloads UNAVAILABLE_DOWNLOADS = new ResultDownloads() {
        @Override public boolean publish(String filename, String frozenUtf8Text) { return false; }
        @Override public String url() { return null; }
        @Override public void clear() { }
    };

    private final DownloadLifetime downloadLifetime;
    private final DownloadLease downloadLease;

    /**
     * Read-only result transport selected by the host. Publish already-frozen text and add its URL
     * to the tester's existing telemetry frame; never create another server or telemetry commit.
     * This lease is revoked before a selected child's cleanup and replaced before a
     * {@link BaseTeleOpTester}'s START hook. Read it from the current {@code ctx}; do not retain a
     * pre-START alias. Ordinary offline contexts and automatically hosted testers that directly
     * implement {@link TeleOpTester} without the base lifecycle expose an unavailable capability
     * whose publication returns false and URL is null.
     */
    public final ResultDownloads downloads;

    /**
     * FTC hardware map.
     */
    public final HardwareMap hw;

    /**
     * Host-selected FTC telemetry sink.
     *
     * <p>A console adapter may mirror the same row-oriented frame to multiple displays. Testers
     * write one frame here and never select a transport themselves.</p>
     */
    public final Telemetry telemetry;

    /** Stable FTC gamepad 1 owned by the selected tester console. */
    public final Gamepad gamepad1;

    /** Stable FTC gamepad 2 owned by the selected tester console. */
    public final Gamepad gamepad2;

    /**
     * Per-loop heartbeat owned by the OpMode that is running the tester.
     *
     * <p>This clock is advanced once per OpMode cycle by the runner. Testers should
     * use it for dt and for per-cycle identity via {@link LoopClock#cycle()}.</p>
     */
    public final LoopClock clock;

    /**
     * Create a tester context.
     *
     * @param hw FTC hardware map
     * @param telemetry host-selected FTC telemetry sink
     * @param gamepad1 stable gamepad 1 from the selected console
     * @param gamepad2 stable gamepad 2 from the selected console
     * @param clock shared per-loop clock advanced by the tester runner
     */
    public TesterContext(
            HardwareMap hw,
            Telemetry telemetry,
            Gamepad gamepad1,
            Gamepad gamepad2,
            LoopClock clock
    ) {
        this(hw, telemetry, gamepad1, gamepad2, clock, UNAVAILABLE_DOWNLOADS);
    }

    /**
     * Advanced host/test construction with an explicitly supplied result transport.
     *
     * <p>Ordinary tester OpModes receive this wiring automatically. The caller owns the supplied
     * transport's root lifetime and reset/STOP invalidation. Nested tester owners issue revocable
     * child scopes without changing the shared hardware, gamepads, telemetry, or clock.</p>
     *
     * @param hw FTC hardware map
     * @param telemetry host-selected telemetry
     * @param gamepad1 stable first gamepad
     * @param gamepad2 stable second gamepad
     * @param clock borrowed shared heartbeat
     * @param downloads host-owned transfer capability; must not be null
     */
    public TesterContext(
            HardwareMap hw,
            Telemetry telemetry,
            Gamepad gamepad1,
            Gamepad gamepad2,
            LoopClock clock,
            ResultDownloads downloads
    ) {
        this(hw, telemetry, gamepad1, gamepad2, clock,
                new DownloadLifetime(Objects.requireNonNull(downloads, "downloads"), null));
    }

    private TesterContext(HardwareMap hw, Telemetry telemetry, Gamepad gamepad1,
                          Gamepad gamepad2, LoopClock clock, DownloadLifetime downloadLifetime) {
        this.hw = hw;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.clock = clock;
        this.downloadLifetime = downloadLifetime;
        this.downloadLease = new DownloadLease(downloadLifetime);
        this.downloads = downloadLease;
    }

    /** Fork a private publication lifetime while preserving every shared runtime object. */
    TesterContext forChild(boolean supportsDownloads) {
        return new TesterContext(hw, telemetry, gamepad1, gamepad2, clock,
                new DownloadLifetime(supportsDownloads ? downloadLifetime.delegate
                        : UNAVAILABLE_DOWNLOADS, downloadLifetime));
    }

    /** Renew the epoch lease; null means a callback revoked the owner before renewal completed. */
    TesterContext forStart() {
        downloadLease.active = false;
        if (downloadLifetime.hasTransport()) {
            try {
                downloadLifetime.delegate.clear();
            } catch (RuntimeException unavailable) {
                // Disable this optional capability, not the tester's normal START/safety work.
                downloadLifetime.available = false;
            }
        }
        return downloadLifetime.isActive()
                ? new TesterContext(hw, telemetry, gamepad1, gamepad2, clock, downloadLifetime)
                : null;
    }

    /** Revoke before cleanup so a stopped child cannot publish or clear a later child's result. */
    void revokeDownloads() {
        downloadLifetime.revoke();
    }

    /** Lifecycle-thread-only publication guard; HTTP never accesses this object or its owner. */
    private static final class DownloadLifetime {
        private final ResultDownloads delegate;
        private final DownloadLifetime parent;
        private boolean active = true;
        private boolean available = true;

        private DownloadLifetime(ResultDownloads delegate, DownloadLifetime parent) {
            this.delegate = delegate;
            this.parent = parent;
        }

        private boolean isActive() {
            return active && (parent == null || parent.isActive());
        }

        private boolean hasTransport() {
            return active && available && (parent == null || parent.hasTransport());
        }

        private void revoke() {
            boolean wasAvailable = hasTransport();
            active = false;
            if (wasAvailable) delegate.clear();
        }
    }

    /** A retained pre-START alias never regains permission through its owner's renewed lease. */
    private static final class DownloadLease implements ResultDownloads {
        private final DownloadLifetime owner;
        private boolean active = true;

        private DownloadLease(DownloadLifetime owner) {
            this.owner = owner;
        }

        private boolean isActive() {
            return active && owner.hasTransport();
        }

        @Override public boolean publish(String filename, String frozenUtf8Text) {
            return isActive() && owner.delegate.publish(filename, frozenUtf8Text);
        }

        @Override public String url() {
            return isActive() ? owner.delegate.url() : null;
        }

        @Override public void clear() {
            if (isActive()) owner.delegate.clear();
        }
    }
}
