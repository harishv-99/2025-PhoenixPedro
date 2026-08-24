package edu.ftcphoenix.fw.ftc;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

/**
 * FTC-boundary construction for explicit module-wide manual bulk-cache ownership.
 *
 * <p>This is an advanced, opt-in lifecycle policy. Declare the returned service first so it sets
 * {@link LynxModule.BulkCachingMode#MANUAL} before other services start, clears before their active
 * reads, and restores modes only after they stop:</p>
 *
 * <pre>{@code
 * program.service(FtcBulkCaching.manual(hardwareMap)); // first service
 * }</pre>
 *
 * <p>While the service is installed, it exclusively owns every discovered module's bulk-caching
 * mode and cache clearing. Other code must not call {@link LynxModule#setBulkCachingMode},
 * {@link LynxModule#clearBulkCache()}, or {@link LynxModule#getBulkData()} during that lifecycle.
 * Discovery happens during construction without reading a mode, changing a mode, clearing a cache,
 * or starting a bulk transaction. START snapshots every exact prior mode before setting all modules
 * to MANUAL and clearing them. In a claimed START/update cycle the service makes at most one
 * periodic owner-issued clear attempt per module, and exactly one each when that pass succeeds.
 * Terminal cleanup is a separate pass and may therefore issue another clear.</p>
 *
 * <p>After START reaches its mutation boundary, STOP makes one final clear pass and then attempts
 * to restore every captured mode. Stop before START has no module effect. Cleanup does not and
 * cannot restore an earlier cached packet or packet history. The owner makes no guarantee about
 * freshness, validity, physical sample coherence, transaction count, timing, or performance.
 * Cached eligible observations include digital/touch state, analog voltage, and motor position,
 * velocity, busy/at-target, and overcurrent state; motor-current and battery-voltage reads are not
 * served by the same bulk packet. SDK fake-packet/fault behavior can surface cached zero or false
 * values, while a cached motor busy result may be true, without a separate validity signal.</p>
 *
 * <p>Manual caching changes module-global observation and failure semantics and can conflict with
 * opaque vendor code, so ordinary programs leave the SDK mode untouched unless they explicitly
 * adopt and validate this owner on their robot.</p>
 */
public final class FtcBulkCaching {

    private FtcBulkCaching() {
        // utility class
    }

    /**
     * Discover all configured Lynx modules and create their one manual-cache lifecycle owner.
     *
     * <p>The factory defensively retains the discovered module order. Construction performs no
     * cache effect. The returned single-use service requires a non-null shared clock at START and
     * update, binds the exact START clock, rejects regressed cycles, and deduplicates repeated
     * successful updates in one cycle. A same-cycle failure is rethrown as the exact retained
     * exception; a later cycle retries its normal full clear pass. Stop before START has no module
     * effect and terminalizes the service.</p>
     *
     * <p>A failed or null START snapshot consumes the one START attempt without mutating a module.
     * Once the complete snapshot has made cleanup eligible, START stops at its first runtime
     * failure and the managed host's fail-stop path invokes the required cleanup. Active updates
     * attempt every module after ordinary runtime failures, retain the first failure, and suppress
     * later failures in module order. STOP claims terminal state before effects, then makes one
     * best-effort global clear pass followed by one best-effort exact-mode restoration pass.
     * Reentrant stop from an in-flight mutating callback defers that cleanup until the callback
     * returns, preventing the callback from writing after restoration.</p>
     *
     * @param hardwareMap FTC hardware map used only for module discovery during this call
     * @return a single-use service to declare first and own exclusively
     * @throws IllegalArgumentException if {@code hardwareMap} is {@code null}
     * @throws IllegalStateException if no Lynx module is configured
     */
    public static RobotProgram.Service manual(HardwareMap hardwareMap) {
        if (hardwareMap == null) {
            throw new IllegalArgumentException(
                    "FtcBulkCaching.manual(...) requires a non-null HardwareMap");
        }

        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
        if (modules.isEmpty()) {
            throw new IllegalStateException(
                    "FtcBulkCaching.manual(...) found no LynxModule; configure at least one "
                            + "LynxModule or leave the bulk-cache owner uninstalled");
        }

        List<FtcBulkCachingHub> hubs = new ArrayList<>(modules.size());
        for (LynxModule module : modules) {
            hubs.add(new LynxHubAdapter(module));
        }
        return new FtcManualBulkCachingService(hubs);
    }

    /** Private concrete FTC adapter; the package-private hub interface is the test seam. */
    private static final class LynxHubAdapter implements FtcBulkCachingHub {
        private final LynxModule module;

        private LynxHubAdapter(LynxModule module) {
            this.module = module;
        }

        @Override
        public LynxModule.BulkCachingMode readMode() {
            return module.getBulkCachingMode();
        }

        @Override
        public void setMode(LynxModule.BulkCachingMode mode) {
            module.setBulkCachingMode(mode);
        }

        @Override
        public void clearCache() {
            module.clearBulkCache();
        }
    }
}
