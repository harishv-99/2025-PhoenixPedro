package edu.ftcphoenix.fw.ftc;

import com.qualcomm.hardware.lynx.LynxModule;

import java.util.ArrayList;
import java.util.List;

import edu.ftcphoenix.fw.core.time.LoopClock;

/** Package-private lifecycle owner behind {@link FtcBulkCaching#manual}. */
final class FtcManualBulkCachingService implements RobotProgram.Service {

    private static final String OWNER = "FtcBulkCaching.manual(...) service";

    private enum CycleResult {
        NONE,
        IN_PROGRESS,
        SUCCEEDED,
        FAILED
    }

    private enum HubOperation {
        READ_MODE,
        SET_MANUAL,
        CLEAR_CACHE
    }

    private final FtcBulkCachingHub[] hubs;
    private final LynxModule.BulkCachingMode[] priorModes;

    private boolean startAttempted;
    private LoopClock boundClock;
    private boolean active;
    private boolean terminal;

    private boolean hubCallbackInProgress;
    private boolean cleanupEligible;
    private boolean cleanupPassClaimed;
    private boolean cleanupPassStarted;

    private boolean updateInProgress;
    private boolean cycleClaimed;
    private long claimedCycle;
    private CycleResult cycleResult = CycleResult.NONE;
    private RuntimeException retainedCycleFailure;

    FtcManualBulkCachingService(List<? extends FtcBulkCachingHub> hubs) {
        if (hubs == null) {
            throw new IllegalArgumentException(OWNER + " requires a hub list");
        }

        List<FtcBulkCachingHub> copy = new ArrayList<>(hubs.size());
        for (int index = 0; index < hubs.size(); index++) {
            FtcBulkCachingHub hub = hubs.get(index);
            if (hub == null) {
                throw new IllegalArgumentException(
                        OWNER + " requires a non-null hub at index " + index);
            }
            copy.add(hub);
        }
        if (copy.isEmpty()) {
            throw new IllegalStateException(OWNER + " requires at least one LynxModule");
        }

        this.hubs = copy.toArray(new FtcBulkCachingHub[0]);
        this.priorModes = new LynxModule.BulkCachingMode[this.hubs.length];
    }

    @Override
    public void start(LoopClock clock) {
        requireStartClock(clock);
        if (terminal) {
            throw lifecycleFailure(
                    "cannot START after stop(); construct and declare a fresh " + OWNER);
        }
        if (startAttempted) {
            throw lifecycleFailure(
                    "allows one START attempt; construct and declare a fresh " + OWNER
                            + " instead of retrying START");
        }

        startAttempted = true;
        boundClock = clock;

        for (int index = 0; index < hubs.length; index++) {
            LynxModule.BulkCachingMode mode = callHub(
                    hubs[index],
                    HubOperation.READ_MODE,
                    null
            );
            if (mode == null) {
                throw lifecycleFailure(
                        "START received a null bulk-caching mode from LynxModule index "
                                + index + "; repair the FTC hub configuration before retrying "
                                + "with a fresh service");
            }
            priorModes[index] = mode;
        }

        cleanupEligible = true;
        claimCycle(clock.cycle());

        try {
            for (FtcBulkCachingHub hub : hubs) {
                callHub(hub, HubOperation.SET_MANUAL, null);
            }
            for (FtcBulkCachingHub hub : hubs) {
                callHub(hub, HubOperation.CLEAR_CACHE, null);
            }
        } catch (RuntimeException failure) {
            retainCycleFailure(failure);
            throw failure;
        }

        cycleResult = CycleResult.SUCCEEDED;
        active = true;
    }

    @Override
    public void update(LoopClock clock) {
        requireUpdateClock(clock);
        if (terminal) {
            throw lifecycleFailure(
                    "cannot update after stop(); construct and declare a fresh " + OWNER);
        }
        if (!active) {
            throw lifecycleFailure(
                    "cannot update before a successful START; let RobotProgram own its "
                            + "managed lifecycle");
        }
        if (clock != boundClock) {
            throw lifecycleFailure(
                    "must update with the exact LoopClock supplied at START; use the managed "
                            + "RobotProgram clock");
        }

        long cycle = clock.cycle();
        if (cycleClaimed && cycle < claimedCycle) {
            throw lifecycleFailure(
                    "cannot update with a regressed LoopClock cycle (received " + cycle
                            + " after " + claimedCycle + "); advance only the managed clock");
        }
        if (updateInProgress) {
            throw lifecycleFailure(
                    "does not allow reentrant update(); return from the current hub callback "
                            + "before the next managed cycle");
        }
        if (cycleClaimed && cycle == claimedCycle) {
            if (cycleResult == CycleResult.SUCCEEDED) {
                return;
            }
            if (cycleResult == CycleResult.FAILED) {
                throw retainedCycleFailure;
            }
            throw lifecycleFailure(
                    "does not allow a second update while cycle " + cycle
                            + " is still in progress; avoid reentrant lifecycle calls");
        }

        claimCycle(cycle);
        updateInProgress = true;
        RuntimeException failure = null;
        try {
            for (FtcBulkCachingHub hub : hubs) {
                try {
                    RuntimeException precedingFailure = failure;
                    callHub(hub, HubOperation.CLEAR_CACHE, precedingFailure);
                } catch (RuntimeException hubFailure) {
                    if (terminal) {
                        retainCycleFailure(hubFailure);
                        throw hubFailure;
                    }
                    failure = mergeFailure(failure, hubFailure);
                }
            }

            if (failure != null) {
                retainCycleFailure(failure);
                throw failure;
            }
            cycleResult = CycleResult.SUCCEEDED;
        } finally {
            updateInProgress = false;
        }
    }

    @Override
    public void stop() {
        if (terminal) {
            return;
        }

        terminal = true;
        active = false;
        if (!cleanupEligible) {
            return;
        }

        cleanupPassClaimed = true;
        if (hubCallbackInProgress) {
            return;
        }

        RuntimeException failure = performClaimedCleanup(null);
        if (failure != null) {
            throw failure;
        }
    }

    private void requireStartClock(LoopClock clock) {
        if (clock == null) {
            throw new IllegalArgumentException(
                    OWNER + " START requires a non-null LoopClock; use RobotProgram's managed "
                            + "service lifecycle");
        }
    }

    private void requireUpdateClock(LoopClock clock) {
        if (clock == null) {
            throw new IllegalArgumentException(
                    OWNER + " update requires a non-null LoopClock; use RobotProgram's managed "
                            + "service lifecycle");
        }
    }

    private void claimCycle(long cycle) {
        cycleClaimed = true;
        claimedCycle = cycle;
        cycleResult = CycleResult.IN_PROGRESS;
        retainedCycleFailure = null;
    }

    private void retainCycleFailure(RuntimeException failure) {
        cycleResult = CycleResult.FAILED;
        retainedCycleFailure = failure;
    }

    private LynxModule.BulkCachingMode callHub(
            FtcBulkCachingHub hub,
            HubOperation operation,
            RuntimeException precedingFailure) {
        hubCallbackInProgress = true;
        LynxModule.BulkCachingMode result = null;
        RuntimeException callbackFailure = null;
        try {
            switch (operation) {
                case READ_MODE:
                    result = hub.readMode();
                    break;
                case SET_MANUAL:
                    hub.setMode(LynxModule.BulkCachingMode.MANUAL);
                    break;
                case CLEAR_CACHE:
                    hub.clearCache();
                    break;
                default:
                    throw new AssertionError("Unhandled bulk-cache hub operation " + operation);
            }
        } catch (RuntimeException failure) {
            callbackFailure = failure;
        } finally {
            hubCallbackInProgress = false;
        }

        if (terminal) {
            RuntimeException failure = mergeFailure(precedingFailure, callbackFailure);
            if (cleanupPassClaimed) {
                failure = performClaimedCleanup(failure);
            }
            if (failure != null) {
                throw failure;
            }
            throw lifecycleFailure(
                    "was stopped reentrantly from a hub callback; do not call stop() from "
                            + "bulk-cache operations");
        }
        if (callbackFailure != null) {
            throw callbackFailure;
        }
        return result;
    }

    private RuntimeException performClaimedCleanup(RuntimeException precedingFailure) {
        if (!cleanupPassClaimed || cleanupPassStarted) {
            return precedingFailure;
        }

        cleanupPassStarted = true;
        RuntimeException failure = precedingFailure;
        for (FtcBulkCachingHub hub : hubs) {
            try {
                hub.clearCache();
            } catch (RuntimeException cleanupFailure) {
                failure = mergeFailure(failure, cleanupFailure);
            }
        }
        for (int index = 0; index < hubs.length; index++) {
            try {
                hubs[index].setMode(priorModes[index]);
            } catch (RuntimeException cleanupFailure) {
                failure = mergeFailure(failure, cleanupFailure);
            }
        }
        return failure;
    }

    private static RuntimeException mergeFailure(
            RuntimeException primary,
            RuntimeException later) {
        if (later == null) {
            return primary;
        }
        if (primary == null) {
            return later;
        }
        if (primary != later) {
            primary.addSuppressed(later);
        }
        return primary;
    }

    private static IllegalStateException lifecycleFailure(String detail) {
        return new IllegalStateException(OWNER + " " + detail);
    }
}
