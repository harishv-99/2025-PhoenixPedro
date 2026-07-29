package edu.ftcphoenix.robots.examples.pedro;

import java.util.Objects;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.ScalarTasks;
import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.Task;

/**
 * Small intake capability used by the basic Pedro Auto reference.
 *
 * <p>The capability creates fresh Tasks while the retained Plant remains the only final actuator
 * owner. Immediate cleanup and deferred Tasks use the Plant's stable
 * {@link Plant#commandTarget() graph-owned command}, so both change the same request. A real robot
 * normally replaces this class with its existing mechanism capability. This reference deliberately
 * uses a completed-Plant constructor as a hardware-neutral portable-host seam. Ordinary FTC
 * mechanisms instead receive {@code HardwareMap} plus their data-only config and construct their
 * private Plants internally.</p>
 */
public final class BasicPedroAutoMechanism {

    private static final double IDLE_POWER = 0.0;

    private final Plant intakePlant;
    private final double collectPower;
    private boolean stopped;

    /**
     * Creates the portable example capability around one normalized-power Plant with a command
     * target. This is an explicitly hardware-neutral adapter seam, not the ordinary FTC mechanism
     * construction pattern.
     *
     * @param intakePlant source-driven Plant whose update/stop lifecycle this mechanism owns
     * @param collectPower finite collection request in {@code [-1, +1]}
     * @throws NullPointerException if the Plant is null or violates its command-target contract
     * @throws IllegalArgumentException if the Plant has no command target or the power is invalid
     */
    public BasicPedroAutoMechanism(Plant intakePlant, double collectPower) {
        this.intakePlant = Objects.requireNonNull(intakePlant, "intakePlant");
        if (!intakePlant.hasCommandTarget()) {
            throw new IllegalArgumentException(
                    "BasicPedroAutoMechanism requires intakePlant to have a command target"
            );
        }
        Objects.requireNonNull(
                intakePlant.commandTarget(),
                "intakePlant.commandTarget()"
        );
        if (!Double.isFinite(collectPower) || collectPower < -1.0 || collectPower > 1.0) {
            throw new IllegalArgumentException(
                    "collectPower must be finite and in [-1, +1], got " + collectPower
            );
        }
        this.collectPower = collectPower;
    }

    /**
     * Creates a fresh Task that collects for the requested duration and returns to idle.
     *
     * <p>Active cancellation also writes the idle request before the Task becomes terminal.</p>
     */
    public Task collectTask(double durationSec) {
        return ScalarTasks.set(intakePlant.commandTarget(), collectPower)
                .forSeconds(durationSec)
                .then(IDLE_POWER)
                .build();
    }

    /** Creates a fresh write-once Task that restores the safe idle request when started. */
    public Task idleTask() {
        return ScalarTasks.set(intakePlant.commandTarget(), IDLE_POWER).build();
    }

    /** Applies the capability's final source-driven Plant target for this loop. */
    public void update(LoopClock clock) {
        if (!stopped) {
            intakePlant.update(clock);
        }
    }

    /**
     * Restores the idle request and immediately stops the Plant, attempting both exactly once.
     */
    public void stop() {
        if (stopped) {
            return;
        }
        stopped = true;

        CleanupActions.attemptAll(
                () -> intakePlant.commandTarget().set(IDLE_POWER),
                intakePlant::stop
        );
    }
}
