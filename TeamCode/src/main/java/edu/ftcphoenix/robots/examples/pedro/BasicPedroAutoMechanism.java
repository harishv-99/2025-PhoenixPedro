package edu.ftcphoenix.robots.examples.pedro;

import java.util.Objects;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PlantTasks;
import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.task.Task;

/**
 * Small intake capability used by the basic Pedro Auto reference.
 *
 * <p>The capability creates fresh Tasks while the retained Plant remains the only final actuator
 * owner. A real robot normally replaces this class with its existing mechanism capability.</p>
 */
public final class BasicPedroAutoMechanism {

    private static final double IDLE_POWER = 0.0;

    private final Plant intakePlant;
    private final double collectPower;
    private boolean stopped;

    /**
     * Creates the example capability around one writable normalized-power Plant.
     *
     * @param intakePlant source-driven Plant updated by the example composition root
     * @param collectPower finite collection request in {@code [-1, +1]}
     */
    public BasicPedroAutoMechanism(Plant intakePlant, double collectPower) {
        this.intakePlant = Objects.requireNonNull(intakePlant, "intakePlant");
        if (!intakePlant.hasWritableTarget()) {
            throw new IllegalArgumentException(
                    "BasicPedroAutoMechanism requires a Plant with a registered writable target"
            );
        }
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
        return PlantTasks.write(intakePlant)
                .to(collectPower)
                .forSeconds(durationSec)
                .then(IDLE_POWER)
                .build();
    }

    /** Creates a fresh immediate Task that restores the intake's safe idle request. */
    public Task idleTask() {
        return PlantTasks.setTarget(intakePlant, IDLE_POWER);
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
                () -> intakePlant.writableTarget().set(IDLE_POWER),
                intakePlant::stop
        );
    }
}
