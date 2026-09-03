package edu.ftcsushi.robots.examples.basicmechanisms;

import java.util.Objects;

import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.Tasks;

/** Auto policy composed entirely from the same semantic lift and claw APIs used in TeleOp. */
public final class BasicAutoRoutines {

    private static final double GUIDE_HOLD_SEC = 0.50;

    private BasicAutoRoutines() {
        // Static robot-owned routine factories.
    }

    /**
     * Builds the course's mechanism-only command-group lesson.
     *
     * <p>The exact sequence is home, HIGH, LOW and CLOSED in parallel, hold for 0.5 seconds, then
     * OPEN and STOWED in parallel. The fixed graph calls these side-effect-free capability Task
     * factories eagerly, but each feedback prerequisite must succeed before the next Task starts.
     * A timeout stays visible and does not quietly continue the routine.</p>
     *
     * @return fresh single-use mechanism-only root Task
     */
    public static Task guide(BasicLift lift, BasicClaw claw) {
        BasicLift requiredLift = Objects.requireNonNull(lift, "lift");
        BasicClaw requiredClaw = Objects.requireNonNull(claw, "claw");

        return Tasks.sequence(
                requiredLift.home(),
                requiredLift.moveTo(BasicLift.Height.HIGH),
                // The lift is the deadline, while the write-once claw request starts concurrently
                // and remains held by the mechanism after that companion Task succeeds.
                Tasks.parallelDeadline(
                        requiredLift.moveTo(BasicLift.Height.LOW),
                        requiredClaw.setStateTask(BasicClaw.State.CLOSED)),
                Tasks.waitForSeconds(GUIDE_HOLD_SEC),
                Tasks.parallelDeadline(
                        requiredLift.moveTo(BasicLift.Height.STOWED),
                        requiredClaw.setStateTask(BasicClaw.State.OPEN)));
    }

}
