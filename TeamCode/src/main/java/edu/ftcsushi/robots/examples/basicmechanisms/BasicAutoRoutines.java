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
     * OPEN and STOWED in parallel. Each feedback prerequisite must succeed before later motion is
     * constructed. A timeout stays visible and does not quietly continue the routine.</p>
     *
     * @return fresh single-use mechanism-only root Task
     */
    public static Task guide(BasicLift lift, BasicClaw claw) {
        BasicLift requiredLift = Objects.requireNonNull(lift, "lift");
        BasicClaw requiredClaw = Objects.requireNonNull(claw, "claw");

        Task home = requiredLift.home();
        return BasicAutoSuccessGate.continueOnlyAfterSuccess(
                "Basic guide after home",
                home,
                () -> guideRaiseHigh(requiredLift, requiredClaw));
    }

    private static Task guideRaiseHigh(BasicLift lift, BasicClaw claw) {
        Task raiseHigh = lift.moveTo(BasicLift.Height.HIGH);
        return BasicAutoSuccessGate.continueOnlyAfterSuccess(
                "Basic guide after HIGH",
                raiseHigh,
                () -> guideLowerAndClose(lift, claw));
    }

    private static Task guideLowerAndClose(BasicLift lift, BasicClaw claw) {
        // The lift is the deadline so its exact terminal outcome remains authoritative while the
        // one-cycle claw request starts concurrently and then persists through the mechanism.
        Task lowerAndClose = Tasks.parallelDeadline(
                lift.moveTo(BasicLift.Height.LOW),
                requestClaw(claw, BasicClaw.State.CLOSED));
        return BasicAutoSuccessGate.continueOnlyAfterSuccess(
                "Basic guide after LOW and CLOSED",
                lowerAndClose,
                () -> guideHoldThenRelease(lift, claw));
    }

    private static Task guideHoldThenRelease(BasicLift lift, BasicClaw claw) {
        Task hold = Tasks.waitForSeconds(GUIDE_HOLD_SEC);
        return BasicAutoSuccessGate.continueOnlyAfterSuccess(
                "Basic guide after hold",
                hold,
                () -> Tasks.parallelDeadline(
                        lift.moveTo(BasicLift.Height.STOWED),
                        requestClaw(claw, BasicClaw.State.OPEN)));
    }

    /** Creates a fresh one-cycle semantic command Task. */
    private static Task requestClaw(BasicClaw claw, BasicClaw.State state) {
        return Tasks.runOnce(() -> claw.setState(state));
    }

}
