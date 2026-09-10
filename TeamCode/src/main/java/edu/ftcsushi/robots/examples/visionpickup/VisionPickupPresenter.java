package edu.ftcsushi.robots.examples.visionpickup;

import java.util.Objects;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.RobotProgram;

/**
 * Required additive driver display distinguishing accepted alignment from failed assistance.
 * Reads one cached pickup snapshot without evaluating guidance, requesting pulses, changing robot
 * state, clearing another presenter's rows, or committing the shared telemetry frame.
 */
public final class VisionPickupPresenter implements RobotProgram.Presenter {
    private final VisionPickup pickup;

    /** Retains the already-constructed example capability; construction samples no source. */
    public VisionPickupPresenter(VisionPickup pickup) {
        this.pickup = Objects.requireNonNull(pickup, "pickup");
    }

    /** Adds the current assist meaning and the independently retained bounded pickup result. */
    @Override
    public void present(LoopClock clock, Telemetry telemetry) {
        VisionPickup.Status snapshot = pickup.status();
        telemetry.addData("assist", snapshot.hasFailure ? "Fault - stopped" : label(snapshot.assistState));
        telemetry.addData("assist.control", controlLabel(snapshot.assistState));
        telemetry.addData("assist.reason", snapshot.assistReason);
        if (snapshot.assistState == VisionPickup.AssistState.LOST) {
            telemetry.addData("assist.retry",
                    "Aim: release/repress. Pickup: new press. No automatic retry.");
        }
        telemetry.addData("pickup.phase", snapshot.phase);
        telemetry.addData("pickup.outcome", snapshot.hasFailure
                ? "LIFECYCLE FAILURE - no normal outcome" : snapshot.outcome);
        telemetry.addData("pickup.reason", snapshot.reason);
    }

    /** Formats only the capability's explicit state, never inferred zero motion or Task outcome. */
    private static String label(VisionPickup.AssistState state) {
        switch (state) {
            case REQUESTED: return "Aim requested - evidence not yet checked";
            case AIMING: return "Aiming - assistance active";
            case ALIGNED: return "Aligned - assistance active";
            case LOST: return "Assist lost - not completed";
            case PICKUP: return "Pickup active - automatic motion";
            case STOPPED: return "Stopped";
            case IDLE: return "Assistance off";
            default: throw new IllegalArgumentException("unrecognized assist state: " + state);
        }
    }

    /** States selected command ownership for both clients without guessing which mode is running. */
    private static String controlLabel(VisionPickup.AssistState state) {
        switch (state) {
            case AIMING:
            case ALIGNED: return "Driver translation; assisted turning";
            case PICKUP: return "Pickup owns drive";
            case STOPPED: return "Zero drive requested";
            case IDLE:
            case REQUESTED:
            case LOST: return "Manual control in TeleOp / zero idle in Auto";
            default: throw new IllegalArgumentException("unrecognized assist state: " + state);
        }
    }
}
