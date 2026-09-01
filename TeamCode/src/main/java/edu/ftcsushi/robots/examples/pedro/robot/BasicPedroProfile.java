package edu.ftcsushi.robots.examples.pedro.robot;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import edu.ftcsushi.fw.integrations.pedro.PedroPathingRuntime;
import edu.ftcsushi.robots.examples.pedro.capability.intake.BasicPedroAutoMechanism;

/**
 * Data-only configuration for the independent basic Pedro Auto reference.
 *
 * <p>{@link #current()} returns a fresh, complete software baseline, not reviewed drivetrain
 * wiring, Pinpoint placement, follower tuning, route clearance, intake safety, or physical STOP.
 * Ordinary managed Sushi route callers cannot currently set Pedro's persistent Follower power
 * limit. Keep {@link #allowRobotMotion} false for physical use until a focused integration
 * improvement supplies that control and every active fact has been reviewed.</p>
 */
public final class BasicPedroProfile {

    /** Complete Pedro runtime configuration for this example robot. */
    public PedroPathingRuntime.Config pedro;

    /** Wiring and collection configuration for the example intake owner. */
    public BasicPedroAutoMechanism.Config intake;

    /** Software gate that must remain false for physical use under the current route API. */
    public boolean allowRobotMotion;

    private BasicPedroProfile() {
        // Use current() so the complete student-reviewed example remains in one place.
    }

    /**
     * Returns a fresh, complete basic Pedro profile with explicit software example values.
     *
     * <p>The runtime starts from its owner-provided defaults, then authors only this example's
     * motor identities, directions, initial drivetrain setting, and brake choice. Pedro restores
     * its separate route-following power when a path starts, so {@code maxPower=0.25} is an initial
     * drivetrain value rather than a durable route-speed limit. The ordinary managed route API
     * does not expose that persistent limit, so physical motion remains blocked pending a focused
     * integration improvement and complete physical review.</p>
     *
     * @return independent mutable runtime and intake configuration graphs with motion disabled
     */
    public static BasicPedroProfile current() {
        BasicPedroProfile profile = new BasicPedroProfile();

        profile.pedro = PedroPathingRuntime.Config.defaults();
        profile.pedro.mecanumConstants.leftFrontMotorName = "frontLeftMotor";
        profile.pedro.mecanumConstants.leftRearMotorName = "backLeftMotor";
        profile.pedro.mecanumConstants.rightFrontMotorName = "frontRightMotor";
        profile.pedro.mecanumConstants.rightRearMotorName = "backRightMotor";
        profile.pedro.mecanumConstants.leftFrontMotorDirection =
                DcMotorSimple.Direction.REVERSE;
        profile.pedro.mecanumConstants.leftRearMotorDirection =
                DcMotorSimple.Direction.REVERSE;
        profile.pedro.mecanumConstants.rightFrontMotorDirection =
                DcMotorSimple.Direction.FORWARD;
        profile.pedro.mecanumConstants.rightRearMotorDirection =
                DcMotorSimple.Direction.FORWARD;
        profile.pedro.mecanumConstants.maxPower = 0.25;
        profile.pedro.mecanumConstants.useBrakeModeInTeleOp = true;

        profile.intake = BasicPedroAutoMechanism.Config.defaults();
        profile.allowRobotMotion = false;
        return profile;
    }
}
