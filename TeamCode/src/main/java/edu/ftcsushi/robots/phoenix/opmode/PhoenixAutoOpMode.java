package edu.ftcsushi.robots.phoenix.opmode;

import java.util.Objects;

import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.ftc.RobotProgram;

/**
 * One managed FTC host for every production and diagnostic Phoenix autonomous entry.
 *
 * <p>A concrete Driver Station entry supplies only a {@link PhoenixAutoSetup}. This base performs
 * no FTC callback override other than the framework's one declarative configuration hook; the
 * inherited final callbacks own INIT, START, loop, STOP, failure cleanup, and telemetry commits.</p>
 */
public abstract class PhoenixAutoOpMode extends FtcRobotOpMode {

    /** Return the fixed or INIT-selectable data specification for this entry. */
    protected abstract PhoenixAutoSetup autoSetup();

    /** Declare the complete Phoenix/Pedro graph once during FTC INIT. */
    @Override
    protected final void configure(RobotProgram program) {
        new PhoenixAutoProgram(
                this,
                Objects.requireNonNull(program, "program"),
                Objects.requireNonNull(autoSetup(), "autoSetup()")
        );
    }
}
