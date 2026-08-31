package edu.ftcsushi.robots.examples.reference.robot;

import java.util.Objects;

import edu.ftcsushi.robots.examples.reference.capability.launcher.ReferenceLauncher;
import edu.ftcsushi.robots.examples.reference.capability.lift.ReferenceLift;

/** Mode-neutral Reference capability families handed from composition to a mode client. */
public final class ReferenceCapabilities {
    private final ReferenceLift lift;
    private final ReferenceLauncher launcher;

    ReferenceCapabilities(ReferenceLift lift, ReferenceLauncher launcher) {
        this.lift = Objects.requireNonNull(lift, "lift");
        this.launcher = Objects.requireNonNull(launcher, "launcher");
    }

    /** Returns the shared lift vocabulary used by TeleOp and Auto. */
    public ReferenceLift lift() {
        return lift;
    }

    /** Returns the shared launcher vocabulary used by TeleOp and Auto. */
    public ReferenceLauncher launcher() {
        return launcher;
    }
}
