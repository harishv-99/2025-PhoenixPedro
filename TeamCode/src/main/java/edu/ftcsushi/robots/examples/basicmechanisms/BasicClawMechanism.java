package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.FtcActuators;
import edu.ftcsushi.fw.ftc.RobotProgram;

/** Owns the claw's semantic request, final resolver, standard-servo Plant, update, and stop. */
public final class BasicClawMechanism implements BasicClaw, RobotProgram.Output {

    /** Data-only standard-servo wiring and named-state positions. */
    public static final class Config {
        /** Nonblank FTC Robot Configuration name for the standard positional Servo. */
        public String servoName;

        /** Logical FTC Servo direction applied once during hardware construction. */
        public Direction direction;

        /** Native Servo command for {@link State#CLOSED}, in inclusive range {@code [0, 1]}. */
        public double closedPosition;

        /** Native Servo command for {@link State#OPEN}, in inclusive range {@code [0, 1]}. */
        public double openPosition;

        /**
         * Semantic request held from construction until a caller selects another state. With the
         * default {@link State#CLOSED} value, the first successful output heartbeat submits
         * {@link #closedPosition}; that initial Servo motion must be reviewed physically.
         */
        public State initialState;

        private Config() {
            // Start from defaults() so the whole software recipe is present.
        }

        /** Returns a complete software baseline, not reviewed physical endpoints. */
        public static Config defaults() {
            Config config = new Config();
            config.servoName = "clawServo";
            config.direction = Direction.FORWARD;
            config.closedPosition = 0.25;
            config.openPosition = 0.70;
            config.initialState = State.CLOSED;
            return config;
        }
    }

    private final Plant claw;
    private final double closedPosition;
    private final double openPosition;
    private State requestedState;
    private double lastAppliedPosition = Double.NaN;
    private boolean stopped;

    /**
     * Validates a defensive configuration snapshot, then privately constructs the servo Plant.
     *
     * @param hardwareMap FTC hardware registry
     * @param config complete data-only claw configuration
     */
    public BasicClawMechanism(HardwareMap hardwareMap, Config config) {
        HardwareMap map = Objects.requireNonNull(hardwareMap, "hardwareMap is required");
        Config c = copyAndValidate(config);

        closedPosition = c.closedPosition;
        openPosition = c.openPosition;
        requestedState = c.initialState;
        claw = FtcActuators.plant(map)
                .servo(c.servoName, c.direction)
                .position()
                .nonPeriodic()
                .bounded(0.0, 1.0)
                .nativeUnits()
                .targetFromNewCommand(positionFor(c.initialState))
                .build();
    }

    @Override
    public void setState(State state) {
        State requested = Objects.requireNonNull(state, "state");
        claw.commandTarget().set(positionFor(requested));
        requestedState = requested;
    }

    @Override
    public Status status() {
        return new Status(requestedState, lastAppliedPosition);
    }

    /** Applies the held command through the one final Plant path. */
    @Override
    public void update(LoopClock clock) {
        if (stopped) {
            return;
        }
        claw.update(clock);
        // Publish command evidence only after the complete Plant heartbeat succeeds.
        lastAppliedPosition = claw.getAppliedTarget();
    }

    /**
     * Stops the Plant lifecycle without inventing a new OPEN or CLOSED request.
     * The standard servo therefore retains its last physically submitted position command.
     */
    @Override
    public void stop() {
        try {
            claw.stop();
        } finally {
            // A terminally stopped Plant cannot publish new applied-command evidence.
            stopped = true;
        }
    }

    private double positionFor(State state) {
        return state == State.OPEN ? openPosition : closedPosition;
    }

    private static Config copyAndValidate(Config source) {
        Config s = Objects.requireNonNull(source, "BasicClawMechanism.Config is required");
        Config c = new Config();
        c.servoName = hardwareName(s.servoName);
        c.direction = Objects.requireNonNull(s.direction, "direction is required");
        c.closedPosition = servoPosition(s.closedPosition, "closedPosition");
        c.openPosition = servoPosition(s.openPosition, "openPosition");
        c.initialState = Objects.requireNonNull(s.initialState, "initialState is required");
        if (Double.compare(c.closedPosition, c.openPosition) == 0) {
            throw new IllegalArgumentException(
                    "closedPosition and openPosition must be distinct so the named states "
                            + "produce different commands");
        }
        return c;
    }

    private static String hardwareName(String value) {
        if (value == null || value.trim().isEmpty()) {
            throw new IllegalArgumentException(
                    "BasicClawMechanism.Config.servoName must be a non-blank FTC hardware name");
        }
        return value;
    }

    private static double servoPosition(double value, String field) {
        if (!Double.isFinite(value) || value < 0.0 || value > 1.0) {
            throw new IllegalArgumentException(
                    field + " must be finite and in [0, 1], got " + value);
        }
        return value;
    }
}
