package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

import edu.ftcsushi.fw.actuation.Plant;
import edu.ftcsushi.fw.actuation.SemanticScalarCommand;
import edu.ftcsushi.fw.actuation.SemanticScalarTasks;
import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.FtcActuators;
import edu.ftcsushi.fw.ftc.RobotProgram;
import edu.ftcsushi.fw.task.Task;

/** Owns the claw's semantic request, final resolver, standard-servo Plant, update, and stop. */
public final class BasicClawMechanism implements BasicClaw, RobotProgram.Output {

    /** Data-only standard-servo wiring and native endpoints for the named states. */
    public static final class Config {
        /** Nonblank FTC Robot Configuration name for the standard positional Servo. */
        public String servoName;

        /** Logical FTC Servo direction applied once during hardware construction. */
        public Direction direction;

        /**
         * Native FTC Servo endpoint mapped from normalized Plant target {@code 0.0}
         * ({@link State#CLOSED}), in inclusive range {@code [0, 1]}. This fact must be backed off
         * from unsafe travel and reviewed on the assembled mechanism.
         */
        public double closedNativePosition;

        /**
         * Native FTC Servo endpoint mapped from normalized Plant target {@code 1.0}
         * ({@link State#OPEN}), in inclusive range {@code [0, 1]}. This fact must be backed off
         * from unsafe travel and reviewed on the assembled mechanism.
         */
        public double openNativePosition;

        /**
         * Semantic request held from construction until a caller selects another state. With the
         * default {@link State#CLOSED} value, the first successful output heartbeat maps target
         * {@code 0.0} to {@link #closedNativePosition}; that initial Servo motion must be reviewed
         * physically.
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
            config.closedNativePosition = 0.25;
            config.openNativePosition = 0.70;
            config.initialState = State.CLOSED;
            return config;
        }
    }

    private static final double CLOSED_TARGET = 0.0;
    private static final double OPEN_TARGET = 1.0;

    private final Plant claw;
    private final SemanticScalarCommand<State> stateCommand;
    private double lastAppliedCoordinate = Double.NaN;
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

        stateCommand = SemanticScalarCommand.forEnum(c.initialState)
                .map(State.CLOSED, CLOSED_TARGET)
                .map(State.OPEN, OPEN_TARGET)
                .build();
        claw = FtcActuators.plant(map)
                .servo(c.servoName, c.direction)
                .position()
                .nonPeriodic()
                .bounded(CLOSED_TARGET, OPEN_TARGET)
                .rangeMapsToNative(c.closedNativePosition, c.openNativePosition)
                .targetExactlyFrom(stateCommand)
                .build();
    }

    @Override
    public void setState(State state) {
        stateCommand.set(Objects.requireNonNull(state, "state"));
    }

    @Override
    public Task setStateTask(State state) {
        return SemanticScalarTasks.set(stateCommand, Objects.requireNonNull(state, "state"))
                .build();
    }

    @Override
    public Status status() {
        return new Status(stateCommand.request().semantic(), lastAppliedCoordinate);
    }

    /** Applies the held command through the one final Plant path. */
    @Override
    public void update(LoopClock clock) {
        if (stopped) {
            return;
        }
        claw.update(clock);
        // Publish normalized target evidence only after the complete Plant heartbeat succeeds.
        lastAppliedCoordinate = claw.getAppliedTarget();
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
            // A terminally stopped Plant cannot publish new applied-target evidence.
            stopped = true;
        }
    }

    private static Config copyAndValidate(Config source) {
        Config s = Objects.requireNonNull(source, "BasicClawMechanism.Config is required");
        Config c = new Config();
        c.servoName = hardwareName(s.servoName);
        c.direction = Objects.requireNonNull(s.direction, "direction is required");
        c.closedNativePosition = servoPosition(
                s.closedNativePosition, "closedNativePosition");
        c.openNativePosition = servoPosition(s.openNativePosition, "openNativePosition");
        c.initialState = Objects.requireNonNull(s.initialState, "initialState is required");
        if (Double.compare(c.closedNativePosition, c.openNativePosition) == 0) {
            throw new IllegalArgumentException(
                    "closedNativePosition and openNativePosition must be distinct so the named states "
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
