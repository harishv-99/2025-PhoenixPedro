package edu.ftcphoenix.fw.tools.tester;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.SortedSet;
import java.util.function.Consumer;

import edu.ftcphoenix.fw.core.lifecycle.CleanupActions;
import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.ftc.ui.MenuItem;
import edu.ftcphoenix.fw.ftc.ui.SelectionMenu;
import edu.ftcphoenix.fw.input.binding.Bindings;

/**
 * Device-first actuator direction and safe-endpoint bring-up wizard.
 *
 * <p>The ordinary public entry point is {@link StandardTesters#createActuatorBringUp()}. Keeping
 * this implementation package-private prevents the implementation class from becoming a second
 * student-facing construction path.</p>
 *
 * <h2>Truthful evidence</h2>
 * <p>This tester submits deliberately small, dead-man commands and records values that a student
 * has physically judged useful. It does not detect a hard stop, prove clearance, edit robot
 * configuration, reset a motor encoder, or treat {@link Servo#getPosition()} as shaft feedback.
 * Motor endpoint readings are FTC direction-adjusted encoder ticks. Standard-servo endpoints are
 * logical {@code setPosition(...)} submissions whose API calls returned normally; that is not
 * proof of physical delivery.</p>
 *
 * <h2>Lifecycle and safety</h2>
 * <p>INIT may enumerate, resolve, and inspect a selected device, but performs no actuator write.
 * In RUN, a fresh A press first prepares and arms a device with known command state. A standard
 * servo whose {@code getPosition()} returns the documented unknown-state NaN instead requires one A
 * press to acknowledge the physical-clearance warning and a second fresh A press to submit SDK
 * command 0.5 and arm. Motor and CR-servo movement requires
 * exactly one held bumper after the bumpers have been observed neutral. Releasing the bumper,
 * holding both bumpers, B, BACK, STOP, a transition, or a runtime failure writes zero to a motor
 * or CR servo. A standard servo has no zero-power command, so those actions stop new jog writes and
 * retain its last request; they do not prove that the shaft has stopped moving.</p>
 */
final class ActuatorBringUpTester extends BaseTeleOpTester {

    static final String LOG_TAG = "PhoenixActuatorBringUp";

    private final Consumer<String> resultLogSink;

    private static final double POWER_MIN = 0.05;
    private static final double POWER_MAX = 0.30;
    private static final double POWER_STEP = 0.05;

    private static final double SERVO_JOG_RATE_INITIAL_PER_SEC = 0.05;
    private static final double SERVO_JOG_RATE_MIN_PER_SEC = 0.01;
    private static final double SERVO_JOG_RATE_MAX_PER_SEC = 0.25;
    private static final double SERVO_JOG_RATE_STEP_PER_SEC = 0.01;
    private static final double SERVO_MAX_STEP_PER_CYCLE = 0.005;
    private static final double SERVO_BOOTSTRAP_COMMAND = 0.5;

    private enum Screen {
        PICKER,
        ACTIVE,
        RESULT
    }

    private enum DeviceKind {
        DC_MOTOR("DC motor"),
        CR_SERVO("CR servo"),
        SERVO("Servo");

        final String label;

        DeviceKind(String label) {
            this.label = label;
        }
    }

    private static final class DeviceChoice {
        final DeviceKind kind;
        final String name;

        DeviceChoice(DeviceKind kind, String name) {
            this.kind = kind;
            this.name = name;
        }

        String stableId() {
            return kind.name() + ":" + name;
        }

        String displayLabel() {
            return "[" + kind.label + "] " + name;
        }
    }

    private final SelectionMenu<DeviceChoice> picker = new SelectionMenu<DeviceChoice>()
            .setTitle("Actuator Bring-up")
            .setHelp("Dpad: select | A: choose | X: refresh | BACK: exit")
            .setEmptyMessage("No configured DC motors, CR servos, or standard servos found.")
            .setMaxVisibleItems(10);

    private Screen screen = Screen.PICKER;
    private boolean opModeStarted;

    private DeviceChoice selected;
    private DcMotor motor;
    private CRServo crServo;
    private Servo servo;

    private DcMotor.RunMode originalMotorMode;
    private DcMotorSimple.Direction originalMotorDirection;
    private DcMotor.ZeroPowerBehavior originalMotorZeroPowerBehavior;
    private DcMotorSimple.Direction motorDirection;

    private DcMotorSimple.Direction originalCrServoDirection;
    private DcMotorSimple.Direction crServoDirection;

    private Servo.Direction originalServoDirection;
    private Servo.Direction servoDirection;
    private double servoLastSuccessfulCommand;
    private boolean servoCommandSubmittedInRun;
    private boolean servoBootstrapRequired;
    private boolean servoBootstrapConfirmed;

    private boolean prepared;
    private boolean hardwareTouched;
    private boolean cleanupAttempted;
    private RuntimeException retainedCleanupFailure;
    private boolean armed;
    private boolean negativeHeld;
    private boolean positiveHeld;
    private boolean motionNeutralSeen;
    private double testPower = POWER_MIN;
    private double servoJogRatePerSec = SERVO_JOG_RATE_INITIAL_PER_SEC;
    private double lastSubmittedPower;
    private boolean powerCommandSubmitted;
    private boolean nonzeroJogSubmitted;

    private boolean plantMinCaptured;
    private boolean plantMaxCaptured;
    private long motorNativeAtPlantMinTicks;
    private long motorNativeAtPlantMaxTicks;
    private double servoNativeAtPlantMin;
    private double servoNativeAtPlantMax;

    private String pickerNotice;
    private String activeNotice;
    private List<String> resultLines = Collections.emptyList();
    private String resultLog;

    ActuatorBringUpTester() {
        this(message -> RobotLog.ii(LOG_TAG, message));
    }

    /** Package-private deterministic seam for exactly-once result logging tests. */
    ActuatorBringUpTester(Consumer<String> resultLogSink) {
        this.resultLogSink = Objects.requireNonNull(resultLogSink, "resultLogSink");
    }

    @Override
    public String name() {
        return "Actuator Bring-up";
    }

    @Override
    protected void onInit() {
        screen = Screen.PICKER;
        opModeStarted = false;
        refreshPicker();

        Bindings.ControlContext pickerControls = bindings.contextWhen(
                BooleanSource.of(() -> screen == Screen.PICKER),
                Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL);
        pickerControls.onRise(gamepads.p1().dpadUp(), picker::up);
        pickerControls.onRise(gamepads.p1().dpadDown(), picker::down);
        pickerControls.onRise(gamepads.p1().a(), () -> picker.selectCurrent(
                item -> selectDevice(item.value)));
        pickerControls.onRise(gamepads.p1().x(), this::refreshPicker);

        Bindings.ControlContext activeControls = bindings.contextWhen(
                BooleanSource.of(() -> screen == Screen.ACTIVE && opModeStarted),
                Bindings.ActivationPolicy.REARM_AFTER_NEUTRAL);

        // Declaration order deliberately makes a simultaneous A+B resolve to disarmed and safe.
        activeControls.onRise(gamepads.p1().a(), this::armOrRearm);
        activeControls.onRise(gamepads.p1().b(), this::disarmNow);
        activeControls.onRise(gamepads.p1().x(), this::toggleDirection);
        activeControls.onRise(gamepads.p1().dpadUp(), () -> adjustRate(true));
        activeControls.onRise(gamepads.p1().dpadDown(), () -> adjustRate(false));
        activeControls.onRise(gamepads.p1().dpadLeft(), () -> captureEndpoint(true));
        activeControls.onRise(gamepads.p1().dpadRight(), () -> captureEndpoint(false));
        activeControls.onRise(gamepads.p1().start(), this::clearCapturesWhenSafe);
        activeControls.onRise(gamepads.p1().y(), this::finalizeResult);

        // Context mirrors, rather than direct gamepad reads, ensure held transition inputs cannot
        // leak into motion. The realization below owns conflict handling and the final write.
        activeControls.mirrorOnChange(
                gamepads.p1().leftBumper(), value -> negativeHeld = value);
        activeControls.mirrorOnChange(
                gamepads.p1().rightBumper(), value -> positiveHeld = value);
    }

    @Override
    protected void onInitLoop(double dtSec) {
        renderCurrentScreen();
    }

    @Override
    protected void onStart() {
        opModeStarted = true;
        armed = false;
        negativeHeld = false;
        positiveHeld = false;
        motionNeutralSeen = false;
        servoCommandSubmittedInRun = false;
        servoBootstrapConfirmed = false;
        nonzeroJogSubmitted = false;
        activeNotice = selected == null
                ? null
                : runReadyNotice();
    }

    @Override
    protected void onLoop(double dtSec) {
        if (screen == Screen.ACTIVE) {
            // B is a level-sensitive safety control as well as an edge-triggered UI action. A B
            // held across selection or START must still win over a later fresh A press.
            if (gamepads.p1().b().getAsBoolean(clock)) {
                disarmNow();
            }
            realizeSelectedOutput(dtSec);
        }
        renderCurrentScreen();
    }

    @Override
    public boolean onBackPressed() {
        if (screen == Screen.PICKER) {
            return false;
        }

        DeviceKind previousKind = selected.kind;
        cleanupSelectedDevice();
        screen = Screen.PICKER;
        pickerNotice = previousKind == DeviceKind.SERVO
                ? "Temporary servo Direction restored. Its last position request is retained and "
                        + "the shaft may still be moving."
                : "Temporary settings restored and output commanded to zero.";
        refreshPickerPreservingNotice();
        return true;
    }

    @Override
    protected void onStop() {
        opModeStarted = false;
        if (selected != null) {
            cleanupSelectedDevice();
        }
    }

    private void refreshPicker() {
        pickerNotice = null;
        refreshPickerPreservingNotice();
    }

    private void refreshPickerPreservingNotice() {
        try {
            List<DeviceChoice> choices = new ArrayList<DeviceChoice>();
            addChoices(choices, DeviceKind.DC_MOTOR, ctx.hw.getAllNames(DcMotor.class));
            addChoices(choices, DeviceKind.CR_SERVO, ctx.hw.getAllNames(CRServo.class));
            addChoices(choices, DeviceKind.SERVO, ctx.hw.getAllNames(Servo.class));

            Collections.sort(choices, new Comparator<DeviceChoice>() {
                @Override
                public int compare(DeviceChoice first, DeviceChoice second) {
                    int label = first.displayLabel().compareToIgnoreCase(second.displayLabel());
                    if (label != 0) return label;
                    return first.stableId().compareTo(second.stableId());
                }
            });

            List<MenuItem<DeviceChoice>> rows = new ArrayList<MenuItem<DeviceChoice>>();
            for (DeviceChoice choice : choices) {
                rows.add(MenuItem.of(
                        choice.stableId(),
                        choice.displayLabel(),
                        helpFor(choice.kind),
                        choice));
            }
            picker.setItemsPreserveSelectionById(rows);
        } catch (RuntimeException failure) {
            picker.setItems(Collections.<MenuItem<DeviceChoice>>emptyList());
            pickerNotice = "Hardware enumeration failed: " + describe(failure);
        }
    }

    private static void addChoices(List<DeviceChoice> choices,
                                   DeviceKind kind,
                                   SortedSet<String> names) {
        if (names == null) return;
        for (String name : names) {
            if (name != null && !name.trim().isEmpty()) {
                choices.add(new DeviceChoice(kind, name));
            }
        }
    }

    private static String helpFor(DeviceKind kind) {
        switch (kind) {
            case DC_MOTOR:
                return "Low-power direction check; optional direction-adjusted encoder endpoints.";
            case CR_SERVO:
                return "Low-power direction check; no positional bounds without external feedback.";
            case SERVO:
                return "Gradual command-space direction and endpoint check; no shaft feedback.";
            default:
                throw new AssertionError("Unhandled actuator kind: " + kind);
        }
    }

    /** Resolve and inspect only. This method deliberately performs no actuator write. */
    private void selectDevice(DeviceChoice choice) {
        clearSelectedState();
        activeNotice = null;

        try {
            switch (choice.kind) {
                case DC_MOTOR:
                    motor = ctx.hw.get(DcMotor.class, choice.name);
                    originalMotorMode = requireObserved(
                            motor.getMode(), "motor run mode", choice);
                    originalMotorDirection = requireObserved(
                            motor.getDirection(), "motor direction", choice);
                    originalMotorZeroPowerBehavior = requireObserved(
                            motor.getZeroPowerBehavior(), "motor zero-power behavior", choice);
                    motorDirection = originalMotorDirection;
                    break;

                case CR_SERVO:
                    crServo = ctx.hw.get(CRServo.class, choice.name);
                    originalCrServoDirection = requireObserved(
                            crServo.getDirection(), "CR-servo direction", choice);
                    crServoDirection = originalCrServoDirection;
                    break;

                case SERVO:
                    servo = ctx.hw.get(Servo.class, choice.name);
                    originalServoDirection = requireObserved(
                            servo.getDirection(), "servo direction", choice);
                    servoDirection = originalServoDirection;
                    double commandSnapshot = servo.getPosition();
                    if (Double.isNaN(commandSnapshot)) {
                        // The Servo API reserves NaN for no known logical command. Only that
                        // explicit sentinel enters the two-press physical-safety bootstrap.
                        servoBootstrapRequired = true;
                        servoLastSuccessfulCommand = Double.NaN;
                    } else if (!Double.isFinite(commandSnapshot)
                            || commandSnapshot < Servo.MIN_POSITION
                            || commandSnapshot > Servo.MAX_POSITION) {
                        throw new IllegalStateException(
                                choice.displayLabel() + " reported invalid SDK command state "
                                        + commandSnapshot + "; expected unknown-state NaN or a finite "
                                        + "value in [0.0, 1.0].");
                    } else {
                        servoBootstrapRequired = false;
                        servoLastSuccessfulCommand = commandSnapshot;
                    }
                    break;

                default:
                    throw new AssertionError("Unhandled actuator kind: " + choice.kind);
            }
        } catch (RuntimeException failure) {
            clearSelectedState();
            pickerNotice = "Cannot inspect " + choice.displayLabel() + ": " + describe(failure);
            screen = Screen.PICKER;
            return;
        }

        selected = choice;
        screen = Screen.ACTIVE;
        prepared = false;
        hardwareTouched = false;
        armed = false;
        negativeHeld = false;
        positiveHeld = false;
        motionNeutralSeen = false;
        testPower = POWER_MIN;
        servoJogRatePerSec = SERVO_JOG_RATE_INITIAL_PER_SEC;
        lastSubmittedPower = 0.0;
        powerCommandSubmitted = false;
        clearCaptures();
        resultLines = Collections.emptyList();
        resultLog = null;
        servoCommandSubmittedInRun = false;
        servoBootstrapConfirmed = false;
        nonzeroJogSubmitted = false;
        activeNotice = opModeStarted
                ? runReadyNotice()
                : initReadyNotice();
    }

    private static <T> T requireObserved(T value, String fact, DeviceChoice choice) {
        if (value == null) {
            throw new IllegalStateException(
                    choice.displayLabel() + " did not report its " + fact + ".");
        }
        return value;
    }

    private void armOrRearm() {
        if (selected == null) return;
        // B is the higher-priority safety input. The raw level check is required here because A's
        // rise callback executes before B's callback; otherwise a confirmed servo bootstrap could
        // submit 0.5 before the later callback disarmed it in a simultaneous A+B cycle.
        if (gamepads.p1().b().getAsBoolean(clock)) {
            disarmNow();
            return;
        }

        if (!prepared) {
            if (selected.kind == DeviceKind.SERVO
                    && servoBootstrapRequired
                    && !servoBootstrapConfirmed) {
                servoBootstrapConfirmed = true;
                activeNotice = "Servo.getPosition() reported unknown state (NaN). Remove the "
                        + "horn/linkage or prove "
                        + "the full possible travel under the current controller PWM mapping and "
                        + "servo programming is clear. Release A, then press A again to submit "
                        + "SDK command 0.5 and arm.";
                return;
            }
            prepareSelectedDevice();
            armed = true;
            motionNeutralSeen = false;
            activeNotice = "ARMED. Release both bumpers once, then hold exactly one to jog.";
            return;
        }

        if (armed) {
            activeNotice = "Already armed. Press B to disarm.";
            return;
        }

        armed = true;
        motionNeutralSeen = false;
        activeNotice = "ARMED. Release both bumpers once, then hold exactly one to jog.";
    }

    private void prepareSelectedDevice() {
        switch (selected.kind) {
            case DC_MOTOR:
                if (originalMotorMode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                    throw new IllegalStateException(
                            "Motor '" + selected.name + "' is in STOP_AND_RESET_ENCODER. "
                                    + "The bring-up wizard will not restore a mode that resets the "
                                    + "encoder; place the motor in a normal run mode first.");
                }
                hardwareTouched = true;
                motor.setPower(0.0);
                lastSubmittedPower = 0.0;
                powerCommandSubmitted = true;
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                    throw new IllegalStateException(
                            "Motor '" + selected.name + "' did not enter RUN_WITHOUT_ENCODER.");
                }
                break;

            case CR_SERVO:
                hardwareTouched = true;
                crServo.setPower(0.0);
                lastSubmittedPower = 0.0;
                powerCommandSubmitted = true;
                break;

            case SERVO:
                if (servoBootstrapRequired) {
                    hardwareTouched = true;
                    servo.setPosition(SERVO_BOOTSTRAP_COMMAND);
                    servoLastSuccessfulCommand = SERVO_BOOTSTRAP_COMMAND;
                    servoCommandSubmittedInRun = true;
                    servoBootstrapRequired = false;
                    servoBootstrapConfirmed = false;
                }
                // Otherwise arming does not write. The first dead-man jog starts gradually from
                // the finite SDK command state observed during selection.
                break;

            default:
                throw new AssertionError("Unhandled actuator kind: " + selected.kind);
        }
        prepared = true;
    }

    private void disarmNow() {
        if (selected == null) return;
        armed = false;
        motionNeutralSeen = false;
        if (!prepared) {
            servoBootstrapConfirmed = false;
        }
        applyDisarmedOutputNow();
        activeNotice = selected.kind == DeviceKind.SERVO
                ? "DISARMED. Servo retains its last submitted command."
                : "DISARMED. Output is zero.";
    }

    private void applyDisarmedOutputNow() {
        if (!prepared) return;
        switch (selected.kind) {
            case DC_MOTOR:
                motor.setPower(0.0);
                lastSubmittedPower = 0.0;
                powerCommandSubmitted = true;
                break;
            case CR_SERVO:
                crServo.setPower(0.0);
                lastSubmittedPower = 0.0;
                powerCommandSubmitted = true;
                break;
            case SERVO:
                // A positional servo has no power-zero command. Stopping the jog retains the
                // existing controller command and therefore the existing hold behavior.
                break;
            default:
                throw new AssertionError("Unhandled actuator kind: " + selected.kind);
        }
    }

    private void toggleDirection() {
        if (!requirePreparedAndDisarmed("change direction")) return;
        if (negativeHeld || positiveHeld) {
            activeNotice = "Release both bumpers before changing direction.";
            return;
        }

        applyDisarmedOutputNow();
        hardwareTouched = true;
        switch (selected.kind) {
            case DC_MOTOR:
                DcMotorSimple.Direction nextMotor = opposite(motorDirection);
                motor.setDirection(nextMotor);
                motorDirection = nextMotor;
                requireDirectionApplied(motor.getDirection(), nextMotor, "motor");
                break;

            case CR_SERVO:
                DcMotorSimple.Direction nextCrServo = opposite(crServoDirection);
                crServo.setDirection(nextCrServo);
                crServoDirection = nextCrServo;
                requireDirectionApplied(crServo.getDirection(), nextCrServo, "CR servo");
                break;

            case SERVO:
                Servo.Direction nextServo = opposite(servoDirection);
                servo.setDirection(nextServo);
                servoDirection = nextServo;
                // FTC direction reflection occurs before its range mapping. Reflecting the logical
                // command lets the next gradual jog begin at the same retained physical pulse.
                servoLastSuccessfulCommand = 1.0 - servoLastSuccessfulCommand;
                servoCommandSubmittedInRun = false;
                requireDirectionApplied(servo.getDirection(), nextServo, "servo");
                break;

            default:
                throw new AssertionError("Unhandled actuator kind: " + selected.kind);
        }

        clearCaptures();
        nonzeroJogSubmitted = false;
        armed = false;
        motionNeutralSeen = false;
        activeNotice = "Direction is temporarily " + directionCode()
                + ". Captures cleared; press A again after neutral to rearm.";
    }

    private static DcMotorSimple.Direction opposite(DcMotorSimple.Direction direction) {
        return direction == DcMotorSimple.Direction.FORWARD
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD;
    }

    private static Servo.Direction opposite(Servo.Direction direction) {
        return direction == Servo.Direction.FORWARD
                ? Servo.Direction.REVERSE
                : Servo.Direction.FORWARD;
    }

    private void requireDirectionApplied(Object observed, Object expected, String family) {
        if (observed != expected) {
            throw new IllegalStateException(
                    "Selected " + family + " did not retain direction " + expected + ".");
        }
    }

    private void adjustRate(boolean increase) {
        if (!requireDisarmed("adjust the test rate")) return;

        if (selected.kind == DeviceKind.SERVO) {
            double delta = increase
                    ? SERVO_JOG_RATE_STEP_PER_SEC
                    : -SERVO_JOG_RATE_STEP_PER_SEC;
            servoJogRatePerSec = clamp(
                    servoJogRatePerSec + delta,
                    SERVO_JOG_RATE_MIN_PER_SEC,
                    SERVO_JOG_RATE_MAX_PER_SEC);
            activeNotice = String.format(
                    Locale.US,
                    "Servo jog rate %.2f native command units/sec.",
                    servoJogRatePerSec);
        } else {
            double delta = increase ? POWER_STEP : -POWER_STEP;
            testPower = clamp(testPower + delta, POWER_MIN, POWER_MAX);
            activeNotice = String.format(Locale.US, "Test power magnitude %.2f.", testPower);
        }
    }

    private void captureEndpoint(boolean plantMinimum) {
        if (!requirePreparedAndDisarmed("capture an endpoint")) return;
        if (negativeHeld || positiveHeld) {
            activeNotice = "Release both bumpers before capturing an endpoint.";
            return;
        }

        switch (selected.kind) {
            case DC_MOTOR:
                final int currentTicks;
                try {
                    currentTicks = motor.getCurrentPosition();
                } catch (RuntimeException failure) {
                    activeNotice = "Encoder capture failed: " + describe(failure);
                    return;
                }
                if (plantMinimum) {
                    motorNativeAtPlantMinTicks = currentTicks;
                    plantMinCaptured = true;
                } else {
                    motorNativeAtPlantMaxTicks = currentTicks;
                    plantMaxCaptured = true;
                }
                activeNotice = (plantMinimum ? "Plant-min" : "Plant-max")
                        + " captured at " + currentTicks + " direction-adjusted ticks.";
                break;

            case CR_SERVO:
                activeNotice = "CR servos provide no positional endpoint without external feedback.";
                break;

            case SERVO:
                if (!servoCommandSubmittedInRun) {
                    activeNotice = "Jog this servo successfully at least once in RUN before "
                            + "capturing an endpoint. The initial getPosition() value is SDK "
                            + "command state, not wizard-submitted evidence.";
                    return;
                }
                if (plantMinimum) {
                    servoNativeAtPlantMin = servoLastSuccessfulCommand;
                    plantMinCaptured = true;
                } else {
                    servoNativeAtPlantMax = servoLastSuccessfulCommand;
                    plantMaxCaptured = true;
                }
                activeNotice = String.format(
                        Locale.US,
                        "%s captured at SDK command %.4f (not measured shaft position).",
                        plantMinimum ? "Plant-min" : "Plant-max",
                        servoLastSuccessfulCommand);
                break;

            default:
                throw new AssertionError("Unhandled actuator kind: " + selected.kind);
        }
    }

    private void clearCapturesWhenSafe() {
        if (!requireDisarmed("clear endpoint captures")) return;
        clearCaptures();
        activeNotice = "Endpoint captures cleared.";
    }

    private void clearCaptures() {
        plantMinCaptured = false;
        plantMaxCaptured = false;
        motorNativeAtPlantMinTicks = 0L;
        motorNativeAtPlantMaxTicks = 0L;
        servoNativeAtPlantMin = Double.NaN;
        servoNativeAtPlantMax = Double.NaN;
    }

    private void finalizeResult() {
        if (!requirePreparedAndDisarmed("finalize the result")) return;
        if (negativeHeld || positiveHeld) {
            activeNotice = "Release both bumpers before finalizing.";
            return;
        }
        if (!nonzeroJogSubmitted) {
            activeNotice = "Submit at least one successful nonzero jog under the current Direction "
                    + "before finalizing.";
            return;
        }
        if (plantMinCaptured != plantMaxCaptured) {
            activeNotice = "Capture both Plant endpoints, or press gamepad START to clear the single capture "
                    + "and finalize direction only.";
            return;
        }

        applyDisarmedOutputNow();
        List<String> lines = new ArrayList<String>();
        lines.add("Device: " + selected.displayLabel());
        lines.add("Direction: " + directionCode());
        lines.add("Copy the tested direction into robot configuration.");
        lines.add("Exit restores temporary Direction/settings; a servo retains its last request.");

        StringBuilder log = new StringBuilder("ACTUATOR_BRING_UP_RESULT")
                .append(",kind=").append(selected.kind.name())
                .append(",name=").append(logSafe(selected.name))
                .append(",direction=").append(directionName())
                .append(",directionEvidence=NONZERO_JOG_SUBMITTED");

        try {
            switch (selected.kind) {
                case DC_MOTOR:
                    appendMotorResult(lines, log);
                    break;
                case CR_SERVO:
                    lines.add("Positional bounds: unavailable without external position feedback.");
                    log.append(",positionEvidence=UNAVAILABLE_WITHOUT_EXTERNAL_FEEDBACK");
                    break;
                case SERVO:
                    appendServoResult(lines, log);
                    break;
                default:
                    throw new AssertionError("Unhandled actuator kind: " + selected.kind);
            }
        } catch (ResultValidationException validation) {
            // A semantic capture mistake is retryable while the device is already disarmed and its
            // type-specific output request is safe. Do not turn it into a lifecycle failure.
            activeNotice = validation.getMessage();
            return;
        }

        resultLines = Collections.unmodifiableList(lines);
        resultLog = log.toString();
        resultLogSink.accept(resultLog);
        screen = Screen.RESULT;
    }

    private void appendMotorResult(List<String> lines, StringBuilder log) {
        if (!plantMinCaptured) {
            lines.add("Endpoint evidence: not captured (direction-only result).");
            log.append(",positionEvidence=NOT_CAPTURED");
            return;
        }

        long signedSpan = motorNativeAtPlantMaxTicks - motorNativeAtPlantMinTicks;
        long absoluteSpan = Math.abs(signedSpan);
        if (signedSpan <= 0L) {
            activeNotice = "Plant-max must be greater than Plant-min under " + directionCode()
                    + ". Change direction or recapture the semantic endpoints.";
            throw new ResultValidationException(activeNotice);
        }

        lines.add("nativeAtPlantMinTicks = " + motorNativeAtPlantMinTicks);
        lines.add("nativeAtPlantMaxTicks = " + motorNativeAtPlantMaxTicks);
        lines.add("signedSafeTravelTicks = " + signedSpan);
        lines.add("absoluteSafeTravelTicks = " + absoluteSpan);
        lines.add("The raw encoder was not reset; runtime homing must establish Plant zero.");
        lines.add("Relative-tick mapping fragment:");
        lines.add(".bounded(0.0, " + signedSpan + ".0)");
        lines.add(".nativeUnits()");
        lines.add(".needsReference(\"mechanism not homed\")");
        lines.add("Meaningful-unit alternative:");
        lines.add(".bounded(0.0, PLANT_TRAVEL)");
        lines.add(".scaleToNative(" + signedSpan + ".0 / PLANT_TRAVEL)");
        lines.add(".needsReference(\"mechanism not homed\")");

        log.append(",nativeAtPlantMinTicks=").append(motorNativeAtPlantMinTicks)
                .append(",nativeAtPlantMaxTicks=").append(motorNativeAtPlantMaxTicks)
                .append(",signedSafeTravelTicks=").append(signedSpan)
                .append(",absoluteSafeTravelTicks=").append(absoluteSpan);
    }

    private void appendServoResult(List<String> lines, StringBuilder log) {
        if (!plantMinCaptured) {
            lines.add("Endpoint evidence: not captured (direction-only result).");
            lines.add("FTC native 0.0..1.0 is the SDK logical command domain.");
            lines.add("Controller PWM mapping and servo programming determine physical travel.");
            log.append(",positionEvidence=NOT_CAPTURED");
            return;
        }
        if (servoNativeAtPlantMin == servoNativeAtPlantMax) {
            activeNotice = "Servo Plant-min and Plant-max commands must be different.";
            throw new ResultValidationException(activeNotice);
        }

        String nativeMin = formatNative(servoNativeAtPlantMin);
        String nativeMax = formatNative(servoNativeAtPlantMax);
        lines.add("nativeAtPlantMin = " + nativeMin);
        lines.add("nativeAtPlantMax = " + nativeMax);
        lines.add("Mapping template (choose meaningful Plant units):");
        lines.add(".bounded(PLANT_MIN, PLANT_MAX)");
        lines.add(".rangeMapsToNative(" + nativeMin + ", " + nativeMax + ")");
        lines.add("Native 0.0..1.0 is the FTC SDK logical command domain.");
        lines.add("Controller PWM mapping and servo programming determine physical travel.");
        lines.add("The map is linear in command space; it does not prove linkage linearity.");

        log.append(",nativeAtPlantMin=").append(nativeMin)
                .append(",nativeAtPlantMax=").append(nativeMax);
    }

    /** Marker caught at the local result boundary so semantic capture errors remain retryable. */
    private static final class ResultValidationException extends RuntimeException {
        ResultValidationException(String message) {
            super(message);
        }
    }

    private void realizeSelectedOutput(double dtSec) {
        if (!prepared) return;

        if (negativeHeld && positiveHeld) {
            motionNeutralSeen = false;
        } else if (!negativeHeld && !positiveHeld) {
            motionNeutralSeen = true;
        }

        boolean oneDirectionHeld = negativeHeld ^ positiveHeld;
        boolean mayMove = armed && motionNeutralSeen && oneDirectionHeld;

        switch (selected.kind) {
            case DC_MOTOR:
                submitMotorPower(mayMove ? signedTestPower() : 0.0);
                break;
            case CR_SERVO:
                submitCrServoPower(mayMove ? signedTestPower() : 0.0);
                break;
            case SERVO:
                if (mayMove) {
                    jogServo(dtSec, positiveHeld ? +1.0 : -1.0);
                }
                break;
            default:
                throw new AssertionError("Unhandled actuator kind: " + selected.kind);
        }
    }

    private double signedTestPower() {
        return positiveHeld ? testPower : -testPower;
    }

    private void submitMotorPower(double power) {
        motor.setPower(power);
        lastSubmittedPower = power;
        powerCommandSubmitted = true;
        if (power != 0.0) nonzeroJogSubmitted = true;
    }

    private void submitCrServoPower(double power) {
        crServo.setPower(power);
        lastSubmittedPower = power;
        powerCommandSubmitted = true;
        if (power != 0.0) nonzeroJogSubmitted = true;
    }

    private void jogServo(double dtSec, double sign) {
        if (!Double.isFinite(dtSec) || dtSec <= 0.0) return;
        double step = Math.min(SERVO_MAX_STEP_PER_CYCLE, servoJogRatePerSec * dtSec);
        double next = clamp(
                servoLastSuccessfulCommand + sign * step,
                Servo.MIN_POSITION,
                Servo.MAX_POSITION);
        // Signed zeros are the same command and prove no directional jog. NaN cannot reach a
        // prepared servo because the explicit unknown state must complete bootstrap first.
        if (next == servoLastSuccessfulCommand) {
            return;
        }

        hardwareTouched = true;
        servo.setPosition(next);
        servoLastSuccessfulCommand = next;
        servoCommandSubmittedInRun = true;
        nonzeroJogSubmitted = true;
    }

    private boolean requirePreparedAndDisarmed(String operation) {
        if (!prepared) {
            activeNotice = "Press A in RUN to prepare before you " + operation + ".";
            return false;
        }
        return requireDisarmed(operation);
    }

    private boolean requireDisarmed(String operation) {
        if (armed) {
            activeNotice = "Press B to disarm before you " + operation + ".";
            return false;
        }
        return true;
    }

    private void cleanupSelectedDevice() {
        if (selected == null) return;
        if (cleanupAttempted) {
            if (retainedCleanupFailure != null) throw retainedCleanupFailure;
            return;
        }
        cleanupAttempted = true;

        armed = false;
        negativeHeld = false;
        positiveHeld = false;
        motionNeutralSeen = false;

        try {
            switch (selected.kind) {
                case DC_MOTOR:
                    if (hardwareTouched) {
                        CleanupActions.attemptAll(
                                () -> motor.setPower(0.0),
                                () -> motor.setDirection(originalMotorDirection),
                                () -> motor.setZeroPowerBehavior(originalMotorZeroPowerBehavior),
                                () -> motor.setMode(originalMotorMode));
                    }
                    break;

                case CR_SERVO:
                    if (hardwareTouched) {
                        CleanupActions.attemptAll(
                                () -> crServo.setPower(0.0),
                                () -> crServo.setDirection(originalCrServoDirection));
                    }
                    break;

                case SERVO:
                    if (hardwareTouched) {
                        Servo.Direction beforeRestore = servoDirection;
                        servo.setDirection(originalServoDirection);
                        if (beforeRestore != originalServoDirection) {
                            // setDirection itself leaves the physical pulse untouched. Reflect the
                            // retained logical command so our internal statement remains truthful.
                            servoLastSuccessfulCommand = 1.0 - servoLastSuccessfulCommand;
                        }
                        servoDirection = originalServoDirection;
                    }
                    break;

                default:
                    throw new AssertionError("Unhandled actuator kind: " + selected.kind);
            }

            clearSelectedState();
        } catch (RuntimeException failure) {
            retainedCleanupFailure = failure;
            throw failure;
        }
    }

    private void clearSelectedState() {
        selected = null;
        motor = null;
        crServo = null;
        servo = null;

        originalMotorMode = null;
        originalMotorDirection = null;
        originalMotorZeroPowerBehavior = null;
        motorDirection = null;
        originalCrServoDirection = null;
        crServoDirection = null;
        originalServoDirection = null;
        servoDirection = null;
        servoLastSuccessfulCommand = Double.NaN;
        servoCommandSubmittedInRun = false;
        servoBootstrapRequired = false;
        servoBootstrapConfirmed = false;

        prepared = false;
        hardwareTouched = false;
        cleanupAttempted = false;
        retainedCleanupFailure = null;
        armed = false;
        negativeHeld = false;
        positiveHeld = false;
        motionNeutralSeen = false;
        lastSubmittedPower = 0.0;
        powerCommandSubmitted = false;
        nonzeroJogSubmitted = false;
        clearCaptures();
        resultLines = Collections.emptyList();
        resultLog = null;
    }

    private void renderCurrentScreen() {
        switch (screen) {
            case PICKER:
                renderPicker();
                break;
            case ACTIVE:
                renderActive();
                break;
            case RESULT:
                renderResult();
                break;
            default:
                throw new AssertionError("Unhandled screen: " + screen);
        }
    }

    private void renderPicker() {
        Telemetry telemetry = ctx.telemetry;
        telemetry.clearAll();
        picker.render(telemetry);
        telemetry.addLine("");
        telemetry.addLine("Use one isolated, unloaded actuator at a time.");
        telemetry.addLine("The wizard records candidate facts; it does not detect safe hard stops.");
        if (pickerNotice != null) {
            telemetry.addLine("");
            telemetry.addLine(pickerNotice);
        }
        telemetry.update();
    }

    private void renderActive() {
        Telemetry telemetry = ctx.telemetry;
        telemetry.clearAll();
        telemetry.addLine("=== Actuator Bring-up ===");
        telemetry.addData("Device", selected.displayLabel());
        telemetry.addData("Phase", opModeStarted ? "RUN" : "INIT (observation only; no writes)");
        telemetry.addData("Prepared", prepared ? "YES" : "NO");
        telemetry.addData("Armed [A]", armed ? "YES" : "NO");
        telemetry.addData("Direction [X while disarmed]", directionCode());
        telemetry.addData(
                "Direction jog evidence",
                nonzeroJogSubmitted ? "YES - nonzero command submitted" : "NO");

        if (selected.kind == DeviceKind.SERVO) {
            if (servoBootstrapRequired) {
                telemetry.addData("SDK command", "unknown (NaN); bootstrap candidate %.1f",
                        SERVO_BOOTSTRAP_COMMAND);
                telemetry.addLine("Before bootstrap: remove the horn/linkage or prove every possible");
                telemetry.addLine("position under the current PWM mapping/programming is clear.");
                telemetry.addLine("Press A once to acknowledge; release, then press A again to submit 0.5.");
            } else {
                telemetry.addData(
                        "SDK command state (not shaft feedback)",
                        "%.4f",
                        servoLastSuccessfulCommand);
            }
            telemetry.addData("Jog rate [Dpad U/D]", "%.2f native/sec", servoJogRatePerSec);
            telemetry.addLine("FTC native 0.0..1.0 is SDK logical command, not shaft position.");
        } else {
            telemetry.addData("Test power [Dpad U/D]", "%.2f", testPower);
            if (powerCommandSubmitted) {
                telemetry.addData("Last submitted power", "%.2f", lastSubmittedPower);
            } else {
                telemetry.addData("Last submitted power", "not submitted by this wizard");
            }
            if (selected.kind == DeviceKind.DC_MOTOR) {
                try {
                    telemetry.addData(
                            "Encoder (direction-adjusted)",
                            motor.getCurrentPosition());
                } catch (RuntimeException failure) {
                    telemetry.addData("Encoder", "unavailable: %s", describe(failure));
                }
            } else {
                telemetry.addLine("CR servo: positional endpoints require external feedback.");
            }
        }

        telemetry.addLine("");
        telemetry.addLine("Hold LB/RB: negative/positive jog | both: motor/CR zero; servo no new jog");
        telemetry.addLine("B: disarm (motor/CR zero; servo retains request)");
        telemetry.addLine("Dpad L/R: capture Plant min/max");
        telemetry.addLine("While disarmed: gamepad START clears captures | Y finalizes");
        telemetry.addLine("BACK: restore temporary settings / return to device list");
        renderCaptureState(telemetry);
        telemetry.addLine("");
        telemetry.addLine("Approach slowly, stop before binding, back away, then capture.");
        if (activeNotice != null) {
            telemetry.addLine("Status: " + activeNotice);
        }
        telemetry.update();
    }

    private void renderCaptureState(Telemetry telemetry) {
        if (selected.kind == DeviceKind.CR_SERVO) return;

        if (selected.kind == DeviceKind.DC_MOTOR) {
            telemetry.addData(
                    "Plant-min capture",
                    plantMinCaptured ? motorNativeAtPlantMinTicks + " ticks" : "not captured");
            telemetry.addData(
                    "Plant-max capture",
                    plantMaxCaptured ? motorNativeAtPlantMaxTicks + " ticks" : "not captured");
        } else {
            telemetry.addData(
                    "Plant-min capture",
                    plantMinCaptured ? formatNative(servoNativeAtPlantMin) : "not captured");
            telemetry.addData(
                    "Plant-max capture",
                    plantMaxCaptured ? formatNative(servoNativeAtPlantMax) : "not captured");
        }
    }

    private void renderResult() {
        Telemetry telemetry = ctx.telemetry;
        telemetry.clearAll();
        telemetry.addLine("=== Actuator Bring-up Result ===");
        for (String line : resultLines) {
            telemetry.addLine(line);
        }
        telemetry.addLine("");
        telemetry.addLine("Logcat tag: " + LOG_TAG);
        telemetry.addLine("No code, profile, preference, or hardware-programmer setting was changed.");
        telemetry.addLine("BACK: restore temporary Direction/settings and choose another device.");
        if (selected.kind == DeviceKind.SERVO) {
            telemetry.addLine("Servo request is retained on BACK; the shaft may still be moving.");
        }
        telemetry.update();
    }

    private String directionCode() {
        return "Direction." + directionName();
    }

    private String directionName() {
        switch (selected.kind) {
            case DC_MOTOR:
                return motorDirection.name();
            case CR_SERVO:
                return crServoDirection.name();
            case SERVO:
                return servoDirection.name();
            default:
                throw new AssertionError("Unhandled actuator kind: " + selected.kind);
        }
    }

    private static String formatNative(double value) {
        return Double.toString(value);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static String describe(Throwable failure) {
        String message = failure.getMessage();
        return failure.getClass().getSimpleName()
                + ((message == null || message.trim().isEmpty()) ? "" : ": " + message);
    }

    private static String logSafe(String value) {
        if (value == null || value.isEmpty()) return "unknown";
        return value.replace(',', '_').replace('\n', ' ').replace('\r', ' ');
    }

    private String initReadyNotice() {
        if (selected.kind == DeviceKind.SERVO && servoBootstrapRequired) {
            return "INIT inspection only; Servo.getPosition() reported unknown state (NaN). "
                    + "Read the bootstrap warning, "
                    + "then start the OpMode.";
        }
        return "INIT inspection only. Start the OpMode, release controls, then press A "
                + "to prepare and arm.";
    }

    private String runReadyNotice() {
        if (selected.kind == DeviceKind.SERVO && servoBootstrapRequired) {
            return "Servo.getPosition() reported unknown state (NaN). Read the physical bootstrap "
                    + "warning, then press A "
                    + "once to acknowledge.";
        }
        return "RUN ready. Release controls, then press A to prepare and arm.";
    }
}
