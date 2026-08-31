package edu.ftcsushi.fw.tools.tester.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.function.Function;

import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.tools.tester.TeleOpTester;
import edu.ftcsushi.fw.tools.tester.TesterContext;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/** Lifecycle and fail-stop regressions for the retained advanced DC-motor diagnostics. */
public final class AdvancedMotorTesterSafetyTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void powerTesterIsReadOnlyInInitAndRequiresFreshPostStartControls() throws Exception {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor("motor", DcMotor.RunMode.RUN_USING_ENCODER, 12);
        Rig rig = new Rig(hardware);
        DcMotorPowerTester tester = new DcMotorPowerTester("motor");

        rig.gamepad.a = true;
        rig.gamepad.dpad_up = true;
        rig.gamepad.left_stick_y = 1.0f;
        tester.init(rig.context());
        rig.initCycle(tester);

        assertNoActuatorWrites(motor);
        assertTrue(Double.isNaN(fieldDouble(tester, "lastSubmittedPower")));

        rig.start(tester);
        assertTrue(motor.onlyZeroPowerWasWritten());
        assertEquals(0.0, fieldDouble(tester, "lastSubmittedPower"), EPSILON);
        assertFalse(motor.modeWrites.isEmpty());
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motor.mode);

        rig.runCycle(tester); // Held INIT controls remain effect-free after PLAY.
        assertEquals(0.0, motor.power, EPSILON);

        rig.neutral();
        rig.runCycle(tester);
        rig.gamepad.dpad_up = true;
        rig.runCycle(tester);
        rig.gamepad.dpad_up = false;
        rig.runCycle(tester);
        rig.gamepad.a = true;
        rig.runCycle(tester);

        assertEquals(0.05, motor.power, EPSILON);

        rig.gamepad.a = false;
        rig.gamepad.left_stick_y = 1.0f;
        rig.runCycle(tester);
        assertEquals(0.05, motor.power, EPSILON);

        tester.stop();
        assertEquals(0.0, motor.power, EPSILON);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.mode);
    }

    @Test
    public void positionTesterAnchorsAtCurrentReadingUsesConservativePowerAndNeverResetsEncoder() {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor("lift", DcMotor.RunMode.RUN_WITHOUT_ENCODER, 1234);
        Rig rig = new Rig(hardware);
        DcMotorPositionTester tester = new DcMotorPositionTester("lift");

        rig.gamepad.a = true;
        rig.gamepad.y = true;
        rig.gamepad.right_stick_y = 1.0f;
        tester.init(rig.context());
        rig.initCycle(tester);

        assertNoActuatorWrites(motor);

        rig.start(tester);
        rig.runCycle(tester); // Held A/Y/stick cannot enable or reset after PLAY.
        assertEquals(0, motor.targetPositionWrites);
        assertFalse(motor.modeWrites.contains(DcMotor.RunMode.RUN_TO_POSITION));
        assertFalse(motor.modeWrites.contains(DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        rig.neutral();
        rig.runCycle(tester);
        rig.gamepad.a = true;
        rig.runCycle(tester);

        assertEquals(1234, motor.targetPosition);
        assertEquals(1, motor.targetPositionWrites);
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, motor.mode);
        assertEquals(0.10, motor.power, EPSILON);

        rig.gamepad.a = false;
        rig.gamepad.right_stick_y = 1.0f;
        rig.runCycle(tester);
        assertEquals(1234, motor.targetPosition);

        rig.gamepad.right_stick_y = 0.0f;
        rig.gamepad.y = false;
        rig.runCycle(tester);
        rig.gamepad.y = true;
        rig.runCycle(tester);
        assertFalse(motor.modeWrites.contains(DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        tester.stop();
        assertEquals(0.0, motor.power, EPSILON);
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motor.mode);
    }

    @Test
    public void positionTesterPreservesFullEncoderDomainAndSaturatesStepsWithoutOverflow() {
        assertExtremePositionAnchor(Integer.MAX_VALUE, true);
        assertExtremePositionAnchor(Integer.MIN_VALUE, false);
    }

    @Test
    public void powerTesterDisarmsAndZerosBeforeCommandSignInversion() {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor("motor", DcMotor.RunMode.RUN_USING_ENCODER, 0);
        Rig rig = new Rig(hardware);
        DcMotorPowerTester tester = new DcMotorPowerTester("motor");
        tester.init(rig.context());
        rig.initCycle(tester);
        rig.start(tester);

        rig.neutral();
        rig.runCycle(tester);
        rig.gamepad.dpad_up = true;
        rig.runCycle(tester);
        rig.gamepad.dpad_up = false;
        rig.runCycle(tester);
        rig.gamepad.a = true;
        rig.runCycle(tester);
        assertEquals(0.05, motor.power, EPSILON);

        rig.gamepad.a = false;
        rig.gamepad.x = true;
        rig.runCycle(tester);
        assertEquals(0.0, motor.power, EPSILON);

        rig.gamepad.x = false;
        rig.runCycle(tester);
        rig.gamepad.a = true;
        rig.runCycle(tester);
        assertEquals(0.0, motor.power, EPSILON);
    }

    @Test
    public void velocityTesterIsReadOnlyInInitAndIgnoresHeldAndStickCommands() {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor("flywheel", DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        Rig rig = new Rig(hardware);
        DcMotorVelocityTester tester = new DcMotorVelocityTester("flywheel");

        rig.gamepad.a = true;
        rig.gamepad.dpad_up = true;
        rig.gamepad.right_stick_y = 1.0f;
        tester.init(rig.context());
        rig.initCycle(tester);

        assertNoActuatorWrites(motor);

        rig.start(tester);
        rig.runCycle(tester);
        assertTrue(motor.onlyZeroVelocityWasWritten());

        rig.neutral();
        rig.runCycle(tester);
        rig.gamepad.dpad_up = true;
        rig.runCycle(tester);
        rig.gamepad.dpad_up = false;
        rig.runCycle(tester);
        rig.gamepad.a = true;
        rig.runCycle(tester);

        assertEquals(50.0, motor.velocity, EPSILON);

        rig.gamepad.a = false;
        rig.gamepad.right_stick_y = 1.0f;
        rig.runCycle(tester);
        assertEquals(50.0, motor.velocity, EPSILON);

        tester.stop();
        assertEquals(0.0, motor.velocity, EPSILON);
        assertEquals(0.0, motor.power, EPSILON);
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motor.mode);
    }

    @Test
    public void everyAdvancedMotorDiagnosticRejectsEncoderResetModeBeforeAnyWrite() {
        assertResetModeRejected(DcMotorPowerTester::new);
        assertResetModeRejected(DcMotorPositionTester::new);
        assertResetModeRejected(DcMotorVelocityTester::new);
    }

    @Test
    public void cleanupAttemptsModeRestoreAfterZeroFailureAndSurfacesTheError() {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor("motor", DcMotor.RunMode.RUN_USING_ENCODER, 0);
        Rig rig = new Rig(hardware);
        DcMotorPowerTester tester = new DcMotorPowerTester("motor");
        tester.init(rig.context());
        rig.initCycle(tester);
        rig.start(tester);

        motor.failNextPowerWrite = new IllegalStateException("zero failed");
        RuntimeException failure = assertThrows(RuntimeException.class, tester::stop);

        assertTrue(failure.getMessage().contains("restore"));
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.mode);
        assertTrue(motor.modeWrites.contains(DcMotor.RunMode.RUN_USING_ENCODER));
    }

    @Test
    public void cleanupFailureIsRetainedWithoutReplayingAnyAdvancedMotorWrites() {
        assertPowerCleanupIsNotReplayed();
        assertPositionCleanupIsNotReplayed();
        assertVelocityCleanupIsNotReplayed();
    }

    @Test
    public void disabledDirectionChangesClearStagedPowerAndVelocityTargets() {
        assertDisabledPowerInversionClearsStagedTarget();
        assertDisabledVelocityDirectionChangeClearsStagedTarget();
    }

    @Test
    public void powerPreparationCleanupFailureRetainsTheSelectedHardwareLatch() {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor(
                "motor", DcMotor.RunMode.RUN_USING_ENCODER, 0);
        Rig rig = new Rig(hardware);
        DcMotorPowerTester tester = new DcMotorPowerTester();
        tester.init(rig.context());
        rig.initCycle(tester);
        rig.start(tester);

        motor.remainingPowerWriteFailures = 2;
        rig.gamepad.a = true;
        RuntimeException preparationFailure = assertThrows(
                RuntimeException.class,
                () -> rig.runCycle(tester));
        assertTrue(preparationFailure.getMessage().contains("cleanup did not complete"));
        int powerWritesAfterFailure = motor.powerWrites.size();
        int modeWritesAfterFailure = motor.modeWrites.size();

        RuntimeException stopFailure = assertThrows(RuntimeException.class, tester::stop);

        assertTrue(stopFailure.getMessage().contains("restore"));
        assertEquals(powerWritesAfterFailure, motor.powerWrites.size());
        assertEquals(modeWritesAfterFailure, motor.modeWrites.size());
    }

    @Test
    public void positionTelemetryMakesUnavailableReadsAndInactiveCommandsExplicit() {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor(
                "lift", DcMotor.RunMode.RUN_WITHOUT_ENCODER, 400);
        Rig rig = new Rig(hardware);
        DcMotorPositionTester tester = new DcMotorPositionTester("lift");
        tester.init(rig.context());
        rig.initCycle(tester);
        rig.telemetry.assertDataContains("Target command", "INIT - no command submitted");
        rig.start(tester);

        rig.neutral();
        motor.failNextCurrentPositionRead = new IllegalStateException("position read failed");
        motor.failNextBusyRead = new IllegalStateException("busy read failed");
        motor.failNextModeRead = new IllegalStateException("mode read failed");
        motor.failNextVelocityRead = new IllegalStateException("velocity read failed");
        rig.runCycle(tester);

        rig.telemetry.assertDataContains("Target command", "not active");
        rig.telemetry.assertDataContains("Current position", "unavailable");
        rig.telemetry.assertDataContains("Position error", "unavailable");
        rig.telemetry.assertDataContains("Busy", "unavailable");
        rig.telemetry.assertDataContains("Mode", "unavailable");
        rig.telemetry.assertDataContains("Velocity", "unavailable");
        tester.stop();
    }

    @Test
    public void velocityTelemetryMakesUnavailableReadsAndInactiveCommandsExplicit() {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor(
                "flywheel", DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        Rig rig = new Rig(hardware);
        DcMotorVelocityTester tester = new DcMotorVelocityTester("flywheel");
        tester.init(rig.context());
        rig.initCycle(tester);
        rig.telemetry.assertDataContains("Velocity command", "INIT - no command submitted");
        rig.start(tester);

        rig.neutral();
        motor.failNextModeRead = new IllegalStateException("mode read failed");
        motor.failNextVelocityRead = new IllegalStateException("velocity read failed");
        rig.runCycle(tester);

        rig.telemetry.assertDataContains("Velocity command", "not active");
        rig.telemetry.assertDataContains("Measured velocity", "unavailable");
        rig.telemetry.assertDataContains("Velocity error", "unavailable");
        rig.telemetry.assertDataContains("Mode", "unavailable");
        tester.stop();
    }

    @Test
    public void operationalDirectionMatchesEverySdkDirectionOrientationCombination() {
        assertEquals(
                DcMotor.Direction.FORWARD,
                DcMotorPowerTester.operationalDirection(
                        DcMotor.Direction.FORWARD, Rotation.CW));
        assertEquals(
                DcMotor.Direction.REVERSE,
                DcMotorPowerTester.operationalDirection(
                        DcMotor.Direction.REVERSE, Rotation.CW));
        assertEquals(
                DcMotor.Direction.REVERSE,
                DcMotorPowerTester.operationalDirection(
                        DcMotor.Direction.FORWARD, Rotation.CCW));
        assertEquals(
                DcMotor.Direction.FORWARD,
                DcMotorPowerTester.operationalDirection(
                        DcMotor.Direction.REVERSE, Rotation.CCW));
        assertThrows(
                IllegalArgumentException.class,
                () -> DcMotorPowerTester.operationalDirection(
                        DcMotor.Direction.FORWARD, null));
    }

    @Test
    public void heldStopHasLevelPriorityAcrossStartInEveryAdvancedMotorDiagnostic() {
        assertPowerHeldStopPriority();
        assertPositionHeldStopPriority();
        assertVelocityHeldStopPriority();
    }

    private static void assertPowerCleanupIsNotReplayed() {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor(
                "motor", DcMotor.RunMode.RUN_USING_ENCODER, 0);
        Rig rig = new Rig(hardware);
        DcMotorPowerTester tester = new DcMotorPowerTester("motor");
        tester.init(rig.context());
        rig.initCycle(tester);
        rig.start(tester);

        motor.failNextPowerWrite = new IllegalStateException("power cleanup failed");
        RuntimeException first = assertThrows(RuntimeException.class, tester::onBackPressed);
        int powerWrites = motor.powerWrites.size();
        int modeWrites = motor.modeWrites.size();
        RuntimeException second = assertThrows(RuntimeException.class, tester::stop);

        assertSame(first, second);
        assertEquals(powerWrites, motor.powerWrites.size());
        assertEquals(modeWrites, motor.modeWrites.size());
    }

    private static void assertPositionCleanupIsNotReplayed() {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor(
                "lift", DcMotor.RunMode.RUN_WITHOUT_ENCODER, 100);
        Rig rig = new Rig(hardware);
        DcMotorPositionTester tester = new DcMotorPositionTester("lift");
        tester.init(rig.context());
        rig.initCycle(tester);
        rig.start(tester);

        motor.failNextPowerWrite = new IllegalStateException("position cleanup failed");
        RuntimeException first = assertThrows(RuntimeException.class, tester::onBackPressed);
        int totalWrites = motor.totalWrites();
        RuntimeException second = assertThrows(RuntimeException.class, tester::stop);

        assertSame(first, second);
        assertEquals(totalWrites, motor.totalWrites());
    }

    private static void assertVelocityCleanupIsNotReplayed() {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor(
                "flywheel", DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        Rig rig = new Rig(hardware);
        DcMotorVelocityTester tester = new DcMotorVelocityTester("flywheel");
        tester.init(rig.context());
        rig.initCycle(tester);
        rig.start(tester);

        motor.failNextVelocityWrite = new IllegalStateException("velocity cleanup failed");
        RuntimeException first = assertThrows(RuntimeException.class, tester::onBackPressed);
        int totalWrites = motor.totalWrites();
        RuntimeException second = assertThrows(RuntimeException.class, tester::stop);

        assertSame(first, second);
        assertEquals(totalWrites, motor.totalWrites());
    }

    private static void assertDisabledPowerInversionClearsStagedTarget() {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor(
                "motor", DcMotor.RunMode.RUN_USING_ENCODER, 0);
        Rig rig = new Rig(hardware);
        DcMotorPowerTester tester = new DcMotorPowerTester("motor");
        tester.init(rig.context());
        rig.initCycle(tester);
        rig.start(tester);

        rig.neutral();
        rig.runCycle(tester);
        rig.gamepad.dpad_up = true;
        rig.runCycle(tester);
        rig.neutral();
        rig.runCycle(tester);
        rig.gamepad.a = true;
        rig.runCycle(tester);
        assertEquals(0.05, motor.power, EPSILON);
        rig.neutral();
        rig.runCycle(tester);
        rig.gamepad.a = true;
        rig.runCycle(tester);
        assertEquals(0.0, motor.power, EPSILON);
        rig.neutral();
        rig.runCycle(tester);
        rig.gamepad.x = true;
        rig.runCycle(tester);
        assertEquals(0.0, motor.power, EPSILON);
        rig.neutral();
        rig.runCycle(tester);
        rig.gamepad.a = true;
        rig.runCycle(tester);
        assertEquals(0.0, motor.power, EPSILON);
        tester.stop();
    }

    private static void assertDisabledVelocityDirectionChangeClearsStagedTarget() {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor(
                "flywheel", DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        Rig rig = new Rig(hardware);
        DcMotorVelocityTester tester = new DcMotorVelocityTester("flywheel");
        tester.init(rig.context());
        rig.initCycle(tester);
        rig.start(tester);

        rig.neutral();
        rig.runCycle(tester);
        rig.gamepad.dpad_up = true;
        rig.runCycle(tester);
        rig.neutral();
        rig.runCycle(tester);
        rig.gamepad.a = true;
        rig.runCycle(tester);
        assertEquals(50.0, motor.velocity, EPSILON);
        rig.neutral();
        rig.runCycle(tester);
        rig.gamepad.a = true;
        rig.runCycle(tester);
        assertEquals(0.0, motor.velocity, EPSILON);
        rig.neutral();
        rig.runCycle(tester);
        rig.gamepad.x = true;
        rig.runCycle(tester);
        assertEquals(0.0, motor.velocity, EPSILON);
        rig.neutral();
        rig.runCycle(tester);
        rig.gamepad.a = true;
        rig.runCycle(tester);
        assertEquals(0.0, motor.velocity, EPSILON);
        tester.stop();
    }

    private static void assertPowerHeldStopPriority() {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor(
                "motor", DcMotor.RunMode.RUN_USING_ENCODER, 0);
        Rig rig = new Rig(hardware);
        DcMotorPowerTester tester = new DcMotorPowerTester("motor");
        rig.gamepad.a = true;
        rig.gamepad.b = true;
        rig.gamepad.dpad_up = true;
        tester.init(rig.context());
        rig.initCycle(tester);
        rig.start(tester);
        rig.runCycle(tester);
        assertEquals(0.0, motor.power, EPSILON);

        rig.gamepad.a = false;
        rig.gamepad.dpad_up = false;
        rig.runCycle(tester);
        rig.gamepad.a = true;
        rig.gamepad.dpad_up = true;
        rig.runCycle(tester);
        assertEquals(0.0, motor.power, EPSILON);

        rig.neutral();
        rig.runCycle(tester);
        rig.gamepad.a = true;
        rig.runCycle(tester);
        assertEquals(0.0, motor.power, EPSILON);
        tester.stop();
    }

    private static void assertPositionHeldStopPriority() {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor(
                "lift", DcMotor.RunMode.RUN_WITHOUT_ENCODER, 100);
        Rig rig = new Rig(hardware);
        DcMotorPositionTester tester = new DcMotorPositionTester("lift");
        rig.gamepad.a = true;
        rig.gamepad.b = true;
        rig.gamepad.dpad_up = true;
        tester.init(rig.context());
        rig.initCycle(tester);
        rig.start(tester);
        rig.runCycle(tester);
        assertEquals(0.0, motor.power, EPSILON);

        rig.gamepad.a = false;
        rig.gamepad.dpad_up = false;
        rig.runCycle(tester);
        rig.gamepad.a = true;
        rig.gamepad.dpad_up = true;
        rig.runCycle(tester);
        assertEquals(0.0, motor.power, EPSILON);

        rig.neutral();
        rig.runCycle(tester);
        rig.gamepad.a = true;
        rig.runCycle(tester);
        assertEquals(0.0, motor.power, EPSILON);
        tester.stop();
    }

    private static void assertVelocityHeldStopPriority() {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor(
                "flywheel", DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0);
        Rig rig = new Rig(hardware);
        DcMotorVelocityTester tester = new DcMotorVelocityTester("flywheel");
        rig.gamepad.a = true;
        rig.gamepad.b = true;
        rig.gamepad.dpad_up = true;
        tester.init(rig.context());
        rig.initCycle(tester);
        rig.start(tester);
        rig.runCycle(tester);
        assertEquals(0.0, motor.velocity, EPSILON);

        rig.gamepad.a = false;
        rig.gamepad.dpad_up = false;
        rig.runCycle(tester);
        rig.gamepad.a = true;
        rig.gamepad.dpad_up = true;
        rig.runCycle(tester);
        assertEquals(0.0, motor.velocity, EPSILON);

        rig.neutral();
        rig.runCycle(tester);
        rig.gamepad.a = true;
        rig.runCycle(tester);
        assertEquals(0.0, motor.velocity, EPSILON);
        tester.stop();
    }

    private static void assertExtremePositionAnchor(int currentPosition, boolean increase) {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor(
                "lift", DcMotor.RunMode.RUN_WITHOUT_ENCODER, currentPosition);
        Rig rig = new Rig(hardware);
        DcMotorPositionTester tester = new DcMotorPositionTester("lift");
        tester.init(rig.context());
        rig.initCycle(tester);
        rig.start(tester);

        rig.neutral();
        rig.runCycle(tester);
        if (increase) {
            rig.gamepad.dpad_up = true;
        } else {
            rig.gamepad.dpad_down = true;
        }
        rig.runCycle(tester);
        rig.neutral();
        rig.runCycle(tester);
        rig.gamepad.a = true;
        rig.runCycle(tester);

        assertEquals(currentPosition, motor.targetPosition);
        assertEquals(0.10, motor.power, EPSILON);
        assertFalse(motor.modeWrites.contains(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        tester.stop();
    }

    private static void assertResetModeRejected(
            Function<String, ? extends TeleOpTester> testerFactory) {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor(
                "motor", DcMotor.RunMode.STOP_AND_RESET_ENCODER, 0);
        Rig rig = new Rig(hardware);
        TeleOpTester tester = testerFactory.apply("motor");
        tester.init(rig.context());
        rig.initCycle(tester);

        assertNoActuatorWrites(motor);
        RuntimeException failure = assertThrows(RuntimeException.class, () -> rig.start(tester));
        assertTrue(failure.getMessage().contains("STOP_AND_RESET_ENCODER"));
        assertNoActuatorWrites(motor);
        assertEquals(DcMotor.RunMode.STOP_AND_RESET_ENCODER, motor.mode);
    }

    private static double fieldDouble(Object target, String name) throws Exception {
        Field field = target.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return field.getDouble(target);
    }

    private static void assertNoActuatorWrites(MotorProbe motor) {
        assertEquals(0, motor.powerWrites.size());
        assertEquals(0, motor.velocityWrites.size());
        assertEquals(0, motor.modeWrites.size());
        assertEquals(0, motor.targetPositionWrites);
        assertEquals(0, motor.directionWrites);
        assertEquals(0, motor.zeroPowerBehaviorWrites);
    }

    private static final class Rig {
        final Gamepad gamepad = new Gamepad();
        final ManualLoopClock time = new ManualLoopClock();
        final RecordingTelemetry telemetry = new RecordingTelemetry();
        private final TesterContext context;

        Rig(HardwareMap hardware) {
            context = new TesterContext(
                    hardware,
                    telemetry.proxy(),
                    gamepad,
                    new Gamepad(),
                    time.clock());
        }

        TesterContext context() {
            return context;
        }

        void initCycle(TeleOpTester tester) {
            time.nextCycle(0.02);
            tester.initLoop(time.clock().dtSec());
        }

        void start(TeleOpTester tester) {
            time.clock().reset(time.clock().nowSec());
            tester.start();
        }

        void runCycle(TeleOpTester tester) {
            time.nextCycle(0.02);
            tester.loop(time.clock().dtSec());
        }

        void neutral() {
            gamepad.a = false;
            gamepad.b = false;
            gamepad.x = false;
            gamepad.y = false;
            gamepad.start = false;
            gamepad.dpad_up = false;
            gamepad.dpad_down = false;
            gamepad.dpad_left = false;
            gamepad.dpad_right = false;
            gamepad.left_bumper = false;
            gamepad.right_bumper = false;
            gamepad.left_stick_x = 0.0f;
            gamepad.left_stick_y = 0.0f;
            gamepad.right_stick_x = 0.0f;
            gamepad.right_stick_y = 0.0f;
        }
    }

    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> devices = new HashMap<>();

        TestHardwareMap() {
            super(null, null);
        }

        MotorProbe addMotor(String name, DcMotor.RunMode mode, int currentPosition) {
            MotorProbe probe = new MotorProbe(mode, currentPosition);
            devices.put(name, probe.motor);
            return probe;
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            HardwareDevice device = devices.get(name);
            if (device == null || !type.isInstance(device)) {
                throw new IllegalArgumentException("No " + type.getSimpleName() + " named " + name);
            }
            return type.cast(device);
        }

        @Override
        public SortedSet<String> getAllNames(Class<? extends HardwareDevice> type) {
            SortedSet<String> names = new TreeSet<>();
            for (Map.Entry<String, HardwareDevice> entry : devices.entrySet()) {
                if (type.isInstance(entry.getValue())) {
                    names.add(entry.getKey());
                }
            }
            return names;
        }
    }

    private static final class MotorProbe {
        final DcMotorEx motor;
        final List<Double> powerWrites = new ArrayList<>();
        final List<Double> velocityWrites = new ArrayList<>();
        final List<DcMotor.RunMode> modeWrites = new ArrayList<>();

        double power;
        double velocity;
        int currentPosition;
        int targetPosition;
        int targetPositionWrites;
        int directionWrites;
        int zeroPowerBehaviorWrites;
        DcMotor.RunMode mode;
        DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        RuntimeException failNextPowerWrite;
        RuntimeException failNextVelocityWrite;
        RuntimeException failNextCurrentPositionRead;
        RuntimeException failNextVelocityRead;
        RuntimeException failNextBusyRead;
        RuntimeException failNextModeRead;
        int remainingPowerWriteFailures;

        MotorProbe(DcMotor.RunMode mode, int currentPosition) {
            this.mode = mode;
            this.currentPosition = currentPosition;
            motor = (DcMotorEx) Proxy.newProxyInstance(
                    DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class},
                    this::invoke);
        }

        boolean onlyZeroPowerWasWritten() {
            if (powerWrites.isEmpty()) return false;
            for (double value : powerWrites) {
                if (value != 0.0) return false;
            }
            return true;
        }

        boolean onlyZeroVelocityWasWritten() {
            if (velocityWrites.isEmpty()) return false;
            for (double value : velocityWrites) {
                if (value != 0.0) return false;
            }
            return true;
        }

        int totalWrites() {
            return powerWrites.size()
                    + velocityWrites.size()
                    + modeWrites.size()
                    + targetPositionWrites
                    + directionWrites
                    + zeroPowerBehaviorWrites;
        }

        private Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, name, args, "MotorProbe");
            }
            if ("setPower".equals(name)) {
                double requested = (Double) args[0];
                powerWrites.add(requested);
                if (remainingPowerWriteFailures > 0) {
                    remainingPowerWriteFailures--;
                    throw new IllegalStateException("injected repeated power write failure");
                }
                if (failNextPowerWrite != null) {
                    RuntimeException failure = failNextPowerWrite;
                    failNextPowerWrite = null;
                    throw failure;
                }
                power = requested;
                return null;
            }
            if ("getPower".equals(name)) return power;
            if ("setVelocity".equals(name)) {
                velocity = (Double) args[0];
                velocityWrites.add(velocity);
                if (failNextVelocityWrite != null) {
                    RuntimeException failure = failNextVelocityWrite;
                    failNextVelocityWrite = null;
                    throw failure;
                }
                return null;
            }
            if ("getVelocity".equals(name)) {
                if (failNextVelocityRead != null) {
                    RuntimeException failure = failNextVelocityRead;
                    failNextVelocityRead = null;
                    throw failure;
                }
                return velocity;
            }
            if ("setMode".equals(name)) {
                mode = (DcMotor.RunMode) args[0];
                modeWrites.add(mode);
                return null;
            }
            if ("getMode".equals(name)) {
                if (failNextModeRead != null) {
                    RuntimeException failure = failNextModeRead;
                    failNextModeRead = null;
                    throw failure;
                }
                return mode;
            }
            if ("setTargetPosition".equals(name)) {
                targetPosition = (Integer) args[0];
                targetPositionWrites++;
                return null;
            }
            if ("getTargetPosition".equals(name)) return targetPosition;
            if ("getCurrentPosition".equals(name)) {
                if (failNextCurrentPositionRead != null) {
                    RuntimeException failure = failNextCurrentPositionRead;
                    failNextCurrentPositionRead = null;
                    throw failure;
                }
                return currentPosition;
            }
            if ("setDirection".equals(name)) {
                direction = (DcMotorSimple.Direction) args[0];
                directionWrites++;
                return null;
            }
            if ("getDirection".equals(name)) return direction;
            if ("setZeroPowerBehavior".equals(name)) {
                zeroPowerBehavior = (DcMotor.ZeroPowerBehavior) args[0];
                zeroPowerBehaviorWrites++;
                return null;
            }
            if ("getZeroPowerBehavior".equals(name)) return zeroPowerBehavior;
            if ("isBusy".equals(name)) {
                if (failNextBusyRead != null) {
                    RuntimeException failure = failNextBusyRead;
                    failNextBusyRead = null;
                    throw failure;
                }
                return false;
            }
            return hardwareDeviceOrDefault(method);
        }
    }

    private static final class RecordingTelemetry {
        private final Map<String, String> data = new HashMap<>();

        Telemetry proxy() {
            return (Telemetry) Proxy.newProxyInstance(
                    Telemetry.class.getClassLoader(),
                    new Class<?>[]{Telemetry.class},
                    (proxy, method, args) -> {
                        if (method.getDeclaringClass() == Object.class) {
                            return objectMethod(proxy, method.getName(), args, "TelemetryProbe");
                        }
                        if ("clearAll".equals(method.getName())) {
                            data.clear();
                        } else if ("addData".equals(method.getName())
                                && args != null && args.length >= 2) {
                            data.put(String.valueOf(args[0]), renderTelemetryValue(args));
                        }
                        if (method.getReturnType() == boolean.class) return true;
                        return defaultValue(method.getReturnType());
                    });
        }

        void assertDataContains(String caption, String expectedText) {
            String actual = data.get(caption);
            assertTrue(
                    "Expected telemetry '" + caption + "' to contain '" + expectedText
                            + "' but was '" + actual + "'",
                    actual != null && actual.contains(expectedText));
        }

        private static String renderTelemetryValue(Object[] args) {
            if (args.length >= 3 && args[1] instanceof String && args[2] instanceof Object[]) {
                return String.format(
                        Locale.US,
                        (String) args[1],
                        (Object[]) args[2]);
            }
            return String.valueOf(args[1]);
        }
    }

    private static Object objectMethod(Object proxy,
                                       String methodName,
                                       Object[] args,
                                       String label) {
        if ("equals".equals(methodName)) return proxy == args[0];
        if ("hashCode".equals(methodName)) return System.identityHashCode(proxy);
        if ("toString".equals(methodName)) return label;
        return null;
    }

    private static Object hardwareDeviceOrDefault(Method method) {
        if ("getManufacturer".equals(method.getName())) return HardwareDevice.Manufacturer.Other;
        if ("getDeviceName".equals(method.getName())) return "TESTER-02 probe";
        if ("getConnectionInfo".equals(method.getName())) return "test";
        if ("getVersion".equals(method.getName())) return 1;
        return defaultValue(method.getReturnType());
    }

    private static Object defaultValue(Class<?> type) {
        if (!type.isPrimitive()) return null;
        if (type == boolean.class) return false;
        if (type == byte.class) return (byte) 0;
        if (type == short.class) return (short) 0;
        if (type == int.class) return 0;
        if (type == long.class) return 0L;
        if (type == float.class) return 0.0f;
        if (type == double.class) return 0.0;
        if (type == char.class) return '\0';
        return null;
    }
}
