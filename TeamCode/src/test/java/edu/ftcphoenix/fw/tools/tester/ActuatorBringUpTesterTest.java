package edu.ftcphoenix.fw.tools.tester;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.SortedSet;
import java.util.TreeSet;

import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.ftc.ui.MenuItem;
import edu.ftcphoenix.fw.ftc.ui.SelectionMenu;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/** Focused software-contract coverage for the canonical actuator bring-up wizard. */
public final class ActuatorBringUpTesterTest {

    private static final double EPSILON = 1e-12;

    @Test
    public void pickerCombinesTypedDevicesSortsThemAndPreservesStableSelection() throws Exception {
        TestHardwareMap hardware = new TestHardwareMap();
        hardware.addMotor("zetaMotor");
        hardware.addCrServo("betaWheel");
        hardware.addServo("gammaClaw", 0.4);
        hardware.addMotor("alphaMotor");

        Rig rig = new Rig(hardware);
        ActuatorBringUpTester tester = new ActuatorBringUpTester(message -> { });
        tester.init(rig.context());
        rig.initCycle(tester);

        SelectionMenu<?> picker = field(tester, "picker", SelectionMenu.class);
        assertEquals(
                Arrays.asList(
                        "[CR servo] betaWheel",
                        "[DC motor] alphaMotor",
                        "[DC motor] zetaMotor",
                        "[Servo] gammaClaw"),
                labels(picker.itemsSnapshot()));

        assertTrue(picker.setSelectedId("SERVO:gammaClaw"));
        hardware.addCrServo("aardvarkWheel");
        rig.gamepad1.x = true;
        rig.initCycle(tester);
        rig.gamepad1.x = false;
        rig.initCycle(tester);

        assertEquals("SERVO:gammaClaw", picker.selectedIdOrNull());
        assertEquals(
                Arrays.asList(
                        "[CR servo] aardvarkWheel",
                        "[CR servo] betaWheel",
                        "[DC motor] alphaMotor",
                        "[DC motor] zetaMotor",
                        "[Servo] gammaClaw"),
                labels(picker.itemsSnapshot()));
    }

    @Test
    public void selectingAndLeavingEveryDeviceDuringInitPerformsNoActuatorWrite() throws Exception {
        assertInitSelectionHasNoWrites("DC_MOTOR:motor", DeviceKindForTest.MOTOR);
        assertInitSelectionHasNoWrites("CR_SERVO:wheel", DeviceKindForTest.CR_SERVO);
        assertInitSelectionHasNoWrites("SERVO:claw", DeviceKindForTest.SERVO);
    }

    @Test
    public void heldSelectionThroughStartRequiresReleaseAndFreshArm() throws Exception {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor("lift");
        Rig rig = new Rig(hardware);
        ActuatorBringUpTester tester = new ActuatorBringUpTester(message -> { });
        tester.init(rig.context());
        rig.initCycle(tester);

        selectDuringInit(tester, rig, "DC_MOTOR:lift", true);
        assertEquals(0, motor.totalWrites());

        rig.start(tester);
        rig.runCycle(tester);
        rig.runCycle(tester);
        assertEquals(0, motor.totalWrites());

        rig.gamepad1.a = false;
        rig.runCycle(tester);
        assertEquals(0, motor.totalWrites());

        rig.gamepad1.a = true;
        rig.runCycle(tester);
        assertTrue(motor.powerWrites > 0);
        assertEquals(0.0, motor.power, EPSILON);
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motor.mode);
        assertFalse(motor.modeWrites.contains(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
    }

    @Test
    public void heldDisarmCannotBeBypassedByFreshArm() throws Exception {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor("lift");
        Rig rig = new Rig(hardware);
        ActuatorBringUpTester tester = readyRunningTester(hardware, rig, "DC_MOTOR:lift");

        rig.gamepad1.b = true;
        arm(tester, rig);
        rig.gamepad1.right_bumper = true;
        rig.runCycle(tester);
        assertEquals(0.0, motor.power, EPSILON);
        assertFalse(fieldBoolean(tester, "armed"));

        rig.gamepad1.b = false;
        rig.gamepad1.right_bumper = false;
        rig.runCycle(tester);
        arm(tester, rig);
        rig.gamepad1.right_bumper = true;
        rig.runCycle(tester);
        assertEquals(0.05, motor.power, EPSILON);
    }

    @Test
    public void motorDeadmanConflictDisarmDirectionCaptureAndBackAreSafe() throws Exception {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor("arm");
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER;
        motor.direction = DcMotorSimple.Direction.FORWARD;
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        Rig rig = new Rig(hardware);
        ActuatorBringUpTester tester = readyRunningTester(hardware, rig, "DC_MOTOR:arm");

        arm(tester, rig);
        rig.gamepad1.right_bumper = true;
        rig.runCycle(tester);
        assertEquals(0.05, motor.power, EPSILON);

        rig.gamepad1.right_bumper = false;
        rig.runCycle(tester);
        assertEquals(0.0, motor.power, EPSILON);

        rig.gamepad1.left_bumper = true;
        rig.gamepad1.right_bumper = true;
        rig.runCycle(tester);
        assertEquals(0.0, motor.power, EPSILON);

        // Resolving a two-button conflict to one held button remains zero until both are neutral.
        rig.gamepad1.left_bumper = false;
        rig.runCycle(tester);
        assertEquals(0.0, motor.power, EPSILON);
        rig.gamepad1.right_bumper = false;
        rig.runCycle(tester);
        rig.gamepad1.right_bumper = true;
        rig.runCycle(tester);
        assertEquals(0.05, motor.power, EPSILON);

        // A is arm/rearm only. It cannot become a second disarm verb.
        rig.gamepad1.a = true;
        rig.runCycle(tester);
        assertEquals(0.05, motor.power, EPSILON);
        rig.gamepad1.a = false;
        rig.gamepad1.right_bumper = false;
        rig.runCycle(tester);

        pressB(tester, rig);
        assertEquals(0.0, motor.power, EPSILON);

        motor.currentPosition = 10;
        tapDpadLeft(tester, rig);
        motor.currentPosition = 20;
        tapDpadRight(tester, rig);
        assertTrue(fieldBoolean(tester, "plantMinCaptured"));
        assertTrue(fieldBoolean(tester, "plantMaxCaptured"));

        tapX(tester, rig);
        assertEquals(DcMotorSimple.Direction.REVERSE, motor.direction);
        assertFalse(fieldBoolean(tester, "plantMinCaptured"));
        assertFalse(fieldBoolean(tester, "plantMaxCaptured"));
        assertEquals(0.0, motor.power, EPSILON);

        assertTrue(tester.onBackPressed());
        assertEquals(0.0, motor.power, EPSILON);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.mode);
        assertEquals(DcMotorSimple.Direction.FORWARD, motor.direction);
        assertEquals(DcMotor.ZeroPowerBehavior.BRAKE, motor.zeroPowerBehavior);
        assertFalse(motor.modeWrites.contains(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
    }

    @Test
    public void motorExtremeEndpointSpanLogsExactlyOnceWithoutResettingEncoder() throws Exception {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor("slide");
        List<String> logs = new ArrayList<String>();
        Rig rig = new Rig(hardware);
        ActuatorBringUpTester tester = readyRunningTester(hardware, rig, "DC_MOTOR:slide", logs);

        arm(tester, rig);
        rig.gamepad1.right_bumper = true;
        rig.runCycle(tester);
        rig.gamepad1.right_bumper = false;
        rig.runCycle(tester);
        pressB(tester, rig);
        motor.currentPosition = Integer.MIN_VALUE;
        tapDpadLeft(tester, rig);
        motor.currentPosition = Integer.MAX_VALUE;
        tapDpadRight(tester, rig);
        tapY(tester, rig);

        assertEquals(1, logs.size());
        assertTrue(logs.get(0).contains("nativeAtPlantMinTicks=-2147483648"));
        assertTrue(logs.get(0).contains("nativeAtPlantMaxTicks=2147483647"));
        assertTrue(logs.get(0).contains("signedSafeTravelTicks=4294967295"));
        assertTrue(logs.get(0).contains("absoluteSafeTravelTicks=4294967295"));
        assertTrue(rig.telemetry.contains(".bounded(0.0, 4294967295.0)"));

        rig.runCycle(tester);
        rig.gamepad1.y = true;
        rig.runCycle(tester);
        assertEquals(1, logs.size());
        assertFalse(motor.modeWrites.contains(DcMotor.RunMode.STOP_AND_RESET_ENCODER));

        tester.stop();
        assertEquals(0.0, motor.power, EPSILON);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.mode);
    }

    @Test
    public void crServoUsesDeadmanHasNoBoundsAndStopRestoresDirection() throws Exception {
        TestHardwareMap hardware = new TestHardwareMap();
        CrServoProbe servo = hardware.addCrServo("roller");
        List<String> logs = new ArrayList<String>();
        Rig rig = new Rig(hardware);
        ActuatorBringUpTester tester = readyRunningTester(hardware, rig, "CR_SERVO:roller", logs);

        arm(tester, rig);
        rig.gamepad1.right_bumper = true;
        rig.runCycle(tester);
        assertEquals(0.05, servo.power, EPSILON);
        rig.gamepad1.right_bumper = false;
        rig.runCycle(tester);
        assertEquals(0.0, servo.power, EPSILON);

        rig.gamepad1.left_bumper = true;
        rig.gamepad1.right_bumper = true;
        rig.runCycle(tester);
        assertEquals(0.0, servo.power, EPSILON);
        rig.gamepad1.left_bumper = false;
        rig.gamepad1.right_bumper = false;
        rig.runCycle(tester);

        pressB(tester, rig);
        tapX(tester, rig);
        assertEquals(DcMotorSimple.Direction.REVERSE, servo.direction);
        tapDpadLeft(tester, rig);
        assertFalse(fieldBoolean(tester, "plantMinCaptured"));
        assertTrue(rig.telemetry.contains("no positional endpoint"));

        arm(tester, rig);
        rig.gamepad1.right_bumper = true;
        rig.runCycle(tester);
        rig.gamepad1.right_bumper = false;
        rig.runCycle(tester);
        pressB(tester, rig);
        tapY(tester, rig);
        assertEquals(1, logs.size());
        assertTrue(logs.get(0).contains("UNAVAILABLE_WITHOUT_EXTERNAL_FEEDBACK"));
        tester.stop();

        assertEquals(0.0, servo.power, EPSILON);
        assertEquals(DcMotorSimple.Direction.FORWARD, servo.direction);
    }

    @Test
    public void servoJogsGraduallyRequiresWizardWriteAndAllowsReversedEndpointMap() throws Exception {
        TestHardwareMap hardware = new TestHardwareMap();
        ServoProbe servo = hardware.addServo("wrist", 0.4);
        List<String> logs = new ArrayList<String>();
        Rig rig = new Rig(hardware);
        ActuatorBringUpTester tester = readyRunningTester(hardware, rig, "SERVO:wrist", logs);

        arm(tester, rig);
        pressB(tester, rig);
        tapDpadLeft(tester, rig);
        assertFalse(fieldBoolean(tester, "plantMinCaptured"));
        assertTrue(rig.telemetry.contains("getPosition() value is SDK command state"));

        arm(tester, rig);
        rig.gamepad1.right_bumper = true;
        rig.runCycle(tester, 0.5);
        rig.gamepad1.right_bumper = false;
        rig.runCycle(tester);
        assertEquals(0.405, servo.lastLogicalCommand, EPSILON);
        assertEquals(1, servo.successfulPositionWrites);

        pressB(tester, rig);
        double physicalBeforeDirection = servo.physicalCommand;
        tapX(tester, rig);
        assertEquals(Servo.Direction.REVERSE, servo.direction);
        assertEquals(physicalBeforeDirection, servo.physicalCommand, EPSILON);
        assertFalse(fieldBoolean(tester, "plantMinCaptured"));
        assertFalse(fieldBoolean(tester, "servoCommandSubmittedInRun"));

        // Under the selected reverse direction, submit a new command before capturing evidence.
        arm(tester, rig);
        rig.gamepad1.right_bumper = true;
        rig.runCycle(tester);
        rig.gamepad1.right_bumper = false;
        rig.runCycle(tester);
        pressB(tester, rig);
        tapDpadLeft(tester, rig);
        double nativeAtPlantMin = fieldDouble(tester, "servoNativeAtPlantMin");

        arm(tester, rig);
        rig.gamepad1.left_bumper = true;
        rig.runCycle(tester);
        rig.gamepad1.left_bumper = false;
        rig.runCycle(tester);
        pressB(tester, rig);
        tapDpadRight(tester, rig);
        double nativeAtPlantMax = fieldDouble(tester, "servoNativeAtPlantMax");
        assertTrue(nativeAtPlantMax < nativeAtPlantMin);

        tapY(tester, rig);
        assertEquals(1, logs.size());
        assertTrue(logs.get(0).contains(
                "nativeAtPlantMin=" + Double.toString(nativeAtPlantMin)));
        assertTrue(logs.get(0).contains(
                "nativeAtPlantMax=" + Double.toString(nativeAtPlantMax)));
        assertTrue(rig.telemetry.contains(
                ".rangeMapsToNative(" + Double.toString(nativeAtPlantMin)
                        + ", " + Double.toString(nativeAtPlantMax) + ")"));

        double physicalBeforeBack = servo.physicalCommand;
        assertTrue(tester.onBackPressed());
        assertEquals(Servo.Direction.FORWARD, servo.direction);
        assertEquals(physicalBeforeBack, servo.physicalCommand, EPSILON);
    }

    @Test
    public void failedServoWriteDoesNotAdvanceCommandOrCaptureEvidence() throws Exception {
        TestHardwareMap hardware = new TestHardwareMap();
        ServoProbe servo = hardware.addServo("gate", 0.25);
        Rig rig = new Rig(hardware);
        ActuatorBringUpTester tester = readyRunningTester(hardware, rig, "SERVO:gate");

        arm(tester, rig);
        servo.failNextPositionWrite = true;
        rig.gamepad1.right_bumper = true;
        assertThrows(IllegalStateException.class, () -> rig.runCycle(tester));

        assertEquals(0.25, servo.physicalCommand, EPSILON);
        assertEquals(0.25, fieldDouble(tester, "servoLastSuccessfulCommand"), EPSILON);
        assertFalse(fieldBoolean(tester, "servoCommandSubmittedInRun"));
        assertEquals(0, servo.successfulPositionWrites);
        tester.stop();
    }

    @Test
    public void uncachedServoBootstrapRequiresTwoFreshPressesAndHeldBResetsConfirmation()
            throws Exception {
        TestHardwareMap hardware = new TestHardwareMap();
        ServoProbe servo = hardware.addServo("newServo", Double.NaN);
        Rig rig = new Rig(hardware);
        ActuatorBringUpTester tester = new ActuatorBringUpTester(message -> { });
        tester.init(rig.context());
        rig.initCycle(tester);
        selectDuringInit(tester, rig, "SERVO:newServo", false);

        assertEquals(0, servo.positionWrites);
        assertTrue(fieldBoolean(tester, "servoBootstrapRequired"));
        assertTrue(Double.isNaN(fieldDouble(tester, "servoLastSuccessfulCommand")));

        rig.start(tester);
        rig.runCycle(tester);
        arm(tester, rig); // First press acknowledges only.
        assertEquals(0, servo.positionWrites);
        assertTrue(fieldBoolean(tester, "servoBootstrapConfirmed"));
        assertFalse(fieldBoolean(tester, "prepared"));

        // B has priority even when the dangerous second A arrives in the same cycle.
        rig.gamepad1.a = true;
        rig.gamepad1.b = true;
        rig.runCycle(tester);
        assertEquals(0, servo.positionWrites);
        assertFalse(fieldBoolean(tester, "servoBootstrapConfirmed"));
        rig.gamepad1.a = false;
        rig.gamepad1.b = false;
        rig.runCycle(tester);

        // A held or freshly pressed while B remains high cannot re-establish confirmation.
        rig.gamepad1.b = true;
        rig.runCycle(tester);
        arm(tester, rig); // Held B must cancel, not allow the second-stage write.
        assertEquals(0, servo.positionWrites);
        assertFalse(fieldBoolean(tester, "servoBootstrapConfirmed"));

        rig.gamepad1.b = false;
        rig.runCycle(tester);
        arm(tester, rig); // Acknowledge again after B.
        assertEquals(0, servo.positionWrites);
        arm(tester, rig); // Second fresh press performs the explicit bootstrap.

        assertEquals(1, servo.positionWrites);
        assertEquals(1, servo.successfulPositionWrites);
        assertEquals(0.5, servo.lastLogicalCommand, EPSILON);
        assertEquals(0.5, fieldDouble(tester, "servoLastSuccessfulCommand"), EPSILON);
        assertTrue(fieldBoolean(tester, "prepared"));
        assertTrue(fieldBoolean(tester, "armed"));
        assertTrue(fieldBoolean(tester, "servoCommandSubmittedInRun"));
        assertFalse(fieldBoolean(tester, "nonzeroJogSubmitted"));

        rig.gamepad1.right_bumper = true;
        rig.runCycle(tester);
        rig.gamepad1.right_bumper = false;
        rig.runCycle(tester);
        pressB(tester, rig);
        tapDpadLeft(tester, rig);
        assertTrue(fieldBoolean(tester, "plantMinCaptured"));
        assertTrue(fieldBoolean(tester, "nonzeroJogSubmitted"));

        assertTrue(tester.onBackPressed());
        assertEquals(Servo.Direction.FORWARD, servo.direction);
        assertTrue(Double.isFinite(servo.physicalCommand));
    }

    @Test
    public void failedServoBootstrapDoesNotCreateSuccessfulCommandEvidence() throws Exception {
        TestHardwareMap hardware = new TestHardwareMap();
        ServoProbe servo = hardware.addServo("newServo", Double.NaN);
        Rig rig = new Rig(hardware);
        ActuatorBringUpTester tester = readyRunningTester(
                hardware, rig, "SERVO:newServo");

        arm(tester, rig); // Acknowledge.
        servo.failNextPositionWrite = true;
        servo.applyPositionBeforeFailure = true;
        rig.gamepad1.a = true;
        assertThrows(IllegalStateException.class, () -> rig.runCycle(tester));

        assertEquals(1, servo.positionWrites);
        assertEquals(0, servo.successfulPositionWrites);
        assertTrue(Double.isNaN(fieldDouble(tester, "servoLastSuccessfulCommand")));
        assertFalse(fieldBoolean(tester, "servoCommandSubmittedInRun"));
        assertFalse(fieldBoolean(tester, "prepared"));
        assertFalse(fieldBoolean(tester, "nonzeroJogSubmitted"));
        tester.stop();
        assertEquals(0.5, servo.physicalCommand, EPSILON);
        assertEquals(1, servo.directionWrites);
    }

    @Test
    public void childSessionPreservesBootstrapFailureAndAllowsReplacementAfterCleanup()
            throws Exception {
        assertBootstrapFailureSession(false);
    }

    @Test
    public void childSessionSuppressesServoCleanupFailureAndBlocksReplacement()
            throws Exception {
        assertBootstrapFailureSession(true);
    }

    @Test
    public void nonNanInvalidServoCommandStateIsRejectedWithoutWrites() throws Exception {
        double[] invalidStates = {-0.01, 1.01, Double.POSITIVE_INFINITY};
        for (double invalidState : invalidStates) {
            TestHardwareMap hardware = new TestHardwareMap();
            ServoProbe servo = hardware.addServo("invalidServo", invalidState);
            Rig rig = new Rig(hardware);
            ActuatorBringUpTester tester = new ActuatorBringUpTester(message -> { });
            tester.init(rig.context());
            rig.initCycle(tester);
            selectDuringInit(tester, rig, "SERVO:invalidServo", false);

            assertEquals(0, servo.totalWrites());
            assertEquals("PICKER", field(tester, "screen", Object.class).toString());
            assertTrue(rig.telemetry.contains("invalid SDK command state"));
        }
    }

    @Test
    public void finalizationRequiresASuccessfulNonzeroJogUnderCurrentDirection()
            throws Exception {
        TestHardwareMap hardware = new TestHardwareMap();
        hardware.addMotor("intake");
        Rig rig = new Rig(hardware);
        List<String> logs = new ArrayList<String>();
        ActuatorBringUpTester tester = readyRunningTester(
                hardware, rig, "DC_MOTOR:intake", logs);

        arm(tester, rig);
        pressB(tester, rig);
        tapY(tester, rig);
        assertEquals(0, logs.size());
        assertTrue(rig.telemetry.contains("successful nonzero jog"));

        arm(tester, rig);
        rig.gamepad1.right_bumper = true;
        rig.runCycle(tester);
        rig.gamepad1.right_bumper = false;
        rig.runCycle(tester);
        pressB(tester, rig);
        tapY(tester, rig);
        assertEquals(1, logs.size());
        assertTrue(logs.get(0).contains("directionEvidence=NONZERO_JOG_SUBMITTED"));
    }

    @Test
    public void signedZeroServoEndpointsAreRejectedAsEqualAndRemainRetryable() throws Exception {
        TestHardwareMap hardware = new TestHardwareMap();
        ServoProbe servo = hardware.addServo("zeroGate", 0.0);
        Rig rig = new Rig(hardware);
        List<String> logs = new ArrayList<String>();
        ActuatorBringUpTester tester = readyRunningTester(hardware, rig, "SERVO:zeroGate", logs);

        // Inject the already-proven capture facts to isolate numeric endpoint validation.
        setField(tester, "prepared", true);
        setField(tester, "armed", false);
        setField(tester, "nonzeroJogSubmitted", true);
        setField(tester, "plantMinCaptured", true);
        setField(tester, "plantMaxCaptured", true);
        setField(tester, "servoNativeAtPlantMin", +0.0);
        setField(tester, "servoNativeAtPlantMax", -0.0);
        tapY(tester, rig);

        assertEquals(0, logs.size());
        assertTrue(rig.telemetry.contains("must be different"));
        assertEquals("ACTIVE", field(tester, "screen", Object.class).toString());
        assertEquals(0, servo.positionWrites);
    }

    @Test
    public void signedZeroClampDoesNotCreateFalseServoDirectionEvidence() throws Exception {
        TestHardwareMap hardware = new TestHardwareMap();
        ServoProbe servo = hardware.addServo("zeroGate", -0.0);
        Rig rig = new Rig(hardware);
        List<String> logs = new ArrayList<String>();
        ActuatorBringUpTester tester = readyRunningTester(
                hardware, rig, "SERVO:zeroGate", logs);

        arm(tester, rig);
        rig.gamepad1.left_bumper = true;
        rig.runCycle(tester);
        rig.gamepad1.left_bumper = false;
        rig.runCycle(tester);
        pressB(tester, rig);

        assertEquals(0, servo.positionWrites);
        assertFalse(fieldBoolean(tester, "nonzeroJogSubmitted"));
        tapY(tester, rig);
        assertEquals(0, logs.size());
        assertTrue(rig.telemetry.contains("successful nonzero jog"));
    }

    private enum DeviceKindForTest {
        MOTOR,
        CR_SERVO,
        SERVO
    }

    private static void assertInitSelectionHasNoWrites(String stableId,
                                                       DeviceKindForTest kind) throws Exception {
        TestHardwareMap hardware = new TestHardwareMap();
        MotorProbe motor = hardware.addMotor("motor");
        CrServoProbe crServo = hardware.addCrServo("wheel");
        ServoProbe servo = hardware.addServo("claw", 0.5);
        Rig rig = new Rig(hardware);
        ActuatorBringUpTester tester = new ActuatorBringUpTester(message -> { });
        tester.init(rig.context());
        rig.initCycle(tester);

        selectDuringInit(tester, rig, stableId, false);
        rig.initCycle(tester);

        assertEquals(0, motor.totalWrites());
        assertEquals(0, crServo.totalWrites());
        assertEquals(0, servo.totalWrites());
        assertTrue(tester.onBackPressed());
        assertEquals(0, motor.totalWrites());
        assertEquals(0, crServo.totalWrites());
        assertEquals(0, servo.totalWrites());

        // Keep the parameter meaningful in failure output and guard accidental fixture changes.
        assertTrue(kind == DeviceKindForTest.MOTOR
                || kind == DeviceKindForTest.CR_SERVO
                || kind == DeviceKindForTest.SERVO);
    }

    private static ActuatorBringUpTester readyRunningTester(TestHardwareMap hardware,
                                                            Rig rig,
                                                            String stableId) throws Exception {
        return readyRunningTester(hardware, rig, stableId, new ArrayList<String>());
    }

    private static ActuatorBringUpTester readyRunningTester(TestHardwareMap hardware,
                                                            Rig rig,
                                                            String stableId,
                                                            List<String> logs) throws Exception {
        ActuatorBringUpTester tester = new ActuatorBringUpTester(logs::add);
        tester.init(rig.context());
        rig.initCycle(tester);
        selectDuringInit(tester, rig, stableId, false);
        rig.start(tester);
        rig.runCycle(tester);
        return tester;
    }

    private static void assertBootstrapFailureSession(boolean cleanupFails) throws Exception {
        TestHardwareMap hardware = new TestHardwareMap();
        ServoProbe servo = hardware.addServo("newServo", Double.NaN);
        Rig rig = new Rig(hardware);
        ActuatorBringUpTester tester = new ActuatorBringUpTester(message -> { });
        TesterChildSession session = new TesterChildSession();
        session.retain(tester);
        assertNull(session.init(rig.context()));
        rig.initCycle(tester);
        selectDuringInit(tester, rig, "SERVO:newServo", false);
        rig.resetClockForStart();
        assertNull(session.start());
        assertNull(rig.runCycle(session));

        arm(tester, rig); // First A acknowledges; the second will fail through the session.
        servo.failNextPositionWrite = true;
        servo.applyPositionBeforeFailure = true;
        servo.failNextDirectionWrite = cleanupFails;
        rig.gamepad1.a = true;
        RuntimeException failure = rig.runCycle(session);

        assertNotNull(failure);
        assertTrue(failure.getMessage().contains("injected servo write failure"));
        assertEquals(1, servo.positionWrites);
        assertEquals(0, servo.successfulPositionWrites);
        assertEquals(0.5, servo.physicalCommand, EPSILON);
        assertEquals(1, servo.directionWrites);
        assertFalse(fieldBoolean(tester, "servoCommandSubmittedInRun"));
        assertFalse(fieldBoolean(tester, "nonzeroJogSubmitted"));

        if (cleanupFails) {
            assertEquals(1, failure.getSuppressed().length);
            assertTrue(failure.getSuppressed()[0].getMessage()
                    .contains("injected servo direction restore failure"));
            assertTrue(session.cleanupBlocked());
            assertFalse(session.canActivate());
            assertThrows(
                    IllegalStateException.class,
                    () -> session.retain(new ActuatorBringUpTester(message -> { })));
        } else {
            assertEquals(0, failure.getSuppressed().length);
            assertFalse(session.cleanupBlocked());
            assertTrue(session.canActivate());
            session.retain(new ActuatorBringUpTester(message -> { }));
        }
    }

    @Test
    public void backCleanupFailureIsRetainedAndAutomaticStopDoesNotReplayHardwareWrites()
            throws Exception {
        TestHardwareMap hardware = new TestHardwareMap();
        ServoProbe servo = hardware.addServo("wrist", 0.4);
        Rig rig = new Rig(hardware);
        ActuatorBringUpTester tester = new ActuatorBringUpTester(message -> { });
        TesterChildSession session = new TesterChildSession();
        session.retain(tester);
        assertNull(session.init(rig.context()));
        rig.initCycle(tester);
        selectDuringInit(tester, rig, "SERVO:wrist", false);
        rig.resetClockForStart();
        assertNull(session.start());
        assertNull(rig.runCycle(session));

        arm(tester, rig);
        pressB(tester, rig);
        tapX(tester, rig);
        assertEquals(1, servo.directionWrites);
        servo.failNextDirectionWrite = true;

        TesterChildSession.BackResult result = session.backPressed();

        assertFalse(result.handled());
        assertNotNull(result.failure());
        assertTrue(result.failure().getMessage()
                .contains("injected servo direction restore failure"));
        assertSame(result.failure(), session.lastFailure());
        assertEquals("cleanup plus automatic stop must attempt restoration only once",
                2, servo.directionWrites);
        assertTrue(session.cleanupBlocked());
        assertFalse(session.canActivate());
        assertThrows(
                IllegalStateException.class,
                () -> session.retain(new ActuatorBringUpTester(message -> { })));
    }

    private static void selectDuringInit(ActuatorBringUpTester tester,
                                         Rig rig,
                                         String stableId,
                                         boolean keepAHeld) throws Exception {
        SelectionMenu<?> picker = field(tester, "picker", SelectionMenu.class);
        assertTrue("Missing picker id " + stableId, picker.setSelectedId(stableId));
        rig.gamepad1.a = true;
        rig.initCycle(tester);
        if (!keepAHeld) {
            rig.gamepad1.a = false;
            rig.initCycle(tester);
        }
    }

    private static void arm(ActuatorBringUpTester tester, Rig rig) {
        rig.gamepad1.a = true;
        rig.runCycle(tester);
        rig.gamepad1.a = false;
        rig.runCycle(tester);
    }

    private static void pressB(ActuatorBringUpTester tester, Rig rig) {
        rig.gamepad1.b = true;
        rig.runCycle(tester);
        rig.gamepad1.b = false;
        rig.runCycle(tester);
    }

    private static void tapX(ActuatorBringUpTester tester, Rig rig) {
        rig.gamepad1.x = true;
        rig.runCycle(tester);
        rig.gamepad1.x = false;
        rig.runCycle(tester);
    }

    private static void tapY(ActuatorBringUpTester tester, Rig rig) {
        rig.gamepad1.y = true;
        rig.runCycle(tester);
        rig.gamepad1.y = false;
        rig.runCycle(tester);
    }

    private static void tapDpadLeft(ActuatorBringUpTester tester, Rig rig) {
        rig.gamepad1.dpad_left = true;
        rig.runCycle(tester);
        rig.gamepad1.dpad_left = false;
        rig.runCycle(tester);
    }

    private static void tapDpadRight(ActuatorBringUpTester tester, Rig rig) {
        rig.gamepad1.dpad_right = true;
        rig.runCycle(tester);
        rig.gamepad1.dpad_right = false;
        rig.runCycle(tester);
    }

    private static List<String> labels(List<? extends MenuItem<?>> items) {
        List<String> labels = new ArrayList<String>();
        for (MenuItem<?> item : items) labels.add(item.label);
        return labels;
    }

    private static boolean fieldBoolean(Object target, String name) throws Exception {
        return (Boolean) field(target, name, Object.class);
    }

    private static double fieldDouble(Object target, String name) throws Exception {
        return (Double) field(target, name, Object.class);
    }

    private static void setField(Object target, String name, Object value) throws Exception {
        Field field = target.getClass().getDeclaredField(name);
        field.setAccessible(true);
        field.set(target, value);
    }

    private static <T> T field(Object target, String name, Class<T> type) throws Exception {
        Field field = target.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return type.cast(field.get(target));
    }

    private static final class Rig {
        final Gamepad gamepad1 = new Gamepad();
        final RecordingTelemetry telemetry = new RecordingTelemetry();
        final LoopClock clock = new LoopClock();
        private final TesterContext context;
        private double nowSec;

        Rig(HardwareMap hardware) {
            clock.reset(0.0);
            context = new TesterContext(
                    hardware,
                    telemetry.proxy(),
                    gamepad1,
                    new Gamepad(),
                    clock);
        }

        TesterContext context() {
            return context;
        }

        void initCycle(TeleOpTester tester) {
            nowSec += 0.02;
            clock.update(nowSec);
            tester.initLoop(clock.dtSec());
        }

        void start(TeleOpTester tester) {
            resetClockForStart();
            tester.start();
        }

        void resetClockForStart() {
            clock.reset(nowSec);
        }

        void runCycle(TeleOpTester tester) {
            runCycle(tester, 0.02);
        }

        void runCycle(TeleOpTester tester, double dtSec) {
            nowSec += dtSec;
            clock.update(nowSec);
            tester.loop(clock.dtSec());
        }

        RuntimeException runCycle(TesterChildSession session) {
            nowSec += 0.02;
            clock.update(nowSec);
            return session.loop(clock.dtSec());
        }
    }

    private static final class RecordingTelemetry {
        private final List<String> entries = new ArrayList<String>();

        Telemetry proxy() {
            return (Telemetry) Proxy.newProxyInstance(
                    Telemetry.class.getClassLoader(),
                    new Class<?>[]{Telemetry.class},
                    (proxy, method, args) -> {
                        if (method.getDeclaringClass() == Object.class) {
                            return objectMethod(proxy, method, args, "TelemetryProbe");
                        }
                        if ("clearAll".equals(method.getName())) {
                            entries.clear();
                        } else if (("addData".equals(method.getName())
                                || "addLine".equals(method.getName())) && args != null) {
                            StringBuilder line = new StringBuilder();
                            for (Object arg : args) {
                                if (arg != null && arg.getClass().isArray()) {
                                    line.append(' ').append(Arrays.deepToString((Object[]) arg));
                                } else {
                                    line.append(' ').append(arg);
                                }
                            }
                            entries.add(line.toString());
                        }
                        if (method.getReturnType() == boolean.class) return true;
                        return defaultValue(method.getReturnType());
                    });
        }

        boolean contains(String text) {
            for (String entry : entries) {
                if (entry.contains(text)) return true;
            }
            return false;
        }
    }

    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> devices = new HashMap<String, HardwareDevice>();

        TestHardwareMap() {
            super(null, null);
        }

        MotorProbe addMotor(String name) {
            MotorProbe probe = new MotorProbe();
            devices.put(name, probe.motor);
            return probe;
        }

        CrServoProbe addCrServo(String name) {
            CrServoProbe probe = new CrServoProbe();
            devices.put(name, probe.servo);
            return probe;
        }

        ServoProbe addServo(String name, double initialPhysicalCommand) {
            ServoProbe probe = new ServoProbe(initialPhysicalCommand);
            devices.put(name, probe.servo);
            return probe;
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            HardwareDevice device = devices.get(name);
            if (device == null || !type.isInstance(device)) {
                throw new IllegalArgumentException(
                        "No " + type.getSimpleName() + " named '" + name + "'.");
            }
            return type.cast(device);
        }

        @Override
        public SortedSet<String> getAllNames(Class<? extends HardwareDevice> type) {
            SortedSet<String> names = new TreeSet<String>();
            for (Map.Entry<String, HardwareDevice> entry : devices.entrySet()) {
                if (type.isInstance(entry.getValue())) names.add(entry.getKey());
            }
            return names;
        }
    }

    private static final class MotorProbe {
        final DcMotorEx motor;
        double power;
        int currentPosition;
        DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;
        DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        int powerWrites;
        int directionWrites;
        int zeroPowerBehaviorWrites;
        final List<DcMotor.RunMode> modeWrites = new ArrayList<DcMotor.RunMode>();

        MotorProbe() {
            motor = (DcMotorEx) Proxy.newProxyInstance(
                    DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class},
                    this::invoke);
        }

        int totalWrites() {
            return powerWrites + directionWrites + zeroPowerBehaviorWrites + modeWrites.size();
        }

        private Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, method, args, "MotorProbe");
            }
            if ("setPower".equals(name)) {
                power = (Double) args[0];
                powerWrites++;
                return null;
            }
            if ("getPower".equals(name)) return power;
            if ("setMode".equals(name)) {
                mode = (DcMotor.RunMode) args[0];
                modeWrites.add(mode);
                return null;
            }
            if ("getMode".equals(name)) return mode;
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
            if ("getCurrentPosition".equals(name)) return currentPosition;
            if ("isBusy".equals(name)) return false;
            if ("setVelocity".equals(name)) return null;
            if ("getVelocity".equals(name)) return 0.0;
            return hardwareDeviceOrDefault(method);
        }
    }

    private static final class CrServoProbe {
        final CRServo servo;
        double power;
        DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        int powerWrites;
        int directionWrites;

        CrServoProbe() {
            servo = (CRServo) Proxy.newProxyInstance(
                    CRServo.class.getClassLoader(),
                    new Class<?>[]{CRServo.class},
                    this::invoke);
        }

        int totalWrites() {
            return powerWrites + directionWrites;
        }

        private Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, method, args, "CrServoProbe");
            }
            if ("setPower".equals(name)) {
                power = (Double) args[0];
                powerWrites++;
                return null;
            }
            if ("getPower".equals(name)) return power;
            if ("setDirection".equals(name)) {
                direction = (DcMotorSimple.Direction) args[0];
                directionWrites++;
                return null;
            }
            if ("getDirection".equals(name)) return direction;
            return hardwareDeviceOrDefault(method);
        }
    }

    private static final class ServoProbe {
        final Servo servo;
        double physicalCommand;
        double lastLogicalCommand;
        Servo.Direction direction = Servo.Direction.FORWARD;
        int positionWrites;
        int successfulPositionWrites;
        int directionWrites;
        boolean failNextPositionWrite;
        boolean applyPositionBeforeFailure;
        boolean failNextDirectionWrite;

        ServoProbe(double initialPhysicalCommand) {
            physicalCommand = initialPhysicalCommand;
            lastLogicalCommand = initialPhysicalCommand;
            servo = (Servo) Proxy.newProxyInstance(
                    Servo.class.getClassLoader(),
                    new Class<?>[]{Servo.class},
                    this::invoke);
        }

        int totalWrites() {
            return positionWrites + directionWrites;
        }

        private Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, method, args, "ServoProbe");
            }
            if ("setPosition".equals(name)) {
                double logical = (Double) args[0];
                positionWrites++;
                if (failNextPositionWrite) {
                    failNextPositionWrite = false;
                    if (applyPositionBeforeFailure) {
                        applyPositionBeforeFailure = false;
                        lastLogicalCommand = logical;
                        physicalCommand = direction == Servo.Direction.REVERSE
                                ? 1.0 - logical
                                : logical;
                    }
                    throw new IllegalStateException("injected servo write failure");
                }
                lastLogicalCommand = logical;
                physicalCommand = direction == Servo.Direction.REVERSE ? 1.0 - logical : logical;
                successfulPositionWrites++;
                return null;
            }
            if ("getPosition".equals(name)) {
                return direction == Servo.Direction.REVERSE
                        ? 1.0 - physicalCommand
                        : physicalCommand;
            }
            if ("setDirection".equals(name)) {
                directionWrites++;
                if (failNextDirectionWrite) {
                    failNextDirectionWrite = false;
                    throw new IllegalStateException(
                            "injected servo direction restore failure");
                }
                direction = (Servo.Direction) args[0];
                return null;
            }
            if ("getDirection".equals(name)) return direction;
            return hardwareDeviceOrDefault(method);
        }
    }

    private static Object objectMethod(Object proxy,
                                       Method method,
                                       Object[] args,
                                       String label) {
        if ("equals".equals(method.getName())) return proxy == args[0];
        if ("hashCode".equals(method.getName())) return System.identityHashCode(proxy);
        if ("toString".equals(method.getName())) return label;
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
