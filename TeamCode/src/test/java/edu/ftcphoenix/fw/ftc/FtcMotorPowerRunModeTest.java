package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.PositionPlant;
import edu.ftcphoenix.fw.core.control.ScalarRegulator;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.hal.PowerOutput;
import edu.ftcphoenix.fw.core.hal.VelocityOutput;
import edu.ftcphoenix.fw.drive.DriveSignal;
import edu.ftcphoenix.fw.drive.MecanumDrivebase;
import edu.ftcphoenix.fw.testing.ManualLoopClock;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies FTC raw motor-power run-mode ownership and public construction-path handoffs. */
public final class FtcMotorPowerRunModeTest {

    private static final double EPSILON = 1.0e-9;
    private static final DcMotor.RunMode RAW_MODE = DcMotor.RunMode.RUN_WITHOUT_ENCODER;

    @Test
    public void constructionConfiguresDirectionWithoutAcquiringModeOrWritingPower() {
        EventLog events = new EventLog();
        MotorProbe motor = new MotorProbe("motor", events, DcMotor.RunMode.RUN_TO_POSITION);

        PowerOutput output = FtcHardware.motorPower(motor.motor(), Direction.REVERSE);

        assertEquals(Arrays.asList("motor.setDirection(REVERSE)"), events.values);
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, motor.runMode);
        assertEquals(0, motor.getModeCalls);
        assertEquals(0, motor.setModeCalls);
        assertEquals(0, motor.setPowerCalls);
        assertEquals(0.0, output.getCommandedPower(), EPSILON);
    }

    @Test
    public void everySupportedStartingModeUsesApprovedTransitionWithoutResettingEncoder() {
        EventLog rawEvents = new EventLog();
        MotorProbe rawMotor = new MotorProbe(
                "motor",
                rawEvents,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PowerOutput rawOutput = FtcHardware.motorPower(rawMotor.motor(), Direction.FORWARD);
        rawEvents.clear();

        rawOutput.setPower(0.625);

        assertEquals(
                Arrays.asList("motor.getMode", "motor.setPower(0.625)"),
                rawEvents.values);
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, rawMotor.runMode);
        assertEquals(0.625, rawMotor.power, EPSILON);
        assertNoEncoderResetModeWrite(rawEvents);

        DcMotor.RunMode[] transitionModes = {
                DcMotor.RunMode.RUN_USING_ENCODER,
                DcMotor.RunMode.RUN_TO_POSITION,
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        };
        for (DcMotor.RunMode startingMode : transitionModes) {
            EventLog events = new EventLog();
            MotorProbe motor = new MotorProbe("motor", events, startingMode);
            PowerOutput output = FtcHardware.motorPower(motor.motor(), Direction.FORWARD);
            events.clear();

            output.setPower(-0.375);

            assertEquals("starting mode " + startingMode,
                    Arrays.asList(
                            "motor.getMode",
                            "motor.setPower(0.0)",
                            "motor.setMode(RUN_WITHOUT_ENCODER)",
                            "motor.getMode",
                            "motor.setPower(-0.375)"
                    ),
                    events.values);
            assertEquals("starting mode " + startingMode, RAW_MODE, motor.runMode);
            assertEquals("starting mode " + startingMode, -0.375, motor.power, EPSILON);
            assertNoEncoderResetModeWrite(events);
        }
    }

    @Test
    public void repeatedRawWritesSkipTransitionsAndModeDriftIsReacquired() {
        EventLog events = new EventLog();
        MotorProbe motor = new MotorProbe("motor", events, RAW_MODE);
        PowerOutput output = FtcHardware.motorPower(motor.motor(), Direction.FORWARD);
        events.clear();

        output.setPower(0.2);
        output.setPower(0.4);

        assertEquals(Arrays.asList(
                "motor.getMode",
                "motor.setPower(0.2)",
                "motor.getMode",
                "motor.setPower(0.4)"
        ), events.values);
        assertEquals(0, motor.setModeCalls);

        motor.forceMode(DcMotor.RunMode.RUN_USING_ENCODER);
        events.clear();

        output.setPower(0.6);

        assertEquals(Arrays.asList(
                "motor.getMode",
                "motor.setPower(0.0)",
                "motor.setMode(RUN_WITHOUT_ENCODER)",
                "motor.getMode",
                "motor.setPower(0.6)"
        ), events.values);
        assertEquals(RAW_MODE, motor.runMode);
        assertEquals(0.6, output.getCommandedPower(), EPSILON);
    }

    @Test
    public void activeZeroCommandAcquiresAndVerifiesRawModeUnlikeLifecycleStop() {
        EventLog events = new EventLog();
        MotorProbe motor = new MotorProbe("motor", events, DcMotor.RunMode.RUN_TO_POSITION);
        PowerOutput output = FtcHardware.motorPower(motor.motor(), Direction.FORWARD);
        events.clear();

        output.setPower(0.0);

        assertEquals(Arrays.asList(
                "motor.getMode",
                "motor.setPower(0.0)",
                "motor.setMode(RUN_WITHOUT_ENCODER)",
                "motor.getMode",
                "motor.setPower(0.0)"
        ), events.values);
        assertEquals(RAW_MODE, motor.runMode);
        assertEquals(0.0, motor.power, EPSILON);
        assertEquals(0.0, output.getCommandedPower(), EPSILON);
        assertNoEncoderResetModeWrite(events);
    }

    @Test
    public void lifecycleStopWritesZeroWithoutAcquiringOrRestoringMode() {
        EventLog events = new EventLog();
        MotorProbe motor = new MotorProbe("motor", events, DcMotor.RunMode.RUN_TO_POSITION);
        motor.power = 0.8;
        PowerOutput output = FtcHardware.motorPower(motor.motor(), Direction.FORWARD);
        events.clear();

        output.stop();

        assertEquals(Arrays.asList("motor.setPower(0.0)"), events.values);
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, motor.runMode);
        assertEquals(0, motor.getModeCalls);
        assertEquals(0, motor.setModeCalls);
        assertEquals(0.0, motor.power, EPSILON);
        assertEquals(0.0, output.getCommandedPower(), EPSILON);
        assertNoEncoderResetModeWrite(events);
    }

    @Test
    public void rawAndDeviceManagedVelocityHandoffStopsEachOwnerBeforeTheNextStarts() {
        EventLog events = new EventLog();
        MotorProbe motor = new MotorProbe("motor", events, DcMotor.RunMode.RUN_TO_POSITION);
        PowerOutput raw = FtcHardware.motorPower(motor.motor(), Direction.FORWARD);
        VelocityOutput velocity = FtcHardware.motorVelocity(motor.motor(), Direction.FORWARD);
        events.clear();

        raw.setPower(0.45);

        assertEquals(Arrays.asList(
                "motor.getMode",
                "motor.setPower(0.0)",
                "motor.setMode(RUN_WITHOUT_ENCODER)",
                "motor.getMode",
                "motor.setPower(0.45)"
        ), events.values);
        assertEquals(RAW_MODE, motor.runMode);
        assertEquals(0.45, motor.power, EPSILON);

        events.clear();
        raw.stop();

        assertEquals(Arrays.asList("motor.setPower(0.0)"), events.values);
        assertEquals(RAW_MODE, motor.runMode);
        assertEquals(0.0, motor.power, EPSILON);

        velocity.setVelocity(1200.0);

        assertEquals(Arrays.asList(
                "motor.setPower(0.0)",
                "motor.setMode(RUN_USING_ENCODER)",
                "motor.setVelocity(1200.0)"
        ), events.values);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.runMode);
        assertEquals(1200.0, motor.velocityTicksPerSec, EPSILON);

        events.clear();
        velocity.stop();

        assertEquals(Arrays.asList(
                "motor.setVelocity(0.0)",
                "motor.setPower(0.0)"
        ), events.values);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.runMode);
        assertEquals(0.0, motor.velocityTicksPerSec, EPSILON);
        assertEquals(0.0, motor.power, EPSILON);

        raw.setPower(-0.25);

        assertEquals(Arrays.asList(
                "motor.setVelocity(0.0)",
                "motor.setPower(0.0)",
                "motor.getMode",
                "motor.setPower(0.0)",
                "motor.setMode(RUN_WITHOUT_ENCODER)",
                "motor.getMode",
                "motor.setPower(-0.25)"
        ), events.values);
        assertEquals(RAW_MODE, motor.runMode);
        assertEquals(-0.25, motor.power, EPSILON);
        assertNoEncoderResetModeWrite(events);
    }

    @Test
    public void initialModeReadFailureAttemptsPhysicalZeroAndReportsRequiredMode() {
        EventLog events = new EventLog();
        MotorProbe motor = new MotorProbe("motor", events, DcMotor.RunMode.RUN_USING_ENCODER);
        RuntimeException readFailure = new IllegalStateException("mode read failed");
        motor.getModeFailures.put(1, readFailure);
        PowerOutput output = FtcHardware.motorPower(motor.motor(), Direction.FORWARD);
        events.clear();

        RuntimeException observed = expectRuntime(() -> output.setPower(0.5));

        assertActionableRawModeFailure(observed);
        assertThrowableGraphContains(observed, readFailure);
        assertEquals(Arrays.asList("motor.getMode", "motor.setPower(0.0)"), events.values);
        assertEquals(0.0, motor.power, EPSILON);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.runMode);
    }

    @Test
    public void transitionZeroFailureRetriesCleanupWithoutSelectingMode() {
        EventLog events = new EventLog();
        MotorProbe motor = new MotorProbe("motor", events, DcMotor.RunMode.RUN_TO_POSITION);
        RuntimeException zeroFailure = new IllegalStateException("transition zero failed");
        motor.setPowerFailures.put(1, zeroFailure);
        PowerOutput output = FtcHardware.motorPower(motor.motor(), Direction.FORWARD);
        events.clear();

        RuntimeException observed = expectRuntime(() -> output.setPower(0.5));

        assertActionableRawModeFailure(observed);
        assertThrowableGraphContains(observed, zeroFailure);
        assertEquals(Arrays.asList(
                "motor.getMode",
                "motor.setPower(0.0)",
                "motor.setPower(0.0)"
        ), events.values);
        assertEquals(0, motor.setModeCalls);
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, motor.runMode);
        assertEquals(0.0, motor.power, EPSILON);
    }

    @Test
    public void modeSelectionFailurePreservesPrimaryAndSuppressesCleanupFailure() {
        EventLog events = new EventLog();
        MotorProbe motor = new MotorProbe("motor", events, DcMotor.RunMode.RUN_USING_ENCODER);
        RuntimeException transitionFailure = new IllegalStateException("set mode failed");
        RuntimeException cleanupFailure = new IllegalStateException("cleanup zero failed");
        motor.setModeFailures.put(1, transitionFailure);
        motor.setPowerFailures.put(2, cleanupFailure);
        PowerOutput output = FtcHardware.motorPower(motor.motor(), Direction.FORWARD);
        events.clear();

        RuntimeException observed = expectRuntime(() -> output.setPower(0.7));

        assertActionableRawModeFailure(observed);
        assertThrowableGraphContains(observed, transitionFailure);
        assertThrowableGraphContains(observed, cleanupFailure);
        assertSuppressedSomewhere(observed, cleanupFailure);
        assertEquals(Arrays.asList(
                "motor.getMode",
                "motor.setPower(0.0)",
                "motor.setMode(RUN_WITHOUT_ENCODER)",
                "motor.setPower(0.0)"
        ), events.values);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.runMode);
        assertNoRequestedPower(events, 0.7);
    }

    @Test
    public void verificationReadFailureFailsClosedAndSuppressesCleanupFailure() {
        EventLog events = new EventLog();
        MotorProbe motor = new MotorProbe("motor", events, DcMotor.RunMode.RUN_USING_ENCODER);
        RuntimeException verificationFailure = new IllegalStateException("verification read failed");
        RuntimeException cleanupFailure = new IllegalStateException("cleanup zero failed");
        motor.getModeFailures.put(2, verificationFailure);
        motor.setPowerFailures.put(2, cleanupFailure);
        PowerOutput output = FtcHardware.motorPower(motor.motor(), Direction.FORWARD);
        events.clear();

        RuntimeException observed = expectRuntime(() -> output.setPower(0.7));

        assertActionableRawModeFailure(observed);
        assertThrowableGraphContains(observed, verificationFailure);
        assertThrowableGraphContains(observed, cleanupFailure);
        assertSuppressedSomewhere(observed, cleanupFailure);
        assertEquals(Arrays.asList(
                "motor.getMode",
                "motor.setPower(0.0)",
                "motor.setMode(RUN_WITHOUT_ENCODER)",
                "motor.getMode",
                "motor.setPower(0.0)"
        ), events.values);
        assertEquals(RAW_MODE, motor.runMode);
        assertNoRequestedPower(events, 0.7);
    }

    @Test
    public void verificationMismatchFailsClosedBeforeRequestedPower() {
        EventLog events = new EventLog();
        MotorProbe motor = new MotorProbe("motor", events, DcMotor.RunMode.RUN_TO_POSITION);
        motor.ignoreModeWrites = true;
        PowerOutput output = FtcHardware.motorPower(motor.motor(), Direction.FORWARD);
        events.clear();

        RuntimeException observed = expectRuntime(() -> output.setPower(-0.55));

        assertActionableRawModeFailure(observed);
        assertEquals(Arrays.asList(
                "motor.getMode",
                "motor.setPower(0.0)",
                "motor.setMode(RUN_WITHOUT_ENCODER)",
                "motor.getMode",
                "motor.setPower(0.0)"
        ), events.values);
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, motor.runMode);
        assertEquals(0.0, motor.power, EPSILON);
        assertNoRequestedPower(events, -0.55);
    }

    @Test
    public void directMotorGroupPreflightsEveryChildBeforeRequestedMotion() {
        EventLog events = new EventLog();
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe left = new MotorProbe("left", events, DcMotor.RunMode.RUN_USING_ENCODER);
        MotorProbe right = new MotorProbe("right", events, DcMotor.RunMode.RUN_TO_POSITION);
        hardwareMap.put("left", left.motor());
        hardwareMap.put("right", right.motor());

        Plant plant = FtcActuators.plant(hardwareMap)
                .motor("left", Direction.FORWARD)
                .andMotor("right", Direction.REVERSE)
                .power()
                .targetedByDefaultWritable(0.65)
                .build();
        events.clear();

        plant.update(new ManualLoopClock().clock());

        int firstRequested = firstIndexOfEither(
                events.values, "left.setPower(0.65)", "right.setPower(0.65)");
        assertTrue("expected requested group power", firstRequested >= 0);
        assertTransitionVerifiedBefore(events.values, "left", firstRequested);
        assertTransitionVerifiedBefore(events.values, "right", firstRequested);
        assertTrue(events.values.indexOf("left.setPower(0.65)") >= firstRequested);
        assertTrue(events.values.indexOf("right.setPower(0.65)") >= firstRequested);
        assertEquals(RAW_MODE, left.runMode);
        assertEquals(RAW_MODE, right.runMode);
        assertEquals(0.65, left.power, EPSILON);
        assertEquals(0.65, right.power, EPSILON);
        assertNoEncoderResetModeWrite(events);
    }

    @Test
    public void groupedActuatorConstructionResolvesEveryMotorBeforeConfiguringAnyMotor() {
        EventLog events = new EventLog();
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe first = new MotorProbe("first", events, DcMotor.RunMode.RUN_TO_POSITION);
        hardwareMap.put("first", first.motor());

        expectRuntime(() -> FtcActuators.plant(hardwareMap)
                .motor("first", Direction.REVERSE)
                .andMotor("missing", Direction.FORWARD)
                .power()
                .targetedByDefaultWritable(0.5)
                .build());

        assertTrue("no motor may be configured before every group member resolves",
                events.values.isEmpty());
        assertEquals(DcMotorSimple.Direction.FORWARD, first.direction);
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, first.runMode);
        assertEquals(0, first.getModeCalls);
        assertEquals(0, first.setModeCalls);
        assertEquals(0, first.setPowerCalls);
    }

    @Test
    public void groupPreflightFailurePreventsRequestedMotionOnEveryChild() {
        EventLog events = new EventLog();
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe left = new MotorProbe("left", events, DcMotor.RunMode.RUN_USING_ENCODER);
        MotorProbe right = new MotorProbe("right", events, DcMotor.RunMode.RUN_TO_POSITION);
        RuntimeException rightModeFailure = new IllegalStateException("right mode failed");
        right.setModeFailures.put(1, rightModeFailure);
        hardwareMap.put("left", left.motor());
        hardwareMap.put("right", right.motor());

        Plant plant = FtcActuators.plant(hardwareMap)
                .motor("left", Direction.FORWARD)
                .andMotor("right", Direction.FORWARD)
                .power()
                .targetedByDefaultWritable(0.7)
                .build();
        events.clear();

        RuntimeException observed =
                expectRuntime(() -> plant.update(new ManualLoopClock().clock()));

        assertActionableRawModeFailure(observed);
        assertThrowableGraphContains(observed, rightModeFailure);
        assertNoRequestedPower(events, 0.7);
        assertOnlyZeroPowerWrites(events);
        assertEquals(0.0, left.power, EPSILON);
        assertEquals(0.0, right.power, EPSILON);
        assertNoEncoderResetModeWrite(events);
    }

    @Test
    public void mecanumLaterWheelModeFailureZerosEveryWheelBeforeAnyRequestedMotion() {
        EventLog events = new EventLog();
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe fl = new MotorProbe("fl", events, DcMotor.RunMode.RUN_USING_ENCODER);
        MotorProbe fr = new MotorProbe("fr", events, DcMotor.RunMode.RUN_USING_ENCODER);
        MotorProbe bl = new MotorProbe("bl", events, DcMotor.RunMode.RUN_USING_ENCODER);
        MotorProbe br = new MotorProbe("br", events, DcMotor.RunMode.RUN_USING_ENCODER);
        RuntimeException brModeFailure = new IllegalStateException("back-right mode failed");
        br.setModeFailures.put(1, brModeFailure);
        hardwareMap.put("fl", fl.motor());
        hardwareMap.put("fr", fr.motor());
        hardwareMap.put("bl", bl.motor());
        hardwareMap.put("br", br.motor());

        MecanumDrivebase drive = FtcDrives.mecanum(
                hardwareMap,
                "fl", Direction.FORWARD,
                "fr", Direction.REVERSE,
                "bl", Direction.FORWARD,
                "br", Direction.REVERSE);
        events.clear();

        RuntimeException observed =
                expectRuntime(() -> drive.drive(new DriveSignal(0.4, 0.0, 0.0)));

        assertActionableRawModeFailure(observed);
        assertThrowableGraphContains(observed, brModeFailure);
        assertOnlyZeroPowerWrites(events);
        for (MotorProbe motor : Arrays.asList(fl, fr, bl, br)) {
            assertTrue(motor.id + " should receive a best-effort zero",
                    events.values.contains(motor.id + ".setPower(0.0)"));
            assertEquals(motor.id, 0.0, motor.power, EPSILON);
        }
        assertNoEncoderResetModeWrite(events);
    }

    @Test
    public void regulatedExternalEncoderChannelRemainsStrictlyMeasurementOnly() {
        EventLog events = new EventLog();
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe powered = new MotorProbe(
                "powered", events, DcMotor.RunMode.RUN_USING_ENCODER);
        MotorProbe encoder = new MotorProbe(
                "encoder", events, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.positionTicks = 1234;
        hardwareMap.put("powered", powered.motor());
        hardwareMap.put("encoder", encoder.motor());
        ScalarRegulator regulator = (setpoint, measurement, clock) -> 0.4;

        Plant plant = FtcActuators.plant(hardwareMap)
                .motor("powered", Direction.FORWARD)
                .velocity()
                .regulated()
                .externalEncoder("encoder")
                .regulator(regulator)
                .unbounded()
                .nativeUnits()
                .targetedByDefaultWritable(2000.0)
                .build();
        events.clear();

        plant.update(new ManualLoopClock().clock());

        assertEquals(Arrays.asList("encoder.getCurrentPosition"), encoder.onlyOwnEvents());
        assertEquals(DcMotor.RunMode.STOP_AND_RESET_ENCODER, encoder.runMode);
        assertEquals(0, encoder.getModeCalls);
        assertEquals(0, encoder.setModeCalls);
        assertEquals(0, encoder.setPowerCalls);
        assertEquals(RAW_MODE, powered.runMode);
        assertEquals(0.4, powered.power, EPSILON);

        events.clear();
        plant.reset();

        assertTrue("Plant reset must not touch either FTC device", events.values.isEmpty());
        assertEquals(DcMotor.RunMode.STOP_AND_RESET_ENCODER, encoder.runMode);

        plant.stop();

        assertEquals(Arrays.asList("powered.setPower(0.0)"), events.values);
        assertEquals(DcMotor.RunMode.STOP_AND_RESET_ENCODER, encoder.runMode);
        assertEquals(0, encoder.setModeCalls);
        assertEquals(0, encoder.setPowerCalls);
    }

    @Test
    public void sameChannelExternalEncoderMeasuresPositionBeforeRawActuationOwnsMode() {
        EventLog events = new EventLog();
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe powered = new MotorProbe(
                "powered", events, DcMotor.RunMode.RUN_USING_ENCODER);
        powered.positionTicks = 1000;
        hardwareMap.put("powered", powered.motor());
        double[] observedMeasurement = {Double.NaN};
        int[] regulatorCalls = {0};
        ScalarRegulator regulator = (setpoint, measurement, clock) -> {
            regulatorCalls[0]++;
            observedMeasurement[0] = measurement;
            return -0.35;
        };

        Plant plant = FtcActuators.plant(hardwareMap)
                .motor("powered", Direction.FORWARD)
                .velocity()
                .regulated()
                .externalEncoder("powered")
                .regulator(regulator)
                .unbounded()
                .nativeUnits()
                .targetedByDefaultWritable(2000.0)
                .build();
        events.clear();
        ManualLoopClock manualClock = new ManualLoopClock();

        plant.update(manualClock.clock());

        assertEquals(Arrays.asList(
                "powered.getCurrentPosition",
                "powered.getMode",
                "powered.setPower(0.0)",
                "powered.setMode(RUN_WITHOUT_ENCODER)",
                "powered.getMode",
                "powered.setPower(-0.35)"
        ), events.values);
        assertEquals(1, regulatorCalls[0]);
        assertEquals(0.0, observedMeasurement[0], EPSILON);
        assertEquals(RAW_MODE, powered.runMode);
        assertEquals(-0.35, powered.power, EPSILON);
        assertNoEncoderResetModeWrite(events);

        powered.positionTicks = 1030;
        events.clear();
        plant.update(manualClock.nextCycle(0.25));

        assertEquals(Arrays.asList(
                "powered.getCurrentPosition",
                "powered.getMode",
                "powered.setPower(-0.35)"
        ), events.values);
        assertEquals(2, regulatorCalls[0]);
        assertEquals(120.0, observedMeasurement[0], EPSILON);
        assertEquals(RAW_MODE, powered.runMode);
        assertEquals(-0.35, powered.power, EPSILON);
        assertNoEncoderResetModeWrite(events);
    }

    @Test
    public void regulatedPositionUsesTheSameRawPowerModeOwnership() {
        EventLog events = new EventLog();
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe motor = new MotorProbe("arm", events, DcMotor.RunMode.RUN_TO_POSITION);
        hardwareMap.put("arm", motor.motor());

        PositionPlant plant = FtcActuators.plant(hardwareMap)
                .motor("arm", Direction.FORWARD)
                .position()
                .regulated()
                .nativeFeedback(clock -> 10.0)
                .regulator((setpoint, measurement, clock) -> -0.3)
                .linear()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced()
                .targetedByDefaultWritable(20.0)
                .build();
        events.clear();

        plant.update(new ManualLoopClock().clock());

        assertEquals(Arrays.asList(
                "arm.getMode",
                "arm.setPower(0.0)",
                "arm.setMode(RUN_WITHOUT_ENCODER)",
                "arm.getMode",
                "arm.setPower(-0.3)"
        ), events.values);
        assertEquals(RAW_MODE, motor.runMode);
        assertEquals(-0.3, motor.power, EPSILON);
    }

    @Test
    public void deviceManagedPositionCalibrationUsesRawModeThenHandsBackToPositionMode() {
        EventLog events = new EventLog();
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe motor = new MotorProbe("lift", events, DcMotor.RunMode.RUN_USING_ENCODER);
        motor.positionTicks = 40;
        hardwareMap.put("lift", motor.motor());

        PositionPlant plant = FtcActuators.plant(hardwareMap)
                .motor("lift", Direction.FORWARD)
                .position()
                .deviceManagedWithDefaults()
                .linear()
                .unbounded()
                .nativeUnits()
                .alreadyReferenced()
                .targetedByDefaultWritable(100.0)
                .build();
        ManualLoopClock clock = new ManualLoopClock();
        events.clear();

        plant.update(clock.clock());
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, motor.runMode);
        assertEquals(100, motor.targetPosition);

        events.clear();
        plant.beginCalibrationSearch(0.25);

        assertEquals(Arrays.asList(
                "lift.setPower(0.0)",
                "lift.setMode(RUN_USING_ENCODER)",
                "lift.getMode",
                "lift.setPower(0.0)",
                "lift.setMode(RUN_WITHOUT_ENCODER)",
                "lift.getMode",
                "lift.setPower(0.25)"
        ), events.values);
        assertEquals(RAW_MODE, motor.runMode);
        assertEquals(0.25, motor.power, EPSILON);

        events.clear();
        plant.endCalibrationSearch(true);

        assertEquals(Arrays.asList("lift.setPower(0.0)"), events.values);
        assertEquals(RAW_MODE, motor.runMode);

        events.clear();
        clock.nextCycle(0.02);
        plant.update(clock.clock());

        assertTrue(events.values.indexOf("lift.setMode(RUN_TO_POSITION)") >= 0);
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, motor.runMode);
        assertEquals(1.0, motor.power, EPSILON);

        events.clear();
        plant.stop();

        assertEquals(Arrays.asList(
                "lift.setPower(0.0)",
                "lift.setMode(RUN_USING_ENCODER)",
                "lift.setPower(0.0)"
        ), events.values);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.runMode);
        assertEquals(0.0, motor.power, EPSILON);
        assertNoEncoderResetModeWrite(events);
    }

    @Test
    public void regulatedFailStopUsesLifecycleStopWithoutStealingMode() {
        EventLog events = new EventLog();
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe motor = new MotorProbe("flywheel", events, DcMotor.RunMode.RUN_TO_POSITION);
        hardwareMap.put("flywheel", motor.motor());

        Plant plant = FtcActuators.plant(hardwareMap)
                .motor("flywheel", Direction.FORWARD)
                .velocity()
                .regulated()
                .nativeFeedback(clock -> 0.0)
                .regulator((setpoint, measurement, clock) -> Double.NaN)
                .unbounded()
                .nativeUnits()
                .targetedByDefaultWritable(1000.0)
                .build();
        events.clear();

        expectRuntime(() -> plant.update(new ManualLoopClock().clock()));

        assertEquals(Arrays.asList("flywheel.setPower(0.0)"), events.values);
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, motor.runMode);
        assertEquals(0, motor.getModeCalls);
        assertEquals(0, motor.setModeCalls);
        assertEquals(0.0, motor.power, EPSILON);
    }

    @Test
    public void simpleMecanumDriveUsesRawModeAndLifecycleStopDoesNotStealLaterModes() {
        EventLog events = new EventLog();
        TestHardwareMap hardwareMap = new TestHardwareMap();
        MotorProbe fl = new MotorProbe("fl", events, DcMotor.RunMode.RUN_TO_POSITION);
        MotorProbe fr = new MotorProbe("fr", events, DcMotor.RunMode.RUN_USING_ENCODER);
        MotorProbe bl = new MotorProbe("bl", events, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorProbe br = new MotorProbe("br", events, RAW_MODE);
        hardwareMap.put("fl", fl.motor());
        hardwareMap.put("fr", fr.motor());
        hardwareMap.put("bl", bl.motor());
        hardwareMap.put("br", br.motor());

        MecanumDrivebase drive = FtcDrives.mecanum(
                hardwareMap,
                "fl", Direction.FORWARD,
                "fr", Direction.REVERSE,
                "bl", Direction.FORWARD,
                "br", Direction.REVERSE);
        assertNoModeOrPowerEvents(events);
        events.clear();

        drive.drive(new DriveSignal(0.4, 0.0, 0.0));

        for (MotorProbe motor : Arrays.asList(fl, fr, bl, br)) {
            assertEquals(motor.id, RAW_MODE, motor.runMode);
            assertEquals(motor.id, 0.4, motor.power, EPSILON);
        }
        assertNoEncoderResetModeWrite(events);

        for (MotorProbe motor : Arrays.asList(fl, fr, bl, br)) {
            motor.forceMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        events.clear();

        // The drive owner releases through each PowerOutput lifecycle stop hook. If a deliberate
        // handoff already selected another mode, cleanup sends zero without stealing that mode back.
        drive.stop();

        for (MotorProbe motor : Arrays.asList(fl, fr, bl, br)) {
            assertEquals(motor.id, DcMotor.RunMode.RUN_USING_ENCODER, motor.runMode);
            assertEquals(motor.id, 0.0, motor.power, EPSILON);
        }
        assertEquals(Arrays.asList(
                "fl.setPower(0.0)",
                "fr.setPower(0.0)",
                "bl.setPower(0.0)",
                "br.setPower(0.0)"
        ), events.values);
        assertNoEncoderResetModeWrite(events);
    }

    private static void assertTransitionVerifiedBefore(List<String> events,
                                                       String motor,
                                                       int firstRequestedPower) {
        int firstRead = nthIndex(events, motor + ".getMode", 1);
        int zero = events.indexOf(motor + ".setPower(0.0)");
        int mode = events.indexOf(motor + ".setMode(RUN_WITHOUT_ENCODER)");
        int verificationRead = nthIndex(events, motor + ".getMode", 2);
        assertTrue(motor + " initial mode read missing", firstRead >= 0);
        assertTrue(motor + " transition zero must follow the initial read", firstRead < zero);
        assertTrue(motor + " raw mode selection must follow zero", zero < mode);
        assertTrue(motor + " verification must follow mode selection", mode < verificationRead);
        assertTrue(motor + " must verify before any group motion",
                verificationRead < firstRequestedPower);
    }

    private static int nthIndex(List<String> events, String value, int occurrence) {
        int seen = 0;
        for (int i = 0; i < events.size(); i++) {
            if (value.equals(events.get(i)) && ++seen == occurrence) return i;
        }
        return -1;
    }

    private static int firstIndexOfEither(List<String> events, String a, String b) {
        int ai = events.indexOf(a);
        int bi = events.indexOf(b);
        if (ai < 0) return bi;
        if (bi < 0) return ai;
        return Math.min(ai, bi);
    }

    private static void assertActionableRawModeFailure(RuntimeException failure) {
        assertTrue("expected IllegalStateException, got " + failure,
                failure instanceof IllegalStateException);
        assertTrue("failure should name required RUN_WITHOUT_ENCODER mode: " + failure.getMessage(),
                String.valueOf(failure.getMessage()).contains("RUN_WITHOUT_ENCODER"));
    }

    private static void assertNoRequestedPower(EventLog events, double power) {
        String suffix = ".setPower(" + Double.toString(power) + ")";
        for (String event : events.values) {
            assertFalse("unexpected requested power event " + event, event.endsWith(suffix));
        }
    }

    private static void assertOnlyZeroPowerWrites(EventLog events) {
        for (String event : events.values) {
            if (event.contains(".setPower(")) {
                assertTrue("expected only fail-closed zero writes, got " + event,
                        event.endsWith(".setPower(0.0)"));
            }
        }
    }

    private static void assertNoEncoderResetModeWrite(EventLog events) {
        for (String event : events.values) {
            assertFalse("raw power must never reset an encoder: " + event,
                    event.contains(".setMode(STOP_AND_RESET_ENCODER)"));
        }
    }

    private static void assertNoModeOrPowerEvents(EventLog events) {
        for (String event : events.values) {
            assertFalse("construction must not acquire mode or command power: " + event,
                    event.contains(".getMode")
                            || event.contains(".setMode(")
                            || event.contains(".setPower("));
        }
    }

    private static RuntimeException expectRuntime(Runnable action) {
        try {
            action.run();
        } catch (RuntimeException failure) {
            return failure;
        }
        fail("Expected RuntimeException");
        return null;
    }

    private static void assertThrowableGraphContains(Throwable root, Throwable expected) {
        assertTrue("Throwable graph did not retain " + expected,
                throwableGraphContains(root, expected,
                        java.util.Collections.newSetFromMap(
                                new IdentityHashMap<Throwable, Boolean>())));
    }

    private static boolean throwableGraphContains(Throwable current,
                                                  Throwable expected,
                                                  Set<Throwable> visited) {
        if (current == null || !visited.add(current)) return false;
        if (current == expected) return true;
        if (throwableGraphContains(current.getCause(), expected, visited)) return true;
        for (Throwable suppressed : current.getSuppressed()) {
            if (throwableGraphContains(suppressed, expected, visited)) return true;
        }
        return false;
    }

    private static void assertSuppressedSomewhere(Throwable root, Throwable expected) {
        assertTrue("Expected cleanup failure to be retained as a suppressed exception",
                suppressedSomewhere(root, expected,
                        java.util.Collections.newSetFromMap(
                                new IdentityHashMap<Throwable, Boolean>())));
    }

    private static boolean suppressedSomewhere(Throwable current,
                                               Throwable expected,
                                               Set<Throwable> visited) {
        if (current == null || !visited.add(current)) return false;
        for (Throwable suppressed : current.getSuppressed()) {
            if (suppressed == expected
                    || suppressedSomewhere(suppressed, expected, visited)) {
                return true;
            }
        }
        return suppressedSomewhere(current.getCause(), expected, visited);
    }

    private static final class EventLog {
        private final List<String> values = new ArrayList<>();

        private void add(String event) {
            values.add(event);
        }

        private void clear() {
            values.clear();
        }
    }

    /** In-memory HardwareMap that avoids Android-only SDK discovery during local JVM tests. */
    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> devices = new HashMap<>();

        private TestHardwareMap() {
            super(null, null);
        }

        @Override
        public void put(String name, HardwareDevice device) {
            devices.put(name, device);
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            HardwareDevice device = devices.get(name);
            if (device == null) {
                throw new IllegalArgumentException("No test device named " + name);
            }
            if (!type.isInstance(device)) {
                throw new IllegalArgumentException(name + " is not a " + type.getSimpleName());
            }
            return type.cast(device);
        }
    }

    /** Ordered-event SDK motor probe with independently injectable read/write failures. */
    private static final class MotorProbe {
        private final String id;
        private final EventLog events;
        private final DcMotorEx motor;

        private final Map<Integer, RuntimeException> getModeFailures = new HashMap<>();
        private final Map<Integer, RuntimeException> setModeFailures = new HashMap<>();
        private final Map<Integer, RuntimeException> setPowerFailures = new HashMap<>();

        private DcMotor.RunMode runMode;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        private boolean ignoreModeWrites;
        private int getModeCalls;
        private int setModeCalls;
        private int setPowerCalls;
        private int positionTicks;
        private int targetPosition;
        private double velocityTicksPerSec;
        private double power;

        private MotorProbe(String id, EventLog events, DcMotor.RunMode startingMode) {
            this.id = id;
            this.events = events;
            this.runMode = startingMode;
            this.motor = (DcMotorEx) Proxy.newProxyInstance(
                    DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class},
                    this::invoke);
        }

        private DcMotorEx motor() {
            return motor;
        }

        private void forceMode(DcMotor.RunMode mode) {
            runMode = mode;
        }

        private List<String> onlyOwnEvents() {
            List<String> own = new ArrayList<>();
            String prefix = id + ".";
            for (String event : events.values) {
                if (event.startsWith(prefix)) own.add(event);
            }
            return own;
        }

        private Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, method, args);
            }
            if ("getMode".equals(name)) {
                getModeCalls++;
                events.add(id + ".getMode");
                RuntimeException failure = getModeFailures.get(getModeCalls);
                if (failure != null) throw failure;
                return runMode;
            }
            if ("setMode".equals(name)) {
                setModeCalls++;
                DcMotor.RunMode requested = (DcMotor.RunMode) args[0];
                events.add(id + ".setMode(" + requested + ")");
                RuntimeException failure = setModeFailures.get(setModeCalls);
                if (failure != null) throw failure;
                if (!ignoreModeWrites) runMode = requested;
                return null;
            }
            if ("setPower".equals(name)) {
                setPowerCalls++;
                double requested = (double) args[0];
                events.add(id + ".setPower(" + Double.toString(requested) + ")");
                RuntimeException failure = setPowerFailures.get(setPowerCalls);
                if (failure != null) throw failure;
                power = requested;
                return null;
            }
            if ("getPower".equals(name)) return power;
            if ("setDirection".equals(name)) {
                direction = (DcMotorSimple.Direction) args[0];
                events.add(id + ".setDirection(" + direction + ")");
                return null;
            }
            if ("getDirection".equals(name)) return direction;
            if ("getCurrentPosition".equals(name)) {
                events.add(id + ".getCurrentPosition");
                return positionTicks;
            }
            if ("setTargetPosition".equals(name)) {
                targetPosition = (int) args[0];
                events.add(id + ".setTargetPosition(" + targetPosition + ")");
                return null;
            }
            if ("getTargetPosition".equals(name)) return targetPosition;
            if ("setVelocity".equals(name)) {
                velocityTicksPerSec = (double) args[0];
                events.add(id + ".setVelocity(" + Double.toString(velocityTicksPerSec) + ")");
                return null;
            }
            if ("getVelocity".equals(name)) {
                events.add(id + ".getVelocity");
                return velocityTicksPerSec;
            }
            if ("getManufacturer".equals(name)) return HardwareDevice.Manufacturer.Other;
            if ("getDeviceName".equals(name)) return id;
            if ("getConnectionInfo".equals(name)) return "test:" + id;
            if ("getVersion".equals(name)) return 1;
            return defaultValue(method.getReturnType());
        }

        private Object objectMethod(Object proxy, Method method, Object[] args) {
            if ("toString".equals(method.getName())) return "MotorProbe(" + id + ")";
            if ("hashCode".equals(method.getName())) return System.identityHashCode(proxy);
            if ("equals".equals(method.getName())) return proxy == args[0];
            return null;
        }
    }

    private static Object defaultValue(Class<?> type) {
        if (type == void.class) return null;
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
