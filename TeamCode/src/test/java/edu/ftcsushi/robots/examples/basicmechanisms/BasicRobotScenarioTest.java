package edu.ftcsushi.robots.examples.basicmechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.Proxy;
import java.nio.charset.StandardCharsets;

import edu.ftcsushi.fw.drive.DriveCommandSink;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.FtcRobotOpMode;
import edu.ftcsushi.fw.task.Task;
import edu.ftcsushi.fw.task.TaskOutcome;
import edu.ftcsushi.fw.testing.ManualLoopClock;
import edu.ftcsushi.fw.testing.ftc.FtcTestHardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Runs the complete basic robot graph without a Control Hub and checks its public safety gates. */
public final class BasicRobotScenarioTest {

    @Test
    public void capabilityProfilesAreFreshIndependentAndBlockEveryKindOfMotion() {
        BasicDriveProfile firstDrive = BasicDriveProfile.current();
        BasicDriveProfile secondDrive = BasicDriveProfile.current();
        BasicLiftProfile firstLift = BasicLiftProfile.current();
        BasicLiftProfile secondLift = BasicLiftProfile.current();
        BasicClawProfile firstClaw = BasicClawProfile.current();
        BasicClawProfile secondClaw = BasicClawProfile.current();

        assertNotSame(firstDrive, secondDrive);
        assertNotSame(firstDrive.drive, secondDrive.drive);
        assertNotSame(firstDrive.drive.wiring, secondDrive.drive.wiring);
        assertNotSame(firstDrive.drive.drivebase, secondDrive.drive.drivebase);
        assertNotSame(firstLift, secondLift);
        assertNotSame(firstLift.lift, secondLift.lift);
        assertNotSame(firstClaw, secondClaw);
        assertNotSame(firstClaw.claw, secondClaw.claw);

        assertFalse(firstDrive.allowDriveMotion);
        assertFalse(firstLift.allowLiftMotion);
        assertFalse(firstClaw.allowClawMotion);

        firstDrive.drive.wiring.frontLeftName = "changed";
        firstLift.lift.lowHeightIn = 9.0;
        firstClaw.claw.openPosition = 0.95;
        assertEquals("frontLeftMotor", secondDrive.drive.wiring.frontLeftName);
        assertEquals(4.0, secondLift.lift.lowHeightIn, 0.0);
        assertEquals(0.70, secondClaw.claw.openPosition, 0.0);
    }

    @Test
    public void publicTeachingHostsStayDisabledAndMotionGatesRunBeforeLookup() {
        Class<?>[] hostTypes = {
                BasicLiftTeleOp.class,
                BasicClawTeleOp.class,
                BasicMechanismsAuto.class,
                BasicRobotTeleOp.class,
                BasicDriveAuto.class,
                BasicRobotAuto.class
        };
        String[] firstPermissions = {
                "BasicDriveProfile.allowDriveMotion",
                "BasicDriveProfile.allowDriveMotion",
                "BasicLiftProfile.allowLiftMotion",
                "BasicDriveProfile.allowDriveMotion",
                "BasicDriveProfile.allowDriveMotion",
                "BasicDriveProfile.allowDriveMotion"
        };

        for (int index = 0; index < hostTypes.length; index++) {
            Class<?> type = hostTypes[index];
            assertTrue(Modifier.isPublic(type.getModifiers()));
            assertTrue(Modifier.isFinal(type.getModifiers()));
            assertNotNull(type.getAnnotation(Disabled.class));

            Constructor<?>[] constructors = type.getDeclaredConstructors();
            assertEquals(1, constructors.length);
            assertTrue(Modifier.isPublic(constructors[0].getModifiers()));
            assertEquals(0, constructors[0].getParameterTypes().length);

            FtcTestHardware hardware = new FtcTestHardware();
            FtcRobotOpMode mode = newHost(type);
            prepare(mode, hardware);
            IllegalStateException failure = expectIllegalState(mode::init);
            assertTrue(failure.getMessage().contains(firstPermissions[index]));
            assertEquals(0, hardware.lookupCalls());
        }
    }

    @Test
    public void liftAndDriveCannotSilentlyOwnTheSameMotorKey() {
        BasicDriveProfile driveProfile = BasicDriveProfile.current();
        BasicLiftProfile liftProfile = BasicLiftProfile.current();
        liftProfile.lift.motorName = "  sharedMotor  ";
        driveProfile.drive.wiring.backRightName = "sharedMotor";

        IllegalStateException failure = expectIllegalState(() ->
                BasicHardwareOwnership.requireDistinctDriveAndLiftMotors(
                        driveProfile.drive, liftProfile.lift, "test mode"));

        assertTrue(failure.getMessage().contains("BasicLiftProfile.lift.motorName"));
        assertTrue(failure.getMessage().contains(
                "BasicDriveProfile.drive.wiring.backRightName"));
        assertTrue(failure.getMessage().contains("\"sharedMotor\""));
    }

    @Test
    public void focusedSourceClosuresStayIndependentAndAutosExposeRootOutcomeTelemetry() {
        assertClassDoesNotReference(
                BasicLiftTeleOp.class, "BasicClaw", "BasicClawProfile", "BasicClawControls");
        assertClassDoesNotReference(
                BasicClawTeleOp.class, "BasicLift", "BasicLiftProfile", "BasicLiftControls");
        assertClassDoesNotReference(
                BasicMechanismsAuto.class, "BasicDrive", "FtcDrives", "DriveCommandSink");
        assertClassDoesNotReference(
                BasicDriveAuto.class, "BasicLift", "BasicClaw", "BasicRobotAutoRoutines");
        assertClassDoesNotReference(
                BasicAutoRoutines.class, "BasicDrive", "DriveCommandSink", "DriveTasks");

        assertClassDoesNotReference(
                BasicDriveProfile.class, "BasicLiftProfile", "BasicClawProfile");
        assertClassDoesNotReference(
                BasicLiftProfile.class, "BasicDriveProfile", "BasicClawProfile");
        assertClassDoesNotReference(
                BasicClawProfile.class, "BasicDriveProfile", "BasicLiftProfile");
        assertClassDoesNotReference(
                BasicDriveControls.class, "BasicLiftControls", "BasicClawControls");
        assertClassDoesNotReference(
                BasicLiftControls.class, "BasicDriveControls", "BasicClawControls");
        assertClassDoesNotReference(
                BasicClawControls.class, "BasicDriveControls", "BasicLiftControls");

        assertClassReferences(BasicMechanismsAuto.class, "auto.complete", "auto.outcome");
        assertClassReferences(
                BasicDriveAuto.class,
                "BasicDriveStopOwner",
                "auto.complete",
                "auto.outcome");
        assertClassReferences(
                BasicRobotAuto.class,
                "BasicDriveStopOwner",
                "auto.complete",
                "auto.outcome");
    }

    @Test
    public void completeAutoRunsAgainstRealPlantsAndStopsDriveBeforeSuccessfulFinish() {
        Scenario f = new Scenario();
        Task root = BasicRobotAutoRoutines.complete(f.lift, f.claw, f.drive);

        beginAndReachTimedDrive(root, f);
        assertConservativeForwardRequestWasScaledByTheRealDrivebase(f);

        heartbeat(root, f, 0.75);
        assertAllDriveMotorsStopped(f);
        heartbeat(root, f, 0.02);
        assertEquals(BasicClaw.State.OPEN, f.claw.status().requestedState);
        assertEquals(BasicLift.Height.STOWED, f.lift.status().requestedHeight);

        f.liftMotor.setCurrentPositionTicks(0);
        heartbeat(root, f, 0.02);
        heartbeat(root, f, 0.02);

        assertEquals(TaskOutcome.SUCCESS, root.getOutcome());
        assertAllDriveMotorsStopped(f);
        assertEquals(f.clawProfile.claw.openPosition, f.clawServo.position(), 0.0);
    }

    @Test
    public void cancellingTheCompleteAutoStopsDriveAndStartsNoReleaseOrStow() {
        Scenario f = new Scenario();
        Task root = BasicRobotAutoRoutines.complete(f.lift, f.claw, f.drive);

        beginAndReachTimedDrive(root, f);
        assertConservativeForwardRequestWasScaledByTheRealDrivebase(f);

        root.cancel();
        root.cancel();

        assertEquals(TaskOutcome.CANCELLED, root.getOutcome());
        assertAllDriveMotorsStopped(f);
        assertEquals(BasicClaw.State.CLOSED, f.claw.status().requestedState);
        assertEquals(BasicLift.Height.HIGH, f.lift.status().requestedHeight);
    }

    private static void beginAndReachTimedDrive(Task root, Scenario f) {
        root.start(f.time.clock());
        f.lift.update(f.time.clock());
        f.claw.update(f.time.clock());

        f.bottomSwitch.setHigh(false);
        heartbeat(root, f, 0.01);
        heartbeat(root, f, 0.03);
        heartbeat(root, f, 0.02);
        assertEquals(BasicLift.Height.HIGH, f.lift.status().requestedHeight);
        assertEquals(BasicClaw.State.CLOSED, f.claw.status().requestedState);

        f.liftMotor.setCurrentPositionTicks(
                (int) Math.round(
                        f.liftProfile.lift.highHeightIn
                                * f.liftProfile.lift.ticksPerIn));
        heartbeat(root, f, 0.02);
        heartbeat(root, f, 0.02);
        heartbeat(root, f, 0.02);
        assertFalse(root.isComplete());
    }

    /** Mimics the managed ACTIVE order: Task graph first, then mechanism output owners. */
    private static void heartbeat(Task root, Scenario f, double dtSec) {
        f.time.nextCycle(dtSec);
        root.update(f.time.clock());
        f.lift.update(f.time.clock());
        f.claw.update(f.time.clock());
    }

    private static void assertConservativeForwardRequestWasScaledByTheRealDrivebase(Scenario f) {
        // The routine requests 0.20; the reviewed profile separately limits max axial to 0.25.
        double expectedWheelCommand = 0.20 * f.driveProfile.drive.drivebase.maxAxial;
        assertEquals(0.05, expectedWheelCommand, 0.0);
        assertEquals(expectedWheelCommand, f.frontLeft.power(), 0.0);
        assertEquals(expectedWheelCommand, f.frontRight.power(), 0.0);
        assertEquals(expectedWheelCommand, f.backLeft.power(), 0.0);
        assertEquals(expectedWheelCommand, f.backRight.power(), 0.0);
    }

    private static void assertAllDriveMotorsStopped(Scenario f) {
        assertEquals(0.0, f.frontLeft.power(), 0.0);
        assertEquals(0.0, f.frontRight.power(), 0.0);
        assertEquals(0.0, f.backLeft.power(), 0.0);
        assertEquals(0.0, f.backRight.power(), 0.0);
    }

    private static FtcRobotOpMode newHost(Class<?> type) {
        try {
            return (FtcRobotOpMode) type.getDeclaredConstructor().newInstance();
        } catch (ReflectiveOperationException failure) {
            throw new AssertionError("Could not construct " + type.getName(), failure);
        }
    }

    private static void prepare(FtcRobotOpMode mode, FtcTestHardware hardware) {
        mode.hardwareMap = hardware;
        mode.telemetry = telemetryProxy();
        mode.gamepad1 = new Gamepad();
        mode.gamepad2 = new Gamepad();
        mode.resetRuntime();
    }

    private static Telemetry telemetryProxy() {
        InvocationHandler handler = new InvocationHandler() {
            @Override
            public Object invoke(Object proxy, Method method, Object[] args) {
                if (method.getDeclaringClass() == Object.class) {
                    if ("equals".equals(method.getName())) {
                        return proxy == args[0];
                    }
                    if ("hashCode".equals(method.getName())) {
                        return System.identityHashCode(proxy);
                    }
                    if ("toString".equals(method.getName())) {
                        return "BasicRobotTelemetry";
                    }
                }
                if (method.getReturnType() == boolean.class) {
                    return true;
                }
                if (method.getReturnType().isPrimitive()) {
                    return 0;
                }
                return null;
            }
        };
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                handler);
    }

    private static IllegalStateException expectIllegalState(Runnable action) {
        try {
            action.run();
            fail("Expected IllegalStateException");
            throw new AssertionError("unreachable");
        } catch (IllegalStateException expected) {
            return expected;
        }
    }

    private static void assertClassDoesNotReference(Class<?> type, String... forbiddenText) {
        String classFile = classFileText(type);
        for (String text : forbiddenText) {
            assertFalse(type.getSimpleName() + " must not reference " + text,
                    classFile.contains(text));
        }
    }

    private static void assertClassReferences(Class<?> type, String... requiredText) {
        String classFile = classFileText(type);
        for (String text : requiredText) {
            assertTrue(type.getSimpleName() + " must reference " + text,
                    classFile.contains(text));
        }
    }

    private static String classFileText(Class<?> type) {
        String resource = type.getSimpleName() + ".class";
        try (InputStream input = type.getResourceAsStream(resource)) {
            assertNotNull("Missing compiled class resource for " + type.getName(), input);
            ByteArrayOutputStream bytes = new ByteArrayOutputStream();
            byte[] buffer = new byte[1024];
            int count;
            while ((count = input.read(buffer)) >= 0) {
                bytes.write(buffer, 0, count);
            }
            return new String(bytes.toByteArray(), StandardCharsets.ISO_8859_1);
        } catch (IOException failure) {
            throw new AssertionError("Could not inspect " + type.getName(), failure);
        }
    }

    private static final class Scenario {
        private final BasicDriveProfile driveProfile = BasicDriveProfile.current();
        private final BasicLiftProfile liftProfile = BasicLiftProfile.current();
        private final BasicClawProfile clawProfile = BasicClawProfile.current();
        private final FtcTestHardware hardware = new FtcTestHardware();
        private final FtcTestHardware.MotorProbe liftMotor;
        private final FtcTestHardware.DigitalProbe bottomSwitch;
        private final FtcTestHardware.ServoProbe clawServo;
        private final FtcTestHardware.MotorProbe frontLeft;
        private final FtcTestHardware.MotorProbe frontRight;
        private final FtcTestHardware.MotorProbe backLeft;
        private final FtcTestHardware.MotorProbe backRight;
        private final BasicLiftMechanism lift;
        private final BasicClawMechanism claw;
        private final DriveCommandSink drive;
        private final ManualLoopClock time = new ManualLoopClock();

        private Scenario() {
            liftProfile.lift.motorName = "lift";
            liftProfile.lift.bottomSwitchName = "bottom";
            liftProfile.lift.maximumHeightIn = 10.0;
            liftProfile.lift.ticksPerIn = 10.0;
            liftProfile.lift.toleranceIn = 0.10;
            liftProfile.lift.stowedHeightIn = 0.0;
            liftProfile.lift.lowHeightIn = 4.0;
            liftProfile.lift.highHeightIn = 8.0;
            liftProfile.lift.homingTimeoutSec = 0.20;
            liftProfile.lift.moveTimeoutSec = 0.20;
            clawProfile.claw.servoName = "claw";
            driveProfile.drive.wiring.frontLeftName = "frontLeft";
            driveProfile.drive.wiring.frontRightName = "frontRight";
            driveProfile.drive.wiring.backLeftName = "backLeft";
            driveProfile.drive.wiring.backRightName = "backRight";

            liftMotor = hardware.addMotor(liftProfile.lift.motorName);
            bottomSwitch = hardware.addDigitalInput(liftProfile.lift.bottomSwitchName);
            clawServo = hardware.addServo(clawProfile.claw.servoName);
            frontLeft = hardware.addMotor(driveProfile.drive.wiring.frontLeftName);
            frontRight = hardware.addMotor(driveProfile.drive.wiring.frontRightName);
            backLeft = hardware.addMotor(driveProfile.drive.wiring.backLeftName);
            backRight = hardware.addMotor(driveProfile.drive.wiring.backRightName);

            lift = new BasicLiftMechanism(hardware, liftProfile.lift);
            claw = new BasicClawMechanism(hardware, clawProfile.claw);
            drive = FtcDrives.mecanum(hardware, driveProfile.drive);
        }
    }
}
