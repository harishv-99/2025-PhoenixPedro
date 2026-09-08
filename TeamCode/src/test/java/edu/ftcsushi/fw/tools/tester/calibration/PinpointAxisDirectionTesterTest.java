package edu.ftcsushi.fw.tools.tester.calibration;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.junit.Test;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import edu.ftcsushi.fw.core.time.LoopClock;
import edu.ftcsushi.fw.ftc.input.Gamepads;
import edu.ftcsushi.fw.ftc.localization.PinpointOdometryPredictor;
import edu.ftcsushi.fw.tools.tester.TesterContext;

import static com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection.REVERSED;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/**
 * CAL-07 maintainer evidence for advice relative to configured encoder signs.
 *
 * <p>The actual tester constructor, defensive Config capture, real Pinpoint predictor, shared
 * clock, sample lifecycle, and telemetry remain active. Reflection replaces FTC hardware
 * acquisition using the predictor's existing device seam and queues the existing binding-edge
 * requests; it does not purport to verify button registration or an FTC hardware connection.
 * Scripted poses are independent observations, not values derived from the configured direction.
 * Consequently these tests prove the displayed software advice, not which way a person moved,
 * how a pod is wired, or whether applying the advice improves physical motion.</p>
 *
 * <p>Only the explicitly named defensive arithmetic test directly supplies private completed
 * result values: SDK finite coordinate limits do not furnish a truthful physical overflow case.</p>
 */
public final class PinpointAxisDirectionTesterTest {
    private static final String HISTORICAL_HEADER = "--- LAST COMPLETED SAMPLES ---";
    private static final String HISTORICAL_NOTICE =
            "Retained until a replacement completes or X clears; not the current attempt.";
    private static final String UNAVAILABLE =
            "UNAVAILABLE: non-finite sample delta; no direction recommendation.";

    @Test
    public void bothAxesDirectionsAndSignsNameTheExactKeepOrOppositeAssignment() throws Exception {
        for (boolean initPhase : new boolean[]{false, true}) {
            for (Axis axis : Axis.values()) {
                for (GoBildaPinpointDriver.EncoderDirection current
                        : new GoBildaPinpointDriver.EncoderDirection[]{FORWARD, REVERSED}) {
                    for (double delta : new double[]{8.0, -8.0}) {
                        PinpointAxisDirectionTester.Config draft = configured(axis, current);
                        Fixture f = new Fixture(draft, initPhase);
                        f.complete(axis, delta);

                        // Deliberately independent four-case oracle, not the production flip helper.
                        GoBildaPinpointDriver.EncoderDirection expected;
                        String action;
                        if (delta > 0.0) {
                            expected = current;
                            action = "Keep";
                        } else if (current == FORWARD) {
                            expected = REVERSED;
                            action = "Change";
                        } else {
                            expected = FORWARD;
                            action = "Change";
                        }
                        assertEquals(Collections.singletonList(assignment(action, axis, expected)),
                                f.advice(axis));
                        assertTrue("the peer encoder must not receive an assignment",
                                f.advice(axis.peer()).isEmpty());
                        assertTrue(f.hasLineContaining(delta > 0.0 ? "OK:" : "WRONG SIGN:"));
                        assertTrue(f.lines.contains(HISTORICAL_HEADER));
                        assertTrue(f.lines.contains(HISTORICAL_NOTICE));
                        assertEquals(draft.pinpoint.forwardPodDirection, f.device.forwardDirection);
                        assertEquals(draft.pinpoint.strafePodDirection, f.device.strafeDirection);
                        f.assertNoConfigurationWritesAfterConstruction();
                    }
                }
            }
        }
    }

    @Test
    public void translationMagnitudeThresholdIsInclusiveForEitherSignAndCustomMinimum()
            throws Exception {
        for (Axis axis : Axis.values()) {
            for (double minimum : new double[]{6.0, 7.25}) {
                for (double sign : new double[]{-1.0, 1.0}) {
                    for (double distance : new double[]{0.0, minimum - 0.001, minimum, minimum + 0.001}) {
                        PinpointAxisDirectionTester.Config draft = configured(axis, REVERSED);
                        draft.minTranslationInches = minimum;
                        Fixture f = new Fixture(draft, false);
                        f.complete(axis, sign * distance);
                        assertNotNull(f.result(axis));
                        if (distance < minimum) {
                            assertTrue(f.advice(axis).isEmpty());
                            assertTrue(f.hasLineContaining("Move farther"));
                            assertFalse(f.hasLineContaining("WRONG SIGN:"));
                            assertFalse(f.hasLineContaining("OK:"));
                        } else {
                            assertEquals(1, f.advice(axis).size());
                            assertFalse(f.hasLineContaining("Move farther"));
                        }
                    }
                }
            }
        }
    }

    @Test
    public void largePeerAxisMotionDoesNotQualifyTheRequestedAxis() throws Exception {
        for (Axis axis : Axis.values()) {
            Fixture f = new Fixture(configured(axis, REVERSED), false);
            f.device.position = pose(0.0, 0.0, 0.0);
            f.queue(axis.mode);
            f.tick();
            f.device.position = axis == Axis.FORWARD
                    ? pose(0.0, -40.0, 0.0) : pose(-40.0, 0.0, 0.0);
            f.queue(axis.mode);
            f.tick();
            assertTrue(f.hasLineContaining("Move farther"));
            assertTrue(f.advice(axis).isEmpty());
            assertTrue(f.advice(axis.peer()).isEmpty());
        }
    }

    @Test
    public void unavailablePoseCannotStartOrCompleteANewSample() throws Exception {
        for (Axis axis : Axis.values()) {
            for (int lossPoint = 0; lossPoint < 3; lossPoint++) {
                for (int invalid = 0; invalid < 4; invalid++) {
                    Fixture f = new Fixture(configured(axis, REVERSED), false);
                    if (lossPoint != 0) {
                        f.queue(axis.mode);
                        f.tick();
                        assertEquals(axis.mode, f.mode());
                    }
                    if (invalid == 0) f.device.status = GoBildaPinpointDriver.DeviceStatus.CALIBRATING;
                    if (invalid == 1) f.device.position = pose(Double.NaN, 0.0, 0.0);
                    if (invalid == 2) f.device.position = pose(0.0, Double.POSITIVE_INFINITY, 0.0);
                    if (invalid == 3) f.device.position = null;
                    // Middle-of-sample loss has no button; start/completion loss has a queued edge.
                    if (lossPoint != 1) f.queue(axis.mode);
                    f.tick();
                    assertEquals("IDLE", f.mode());
                    assertNull(f.result(axis));
                    assertTrue(f.advice(axis).isEmpty());
                    assertTrue(f.hasLineContaining("Wait for Pinpoint READY"));

                    f.device.status = GoBildaPinpointDriver.DeviceStatus.READY;
                    f.device.position = axis.pose(-8.0);
                    f.tick();
                    assertEquals("recovery alone cannot complete the cancelled attempt", "IDLE", f.mode());
                    assertNull(f.result(axis));
                    f.assertNoConfigurationWritesAfterConstruction();
                }
            }
        }
    }

    @Test
    public void readyPoseWithoutVelocityRemainsEnoughForAnUnpoweredAxisSample() throws Exception {
        for (Axis axis : Axis.values()) {
            Fixture f = new Fixture(configured(axis, FORWARD), false);
            f.device.velocity = Double.NaN;
            f.complete(axis, -8.0);
            assertTrue(f.predictor.getKinematicSnapshot().hasPose);
            assertFalse(f.predictor.getKinematicSnapshot().hasVelocity);
            assertEquals(Collections.singletonList(assignment("Change", axis, REVERSED)), f.advice(axis));
        }
    }

    @Test
    public void activeAndInterruptedReplacementRetainOnlyTheLastCompletedAdvice() throws Exception {
        for (Axis axis : Axis.values()) {
            Fixture f = new Fixture(configured(axis, FORWARD), false);
            f.complete(axis, -8.0);
            Object completed = f.result(axis);
            List<String> oldAdvice = f.advice(axis);
            f.queue(axis.mode);
            f.tick();
            assertEquals(axis.mode, f.mode());
            assertSame(completed, f.result(axis));
            assertEquals(oldAdvice, f.advice(axis));
            assertTrue(f.lines.contains(HISTORICAL_NOTICE));

            f.device.status = GoBildaPinpointDriver.DeviceStatus.CALIBRATING;
            f.queue(axis.mode);
            f.tick();
            assertEquals("IDLE", f.mode());
            assertSame(completed, f.result(axis));
            assertEquals(oldAdvice, f.advice(axis));
            assertTrue(f.lines.contains(HISTORICAL_HEADER));
            assertTrue(f.lines.contains(HISTORICAL_NOTICE));
            assertTrue(f.hasLineContaining("Wait for Pinpoint READY"));

            f.device.status = GoBildaPinpointDriver.DeviceStatus.READY;
            f.complete(axis, 8.0);
            assertNotSame(completed, f.result(axis));
            assertEquals(Collections.singletonList(assignment("Keep", axis, FORWARD)), f.advice(axis));
        }
    }

    @Test
    public void selectingAnotherAxisCancelsTheAttemptWithoutErasingCompletedAxes() throws Exception {
        Fixture f = new Fixture(configured(Axis.FORWARD, FORWARD), false);
        f.complete(Axis.FORWARD, -8.0);
        Object completedForward = f.result(Axis.FORWARD);
        f.queue(Axis.FORWARD.mode);
        f.tick();
        f.device.position = pose(-2.0, 0.0, 0.0);
        f.queue(Axis.LEFT.mode);
        f.tick();
        assertEquals(Axis.LEFT.mode, f.mode());
        assertSame(completedForward, f.result(Axis.FORWARD));
        assertNull(f.result(Axis.LEFT));
        f.device.position = pose(-2.0, 8.0, 0.0);
        f.queue(Axis.LEFT.mode);
        f.tick();
        assertSame(completedForward, f.result(Axis.FORWARD));
        assertNotNull(f.result(Axis.LEFT));
        assertEquals(Collections.singletonList(assignment("Change", Axis.FORWARD, REVERSED)),
                f.advice(Axis.FORWARD));
        assertEquals(Collections.singletonList(assignment("Keep", Axis.LEFT, REVERSED)),
                f.advice(Axis.LEFT));
        assertTrue(f.lines.contains(HISTORICAL_NOTICE));
    }

    @Test
    public void xClearsEveryCompletedResultAndOnlyPerformsItsDocumentedPoseRebase() throws Exception {
        Fixture f = new Fixture(configured(Axis.FORWARD, FORWARD), false);
        f.complete(Axis.FORWARD, -8.0);
        f.complete(Axis.LEFT, 8.0);
        f.completeRotation(0.0, Math.toRadians(30.0));
        assertNotNull(f.result(Axis.FORWARD));
        assertNotNull(f.result(Axis.LEFT));
        assertNotNull(field(f.owner, "rotateResult"));
        f.assertNoConfigurationWritesAfterConstruction();

        setField(f.owner, "resetRequested", true);
        f.tick();
        assertEquals("IDLE", f.mode());
        assertNull(f.result(Axis.FORWARD));
        assertNull(f.result(Axis.LEFT));
        assertNull(field(f.owner, "rotateResult"));
        assertTrue(f.advice(Axis.FORWARD).isEmpty());
        assertTrue(f.advice(Axis.LEFT).isEmpty());
        List<String> expected = new ArrayList<>(f.constructedEffects);
        expected.add("setPosition");
        assertEquals(expected, f.device.configurationEffects);
        assertEquals(0.0, f.predictor.getEstimate().fieldToRobotPose.xInches, 0.0);
        assertEquals(0.0, f.predictor.getEstimate().fieldToRobotPose.yInches, 0.0);
        assertEquals(FORWARD, f.device.forwardDirection);
        assertEquals(REVERSED, f.device.strafeDirection);
    }

    @Test
    public void oldOwnerKeepsItsCapturedDirectionAndFreshOwnerAdoptsRebuiltDraft() throws Exception {
        for (Axis axis : Axis.values()) {
            PinpointAxisDirectionTester.Config draft = configured(axis, FORWARD);
            Fixture old = new Fixture(draft, false);
            draft.pinpoint.forwardPodDirection = REVERSED;
            draft.pinpoint.strafePodDirection = REVERSED;
            Fixture fresh = new Fixture(draft, false);
            old.complete(axis, -8.0);
            fresh.complete(axis, -8.0);
            assertEquals(Collections.singletonList(assignment("Change", axis, REVERSED)), old.advice(axis));
            assertEquals(Collections.singletonList(assignment("Change", axis, FORWARD)), fresh.advice(axis));
            assertEquals(FORWARD, axis.direction(old.captured.pinpoint));
            assertEquals(REVERSED, axis.direction(fresh.captured.pinpoint));
            assertEquals(REVERSED, draft.pinpoint.forwardPodDirection);
            assertEquals(REVERSED, draft.pinpoint.strafePodDirection);
            old.assertNoConfigurationWritesAfterConstruction();
            fresh.assertNoConfigurationWritesAfterConstruction();
        }
    }

    @Test
    public void ccwSignAdviceNeverChangesEncoderDirectionsOrMakesYawScalarNegative() throws Exception {
        for (boolean initPhase : new boolean[]{false, true}) {
            for (double sign : new double[]{-1.0, 1.0}) {
                Fixture f = new Fixture(configured(Axis.FORWARD, REVERSED), initPhase);
                f.completeRotation(0.0, sign * Math.toRadians(30.0));
                if (sign > 0.0) {
                    assertTrue(f.hasLineContaining("OK: heading is CCW-positive"));
                    assertFalse(f.hasLineContaining("WRONG SIGN:"));
                } else {
                    assertTrue(f.hasLineContaining("WRONG SIGN: CCW rotation produced negative heading"));
                    assertTrue(f.hasLineContaining("mounting orientation, firmware axis settings"));
                    assertTrue(f.hasLineContaining("yawScalar must remain positive"));
                }
                assertTrue(f.advice(Axis.FORWARD).isEmpty());
                assertTrue(f.advice(Axis.LEFT).isEmpty());
                assertFalse(f.hasLineContaining("yawScalar = -"));
                f.assertNoConfigurationWritesAfterConstruction();
            }
        }
    }

    @Test
    public void rotationThresholdIsInclusiveAndKeepsItsOwnMoveFartherAdvice() throws Exception {
        for (double sign : new double[]{-1.0, 1.0}) {
            for (double degrees : new double[]{0.0, 19.999, 20.0, 20.001}) {
                Fixture f = new Fixture(configured(Axis.FORWARD, FORWARD), false);
                f.completeRotation(0.0, sign * Math.toRadians(degrees));
                assertNotNull(field(f.owner, "rotateResult"));
                assertEquals(degrees < 20.0, f.hasLineContaining("Rotate more"));
                if (degrees < 20.0) {
                    assertFalse(f.hasLineContaining("OK:"));
                    assertFalse(f.hasLineContaining("WRONG SIGN:"));
                } else {
                    assertTrue(f.hasLineContaining(sign > 0.0 ? "OK:" : "WRONG SIGN:"));
                }
            }
        }
    }

    @Test
    public void ccwAdviceUsesUnwrappedHeadingAcrossEitherPiBoundary() throws Exception {
        for (double sign : new double[]{-1.0, 1.0}) {
            Fixture f = new Fixture(configured(Axis.FORWARD, FORWARD), false);
            f.completeRotation(sign * Math.toRadians(170.0), -sign * Math.toRadians(150.0));
            Object completed = field(f.owner, "rotateResult");
            assertEquals(sign * Math.toRadians(40.0), (Double) field(completed, "dHeadingRad"), 1e-12);
            assertTrue(f.hasLineContaining(sign > 0.0 ? "OK: heading is CCW-positive"
                    : "WRONG SIGN: CCW rotation produced negative heading"));
        }
    }

    @Test
    public void repeatedRenderingDoesNotRepollReconfigureOrCompleteAnotherSample() throws Exception {
        Fixture f = new Fixture(configured(Axis.FORWARD, REVERSED), false);
        f.complete(Axis.FORWARD, -8.0);
        int polls = f.device.polls;
        Object result = f.result(Axis.FORWARD);
        List<String> lines = new ArrayList<>(f.lines);
        f.device.position = pose(40.0, 0.0, 0.0);
        for (int repeat = 0; repeat < 4; repeat++) {
            f.owner.loop(500.0);
            assertEquals(polls, f.device.polls);
            assertSame(result, f.result(Axis.FORWARD));
            assertEquals(lines, f.lines);
            assertEquals("IDLE", f.mode());
            f.assertNoConfigurationWritesAfterConstruction();
        }
    }

    @Test
    public void defensiveNonfiniteCompletedArithmeticNeverRendersAnOkOrAssignment() throws Exception {
        // Deliberately test the defensive result boundary, not an invented SDK overflow reading.
        for (String slot : new String[]{"forwardResult", "leftResult", "rotateResult"}) {
            for (double invalid : new double[]{Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
                for (int coordinate = 0; coordinate < 3; coordinate++) {
                    Fixture f = new Fixture(configured(Axis.FORWARD, REVERSED), false);
                    double[] deltas = {8.0, 8.0, Math.toRadians(30.0)};
                    deltas[coordinate] = invalid;
                    setField(f.owner, slot, completedResult(deltas[0], deltas[1], deltas[2]));
                    f.tick();
                    assertTrue(f.hasLineContaining(UNAVAILABLE));
                    assertFalse(f.hasLineContaining("OK:"));
                    assertFalse(f.hasLineContaining("WRONG SIGN:"));
                    assertTrue(f.advice(Axis.FORWARD).isEmpty());
                    assertTrue(f.advice(Axis.LEFT).isEmpty());
                    f.assertNoConfigurationWritesAfterConstruction();
                }
            }
        }
        Fixture degreesOverflow = new Fixture(configured(Axis.FORWARD, FORWARD), false);
        setField(degreesOverflow.owner, "rotateResult", completedResult(0.0, 0.0, Double.MAX_VALUE));
        degreesOverflow.tick();
        assertTrue(degreesOverflow.hasLineContaining(UNAVAILABLE));
        assertFalse(degreesOverflow.hasLineContaining("OK:"));
    }

    private enum Axis {
        FORWARD("SAMPLE_FORWARD", "forwardResult", "forwardPodDirection"),
        LEFT("SAMPLE_LEFT", "leftResult", "strafePodDirection");

        final String mode;
        final String resultField;
        final String directionField;

        Axis(String mode, String resultField, String directionField) {
            this.mode = mode;
            this.resultField = resultField;
            this.directionField = directionField;
        }

        Axis peer() { return this == FORWARD ? LEFT : FORWARD; }
        Pose2D pose(double delta) {
            return this == FORWARD ? PinpointAxisDirectionTesterTest.pose(delta, 0.0, 0.0)
                    : PinpointAxisDirectionTesterTest.pose(0.0, delta, 0.0);
        }
        GoBildaPinpointDriver.EncoderDirection direction(PinpointOdometryPredictor.Config config) {
            return this == FORWARD ? config.forwardPodDirection : config.strafePodDirection;
        }
    }

    private static PinpointAxisDirectionTester.Config configured(
            Axis axis, GoBildaPinpointDriver.EncoderDirection direction) {
        PinpointAxisDirectionTester.Config config = PinpointAxisDirectionTester.Config.defaults();
        GoBildaPinpointDriver.EncoderDirection peerDirection = direction == FORWARD ? REVERSED : FORWARD;
        config.pinpoint.forwardPodDirection = axis == Axis.FORWARD ? direction : peerDirection;
        config.pinpoint.strafePodDirection = axis == Axis.LEFT ? direction : peerDirection;
        return config;
    }

    private static String assignment(String action, Axis axis,
                                     GoBildaPinpointDriver.EncoderDirection direction) {
        return action + ": cfg.pinpoint." + axis.directionField
                + " = GoBildaPinpointDriver.EncoderDirection." + direction + ";";
    }

    /** A hardware-substitution fixture; ordinary tests drive observations through real owner loops. */
    private static final class Fixture {
        final LoopClock clock = new LoopClock();
        final List<String> lines = new ArrayList<>();
        final Device device = new Device();
        final PinpointAxisDirectionTester owner;
        final PinpointAxisDirectionTester.Config captured;
        final PinpointOdometryPredictor predictor;
        final List<String> constructedEffects;
        final boolean initPhase;

        Fixture(PinpointAxisDirectionTester.Config draft, boolean initPhase) throws Exception {
            this.initPhase = initPhase;
            clock.reset(0.0);
            owner = new PinpointAxisDirectionTester(draft);
            captured = (PinpointAxisDirectionTester.Config) field(owner, "cfg");
            predictor = device.predictor(captured.pinpoint);
            constructedEffects = new ArrayList<>(device.configurationEffects);
            Telemetry telemetry = (Telemetry) Proxy.newProxyInstance(Telemetry.class.getClassLoader(),
                    new Class<?>[]{Telemetry.class}, (proxy, method, args) -> {
                        if ("clearAll".equals(method.getName())) lines.clear();
                        if ("addLine".equals(method.getName()) && args != null) {
                            lines.add(String.valueOf(args[0]).trim());
                        }
                        if ("addData".equals(method.getName()) && args != null) {
                            Object value = args[1];
                            if (args.length == 3 && args[1] instanceof String && args[2] instanceof Object[]) {
                                value = String.format(Locale.US, (String) args[1], (Object[]) args[2]);
                            }
                            lines.add(args[0] + " = " + value);
                        }
                        return defaultValue(method.getReturnType());
                    });
            TesterContext context = new TesterContext(null, telemetry, new Gamepad(), new Gamepad(), clock);
            setField(owner, "ctx", context);
            setField(owner, "clock", clock);
            setField(owner, "gamepads", Gamepads.create(context.gamepad1, context.gamepad2));
            setField(owner, "pinpoint", predictor);
            owner.start();
        }

        void tick() {
            clock.update(clock.nowSec() + 0.02);
            if (initPhase) owner.initLoop(clock.dtSec());
            else owner.loop(clock.dtSec());
        }

        void queue(String mode) throws Exception {
            Field requested = reflectedField(owner, "sampleToggleRequested");
            for (Object candidate : requested.getType().getEnumConstants()) {
                if (mode.equals(candidate.toString())) {
                    requested.set(owner, candidate);
                    return;
                }
            }
            throw new AssertionError("Unknown test mode " + mode);
        }

        void complete(Axis axis, double delta) throws Exception {
            device.position = pose(0.0, 0.0, 0.0);
            queue(axis.mode);
            tick();
            assertEquals(axis.mode, mode());
            device.position = axis.pose(delta);
            queue(axis.mode);
            tick();
            assertEquals("IDLE", mode());
        }

        void completeRotation(double startRad, double endRad) throws Exception {
            device.position = pose(0.0, 0.0, startRad);
            queue("SAMPLE_ROTATE");
            tick();
            assertEquals("SAMPLE_ROTATE", mode());
            device.position = pose(0.0, 0.0, endRad);
            queue("SAMPLE_ROTATE");
            tick();
            assertEquals("IDLE", mode());
        }

        String mode() throws Exception { return field(owner, "mode").toString(); }
        Object result(Axis axis) throws Exception { return field(owner, axis.resultField); }

        List<String> advice(Axis axis) {
            List<String> result = new ArrayList<>();
            for (String line : lines) {
                if (line.contains("cfg.pinpoint." + axis.directionField + " = ")) result.add(line);
            }
            return result;
        }

        boolean hasLineContaining(String text) {
            for (String line : lines) if (line.contains(text)) return true;
            return false;
        }

        void assertNoConfigurationWritesAfterConstruction() {
            assertEquals("sampling/rendering advice must not reconfigure the device",
                    constructedEffects, device.configurationEffects);
        }
    }

    /** Independently scripted device observations; direction configuration never transforms them. */
    private static final class Device {
        final List<String> configurationEffects = new ArrayList<>();
        Pose2D position = pose(0.0, 0.0, 0.0);
        GoBildaPinpointDriver.DeviceStatus status = GoBildaPinpointDriver.DeviceStatus.READY;
        GoBildaPinpointDriver.EncoderDirection forwardDirection;
        GoBildaPinpointDriver.EncoderDirection strafeDirection;
        double velocity;
        int polls;

        PinpointOdometryPredictor predictor(PinpointOdometryPredictor.Config config) throws Exception {
            Class<?> deviceType = Class.forName(PinpointOdometryPredictor.class.getName() + "$PinpointDevice");
            Class<?> lookupType = Class.forName(PinpointOdometryPredictor.class.getName() + "$PinpointDeviceLookup");
            Object device = Proxy.newProxyInstance(deviceType.getClassLoader(), new Class<?>[]{deviceType},
                    (proxy, method, args) -> {
                        switch (method.getName()) {
                            case "update": polls++; return null;
                            case "getDeviceStatus": return status;
                            case "getPosition": return position;
                            case "getVelXInchesPerSec":
                            case "getVelYInchesPerSec":
                            case "getHeadingVelocityRadPerSec": return velocity;
                            case "setEncoderDirections":
                                forwardDirection = (GoBildaPinpointDriver.EncoderDirection) args[0];
                                strafeDirection = (GoBildaPinpointDriver.EncoderDirection) args[1];
                                configurationEffects.add("directions:" + forwardDirection + "," + strafeDirection);
                                return null;
                            case "setPosition":
                                position = (Pose2D) args[0];
                                configurationEffects.add("setPosition");
                                return null;
                            default:
                                if (method.getName().startsWith("set") || method.getName().startsWith("reset")
                                        || method.getName().startsWith("recalibrate")) {
                                    configurationEffects.add(method.getName());
                                }
                                return defaultValue(method.getReturnType());
                        }
                    });
            Object lookup = Proxy.newProxyInstance(lookupType.getClassLoader(), new Class<?>[]{lookupType},
                    (proxy, method, args) -> device);
            Constructor<PinpointOdometryPredictor> constructor = PinpointOdometryPredictor.class
                    .getDeclaredConstructor(lookupType, PinpointOdometryPredictor.Config.class);
            constructor.setAccessible(true);
            return constructor.newInstance(lookup, config);
        }
    }

    private static Pose2D pose(double xInches, double yInches, double headingRad) {
        return new Pose2D(DistanceUnit.INCH, xInches, yInches, AngleUnit.RADIANS, headingRad);
    }

    private static Object completedResult(double dx, double dy, double heading) throws Exception {
        Class<?> resultType = Class.forName(PinpointAxisDirectionTester.class.getName() + "$SampleResult");
        Constructor<?> constructor = resultType.getDeclaredConstructor(double.class, double.class, double.class);
        constructor.setAccessible(true);
        return constructor.newInstance(dx, dy, heading);
    }

    private static Field reflectedField(Object owner, String name) throws Exception {
        for (Class<?> type = owner.getClass(); type != null; type = type.getSuperclass()) {
            try {
                Field field = type.getDeclaredField(name);
                field.setAccessible(true);
                return field;
            } catch (NoSuchFieldException missing) {
                // FTC context belongs to the base owner; sample state belongs to the concrete tester.
            }
        }
        throw new NoSuchFieldException(name);
    }

    private static Object field(Object owner, String name) throws Exception {
        return reflectedField(owner, name).get(owner);
    }

    private static void setField(Object owner, String name, Object value) throws Exception {
        reflectedField(owner, name).set(owner, value);
    }

    private static Object defaultValue(Class<?> type) {
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
