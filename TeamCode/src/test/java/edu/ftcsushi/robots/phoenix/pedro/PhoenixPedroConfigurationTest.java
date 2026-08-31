package edu.ftcsushi.robots.phoenix.pedro;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;

import edu.ftcsushi.fw.core.hal.Direction;
import edu.ftcsushi.fw.ftc.FtcDrives;
import edu.ftcsushi.fw.ftc.localization.PinpointOdometryPredictor;
import edu.ftcsushi.fw.integrations.pedro.PedroFieldTransform;
import edu.ftcsushi.fw.integrations.pedro.PedroPathingRuntime;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Verifies the project exposes one pure Phoenix-to-Pedro configuration path. */
public final class PhoenixPedroConfigurationTest {

    private static final double EPSILON = 1e-9;

    @Test
    public void publicSurfaceIsOnePureMapperWithNoMutableGlobals() throws Exception {
        List<String> publicFields = new ArrayList<String>();
        for (Field field : PhoenixPedroConfiguration.class.getDeclaredFields()) {
            if (Modifier.isPublic(field.getModifiers())) {
                publicFields.add(field.getName());
            }
        }
        assertTrue(publicFields.toString(), publicFields.isEmpty());

        List<String> publicMethods = new ArrayList<String>();
        for (Method method : PhoenixPedroConfiguration.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                publicMethods.add(method.getName());
            }
        }
        assertEquals(1, publicMethods.size());
        assertEquals("phoenixAutoRuntimeConfig", publicMethods.get(0));

        Method mapper = PhoenixPedroConfiguration.class.getDeclaredMethod(
                "phoenixAutoRuntimeConfig",
                PinpointOdometryPredictor.Config.class,
                FtcDrives.MecanumWiringConfig.class,
                boolean.class
        );
        assertEquals(Modifier.PUBLIC | Modifier.STATIC, mapper.getModifiers());
        assertSame(PedroPathingRuntime.Config.class, mapper.getReturnType());
        assertEquals(3, mapper.getParameterTypes().length);
        assertSame(PinpointOdometryPredictor.Config.class, mapper.getParameterTypes()[0]);
        assertSame(FtcDrives.MecanumWiringConfig.class, mapper.getParameterTypes()[1]);
        assertSame(boolean.class, mapper.getParameterTypes()[2]);
        assertEquals(0, mapper.getExceptionTypes().length);
        assertFalse(mapper.isVarArgs());

        Method nativeToolFactory = PhoenixPedroConfiguration.class.getDeclaredMethod(
                "createToolOnlyNativeFollower",
                HardwareMap.class
        );
        assertFalse(Modifier.isPublic(nativeToolFactory.getModifiers()));
        assertFalse(Modifier.isProtected(nativeToolFactory.getModifiers()));
        assertFalse(Modifier.isPrivate(nativeToolFactory.getModifiers()));
    }

    @Test
    public void mapperSnapshotsOnlyPedroOwnedFactsAndCheckedInTuning() {
        FtcDrives.MecanumWiringConfig wiring = FtcDrives.MecanumWiringConfig.defaults();
        wiring.frontLeftName = "profile-fl";
        wiring.frontRightName = "profile-fr";
        wiring.backLeftName = "profile-bl";
        wiring.backRightName = "profile-br";
        wiring.frontLeftDirection = Direction.FORWARD;
        wiring.frontRightDirection = Direction.REVERSE;
        wiring.backLeftDirection = Direction.FORWARD;
        wiring.backRightDirection = Direction.REVERSE;

        PinpointOdometryPredictor.Config predictor = PinpointOdometryPredictor.Config.defaults();
        predictor.hardwareMapName = "profile-pinpoint";
        predictor.forwardPodOffsetLeftInches = 3.25;
        predictor.strafePodOffsetForwardInches = -4.5;
        predictor.encoderResolution =
                PinpointOdometryPredictor.EncoderResolution.ticksPerInch(1234.5);
        predictor.forwardPodDirection =
                GoBildaPinpointDriver.EncoderDirection.REVERSED;
        predictor.strafePodDirection =
                GoBildaPinpointDriver.EncoderDirection.REVERSED;
        predictor.yawScalar = 1.001;
        predictor.quality = 0.63;

        PedroPathingRuntime.Config mapped = PhoenixPedroConfiguration.phoenixAutoRuntimeConfig(
                predictor,
                wiring,
                true
        );

        assertNotSame(predictor, mapped.predictor);
        assertEquals("profile-pinpoint", mapped.predictor.hardwareMapName);
        assertEquals(3.25, mapped.predictor.forwardPodOffsetLeftInches, EPSILON);
        assertEquals(-4.5, mapped.predictor.strafePodOffsetForwardInches, EPSILON);
        assertSame(predictor.encoderResolution, mapped.predictor.encoderResolution);
        assertSame(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                mapped.predictor.forwardPodDirection);
        assertSame(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                mapped.predictor.strafePodDirection);
        assertEquals(1.001, mapped.predictor.yawScalar, EPSILON);
        assertEquals(0.63, mapped.predictor.quality, EPSILON);

        MecanumConstants drive = mapped.mecanumConstants;
        assertEquals("profile-fl", drive.leftFrontMotorName);
        assertEquals("profile-fr", drive.rightFrontMotorName);
        assertEquals("profile-bl", drive.leftRearMotorName);
        assertEquals("profile-br", drive.rightRearMotorName);
        assertSame(DcMotorSimple.Direction.FORWARD, drive.leftFrontMotorDirection);
        assertSame(DcMotorSimple.Direction.REVERSE, drive.rightFrontMotorDirection);
        assertSame(DcMotorSimple.Direction.FORWARD, drive.leftRearMotorDirection);
        assertSame(DcMotorSimple.Direction.REVERSE, drive.rightRearMotorDirection);
        assertTrue(drive.useBrakeModeInTeleOp);

        assertEquals(9.616158, mapped.followerConstants.mass, EPSILON);
        assertEquals(0.99, mapped.pathConstraints.getTValueConstraint(), EPSILON);
        assertEquals(0.1, mapped.pathConstraints.getVelocityConstraint(), EPSILON);
        assertEquals(0.1, mapped.pathConstraints.getTranslationalConstraint(), EPSILON);
        assertEquals(0.007, mapped.pathConstraints.getHeadingConstraint(), EPSILON);
        assertEquals(100.0, mapped.pathConstraints.getTimeoutConstraint(), EPSILON);
        assertEquals(1.0, mapped.pathConstraints.getBrakingStrength(), EPSILON);
        assertEquals(10, mapped.pathConstraints.getBEZIER_CURVE_SEARCH_LIMIT());
        assertEquals(1.0, mapped.pathConstraints.getBrakingStart(), EPSILON);
        assertSame(PedroFieldTransform.decodeInvertedFtc(), mapped.fieldTransform);
    }

    @Test
    public void mapperPreservesRelevantNullAndInvalidDraftEvidenceForRuntimeValidation() {
        assertNull(PhoenixPedroConfiguration.phoenixAutoRuntimeConfig(
                null,
                FtcDrives.MecanumWiringConfig.defaults(),
                true
        ).predictor);
        assertNull(PhoenixPedroConfiguration.phoenixAutoRuntimeConfig(
                PinpointOdometryPredictor.Config.defaults(),
                null,
                true
        ).mecanumConstants);

        PinpointOdometryPredictor.Config predictor = PinpointOdometryPredictor.Config.defaults();
        predictor.quality = Double.longBitsToDouble(0x7ff8000000000042L);
        FtcDrives.MecanumWiringConfig wiring = FtcDrives.MecanumWiringConfig.defaults();
        wiring.frontLeftName = null;
        wiring.frontLeftDirection = null;
        PedroPathingRuntime.Config raw = PhoenixPedroConfiguration.phoenixAutoRuntimeConfig(
                predictor,
                wiring,
                false
        );

        assertEquals(
                Double.doubleToRawLongBits(predictor.quality),
                Double.doubleToRawLongBits(raw.predictor.quality)
        );
        assertNull(raw.mecanumConstants.leftFrontMotorName);
        assertNull(raw.mecanumConstants.leftFrontMotorDirection);
    }

    @Test
    public void mapperReturnsIndependentGraphsAndDoesNotMutateItsSource() {
        PinpointOdometryPredictor.Config predictor = PinpointOdometryPredictor.Config.defaults();
        FtcDrives.MecanumWiringConfig wiring = FtcDrives.MecanumWiringConfig.defaults();
        PinpointOdometryPredictor.Config sourcePredictor = predictor.copy();
        FtcDrives.MecanumWiringConfig sourceWiring = wiring.copy();

        PedroPathingRuntime.Config first = PhoenixPedroConfiguration.phoenixAutoRuntimeConfig(
                predictor,
                wiring,
                true
        );
        PedroPathingRuntime.Config second = PhoenixPedroConfiguration.phoenixAutoRuntimeConfig(
                predictor,
                wiring,
                true
        );

        assertNotSame(first, second);
        assertNotSame(first.predictor, second.predictor);
        assertNotSame(first.followerConstants, second.followerConstants);
        assertNotSame(first.followerConstants.coefficientsTranslationalPIDF,
                second.followerConstants.coefficientsTranslationalPIDF);
        assertNotSame(first.mecanumConstants, second.mecanumConstants);
        assertNotSame(first.mecanumConstants.frontLeftVector,
                second.mecanumConstants.frontLeftVector);
        assertNotSame(first.pathConstraints, second.pathConstraints);

        first.predictor.hardwareMapName = "mutated-copy-pinpoint";
        first.followerConstants.mass = 123.0;
        first.mecanumConstants.leftFrontMotorName = "mutated-copy-motor";
        first.pathConstraints.setTimeoutConstraint(4321.0);

        assertEquals(sourcePredictor.hardwareMapName, second.predictor.hardwareMapName);
        assertEquals(sourceWiring.frontLeftName, second.mecanumConstants.leftFrontMotorName);
        assertEquals(9.616158, second.followerConstants.mass, EPSILON);
        assertEquals(100.0, second.pathConstraints.getTimeoutConstraint(), EPSILON);
        assertPredictorEquals(sourcePredictor, predictor);
        assertWiringEquals(sourceWiring, wiring);

        predictor.hardwareMapName = "mutated-source-pinpoint";
        wiring.frontLeftName = "mutated-source-motor";
        assertEquals(sourcePredictor.hardwareMapName, second.predictor.hardwareMapName);
        assertEquals(sourceWiring.frontLeftName, second.mecanumConstants.leftFrontMotorName);
    }

    @Test
    public void mapperRecipeIsIndependentOfMutablePedroDefaultConstraints() {
        PathConstraints original = PathConstraints.defaultConstraints;
        PathConstraints poison = new PathConstraints(
                0.123,
                2.0,
                3.0,
                4.0,
                5.0,
                6.0,
                7,
                8.0
        );
        try {
            PathConstraints.setDefaultConstraints(poison);

            PedroPathingRuntime.Config mapped = freshMappedRuntime();

            assertSame(poison, PathConstraints.defaultConstraints);
            assertEquals(0.99, mapped.pathConstraints.getTValueConstraint(), EPSILON);
            assertEquals(0.1, mapped.pathConstraints.getVelocityConstraint(), EPSILON);
            assertEquals(0.1, mapped.pathConstraints.getTranslationalConstraint(), EPSILON);
            assertEquals(0.007, mapped.pathConstraints.getHeadingConstraint(), EPSILON);
            assertEquals(100.0, mapped.pathConstraints.getTimeoutConstraint(), EPSILON);
            assertEquals(1.0, mapped.pathConstraints.getBrakingStrength(), EPSILON);
            assertEquals(10, mapped.pathConstraints.getBEZIER_CURVE_SEARCH_LIMIT());
            assertEquals(1.0, mapped.pathConstraints.getBrakingStart(), EPSILON);

            mapped.pathConstraints.setTValueConstraint(0.5);
            assertSame(poison, PathConstraints.defaultConstraints);
            assertEquals(0.123, poison.getTValueConstraint(), EPSILON);
            assertEquals(5.0, poison.getTimeoutConstraint(), EPSILON);
        } finally {
            PathConstraints.setDefaultConstraints(original);
        }
        assertSame(original, PathConstraints.defaultConstraints);
    }

    @Test
    public void nativePinpointTranslationPreservesCustomAndPresetResolution() throws Exception {
        PinpointOdometryPredictor.Config custom =
                PinpointOdometryPredictor.Config.defaults();
        custom.hardwareMapName = "profile-pinpoint";
        custom.forwardPodOffsetLeftInches = 3.25;
        custom.strafePodOffsetForwardInches = -4.5;
        custom.encoderResolution =
                PinpointOdometryPredictor.EncoderResolution.ticksPerInch(1234.5);
        custom.forwardPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        custom.strafePodDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        custom.yawScalar = 1.001;

        PinpointConstants translatedCustom = invokePinpointConstantsFrom(
                custom.validatedCopy("test.predictor")
        );
        assertEquals("profile-pinpoint", translatedCustom.hardwareMapName);
        assertEquals(3.25, translatedCustom.forwardPodY, EPSILON);
        assertEquals(-4.5, translatedCustom.strafePodX, EPSILON);
        assertTrue(translatedCustom.customEncoderResolution.isPresent());
        assertEquals(1234.5,
                translatedCustom.customEncoderResolution.getAsDouble(), EPSILON);
        assertSame(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                translatedCustom.forwardEncoderDirection);
        assertSame(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                translatedCustom.strafeEncoderDirection);
        assertTrue(translatedCustom.yawScalar.isPresent());
        assertEquals(1.001, translatedCustom.yawScalar.getAsDouble(), EPSILON);

        PinpointOdometryPredictor.Config preset =
                PinpointOdometryPredictor.Config.defaults();
        preset.encoderResolution = PinpointOdometryPredictor.EncoderResolution.forGoBildaPod(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
        );
        PinpointConstants translatedPreset = invokePinpointConstantsFrom(
                preset.validatedCopy("test.predictor")
        );
        assertFalse(translatedPreset.customEncoderResolution.isPresent());
        assertSame(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD,
                translatedPreset.encoderResolution);
    }

    @Test
    public void nativeToolValidatesBeforeEffectsThenBuildsFreshGraphsInExactOrder() {
        HardwareMap hardwareMap = new HardwareMap(null, null);
        PedroPathingRuntime.Config runtimeConfig = freshMappedRuntime();
        RecordingNativeConstruction construction = new RecordingNativeConstruction();

        Follower first = PhoenixPedroConfiguration.createToolOnlyNativeFollowerForTest(
                hardwareMap,
                runtimeConfig,
                construction
        );
        Follower second = PhoenixPedroConfiguration.createToolOnlyNativeFollowerForTest(
                hardwareMap,
                runtimeConfig,
                construction
        );

        assertNotNull(first);
        assertNotNull(second);
        assertNotSame(first, second);
        assertEquals(
                asList("mecanum", "pinpoint", "follower", "follower.vendorBreak",
                        "mecanum", "pinpoint", "follower", "follower.vendorBreak"),
                construction.events
        );
        assertFalse(construction.events.contains("factory.cleanup"));
        assertEquals(2, construction.mecanumConstants.size());
        assertNotSame(construction.mecanumConstants.get(0),
                construction.mecanumConstants.get(1));
        assertNotSame(construction.mecanumConstants.get(0).frontLeftVector,
                construction.mecanumConstants.get(1).frontLeftVector);
        assertNotSame(construction.pinpointConstants.get(0),
                construction.pinpointConstants.get(1));
        assertNotSame(construction.followerConstants.get(0),
                construction.followerConstants.get(1));
        assertNotSame(
                construction.followerConstants.get(0).coefficientsTranslationalPIDF,
                construction.followerConstants.get(1).coefficientsTranslationalPIDF
        );
        assertNotSame(construction.pathConstraints.get(0),
                construction.pathConstraints.get(1));
    }

    @Test
    public void nativeToolInvalidRuntimeDraftProducesNoConstructionEffect() {
        FtcDrives.MecanumWiringConfig wiring = FtcDrives.MecanumWiringConfig.defaults();
        wiring.frontRightName = wiring.frontLeftName;
        PedroPathingRuntime.Config runtimeConfig = PhoenixPedroConfiguration.phoenixAutoRuntimeConfig(
                PinpointOdometryPredictor.Config.defaults(),
                wiring,
                true
        );
        RecordingNativeConstruction construction = new RecordingNativeConstruction();

        try {
            PhoenixPedroConfiguration.createToolOnlyNativeFollowerForTest(
                    new HardwareMap(null, null),
                    runtimeConfig,
                    construction
            );
            fail("expected duplicate motor identity to fail preflight");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("mecanumConstants"));
            assertTrue(expected.getMessage().contains("MotorName"));
        }

        assertTrue(construction.events.toString(), construction.events.isEmpty());
    }

    @Test
    public void nativeToolRelevantNullsUseCanonicalDiagnosticsBeforeAnyRoleEffect() {
        String canonicalRoot = PedroPathingRuntime.Config.class.getCanonicalName();

        assertCanonicalNullPreflight(
                PhoenixPedroConfiguration.phoenixAutoRuntimeConfig(
                        null,
                        FtcDrives.MecanumWiringConfig.defaults(),
                        true
                ),
                canonicalRoot + ".predictor must not be null"
        );

        assertCanonicalNullPreflight(
                PhoenixPedroConfiguration.phoenixAutoRuntimeConfig(
                        PinpointOdometryPredictor.Config.defaults(),
                        null,
                        true
                ),
                canonicalRoot + ".mecanumConstants must not be null"
        );

        PinpointOdometryPredictor.Config invalidPredictor =
                PinpointOdometryPredictor.Config.defaults();
        invalidPredictor.quality = Double.NaN;
        assertCanonicalInvalidPreflight(
                PhoenixPedroConfiguration.phoenixAutoRuntimeConfig(
                        invalidPredictor,
                        FtcDrives.MecanumWiringConfig.defaults(),
                        true
                ),
                canonicalRoot + ".predictor.quality must be finite and in [0, 1], got NaN"
        );
    }

    @Test
    public void nativeToolStopsMecanumWhenPinpointOrFollowerConstructionFails() {
        RuntimeException pinpointFailure = new RuntimeException("pinpoint failed");
        RecordingNativeConstruction pinpointConstruction = new RecordingNativeConstruction();
        pinpointConstruction.pinpointFailure = pinpointFailure;
        assertWrappedFailure(pinpointConstruction, pinpointFailure);
        assertEquals(asList("mecanum", "pinpoint", "factory.cleanup"),
                pinpointConstruction.events);

        RuntimeException followerFailure = new RuntimeException("follower failed");
        RecordingNativeConstruction followerConstruction = new RecordingNativeConstruction();
        followerConstruction.followerFailure = followerFailure;
        assertWrappedFailure(followerConstruction, followerFailure);
        assertEquals(asList("mecanum", "pinpoint", "follower", "factory.cleanup"),
                followerConstruction.events);
    }

    @Test
    public void nativeToolPreservesPrimaryFailureAndSuppressesDistinctStopFailure() {
        RuntimeException pinpointFailure = new RuntimeException("pinpoint failed");
        RuntimeException stopFailure = new RuntimeException("stop failed");
        RecordingNativeConstruction construction = new RecordingNativeConstruction();
        construction.pinpointFailure = pinpointFailure;
        construction.stopFailure = stopFailure;

        assertWrappedFailure(construction, pinpointFailure);

        assertEquals(asList("mecanum", "pinpoint", "factory.cleanup"),
                construction.events);
        assertEquals(1, pinpointFailure.getSuppressed().length);
        assertSame(stopFailure, pinpointFailure.getSuppressed()[0]);
    }

    private static void assertPredictorEquals(PinpointOdometryPredictor.Config expected,
                                              PinpointOdometryPredictor.Config actual) {
        assertEquals(expected.hardwareMapName, actual.hardwareMapName);
        assertEquals(expected.forwardPodOffsetLeftInches,
                actual.forwardPodOffsetLeftInches, 0.0);
        assertEquals(expected.strafePodOffsetForwardInches,
                actual.strafePodOffsetForwardInches, 0.0);
        assertSame(expected.encoderResolution, actual.encoderResolution);
        assertSame(expected.forwardPodDirection, actual.forwardPodDirection);
        assertSame(expected.strafePodDirection, actual.strafePodDirection);
        assertEquals(expected.yawScalar, actual.yawScalar);
        assertEquals(expected.quality, actual.quality, 0.0);
    }

    private static void assertWiringEquals(FtcDrives.MecanumWiringConfig expected,
                                           FtcDrives.MecanumWiringConfig actual) {
        assertEquals(expected.frontLeftName, actual.frontLeftName);
        assertEquals(expected.frontRightName, actual.frontRightName);
        assertEquals(expected.backLeftName, actual.backLeftName);
        assertEquals(expected.backRightName, actual.backRightName);
        assertSame(expected.frontLeftDirection, actual.frontLeftDirection);
        assertSame(expected.frontRightDirection, actual.frontRightDirection);
        assertSame(expected.backLeftDirection, actual.backLeftDirection);
        assertSame(expected.backRightDirection, actual.backRightDirection);
    }

    private static PinpointConstants invokePinpointConstantsFrom(
            PinpointOdometryPredictor.Config predictor
    ) throws Exception {
        Method method = PhoenixPedroConfiguration.class.getDeclaredMethod(
                "pinpointConstantsFrom",
                PinpointOdometryPredictor.Config.class
        );
        method.setAccessible(true);
        return (PinpointConstants) method.invoke(null, predictor);
    }

    private static void assertWrappedFailure(RecordingNativeConstruction construction,
                                             RuntimeException expectedCause) {
        try {
            PhoenixPedroConfiguration.createToolOnlyNativeFollowerForTest(
                    new HardwareMap(null, null),
                    freshMappedRuntime(),
                    construction
            );
            fail("expected native-tool construction failure");
        } catch (IllegalStateException expected) {
            assertSame(expectedCause, expected.getCause());
        }
    }

    private static void assertCanonicalNullPreflight(PedroPathingRuntime.Config runtimeConfig,
                                                     String exactMessage) {
        RecordingNativeConstruction construction = new RecordingNativeConstruction();
        try {
            PhoenixPedroConfiguration.createToolOnlyNativeFollowerForTest(
                    new HardwareMap(null, null),
                    runtimeConfig,
                    construction
            );
            fail("expected canonical preflight rejection: " + exactMessage);
        } catch (NullPointerException expected) {
            assertEquals(exactMessage, expected.getMessage());
        }
        assertTrue(construction.events.toString(), construction.events.isEmpty());
    }

    private static void assertCanonicalInvalidPreflight(
            PedroPathingRuntime.Config runtimeConfig,
                                                        String exactMessage) {
        RecordingNativeConstruction construction = new RecordingNativeConstruction();
        try {
            PhoenixPedroConfiguration.createToolOnlyNativeFollowerForTest(
                    new HardwareMap(null, null),
                    runtimeConfig,
                    construction
            );
            fail("expected canonical preflight rejection: " + exactMessage);
        } catch (IllegalArgumentException expected) {
            assertEquals(exactMessage, expected.getMessage());
        }
        assertTrue(construction.events.toString(), construction.events.isEmpty());
    }

    private static PedroPathingRuntime.Config freshMappedRuntime() {
        return PhoenixPedroConfiguration.phoenixAutoRuntimeConfig(
                PinpointOdometryPredictor.Config.defaults(),
                FtcDrives.MecanumWiringConfig.defaults(),
                true
        );
    }

    private static List<String> asList(String... values) {
        List<String> result = new ArrayList<String>();
        for (String value : values) {
            result.add(value);
        }
        return result;
    }

    private static final class RecordingNativeConstruction
            implements PhoenixPedroConfiguration.NativeToolConstruction {
        final List<String> events = new ArrayList<String>();
        final List<MecanumConstants> mecanumConstants =
                new ArrayList<MecanumConstants>();
        final List<PinpointConstants> pinpointConstants =
                new ArrayList<PinpointConstants>();
        final List<FollowerConstants> followerConstants =
                new ArrayList<FollowerConstants>();
        final List<PathConstraints> pathConstraints =
                new ArrayList<PathConstraints>();

        RuntimeException pinpointFailure;
        RuntimeException followerFailure;
        RuntimeException stopFailure;
        RecordingDrivetrain currentDrivetrain;

        @Override
        public Drivetrain createMecanum(HardwareMap hardwareMap,
                                        MecanumConstants constants) {
            events.add("mecanum");
            mecanumConstants.add(constants);
            currentDrivetrain = new RecordingDrivetrain(events, stopFailure);
            return currentDrivetrain;
        }

        @Override
        public Localizer createPinpoint(HardwareMap hardwareMap,
                                        PinpointConstants constants) {
            events.add("pinpoint");
            pinpointConstants.add(constants);
            if (pinpointFailure != null) {
                throw pinpointFailure;
            }
            return new RecordingLocalizer();
        }

        @Override
        public Follower createFollower(FollowerConstants constants,
                                       Localizer localizer,
                                       Drivetrain drivetrain,
                                       PathConstraints constraints) {
            events.add("follower");
            followerConstants.add(constants);
            pathConstraints.add(constraints);
            if (followerFailure != null) {
                throw followerFailure;
            }
            currentDrivetrain.insideFollowerConstructor = true;
            try {
                return new Follower(constants, localizer, drivetrain, constraints);
            } finally {
                currentDrivetrain.insideFollowerConstructor = false;
            }
        }
    }

    private static final class RecordingDrivetrain extends Drivetrain {
        private final List<String> events;
        private final RuntimeException stopFailure;
        private boolean insideFollowerConstructor;
        private double xVelocity;
        private double yVelocity;

        RecordingDrivetrain(List<String> events, RuntimeException stopFailure) {
            this.events = events;
            this.stopFailure = stopFailure;
            setMaxPowerScaling(1.0);
            setNominalVoltage(12.0);
        }

        @Override
        public double[] calculateDrive(Vector correctivePower,
                                       Vector headingPower,
                                       Vector pathingPower,
                                       double robotHeading) {
            return new double[4];
        }

        @Override
        public void updateConstants() {
        }

        @Override
        public void breakFollowing() {
            if (insideFollowerConstructor) {
                events.add("follower.vendorBreak");
                return;
            }
            events.add("factory.cleanup");
            if (stopFailure != null) {
                throw stopFailure;
            }
        }

        @Override
        public void runDrive(double[] drivePowers) {
        }

        @Override
        public void startTeleopDrive() {
        }

        @Override
        public void startTeleopDrive(boolean brakeMode) {
        }

        @Override
        public double xVelocity() {
            return xVelocity;
        }

        @Override
        public double yVelocity() {
            return yVelocity;
        }

        @Override
        public void setXVelocity(double xMovement) {
            xVelocity = xMovement;
        }

        @Override
        public void setYVelocity(double yMovement) {
            yVelocity = yMovement;
        }

        @Override
        public double getVoltage() {
            return 12.0;
        }

        @Override
        public String debugString() {
            return "RecordingDrivetrain";
        }
    }

    private static final class RecordingLocalizer implements Localizer {
        private Pose pose = new Pose();

        @Override
        public Pose getPose() {
            return pose;
        }

        @Override
        public Pose getVelocity() {
            return new Pose();
        }

        @Override
        public Vector getVelocityVector() {
            return getVelocity().getAsVector();
        }

        @Override
        public void setStartPose(Pose startPose) {
            pose = startPose;
        }

        @Override
        public void setPose(Pose nextPose) {
            pose = nextPose;
        }

        @Override
        public void update() {
        }

        @Override
        public double getTotalHeading() {
            return pose.getHeading();
        }

        @Override
        public double getForwardMultiplier() {
            return 1.0;
        }

        @Override
        public double getLateralMultiplier() {
            return 1.0;
        }

        @Override
        public double getTurningMultiplier() {
            return 1.0;
        }

        @Override
        public void resetIMU() {
        }

        @Override
        public double getIMUHeading() {
            return pose.getHeading();
        }

        @Override
        public boolean isNAN() {
            return false;
        }
    }
}
