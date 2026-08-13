package edu.ftcphoenix.fw.ftc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.ftcphoenix.fw.actuation.Plant;
import edu.ftcphoenix.fw.actuation.Plants;
import edu.ftcphoenix.fw.actuation.ScalarRange;
import edu.ftcphoenix.fw.core.hal.Direction;
import edu.ftcphoenix.fw.core.source.ScalarSource;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Focused TUNE-02 coverage for the narrow FTC velocity-PIDF configuration capability. */
public final class FtcMotorVelocityPidfTest {

    private static final double COEFFICIENT_SCALE = 65536.0;
    private static final double MAX_COEFFICIENT = Integer.MAX_VALUE / COEFFICIENT_SCALE;
    private static final double MIN_COEFFICIENT = -MAX_COEFFICIENT;

    @Test
    public void publicSurfaceIsOneFactoryAndAConfigurationOnlyCapability() throws Exception {
        Method factory = FtcMotorControllers.class.getMethod(
                "velocityPidf", Plant.class);
        assertEquals(FtcMotorVelocityPidf.class, factory.getReturnType());

        assertEquals(1, publicDeclaredMethodCount(FtcMotorControllers.class));
        assertEquals(7, publicDeclaredMethodCount(FtcMotorVelocityPidf.class));
        assertPublicMethod(FtcMotorVelocityPidf.class, "setGains", void.class,
                double.class, double.class, double.class, double.class);
        assertPublicMethod(FtcMotorVelocityPidf.class, "plantTargetRange", ScalarRange.class);
        assertPublicMethod(FtcMotorVelocityPidf.class, "getKP", double.class);
        assertPublicMethod(FtcMotorVelocityPidf.class, "getKI", double.class);
        assertPublicMethod(FtcMotorVelocityPidf.class, "getKD", double.class);
        assertPublicMethod(FtcMotorVelocityPidf.class, "getKF", double.class);
        assertPublicMethod(FtcMotorVelocityPidf.class, "restoreInitial", void.class);
        assertTrue(Modifier.isPublic(FtcMotorVelocityPidf.class.getModifiers()));
        assertTrue(Modifier.isFinal(FtcMotorVelocityPidf.class.getModifiers()));
        assertEquals(0, Arrays.stream(FtcMotorVelocityPidf.class.getDeclaredConstructors())
                .filter(constructor -> Modifier.isPublic(constructor.getModifiers()))
                .count());

        TestHardwareMap hardwareMap = mapWithMotor(
                "flywheel", coefficients(1.0, 2.0, 3.0, 4.0, MotorControlAlgorithm.PIDF));
        Plant plant = deviceManagedVelocityPlant(hardwareMap, "flywheel");
        FtcMotorVelocityPidf handle =
                FtcMotorControllers.velocityPidf(plant);
        assertEquals(FtcMotorVelocityPidf.class, handle.getClass());
    }

    @Test
    public void factoryAcceptsTheExplicitDeviceManagedPidfBranchAndCapturesItsResult() {
        TestHardwareMap hardwareMap = mapWithMotor(
                "flywheel", coefficients(1.0, 2.0, 3.0, 4.0, MotorControlAlgorithm.PIDF));
        MotorProbe probe = hardwareMap.motor("flywheel");
        Plant plant = FtcActuators.plant(hardwareMap)
                .motor("flywheel", Direction.FORWARD)
                .velocity()
                .deviceManaged()
                .velocityPidf(5.0, 6.0, 7.0, 8.0)
                .bounded(0.0, 5000.0)
                .nativeUnits()
                .velocityTolerance(1.0)
                .targetFromNewCommand(0.0)
                .build();
        hardwareMap.resetObservations();

        FtcMotorVelocityPidf handle = FtcMotorControllers.velocityPidf(plant);

        assertRawDoubleEquals(5.0, handle.getKP());
        assertRawDoubleEquals(6.0, handle.getKI());
        assertRawDoubleEquals(7.0, handle.getKD());
        assertRawDoubleEquals(8.0, handle.getKF());
        assertEquals(1, probe.pidfReadAttempts);
        probe.assertNoWritesOrActuation();
    }

    @Test
    public void factoryRejectsMissingPlant() {
        IllegalArgumentException missingPlant = assertThrows(
                IllegalArgumentException.class,
                () -> FtcMotorControllers.velocityPidf(null));
        assertContains(missingPlant, "requires a Plant");
    }

    @Test
    public void factoryRejectsHardwareNeutralVelocityPlantBeforeControllerReadback() {
        TestHardwareMap hardwareMap = mapWithMotor(
                "flywheel", coefficients(1.0, 2.0, 3.0, 4.0, MotorControlAlgorithm.PIDF));
        MotorProbe probe = hardwareMap.motor("flywheel");
        Plant rawPlant = Plants.fromOutputs()
                .deviceManagedVelocity(
                        FtcHardware.motorVelocity(probe.motor, Direction.FORWARD),
                        ScalarSource.constant(0.0))
                .bounded(-1000.0, 1000.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetFromNewCommand(0.0)
                .build();
        hardwareMap.resetObservations();

        assertIneligiblePlant(rawPlant, probe);
    }

    @Test
    public void factoryRejectsFtcPowerAndRegulatedVelocityPlantsBeforeControllerReadback() {
        TestHardwareMap powerMap = mapWithMotor(
                "powerMotor", coefficients(1.0, 2.0, 3.0, 4.0, MotorControlAlgorithm.PIDF));
        MotorProbe powerProbe = powerMap.motor("powerMotor");
        Plant powerPlant = FtcActuators.plant(powerMap)
                .motor("powerMotor", Direction.FORWARD)
                .power()
                .targetFromNewCommand(0.0)
                .build();
        powerMap.resetObservations();

        assertIneligiblePlant(powerPlant, powerProbe);

        TestHardwareMap regulatedMap = mapWithMotor(
                "regulatedMotor",
                coefficients(1.0, 2.0, 3.0, 4.0, MotorControlAlgorithm.PIDF));
        MotorProbe regulatedProbe = regulatedMap.motor("regulatedMotor");
        Plant regulatedPlant = FtcActuators.plant(regulatedMap)
                .motor("regulatedMotor", Direction.FORWARD)
                .velocity()
                .regulated()
                .nativeFeedback(ScalarSource.constant(0.0))
                .regulator((setpoint, measurement, clock) -> 0.0)
                .bounded(-1000.0, 1000.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetFromNewCommand(0.0)
                .build();
        regulatedMap.resetObservations();

        assertIneligiblePlant(regulatedPlant, regulatedProbe);
    }

    @Test
    public void factoryRejectsMultiMotorDeviceManagedVelocityPlantBeforeControllerReadback() {
        PIDFCoefficients initial = coefficients(
                1.0, 2.0, 3.0, 4.0, MotorControlAlgorithm.PIDF);
        TestHardwareMap hardwareMap = mapWithMotor("left", initial);
        MotorProbe left = hardwareMap.motor("left");
        MotorProbe right = hardwareMap.addMotor("right", initial);
        Plant grouped = FtcActuators.plant(hardwareMap)
                .motor("left", Direction.FORWARD)
                .andMotor("right", Direction.REVERSE)
                .velocity()
                .deviceManagedWithDefaults()
                .bounded(-1000.0, 1000.0)
                .nativeUnits()
                .velocityTolerance(0.0)
                .targetFromNewCommand(0.0)
                .build();
        hardwareMap.resetObservations();

        assertIneligiblePlant(grouped, left, right);
    }

    @Test
    public void handleReportsTheExactBoundPlantsTargetRangeInPlantUnits() {
        TestHardwareMap hardwareMap = mapWithMotor(
                "flywheel", coefficients(1.0, 2.0, 3.0, 4.0, MotorControlAlgorithm.PIDF));
        MotorProbe probe = hardwareMap.motor("flywheel");
        Plant plant = deviceManagedVelocityPlant(
                hardwareMap,
                "flywheel",
                -0.0,
                4321.25,
                7.5);

        FtcMotorVelocityPidf handle = FtcMotorControllers.velocityPidf(plant);
        ScalarRange range = handle.plantTargetRange();

        assertSame(range, handle.plantTargetRange());
        assertTrue(range.valid);
        assertRawDoubleEquals(-0.0, range.minValue);
        assertRawDoubleEquals(4321.25, range.maxValue);
        assertEquals("bounded", range.reason);
        assertEquals(1, probe.pidfReadAttempts);
        probe.assertNoWritesOrActuation();
    }

    @Test
    public void factoryRejectsASecondConfigurationOwnerBeforeAnotherReadback() {
        TestHardwareMap hardwareMap = mapWithMotor(
                "flywheel", coefficients(1.0, 2.0, 3.0, 4.0, MotorControlAlgorithm.PIDF));
        MotorProbe probe = hardwareMap.motor("flywheel");
        Plant plant = deviceManagedVelocityPlant(hardwareMap, "flywheel");
        assertNotNull(FtcMotorControllers.velocityPidf(plant));

        IllegalStateException failure = assertThrows(
                IllegalStateException.class,
                () -> FtcMotorControllers.velocityPidf(plant));

        assertContains(failure, "already has a velocity PIDF configuration owner");
        assertEquals(1, probe.pidfReadAttempts);
        probe.assertNoWritesOrActuation();
    }

    @Test
    public void factoryCapturesExactInitialReadbackWithoutAnyWriteOrActuationEffect() {
        PIDFCoefficients initial = coefficients(
                MIN_COEFFICIENT, -0.0, +0.0, MAX_COEFFICIENT,
                MotorControlAlgorithm.LegacyPID);
        TestHardwareMap hardwareMap = mapWithMotor("flywheel", initial);
        MotorProbe probe = hardwareMap.motor("flywheel");
        Plant plant = deviceManagedVelocityPlant(hardwareMap, "flywheel");

        FtcMotorVelocityPidf handle =
                FtcMotorControllers.velocityPidf(plant);

        assertRawDoubleEquals(initial.p, handle.getKP());
        assertRawDoubleEquals(initial.i, handle.getKI());
        assertRawDoubleEquals(initial.d, handle.getKD());
        assertRawDoubleEquals(initial.f, handle.getKF());
        assertEquals(0, hardwareMap.lookupCalls);
        assertEquals(1, probe.pidfReadAttempts);
        probe.assertNoWritesOrActuation();
    }

    @Test
    public void setGainsValidatesTheCompleteTupleBeforeAnySdkEffectAndKeepsCachedReadback() {
        PIDFCoefficients initial = coefficients(
                1.0, 2.0, 3.0, 4.0, MotorControlAlgorithm.PIDF);
        TestHardwareMap hardwareMap = mapWithMotor("flywheel", initial);
        MotorProbe probe = hardwareMap.motor("flywheel");
        Plant plant = deviceManagedVelocityPlant(hardwareMap, "flywheel");
        FtcMotorVelocityPidf handle =
                FtcMotorControllers.velocityPidf(plant);

        double[] invalid = {
                Double.NaN,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                Math.nextDown(MIN_COEFFICIENT),
                Math.nextUp(MAX_COEFFICIENT)
        };
        for (int slot = 0; slot < 4; slot++) {
            for (double bad : invalid) {
                double[] candidate = {8.0, 9.0, 10.0, 11.0};
                candidate[slot] = bad;
                assertThrows(IllegalArgumentException.class,
                        () -> handle.setGains(
                                candidate[0], candidate[1], candidate[2], candidate[3]));
            }
        }

        assertEquals(0, probe.velocityPidfSetAttempts);
        assertEquals(1, probe.pidfReadAttempts);
        assertRawDoubleEquals(initial.p, handle.getKP());
        assertRawDoubleEquals(initial.i, handle.getKI());
        assertRawDoubleEquals(initial.d, handle.getKD());
        assertRawDoubleEquals(initial.f, handle.getKF());
        probe.assertNoWritesOrActuation();
    }

    @Test
    public void setGainsUsesOneSdkTupleCallThenCachesExactControllerReadback() {
        TestHardwareMap hardwareMap = mapWithMotor(
                "flywheel", coefficients(1.0, 2.0, 3.0, 4.0, MotorControlAlgorithm.PIDF));
        MotorProbe probe = hardwareMap.motor("flywheel");
        Plant plant = deviceManagedVelocityPlant(hardwareMap, "flywheel");
        FtcMotorVelocityPidf handle =
                FtcMotorControllers.velocityPidf(plant);
        probe.events.clear();
        probe.nextReadback = coefficients(
                10.000015258789062,
                -2.5000152587890625,
                -0.0,
                0.123443603515625,
                MotorControlAlgorithm.PIDF);

        handle.setGains(10.00002, -2.50002, +0.0, 0.12345);

        assertEquals(Arrays.asList("setVelocityPIDFCoefficients", "getPIDFCoefficients"),
                probe.events);
        assertEquals(1, probe.velocityPidfSetAttempts);
        assertEquals(1, probe.velocityPidfSetWrites);
        assertRawArrayEquals(
                new double[]{10.00002, -2.50002, +0.0, 0.12345},
                probe.lastVelocityPidf);
        assertRawDoubleEquals(10.000015258789062, handle.getKP());
        assertRawDoubleEquals(-2.5000152587890625, handle.getKI());
        assertRawDoubleEquals(-0.0, handle.getKD());
        assertRawDoubleEquals(0.123443603515625, handle.getKF());
        assertEquals(0, probe.generalPidfSetAttempts);
        probe.assertNoActuation();
    }

    @Test
    public void restoreUsesExactInitialTupleAndAlgorithmThenCachesItsReadback() {
        PIDFCoefficients initial = coefficients(
                7.0, -8.0, +0.0, -0.0, MotorControlAlgorithm.LegacyPID);
        TestHardwareMap hardwareMap = mapWithMotor("flywheel", initial);
        MotorProbe probe = hardwareMap.motor("flywheel");
        Plant plant = deviceManagedVelocityPlant(hardwareMap, "flywheel");
        FtcMotorVelocityPidf handle =
                FtcMotorControllers.velocityPidf(plant);
        handle.setGains(1.0, 2.0, 3.0, 4.0);
        probe.events.clear();

        handle.restoreInitial();

        assertEquals(Arrays.asList("setPIDFCoefficients", "getPIDFCoefficients"), probe.events);
        assertEquals(1, probe.generalPidfSetAttempts);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, probe.lastGeneralMode);
        assertCoefficientsRawEqual(initial, probe.lastGeneralPidf);
        assertEquals(MotorControlAlgorithm.LegacyPID, probe.lastGeneralPidf.algorithm);
        assertRawDoubleEquals(initial.p, handle.getKP());
        assertRawDoubleEquals(initial.i, handle.getKI());
        assertRawDoubleEquals(initial.d, handle.getKD());
        assertRawDoubleEquals(initial.f, handle.getKF());
        probe.assertNoActuation();

        // The implementation must pass a fresh copy rather than expose its retained snapshot.
        probe.lastGeneralPidf.p = 999.0;
        probe.events.clear();
        handle.restoreInitial();
        assertRawDoubleEquals(initial.p, probe.lastGeneralPidf.p);
        assertEquals(MotorControlAlgorithm.LegacyPID, probe.lastGeneralPidf.algorithm);
    }

    @Test
    public void factoryReadFailureNamesMotorAndDoesNotClaimAnUncertainWrite() {
        TestHardwareMap hardwareMap = mapWithMotor(
                "flywheel", coefficients(1.0, 2.0, 3.0, 4.0, MotorControlAlgorithm.PIDF));
        MotorProbe probe = hardwareMap.motor("flywheel");
        Plant plant = deviceManagedVelocityPlant(hardwareMap, "flywheel");
        UnsupportedOperationException sdkFailure =
                new UnsupportedOperationException("controller does not support PIDF readback");
        probe.pidfReadFailure = sdkFailure;

        IllegalStateException failure = assertThrows(
                IllegalStateException.class,
                () -> FtcMotorControllers.velocityPidf(plant));

        assertContains(failure, "flywheel");
        assertContains(failure, "No controller configuration was changed");
        assertSame(sdkFailure, failure.getCause());
        probe.assertNoWritesOrActuation();
    }

    @Test
    public void applyAndReadbackFailuresRetainPriorCacheAndReportUncertainControllerState() {
        PIDFCoefficients initial = coefficients(
                1.0, 2.0, 3.0, 4.0, MotorControlAlgorithm.PIDF);

        TestHardwareMap applyMap = mapWithMotor("applyMotor", initial);
        MotorProbe applyProbe = applyMap.motor("applyMotor");
        Plant applyPlant = deviceManagedVelocityPlant(applyMap, "applyMotor");
        FtcMotorVelocityPidf applyHandle =
                FtcMotorControllers.velocityPidf(applyPlant);
        RuntimeException applyCause = new RuntimeException("write transport failed");
        applyProbe.velocityPidfSetFailure = applyCause;

        IllegalStateException applyFailure = assertThrows(
                IllegalStateException.class,
                () -> applyHandle.setGains(5.0, 6.0, 7.0, 8.0));
        assertUncertainFailure(applyFailure, "applyMotor", applyCause);
        assertRawDoubleEquals(initial.p, applyHandle.getKP());
        assertRawDoubleEquals(initial.i, applyHandle.getKI());
        assertRawDoubleEquals(initial.d, applyHandle.getKD());
        assertRawDoubleEquals(initial.f, applyHandle.getKF());

        TestHardwareMap readMap = mapWithMotor("readMotor", initial);
        MotorProbe readProbe = readMap.motor("readMotor");
        Plant readPlant = deviceManagedVelocityPlant(readMap, "readMotor");
        FtcMotorVelocityPidf readHandle =
                FtcMotorControllers.velocityPidf(readPlant);
        RuntimeException readCause = new RuntimeException("read transport failed");
        readProbe.pidfReadFailure = readCause;

        IllegalStateException readFailure = assertThrows(
                IllegalStateException.class,
                () -> readHandle.setGains(5.0, 6.0, 7.0, 8.0));
        assertUncertainFailure(readFailure, "readMotor", readCause);
        assertEquals(1, readProbe.velocityPidfSetWrites);
        assertRawDoubleEquals(initial.p, readHandle.getKP());
        assertRawDoubleEquals(initial.i, readHandle.getKI());
        assertRawDoubleEquals(initial.d, readHandle.getKD());
        assertRawDoubleEquals(initial.f, readHandle.getKF());
        readProbe.assertNoActuation();
    }

    @Test
    public void unexpectedAppliedAlgorithmIsAnUncertainFailureAndDoesNotCommitTheCache() {
        PIDFCoefficients initial = coefficients(
                1.0, 2.0, 3.0, 4.0, MotorControlAlgorithm.PIDF);
        TestHardwareMap hardwareMap = mapWithMotor("flywheel", initial);
        MotorProbe probe = hardwareMap.motor("flywheel");
        Plant plant = deviceManagedVelocityPlant(hardwareMap, "flywheel");
        FtcMotorVelocityPidf handle =
                FtcMotorControllers.velocityPidf(plant);
        probe.nextReadback = coefficients(
                5.0, 6.0, 7.0, 0.0, MotorControlAlgorithm.LegacyPID);

        IllegalStateException failure = assertThrows(
                IllegalStateException.class,
                () -> handle.setGains(5.0, 6.0, 7.0, 8.0));

        assertContains(failure, "flywheel");
        assertContains(failure, "may be uncertain");
        assertContains(failure.getCause(), "instead of PIDF");
        assertRawDoubleEquals(initial.p, handle.getKP());
        assertRawDoubleEquals(initial.i, handle.getKI());
        assertRawDoubleEquals(initial.d, handle.getKD());
        assertRawDoubleEquals(initial.f, handle.getKF());
    }

    @Test
    public void restoreFailureRetainsPriorCacheAndReportsUncertainControllerState() {
        PIDFCoefficients initial = coefficients(
                1.0, 2.0, 3.0, 4.0, MotorControlAlgorithm.PIDF);
        TestHardwareMap hardwareMap = mapWithMotor("flywheel", initial);
        MotorProbe probe = hardwareMap.motor("flywheel");
        Plant plant = deviceManagedVelocityPlant(hardwareMap, "flywheel");
        FtcMotorVelocityPidf handle =
                FtcMotorControllers.velocityPidf(plant);
        handle.setGains(5.0, 6.0, 7.0, 8.0);
        RuntimeException restoreCause = new RuntimeException("restore transport failed");
        probe.generalPidfSetFailure = restoreCause;

        IllegalStateException failure = assertThrows(
                IllegalStateException.class,
                handle::restoreInitial);

        assertUncertainFailure(failure, "flywheel", restoreCause);
        assertRawDoubleEquals(5.0, handle.getKP());
        assertRawDoubleEquals(6.0, handle.getKI());
        assertRawDoubleEquals(7.0, handle.getKD());
        assertRawDoubleEquals(8.0, handle.getKF());
        probe.assertNoActuation();
    }

    @Test
    public void restoreReadbackMustMatchTheCapturedTupleAndAlgorithm() {
        PIDFCoefficients initial = coefficients(
                1.0, 2.0, 3.0, 0.0, MotorControlAlgorithm.LegacyPID);
        TestHardwareMap hardwareMap = mapWithMotor("flywheel", initial);
        MotorProbe probe = hardwareMap.motor("flywheel");
        Plant plant = deviceManagedVelocityPlant(hardwareMap, "flywheel");
        FtcMotorVelocityPidf handle =
                FtcMotorControllers.velocityPidf(plant);
        handle.setGains(5.0, 6.0, 7.0, 8.0);
        probe.generalSetReadbackOverride = coefficients(
                initial.p, initial.i, initial.d, initial.f, MotorControlAlgorithm.PIDF);

        IllegalStateException failure = assertThrows(
                IllegalStateException.class,
                handle::restoreInitial);

        assertContains(failure, "flywheel");
        assertContains(failure, "may be uncertain");
        assertContains(failure.getCause(), "did not match");
        assertRawDoubleEquals(5.0, handle.getKP());
        assertRawDoubleEquals(6.0, handle.getKI());
        assertRawDoubleEquals(7.0, handle.getKD());
        assertRawDoubleEquals(8.0, handle.getKF());
    }

    private static PIDFCoefficients coefficients(double p,
                                                 double i,
                                                 double d,
                                                 double f,
                                                 MotorControlAlgorithm algorithm) {
        return new PIDFCoefficients(p, i, d, f, algorithm);
    }

    private static int publicDeclaredMethodCount(Class<?> type) {
        int count = 0;
        for (Method method : type.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                count++;
            }
        }
        return count;
    }

    private static void assertPublicMethod(Class<?> owner,
                                           String name,
                                           Class<?> returnType,
                                           Class<?>... parameterTypes) throws Exception {
        Method method = owner.getDeclaredMethod(name, parameterTypes);
        assertTrue(Modifier.isPublic(method.getModifiers()));
        assertEquals(returnType, method.getReturnType());
    }

    private static TestHardwareMap mapWithMotor(String name, PIDFCoefficients initial) {
        TestHardwareMap hardwareMap = new TestHardwareMap();
        hardwareMap.addMotor(name, initial);
        return hardwareMap;
    }

    private static Plant deviceManagedVelocityPlant(TestHardwareMap hardwareMap,
                                                     String motorName) {
        return deviceManagedVelocityPlant(
                hardwareMap,
                motorName,
                -5000.0,
                5000.0,
                1.0);
    }

    private static Plant deviceManagedVelocityPlant(TestHardwareMap hardwareMap,
                                                     String motorName,
                                                     double minTarget,
                                                     double maxTarget,
                                                     double nativeUnitsPerPlantUnit) {
        Plant plant = FtcActuators.plant(hardwareMap)
                .motor(motorName, Direction.FORWARD)
                .velocity()
                .deviceManagedWithDefaults()
                .bounded(minTarget, maxTarget)
                .scaleToNative(nativeUnitsPerPlantUnit)
                .velocityTolerance(0.0)
                .targetFromNewCommand(0.0)
                .build();
        hardwareMap.resetObservations();
        return plant;
    }

    private static void assertIneligiblePlant(Plant plant, MotorProbe... probes) {
        IllegalArgumentException failure = assertThrows(
                IllegalArgumentException.class,
                () -> FtcMotorControllers.velocityPidf(plant));
        assertContains(failure, "single-motor FTC device-managed velocity Plant");
        for (MotorProbe probe : probes) {
            assertEquals(0, probe.pidfReadAttempts);
            probe.assertNoWritesOrActuation();
        }
    }

    private static void assertUncertainFailure(IllegalStateException failure,
                                               String motorName,
                                               RuntimeException cause) {
        assertContains(failure, motorName);
        assertContains(failure, "may be uncertain");
        assertSame(cause, failure.getCause());
    }

    private static void assertContains(Throwable failure, String expected) {
        assertNotNull(failure.getMessage());
        assertTrue("Expected <" + failure.getMessage() + "> to contain <" + expected + ">",
                failure.getMessage().contains(expected));
    }

    private static <T extends Throwable> T assertThrows(Class<T> type, Runnable action) {
        try {
            action.run();
            fail("Expected " + type.getSimpleName());
            return null;
        } catch (Throwable failure) {
            if (!type.isInstance(failure)) {
                AssertionError assertion = new AssertionError(
                        "Expected " + type.getSimpleName() + " but got " + failure, failure);
                throw assertion;
            }
            return type.cast(failure);
        }
    }

    private static void assertCoefficientsRawEqual(PIDFCoefficients expected,
                                                   PIDFCoefficients actual) {
        assertNotNull(actual);
        assertRawDoubleEquals(expected.p, actual.p);
        assertRawDoubleEquals(expected.i, actual.i);
        assertRawDoubleEquals(expected.d, actual.d);
        assertRawDoubleEquals(expected.f, actual.f);
        assertEquals(expected.algorithm, actual.algorithm);
    }

    private static void assertRawArrayEquals(double[] expected, double[] actual) {
        assertNotNull(actual);
        assertEquals(expected.length, actual.length);
        for (int i = 0; i < expected.length; i++) {
            assertRawDoubleEquals(expected[i], actual[i]);
        }
    }

    private static void assertRawDoubleEquals(double expected, double actual) {
        assertEquals(Double.doubleToRawLongBits(expected), Double.doubleToRawLongBits(actual));
    }

    /** In-memory HardwareMap with one observable dynamic DcMotorEx per configured name. */
    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, MotorProbe> motors = new HashMap<>();
        private int lookupCalls;

        private TestHardwareMap() {
            super(null, null);
        }

        private MotorProbe addMotor(String name, PIDFCoefficients initial) {
            MotorProbe probe = new MotorProbe(initial);
            motors.put(name, probe);
            return probe;
        }

        private MotorProbe motor(String name) {
            MotorProbe probe = motors.get(name);
            assertNotNull("No test motor named " + name, probe);
            return probe;
        }

        private void resetObservations() {
            lookupCalls = 0;
            for (MotorProbe probe : motors.values()) {
                probe.resetObservations();
            }
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            lookupCalls++;
            MotorProbe probe = motors.get(name);
            if (probe == null) {
                throw new IllegalArgumentException("No test motor named " + name);
            }
            if (!type.isInstance(probe.motor)) {
                throw new IllegalArgumentException("Wrong test device type for " + name);
            }
            return type.cast(probe.motor);
        }
    }

    /** Dynamic DcMotorEx probe that separates configuration calls from actuation calls. */
    private static final class MotorProbe {
        private final DcMotorEx motor;
        private final List<String> events = new ArrayList<>();
        private PIDFCoefficients controllerPidf;
        private PIDFCoefficients nextReadback;
        private PIDFCoefficients generalSetReadbackOverride;
        private PIDFCoefficients lastGeneralPidf;
        private DcMotor.RunMode lastGeneralMode;
        private double[] lastVelocityPidf;
        private int pidfReadAttempts;
        private int velocityPidfSetAttempts;
        private int velocityPidfSetWrites;
        private int generalPidfSetAttempts;
        private int directionWrites;
        private int modeWrites;
        private int powerWrites;
        private int targetWrites;
        private int velocityWrites;
        private RuntimeException pidfReadFailure;
        private RuntimeException velocityPidfSetFailure;
        private RuntimeException generalPidfSetFailure;

        private MotorProbe(PIDFCoefficients initial) {
            controllerPidf = new PIDFCoefficients(initial);
            motor = (DcMotorEx) Proxy.newProxyInstance(
                    DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class},
                    this::invoke);
        }

        private void resetObservations() {
            events.clear();
            lastGeneralPidf = null;
            lastGeneralMode = null;
            lastVelocityPidf = null;
            pidfReadAttempts = 0;
            velocityPidfSetAttempts = 0;
            velocityPidfSetWrites = 0;
            generalPidfSetAttempts = 0;
            directionWrites = 0;
            modeWrites = 0;
            powerWrites = 0;
            targetWrites = 0;
            velocityWrites = 0;
        }

        private Object invoke(Object proxy, Method method, Object[] args) {
            String name = method.getName();
            if (method.getDeclaringClass() == Object.class) {
                return objectMethod(proxy, name, args, "MotorProbe");
            }
            if ("getPIDFCoefficients".equals(name)) {
                events.add(name);
                pidfReadAttempts++;
                if (pidfReadFailure != null) throw pidfReadFailure;
                return new PIDFCoefficients(controllerPidf);
            }
            if ("setVelocityPIDFCoefficients".equals(name)) {
                events.add(name);
                velocityPidfSetAttempts++;
                if (velocityPidfSetFailure != null) throw velocityPidfSetFailure;
                lastVelocityPidf = new double[]{
                        (double) args[0], (double) args[1],
                        (double) args[2], (double) args[3]
                };
                velocityPidfSetWrites++;
                controllerPidf = nextReadback == null
                        ? coefficients(
                                lastVelocityPidf[0],
                                lastVelocityPidf[1],
                                lastVelocityPidf[2],
                                lastVelocityPidf[3],
                                MotorControlAlgorithm.PIDF)
                        : new PIDFCoefficients(nextReadback);
                nextReadback = null;
                return null;
            }
            if ("setPIDFCoefficients".equals(name)) {
                events.add(name);
                generalPidfSetAttempts++;
                if (generalPidfSetFailure != null) throw generalPidfSetFailure;
                lastGeneralMode = (DcMotor.RunMode) args[0];
                lastGeneralPidf = new PIDFCoefficients((PIDFCoefficients) args[1]);
                controllerPidf = generalSetReadbackOverride == null
                        ? new PIDFCoefficients(lastGeneralPidf)
                        : new PIDFCoefficients(generalSetReadbackOverride);
                generalSetReadbackOverride = null;
                return null;
            }
            if ("setDirection".equals(name)) {
                directionWrites++;
                return null;
            }
            if ("getDirection".equals(name)) return DcMotorSimple.Direction.FORWARD;
            if ("setMode".equals(name)) {
                modeWrites++;
                return null;
            }
            if ("getMode".equals(name)) return DcMotor.RunMode.RUN_WITHOUT_ENCODER;
            if ("setPower".equals(name)) {
                powerWrites++;
                return null;
            }
            if ("getPower".equals(name)) return 0.0;
            if ("setTargetPosition".equals(name)) {
                targetWrites++;
                return null;
            }
            if ("getTargetPosition".equals(name)) return 0;
            if ("setVelocity".equals(name)) {
                velocityWrites++;
                return null;
            }
            if ("getVelocity".equals(name)) return 0.0;
            if ("getZeroPowerBehavior".equals(name)) return DcMotor.ZeroPowerBehavior.FLOAT;
            if ("isBusy".equals(name)) return false;
            return hardwareDeviceOrDefault(method, "MotorProbe");
        }

        private void assertNoWritesOrActuation() {
            assertEquals(0, velocityPidfSetAttempts);
            assertEquals(0, generalPidfSetAttempts);
            assertNoActuation();
        }

        private void assertNoActuation() {
            assertEquals(0, directionWrites);
            assertEquals(0, modeWrites);
            assertEquals(0, powerWrites);
            assertEquals(0, targetWrites);
            assertEquals(0, velocityWrites);
        }
    }

    private static Object objectMethod(Object proxy, String name, Object[] args, String label) {
        if ("equals".equals(name)) return proxy == args[0];
        if ("hashCode".equals(name)) return System.identityHashCode(proxy);
        if ("toString".equals(name)) return label;
        return null;
    }

    private static Object hardwareDeviceOrDefault(Method method, String label) {
        String name = method.getName();
        if ("getManufacturer".equals(name)) return HardwareDevice.Manufacturer.Other;
        if ("getDeviceName".equals(name)) return label;
        if ("getConnectionInfo".equals(name)) return "test";
        if ("getVersion".equals(name)) return 1;
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
