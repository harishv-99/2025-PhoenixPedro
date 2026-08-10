package edu.ftcphoenix.robots.phoenix;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.source.Source;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/** Fault-injection coverage for ScoringPath's private Plant construction and stop ownership. */
public final class ScoringPathCleanupTest {

    @Test
    public void eachLaterPlantConstructionFailureStopsEveryCompletedPlantInStableOrder() {
        PhoenixProfile profile = PhoenixProfile.defaults();
        PhoenixProfile.ScoringPathConfig config = profile.scoring.copy();

        assertConstructionRollback(
                profile,
                config,
                config.nameCrServoIntakeTransfer,
                Arrays.asList("intake.power")
        );
        assertConstructionRollback(
                profile,
                config,
                config.nameCrServoShooterTransferRight,
                Arrays.asList("intake.power", "intakeTransfer.power")
        );
        assertConstructionRollback(
                profile,
                config,
                config.nameMotorShooterWheel,
                Arrays.asList(
                        "intake.power",
                        "intakeTransfer.power",
                        "shooterTransferRight.power",
                        "shooterTransferLeft.power"
                )
        );
    }

    @Test
    public void constructionRollbackRetainsPrimaryAndSuppressesCleanupFailuresInOrder() {
        PhoenixProfile profile = PhoenixProfile.defaults();
        PhoenixProfile.ScoringPathConfig config = profile.scoring.copy();
        TestHardwareMap hardwareMap = TestHardwareMap.forScoring(config);
        hardwareMap.remove(config.nameMotorShooterWheel);

        RuntimeException intakeFailure = new RuntimeException("intake rollback failure");
        RuntimeException intakeTransferFailure =
                new RuntimeException("intake-transfer rollback failure");
        RuntimeException shooterTransferFailure =
                new RuntimeException("shooter-transfer rollback failure");
        hardwareMap.state(config.nameMotorIntake).zeroPowerFailure = intakeFailure;
        hardwareMap.state(config.nameCrServoIntakeTransfer).zeroPowerFailure =
                intakeTransferFailure;
        hardwareMap.state(config.nameCrServoShooterTransferRight).zeroPowerFailure =
                shooterTransferFailure;

        try {
            new ScoringPath(hardwareMap, config, targetingFor(profile));
            fail("expected missing flywheel construction to fail");
        } catch (RuntimeException primaryFailure) {
            assertTrue(primaryFailure.getMessage().contains(config.nameMotorShooterWheel));
            assertEquals(3, primaryFailure.getSuppressed().length);
            assertSame(intakeFailure, primaryFailure.getSuppressed()[0]);
            assertSame(intakeTransferFailure, primaryFailure.getSuppressed()[1]);
            assertSame(shooterTransferFailure, primaryFailure.getSuppressed()[2]);
        }

        assertEquals(
                Arrays.asList(
                        "intake.power",
                        "intakeTransfer.power",
                        "shooterTransferRight.power",
                        "shooterTransferLeft.power"
                ),
                hardwareMap.stopEvents
        );
    }

    @Test
    public void stopIsTerminalIdempotentAndAttemptsEveryPlantAfterFailures() {
        PhoenixProfile profile = PhoenixProfile.defaults();
        PhoenixProfile.ScoringPathConfig config = profile.scoring.copy();
        TestHardwareMap hardwareMap = TestHardwareMap.forScoring(config);
        ScoringPath scoring = new ScoringPath(hardwareMap, config, targetingFor(profile));

        RuntimeException flywheelFailure = new RuntimeException("flywheel stop failure");
        RuntimeException intakeFailure = new RuntimeException("intake stop failure");
        RuntimeException intakeTransferFailure = new RuntimeException("intake-transfer stop failure");
        RuntimeException shooterTransferFailure = new RuntimeException("shooter-transfer stop failure");
        hardwareMap.state(config.nameMotorShooterWheel).zeroVelocityFailure = flywheelFailure;
        hardwareMap.state(config.nameMotorIntake).zeroPowerFailure = intakeFailure;
        hardwareMap.state(config.nameCrServoIntakeTransfer).zeroPowerFailure =
                intakeTransferFailure;
        hardwareMap.state(config.nameCrServoShooterTransferRight).zeroPowerFailure =
                shooterTransferFailure;

        try {
            scoring.stop();
            fail("expected injected stop failures");
        } catch (RuntimeException actual) {
            assertSame(flywheelFailure, actual);
            assertEquals(3, actual.getSuppressed().length);
            assertSame(intakeFailure, actual.getSuppressed()[0]);
            assertSame(intakeTransferFailure, actual.getSuppressed()[1]);
            assertSame(shooterTransferFailure, actual.getSuppressed()[2]);
        }

        assertEquals(
                Arrays.asList(
                        "flywheel.velocity",
                        "intake.power",
                        "intakeTransfer.power",
                        "shooterTransferRight.power",
                        "shooterTransferLeft.power"
                ),
                hardwareMap.stopEvents
        );
        assertFalse(scoring.status().flywheelEnabled);
        assertFalse(scoring.status().flywheelRequested);

        List<String> firstStopEvents = new ArrayList<String>(hardwareMap.stopEvents);
        scoring.stop();
        assertEquals(firstStopEvents, hardwareMap.stopEvents);

        LoopClock clock = new LoopClock();
        clock.reset(0.0);
        try {
            scoring.update(clock);
            fail("expected update after terminal stop to fail");
        } catch (IllegalStateException expected) {
            assertTrue(expected.getMessage().contains("after stop"));
        }
    }

    private static void assertConstructionRollback(PhoenixProfile profile,
                                                   PhoenixProfile.ScoringPathConfig config,
                                                   String missingName,
                                                   List<String> expectedStopEvents) {
        TestHardwareMap hardwareMap = TestHardwareMap.forScoring(config);
        hardwareMap.remove(missingName);

        try {
            new ScoringPath(hardwareMap, config, targetingFor(profile));
            fail("expected missing device construction failure for " + missingName);
        } catch (RuntimeException expected) {
            assertTrue(expected.getMessage().contains(missingName));
            assertEquals(0, expected.getSuppressed().length);
        }

        assertEquals(expectedStopEvents, hardwareMap.stopEvents);
    }

    private static ScoringTargeting targetingFor(PhoenixProfile profile) {
        return new ScoringTargeting(
                profile.autoAim,
                profile.localization.aprilTags.fieldPoseSolver.copy(),
                new EmptyAprilTagSensor(),
                CameraMountConfig.identity(),
                new NoPoseEstimator(),
                profile.field.fixedAprilTagLayout,
                Source.constant(profile.autoAim.scoringTagIds()),
                BooleanSource.constant(true),
                BooleanSource.constant(false),
                profile.autoAim.shotVelocityTable
        );
    }

    private static final class EmptyAprilTagSensor implements AprilTagSensor {
        @Override
        public AprilTagDetections get(LoopClock clock) {
            return AprilTagDetections.none();
        }
    }

    private static final class NoPoseEstimator implements AbsolutePoseEstimator {
        private final PoseEstimate estimate = PoseEstimate.noPose(LoopTimestamp.unavailable());

        @Override
        public void update(LoopClock clock) {
            // No localization state is needed for cleanup tests.
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }
    }

    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> devices = new HashMap<String, HardwareDevice>();
        private final Map<String, DeviceState> states = new HashMap<String, DeviceState>();
        private final List<String> stopEvents = new ArrayList<String>();

        private TestHardwareMap() {
            super(null, null);
        }

        static TestHardwareMap forScoring(PhoenixProfile.ScoringPathConfig config) {
            TestHardwareMap map = new TestHardwareMap();
            map.add(config.nameMotorIntake, DcMotorEx.class, "intake");
            map.add(config.nameMotorShooterWheel, DcMotorEx.class, "flywheel");
            map.add(config.nameCrServoIntakeTransfer, CRServo.class, "intakeTransfer");
            map.add(
                    config.nameCrServoShooterTransferRight,
                    CRServo.class,
                    "shooterTransferRight"
            );
            map.add(
                    config.nameCrServoShooterTransferLeft,
                    CRServo.class,
                    "shooterTransferLeft"
            );
            return map;
        }

        private <T extends HardwareDevice> void add(String name, Class<T> type, String label) {
            DeviceState state = new DeviceState(label, stopEvents);
            states.put(name, state);
            devices.put(name, deviceProxy(type, state));
        }

        DeviceState state(String name) {
            DeviceState state = states.get(name);
            if (state == null) {
                throw new AssertionError("No test state for " + name);
            }
            return state;
        }

        void remove(String name) {
            devices.remove(name);
        }

        @Override
        public <T> T get(Class<? extends T> type, String name) {
            HardwareDevice device = devices.get(name);
            if (device == null || !type.isInstance(device)) {
                throw new IllegalArgumentException("No test " + type.getSimpleName() + " named " + name);
            }
            return type.cast(device);
        }
    }

    private static final class DeviceState {
        final String label;
        final List<String> stopEvents;
        DcMotor.RunMode motorMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        RuntimeException zeroPowerFailure;
        RuntimeException zeroVelocityFailure;

        DeviceState(String label, List<String> stopEvents) {
            this.label = label;
            this.stopEvents = stopEvents;
        }
    }

    private static <T extends HardwareDevice> T deviceProxy(Class<T> type, DeviceState state) {
        InvocationHandler handler = new InvocationHandler() {
            @Override
            public Object invoke(Object proxy, Method method, Object[] args) {
                String name = method.getName();
                if (method.getDeclaringClass() == Object.class) {
                    if ("equals".equals(name)) {
                        return proxy == args[0];
                    }
                    if ("hashCode".equals(name)) {
                        return System.identityHashCode(proxy);
                    }
                    if ("toString".equals(name)) {
                        return "TestDevice(" + state.label + ")";
                    }
                }
                if ("getManufacturer".equals(name)) {
                    return HardwareDevice.Manufacturer.Other;
                }
                if ("getDeviceName".equals(name)) {
                    return state.label;
                }
                if ("getConnectionInfo".equals(name)) {
                    return "test";
                }
                if ("getVersion".equals(name)) {
                    return 1;
                }
                if ("setMode".equals(name)) {
                    state.motorMode = (DcMotor.RunMode) args[0];
                    return null;
                }
                if ("getMode".equals(name)) {
                    return state.motorMode;
                }
                if ("setDirection".equals(name)) {
                    state.direction = (DcMotorSimple.Direction) args[0];
                    return null;
                }
                if ("getDirection".equals(name)) {
                    return state.direction;
                }
                if ("setPower".equals(name)) {
                    double power = (Double) args[0];
                    if (power == 0.0) {
                        state.stopEvents.add(state.label + ".power");
                        if (state.zeroPowerFailure != null) {
                            throw state.zeroPowerFailure;
                        }
                    }
                    return null;
                }
                if ("getPower".equals(name)) {
                    return 0.0;
                }
                if ("setVelocity".equals(name)) {
                    double velocity = (Double) args[0];
                    if (velocity == 0.0) {
                        state.stopEvents.add(state.label + ".velocity");
                        if (state.zeroVelocityFailure != null) {
                            throw state.zeroVelocityFailure;
                        }
                    }
                    return null;
                }
                if ("getVelocity".equals(name)) {
                    return 0.0;
                }
                return defaultValue(method.getReturnType());
            }
        };
        return type.cast(Proxy.newProxyInstance(
                type.getClassLoader(),
                new Class<?>[]{type},
                handler
        ));
    }

    private static Object defaultValue(Class<?> type) {
        if (!type.isPrimitive()) {
            return null;
        }
        if (type == boolean.class) {
            return false;
        }
        if (type == byte.class) {
            return (byte) 0;
        }
        if (type == short.class) {
            return (short) 0;
        }
        if (type == int.class) {
            return 0;
        }
        if (type == long.class) {
            return 0L;
        }
        if (type == float.class) {
            return 0.0f;
        }
        if (type == double.class) {
            return 0.0;
        }
        if (type == char.class) {
            return '\0';
        }
        return null;
    }
}
