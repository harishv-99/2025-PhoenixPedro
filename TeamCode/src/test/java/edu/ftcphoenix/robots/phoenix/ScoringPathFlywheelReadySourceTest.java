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
import java.util.HashMap;
import java.util.Map;

import edu.ftcphoenix.fw.core.source.BooleanSource;
import edu.ftcphoenix.fw.core.time.LoopClock;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;
import edu.ftcphoenix.fw.localization.AbsolutePoseEstimator;
import edu.ftcphoenix.fw.localization.PoseEstimate;
import edu.ftcphoenix.fw.sensing.vision.CameraMountConfig;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagDetections;
import edu.ftcphoenix.fw.sensing.vision.apriltag.AprilTagSensor;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Robot-level regression coverage for the source-backed flywheel-ready gate. */
public final class ScoringPathFlywheelReadySourceTest {

    @Test
    public void readinessIsDisabledDebouncedCycleStableAndRestartedAfterStop() {
        PhoenixProfile profile = PhoenixProfile.defaults();
        PhoenixProfile.ScoringPathConfig config = profile.scoring.copy();
        config.readyPredictLeadSec = 0.0;
        config.readyStableSec = 0.05;

        TestHardwareMap hardwareMap = TestHardwareMap.forScoring(config);
        hardwareMap.setShooterVelocity(config.velocityMin);
        LoopClock clock = new LoopClock();
        ScoringPath scoring = new ScoringPath(
                hardwareMap,
                config,
                targetingFor(profile),
                clock
        );
        BooleanSource ready = scoring.flywheelReady();

        clock.reset(0.0);
        scoring.update(clock);
        assertFalse("matching velocity must not report ready while disabled", ready.getAsBoolean(clock));

        scoring.setFlywheelEnabled(true);
        clock.update(0.02);
        scoring.update(clock);
        assertFalse("readiness must wait for the configured stable interval", ready.getAsBoolean(clock));
        assertFalse("a repeated same-cycle read must not advance debounce time", ready.getAsBoolean(clock));

        clock.update(0.04);
        scoring.update(clock);
        assertFalse("readiness must remain false before the full delay", ready.getAsBoolean(clock));

        clock.update(0.06);
        scoring.update(clock);
        assertTrue("continuous readiness beyond the delay must become ready", ready.getAsBoolean(clock));
        assertTrue("a repeated completed same-cycle read must stay ready", ready.getAsBoolean(clock));

        scoring.setFlywheelEnabled(false);
        scoring.update(clock);
        assertFalse("the repeated update applies the disabled request", scoring.status().flywheelEnabled);
        assertFalse("disabling explicitly resets readiness in the same cycle", ready.getAsBoolean(clock));

        scoring.setFlywheelEnabled(true);
        clock.update(0.07);
        scoring.update(clock);
        assertFalse("restart must earn the readiness delay again", ready.getAsBoolean(clock));

        clock.update(0.13);
        scoring.update(clock);
        assertTrue("restart may become ready after a new stable interval", ready.getAsBoolean(clock));

        scoring.stop();
        assertFalse("stop must reset a same-cycle ready observation", ready.getAsBoolean(clock));

        scoring.setFlywheelEnabled(true);
        clock.update(0.14);
        scoring.update(clock);
        assertFalse("restart after stop must earn the readiness delay again", ready.getAsBoolean(clock));

        clock.update(0.20);
        scoring.update(clock);
        assertTrue("restart after stop may become ready after the delay", ready.getAsBoolean(clock));
    }

    private static ScoringTargeting targetingFor(PhoenixProfile profile) {
        return new ScoringTargeting(
                profile.autoAim,
                profile.localization.aprilTags.fieldPoseSolver.copy(),
                new EmptyAprilTagSensor(),
                CameraMountConfig.identity(),
                new NoPoseEstimator(),
                profile.field.fixedAprilTagLayout,
                BooleanSource.constant(true),
                BooleanSource.constant(false),
                profile.autoAim.shotVelocityTable
        );
    }

    /** Named test boundary adapter; no camera resource is needed for flywheel readiness. */
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
            // No localization state is needed for flywheel readiness.
        }

        @Override
        public PoseEstimate getEstimate() {
            return estimate;
        }
    }

    /** In-memory FTC boundary containing only the scoring devices constructed by ScoringPath. */
    private static final class TestHardwareMap extends HardwareMap {
        private final Map<String, HardwareDevice> devices = new HashMap<String, HardwareDevice>();
        private DeviceState shooterState;

        private TestHardwareMap() {
            super(null, null);
        }

        static TestHardwareMap forScoring(PhoenixProfile.ScoringPathConfig config) {
            TestHardwareMap map = new TestHardwareMap();
            map.devices.put(config.nameMotorIntake, deviceProxy(DcMotorEx.class, "intake", new DeviceState()));
            map.shooterState = new DeviceState();
            map.devices.put(
                    config.nameMotorShooterWheel,
                    deviceProxy(DcMotorEx.class, "shooter", map.shooterState)
            );
            map.devices.put(
                    config.nameCrServoIntakeTransfer,
                    deviceProxy(CRServo.class, "intakeTransfer", new DeviceState())
            );
            map.devices.put(
                    config.nameCrServoShooterTransferLeft,
                    deviceProxy(CRServo.class, "shooterTransferLeft", new DeviceState())
            );
            map.devices.put(
                    config.nameCrServoShooterTransferRight,
                    deviceProxy(CRServo.class, "shooterTransferRight", new DeviceState())
            );
            return map;
        }

        void setShooterVelocity(double velocity) {
            shooterState.measuredVelocity = velocity;
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
        DcMotor.RunMode motorMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        double power;
        double measuredVelocity;
    }

    private static <T extends HardwareDevice> T deviceProxy(Class<T> type,
                                                             String label,
                                                             DeviceState state) {
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
                        return "TestDevice(" + label + ")";
                    }
                }
                if ("getManufacturer".equals(name)) {
                    return HardwareDevice.Manufacturer.Other;
                }
                if ("getDeviceName".equals(name)) {
                    return label;
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
                    state.power = (Double) args[0];
                    return null;
                }
                if ("getPower".equals(name)) {
                    return state.power;
                }
                if ("getVelocity".equals(name)) {
                    return state.measuredVelocity;
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
