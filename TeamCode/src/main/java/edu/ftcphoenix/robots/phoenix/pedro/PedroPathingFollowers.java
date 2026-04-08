package edu.ftcphoenix.robots.phoenix.pedro;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Objects;

/**
 * Reflection helpers for creating Pedro {@link Follower} instances without hard-wiring one
 * team-specific constants class import into Phoenix.
 *
 * <p>The current Pedro docs create followers via a static {@code Constants.createFollower(hardwareMap)}
 * helper, but different projects place that class in different packages. This helper lets Phoenix
 * sample OpModes point at a factory class name string instead of forcing a compile-time dependency
 * on one specific TeamCode package layout.</p>
 */
public final class PedroPathingFollowers {

    private PedroPathingFollowers() {
        // utility class
    }

    /**
     * Creates a Pedro follower from one explicit factory-class name.
     *
     * @param hardwareMap      FTC hardware map to pass through to Pedro
     * @param factoryClassName fully-qualified class name that exposes
     *                         {@code static Follower createFollower(HardwareMap)}
     * @return created follower
     */
    public static Follower createFollower(HardwareMap hardwareMap,
                                          String factoryClassName) {
        Objects.requireNonNull(factoryClassName, "factoryClassName");
        return createFollower(hardwareMap, new String[]{factoryClassName});
    }

    /**
     * Creates a Pedro follower by trying a list of candidate factory classes in order.
     *
     * <p>If none of the candidates succeed, this helper falls back to a reflective
     * {@code new Follower(hardwareMap)} construction attempt for older Pedro setups that still use
     * the direct constructor.</p>
     *
     * @param hardwareMap       FTC hardware map to pass through to Pedro
     * @param factoryClassNames candidate fully-qualified factory class names to try in order
     * @return created follower
     */
    public static Follower createFollower(HardwareMap hardwareMap,
                                          String... factoryClassNames) {
        Objects.requireNonNull(hardwareMap, "hardwareMap");
        Objects.requireNonNull(factoryClassNames, "factoryClassNames");

        RuntimeException lastError = null;
        for (String className : factoryClassNames) {
            if (className == null || className.trim().isEmpty()) {
                continue;
            }
            try {
                return createViaFactoryClass(hardwareMap, className.trim());
            } catch (RuntimeException e) {
                lastError = e;
            }
        }

        try {
            return createViaFollowerConstructor(hardwareMap);
        } catch (RuntimeException e) {
            if (lastError != null) {
                e.addSuppressed(lastError);
            }
            throw e;
        }
    }

    private static Follower createViaFactoryClass(HardwareMap hardwareMap,
                                                  String factoryClassName) {
        try {
            Class<?> cls = Class.forName(factoryClassName);
            Method factory = cls.getMethod("createFollower", HardwareMap.class);
            Object created = factory.invoke(null, hardwareMap);
            if (!(created instanceof Follower)) {
                throw new IllegalStateException(
                        "Pedro factory returned "
                                + (created != null ? created.getClass().getName() : "null")
                                + " instead of com.pedropathing.follower.Follower"
                );
            }
            return (Follower) created;
        } catch (ClassNotFoundException e) {
            throw new IllegalStateException("Pedro factory class not found: " + factoryClassName, e);
        } catch (NoSuchMethodException e) {
            throw new IllegalStateException(
                    "Pedro factory class does not expose createFollower(HardwareMap): " + factoryClassName,
                    e
            );
        } catch (IllegalAccessException | InvocationTargetException e) {
            throw new IllegalStateException("Unable to create Pedro follower via " + factoryClassName, e);
        }
    }

    private static Follower createViaFollowerConstructor(HardwareMap hardwareMap) {
        try {
            Constructor<Follower> ctor = Follower.class.getConstructor(HardwareMap.class);
            return ctor.newInstance(hardwareMap);
        } catch (NoSuchMethodException e) {
            throw new IllegalStateException(
                    "Unable to create Pedro follower. Configure a Constants.createFollower(HardwareMap) factory class "
                            + "for your project or update PedroPathingFollowers with your follower construction method.",
                    e
            );
        } catch (InstantiationException | IllegalAccessException | InvocationTargetException e) {
            throw new IllegalStateException("Unable to instantiate Pedro follower via constructor", e);
        }
    }
}
