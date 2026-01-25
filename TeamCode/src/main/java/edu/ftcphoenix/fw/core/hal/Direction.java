package edu.ftcphoenix.fw.core.hal;

/**
 * Logical direction for an actuator channel.
 *
 * <p>This is a HAL-level concept used by adapter layers (for example, FTC hardware adapters)
 * to normalize the meaning of positive commands across motors and servos.
 *
 * <p>{@link #FORWARD} means “positive commands move forward” in the mechanism’s chosen
 * coordinate system. {@link #REVERSE} flips the underlying hardware direction so that
 * the rest of the framework can still treat positive values as forward.
 */
public enum Direction {
    FORWARD,
    REVERSE
}
