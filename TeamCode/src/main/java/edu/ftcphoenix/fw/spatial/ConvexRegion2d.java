package edu.ftcphoenix.fw.spatial;

/**
 * Marker interface for <b>convex</b> 2D regions.
 *
 * <p>Phoenix's zone/safety APIs intentionally bias toward convex geometry because it enables
 * simple, reliable predicates such as:</p>
 * <ul>
 *   <li><b>"robot footprint fully inside"</b>: for convex regions, checking all robot footprint
 *       vertices are inside implies the whole footprint is inside.</li>
 *   <li><b>signed-distance hysteresis</b>: convex regions have a well-behaved boundary for
 *       stable "enter/exit" latching.</li>
 * </ul>
 *
 * <p>This is a marker on top of {@link Region2d}. Implementations should follow the same
 * signed-distance convention (positive inside, negative outside).</p>
 */
public interface ConvexRegion2d extends Region2d {
    // Marker.
}
