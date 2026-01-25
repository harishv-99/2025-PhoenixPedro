package edu.ftcphoenix.fw.spatial;

/**
 * Robot footprint model used for zone checks.
 *
 * <p>The footprint is expressed in the <b>robot frame</b> where +X is forward and +Y is left.
 * This is intentionally a small, simple abstraction: it exists to support common
 * geometric questions like "overlaps zone" and "fully inside zone".</p>
 */
public interface RobotFootprint2d {
    // Marker.
}
