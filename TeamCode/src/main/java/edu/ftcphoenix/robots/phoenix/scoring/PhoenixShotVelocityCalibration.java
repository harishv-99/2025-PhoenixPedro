package edu.ftcphoenix.robots.phoenix.scoring;

import edu.ftcphoenix.fw.core.math.InterpolatingTable1D;

/** Owns Phoenix's reviewed range-to-flywheel-velocity calibration evidence. */
final class PhoenixShotVelocityCalibration {

    private static final InterpolatingTable1D CURRENT = InterpolatingTable1D.ofSortedPairs(
            28.06, 1505.6,
            36.52, 1427.4,
            42.3, 1424.35,
            50.3, 1450.0,
            56.5, 1461.0,
            62.9, 1538.0,
            65.8, 1535.7,
            70.0, 1530.0,
            74.2, 1575.0,
            79.5, 1600.0,
            83.4, 1625.0,
            93.6, 1700.0,
            96.6, 1700.0,
            103.2, 1775.0,
            104.7, 1800.0,
            109.2, 1800.0,
            112.0, 1818.0,
            115.0, 1825.0,
            120.0, 1850.0,
            130.0, 1875.0
    );

    private PhoenixShotVelocityCalibration() {
        // Configuration evidence holder.
    }

    static InterpolatingTable1D currentTable() {
        return CURRENT;
    }
}
