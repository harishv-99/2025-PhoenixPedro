package edu.ftcsushi.robots.phoenix.scoring;

import edu.ftcsushi.fw.core.math.InterpolatingTable1D;

/** Owns Phoenix's reviewed range-to-flywheel-velocity calibration evidence. */
final class PhoenixShotVelocityCalibration {

    private static final InterpolatingTable1D CURRENT = InterpolatingTable1D.ofSorted(
            new double[]{
                    28.06, 36.52, 42.3, 50.3, 56.5,
                    62.9, 65.8, 70.0, 74.2, 79.5,
                    83.4, 93.6, 96.6, 103.2, 104.7,
                    109.2, 112.0, 115.0, 120.0, 130.0
            },
            new double[]{
                    1505.6, 1427.4, 1424.35, 1450.0, 1461.0,
                    1538.0, 1535.7, 1530.0, 1575.0, 1600.0,
                    1625.0, 1700.0, 1700.0, 1775.0, 1800.0,
                    1800.0, 1818.0, 1825.0, 1850.0, 1875.0
            }
    );

    private PhoenixShotVelocityCalibration() {
        // Configuration evidence holder.
    }

    static InterpolatingTable1D currentTable() {
        return CURRENT;
    }
}
