package edu.ftcphoenix.robots.examples.pedro.capability.intake;

import edu.ftcphoenix.fw.actuation.Plant;

/** Cross-package test support for the package-private hardware-neutral mechanism seam. */
public final class BasicPedroMechanismTestFactory {
    private BasicPedroMechanismTestFactory() {
    }

    /** Creates a mechanism through its hardware-neutral package seam. */
    public static BasicPedroAutoMechanism fromPlant(Plant plant) {
        return new BasicPedroAutoMechanism(plant);
    }
}
