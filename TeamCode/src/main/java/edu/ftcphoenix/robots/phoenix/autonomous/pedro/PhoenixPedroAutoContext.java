package edu.ftcphoenix.robots.phoenix.autonomous.pedro;

import java.util.Objects;

import edu.ftcphoenix.fw.integrations.pedro.PedroPathingDriveAdapter;
import edu.ftcphoenix.robots.phoenix.PhoenixAutoConfig;
import edu.ftcphoenix.robots.phoenix.PhoenixCapabilities;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoSpec;

/**
 * Immutable context passed to Pedro autonomous routine builders.
 *
 * <p>The context keeps routine factories from reaching back into an OpMode or raw robot internals.
 * It exposes only the selected Auto spec, a defensive Auto-policy snapshot, capability families,
 * the Pedro drive adapter, the robot-owned path factory, and the fixed path set created for the
 * selected spec. The retained path factory lets routines request live-start geometry without
 * reaching into the OpMode or raw Follower.</p>
 */
public final class PhoenixPedroAutoContext {

    private final PhoenixAutoSpec spec;
    private final PhoenixAutoConfig autoConfig;
    private final PhoenixCapabilities capabilities;
    private final PedroPathingDriveAdapter driveAdapter;
    private final PhoenixPedroPathFactory pathFactory;
    private final PhoenixPedroPathFactory.Paths paths;

    /**
     * Create an autonomous context.
     */
    public PhoenixPedroAutoContext(PhoenixAutoSpec spec,
                                   PhoenixAutoConfig autoConfig,
                                   PhoenixCapabilities capabilities,
                                   PedroPathingDriveAdapter driveAdapter,
                                   PhoenixPedroPathFactory pathFactory,
                                   PhoenixPedroPathFactory.Paths paths) {
        this.spec = Objects.requireNonNull(spec, "spec");
        this.autoConfig = Objects.requireNonNull(autoConfig, "autoConfig").copy();
        this.capabilities = Objects.requireNonNull(capabilities, "capabilities");
        this.driveAdapter = Objects.requireNonNull(driveAdapter, "driveAdapter");
        this.pathFactory = Objects.requireNonNull(pathFactory, "pathFactory");
        this.paths = Objects.requireNonNull(paths, "paths");
    }

    /**
     * Selected autonomous setup.
     */
    public PhoenixAutoSpec spec() {
        return spec;
    }

    /**
     * Return a defensive copy of the Auto policy captured for this routine context.
     */
    public PhoenixAutoConfig autoConfig() {
        return autoConfig.copy();
    }

    /**
     * Shared Phoenix capability families.
     */
    public PhoenixCapabilities capabilities() {
        return capabilities;
    }

    /**
     * Pedro route/drive adapter.
     */
    public PedroPathingDriveAdapter driveAdapter() {
        return driveAdapter;
    }

    /**
     * Robot-owned Pedro path factory used for start-time geometry construction.
     */
    public PhoenixPedroPathFactory pathFactory() {
        return pathFactory;
    }

    /**
     * Fixed Pedro path set and start pose for the selected spec.
     */
    public PhoenixPedroPathFactory.Paths paths() {
        return paths;
    }
}
