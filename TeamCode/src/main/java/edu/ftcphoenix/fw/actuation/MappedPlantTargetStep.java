package edu.ftcphoenix.fw.actuation;

import edu.ftcphoenix.fw.core.source.ScalarSource;
import edu.ftcphoenix.fw.core.source.ScalarTarget;

/**
 * Final target-selection stage shared by hardware-neutral mapped Plant builders.
 *
 * <p>Optional Plant target guards are configured before choosing the one final target graph.
 * Choosing that graph advances to {@link MappedPlantBuildStep}, so ordinary fluent callers cannot
 * build before answering the target-source question.</p>
 *
 * @param <P> concrete mapped Plant type produced after target selection
 */
public interface MappedPlantTargetStep<P extends Plant> {

    /**
     * Set dynamic Plant-level target guards and remain at target selection.
     */
    MappedPlantTargetStep<P> targetGuards(PlantTargetGuards targetGuards);

    /**
     * Use a command target as the exact final source.
     */
    MappedPlantBuildStep<P> targetedBy(ScalarTarget targetSource);

    /**
     * Use a scalar source as an exact final target source.
     */
    MappedPlantBuildStep<P> targetedBy(ScalarSource targetSource);

    /**
     * Use a Plant-aware final target graph.
     */
    MappedPlantBuildStep<P> targetedBy(PlantTargetSource targetSource);
}
