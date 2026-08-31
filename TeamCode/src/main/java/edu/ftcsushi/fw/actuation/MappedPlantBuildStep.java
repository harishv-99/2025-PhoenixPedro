package edu.ftcsushi.fw.actuation;

/**
 * Final build step shared by hardware-neutral mapped Plant builders.
 *
 * @param <P> concrete mapped Plant type returned by {@link #build()}
 */
interface MappedPlantBuildStep<P extends Plant> {

    /**
     * Build the fully configured mapped Plant.
     */
    P build();
}
