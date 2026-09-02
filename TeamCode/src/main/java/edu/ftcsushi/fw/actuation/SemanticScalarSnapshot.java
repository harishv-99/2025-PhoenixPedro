package edu.ftcsushi.fw.actuation;

import java.util.Objects;

/**
 * Immutable composition of one semantic scalar request and one Plant status snapshot.
 *
 * <p>The generic Plant snapshot remains intact, so a positional mechanism retains compile-time
 * access to {@link PositionPlantSnapshot} facts without a separate generic framework
 * {@code SemanticPositionSnapshot} subtype. Current-request selection and arrival are correlated
 * to the exact immutable request identity; numeric equality with an older request, overlay,
 * fallback, or hold is not sufficient.</p>
 *
 * <p>A robot capability should normally wrap this value in one thin, immutable status type with
 * domain-named accessors such as {@code requestedHeight()} or {@code mode()}. That preserves this
 * class as the reusable coherence/provenance mechanism without requiring ordinary presenters or
 * autonomous routines to navigate framework-generic request and Plant objects.</p>
 *
 * @param <S> semantic request type
 * @param <P> concrete Plant snapshot type
 */
public final class SemanticScalarSnapshot<S, P extends PlantSnapshot> {

    private final SemanticScalarCommand.Request<S> request;
    private final P plant;
    private final boolean currentRequestSelected;

    SemanticScalarSnapshot(SemanticScalarCommand.Request<S> request,
                           P plant,
                           boolean currentRequestSelected) {
        this.request = Objects.requireNonNull(request, "request");
        this.plant = Objects.requireNonNull(plant, "plant");
        this.currentRequestSelected = currentRequestSelected;
    }

    /** Return the immutable semantic/numeric request captured for this snapshot. */
    public SemanticScalarCommand.Request<S> request() {
        return request;
    }

    /** Return the immutable Plant facts composed with this semantic request. */
    public P plant() {
        return plant;
    }

    /**
     * Whether this exact request produced the Plant's most recent resolved target.
     *
     * <p>This describes target-graph selection, not physical arrival. An open-loop Plant may select
     * the request while remaining unable to prove arrival.</p>
     */
    public boolean currentRequestSelected() {
        return currentRequestSelected;
    }

    /**
     * Whether this exact current request was selected and the Plant proves physical arrival.
     */
    public boolean currentRequestAtTarget() {
        return currentRequestSelected && plant.atTarget();
    }
}
