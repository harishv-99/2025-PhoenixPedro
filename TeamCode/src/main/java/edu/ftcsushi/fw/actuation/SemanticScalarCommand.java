package edu.ftcsushi.fw.actuation;

import java.util.Objects;
import java.util.function.ToDoubleFunction;

/**
 * Owns one semantic mechanism request and its mapped scalar Plant target.
 *
 * <p>Use this type when the capability request is a named value such as a lift {@code Height},
 * claw {@code State}, or launcher {@code Speed}, while the final {@link Plant} consumes one numeric
 * target. The mechanism owns this command and maps every public direct or Task request through
 * this same owner and mapper. Direct APIs use {@link #set(Object)}; {@link SemanticScalarTasks}
 * uses owner-bound preparation and publication so Task builders validate without publishing.
 * Bind the command into the final Plant graph with
 * {@link PlantTargets#exact(SemanticScalarCommand)} or one of the corresponding overlay or
 * equivalent-position overloads.</p>
 *
 * <p>This is mechanism infrastructure, not the ordinary capability vocabulary. Robot clients
 * should call domain methods such as {@code setHeight(Height)} or a fresh
 * {@code moveTo(Height)} Task factory and observe a capability-shaped status.</p>
 *
 * <p>Each successful set maps and validates the semantic value before publishing one immutable
 * semantic/numeric pair. Publication replaces one volatile reference, so a reader never observes
 * a new semantic value with an old numeric target. Even an identical repeated request receives a
 * fresh private identity; status therefore cannot mistake arrival for an earlier request as
 * arrival for the newly issued one. A throwing mapper or a non-finite mapped target leaves the
 * previous request unchanged.</p>
 *
 * <p>The semantic value itself should be immutable, as an enum or immutable value object. This
 * generic owner cannot defensively copy an arbitrary {@code S}.</p>
 *
 * @param <S> semantic request type
 */
public final class SemanticScalarCommand<S> {

    private final ToDoubleFunction<? super S> commandTargetFor;
    private volatile Request<S> request;

    private SemanticScalarCommand(S initialSemantic,
                                  ToDoubleFunction<? super S> commandTargetFor) {
        this.commandTargetFor = Objects.requireNonNull(commandTargetFor, "commandTargetFor");
        this.request = map(initialSemantic);
    }

    /**
     * Create a semantic scalar command and publish its validated initial request.
     *
     * <p>The mapper is invoked exactly once for the initial semantic value.</p>
     */
    public static <S> SemanticScalarCommand<S> create(
            S initialSemantic,
            ToDoubleFunction<? super S> commandTargetFor) {
        return new SemanticScalarCommand<>(initialSemantic, commandTargetFor);
    }

    /**
     * Map, validate, and atomically publish a fresh request.
     *
     * @return the exact immutable request object that was published
     * @throws NullPointerException if {@code semantic} is null
     * @throws IllegalArgumentException if the mapper returns NaN or infinity
     */
    public Request<S> set(S semantic) {
        Request<S> next = map(semantic);
        request = next;
        return next;
    }

    /**
     * Return the currently published immutable semantic/numeric request.
     */
    public Request<S> request() {
        return request;
    }

    /**
     * Map and validate one request without publishing it.
     *
     * <p>This package-private seam lets {@link SemanticScalarTasks} prepare every semantic/numeric
     * pair while its builder stage is selected, then publish a fresh request occurrence only when
     * the built Task starts. The returned template is bound to this command and cannot be
     * published through another owner.</p>
     */
    PreparedRequest<S> prepare(S semantic) {
        S actualSemantic = Objects.requireNonNull(semantic, "semantic");
        return new PreparedRequest<>(this, actualSemantic, mappedTargetFor(actualSemantic));
    }

    /** Publish one fresh occurrence of a previously validated, owner-bound request template. */
    Request<S> publish(PreparedRequest<S> prepared) {
        PreparedRequest<S> actual = Objects.requireNonNull(prepared, "prepared");
        if (actual.owner != this) {
            throw new IllegalArgumentException(
                    "Prepared semantic request belongs to a different SemanticScalarCommand");
        }
        Request<S> next = new Request<>(actual.semantic, actual.commandTarget);
        request = next;
        return next;
    }

    /**
     * Compose the current request with one immutable Plant snapshot.
     *
     * <p>Current-request arrival is true only when the Plant resolution carries this command's
     * exact request identity and the Plant independently proves physical arrival. Calling
     * {@link #set(Object)} immediately makes an older Plant snapshot ineligible, even when the new
     * request maps to the same numeric target.</p>
     */
    public <P extends PlantSnapshot> SemanticScalarSnapshot<S, P> snapshot(P plant) {
        P actualPlant = Objects.requireNonNull(plant, "plant");
        Request<S> currentRequest = request;
        boolean selected = actualPlant.targetResolution()
                .satisfiesSemanticCommand(this, currentRequest);
        return new SemanticScalarSnapshot<>(currentRequest, actualPlant, selected);
    }

    private Request<S> map(S semantic) {
        S actualSemantic = Objects.requireNonNull(semantic, "semantic");
        return new Request<>(actualSemantic, mappedTargetFor(actualSemantic));
    }

    private double mappedTargetFor(S actualSemantic) {
        double commandTarget = commandTargetFor.applyAsDouble(actualSemantic);
        if (!Double.isFinite(commandTarget)) {
            throw new IllegalArgumentException(
                    "commandTargetFor must return a finite target, got " + commandTarget
                            + " for semantic request " + actualSemantic);
        }
        return commandTarget;
    }

    /** Immutable mapped request template retained only by framework Task builders. */
    static final class PreparedRequest<S> {
        private final SemanticScalarCommand<S> owner;
        private final S semantic;
        private final double commandTarget;

        private PreparedRequest(SemanticScalarCommand<S> owner,
                                S semantic,
                                double commandTarget) {
            this.owner = Objects.requireNonNull(owner, "owner");
            this.semantic = Objects.requireNonNull(semantic, "semantic");
            this.commandTarget = commandTarget;
        }

        S semantic() {
            return semantic;
        }
    }

    /**
     * Immutable semantic/numeric request published by a {@link SemanticScalarCommand}.
     *
     * <p>Callers may retain the exact returned object to correlate work issued at Task start with a
     * later {@link SemanticScalarSnapshot#request()} by identity. Use {@link #semantic()} and
     * {@link #commandTarget()} for value inspection; identity is request-occurrence evidence, not
     * semantic equality or a public numeric revision.</p>
     */
    public static final class Request<S> {
        private final S semantic;
        private final double commandTarget;

        private Request(S semantic, double commandTarget) {
            this.semantic = semantic;
            this.commandTarget = commandTarget;
        }

        /** Return the named capability request. */
        public S semantic() {
            return semantic;
        }

        /** Return the corresponding finite target in Plant units. */
        public double commandTarget() {
            return commandTarget;
        }
    }
}
