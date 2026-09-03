package edu.ftcsushi.fw.actuation;

import java.util.EnumMap;
import java.util.EnumSet;
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
 * Bind an ordinary literal target with
 * {@link Plants.TargetStep#targetExactlyFrom(SemanticScalarCommand)}. Use the corresponding
 * {@link PlantTargets} overlay or equivalent-position forms only when the mechanism needs that
 * additional resolution policy.</p>
 *
 * <p>For a fixed enum-to-scalar table, prefer {@link #forEnum(Enum)} so construction verifies
 * that every enum value has exactly one finite mapping. Keep {@link #create(Object,
 * ToDoubleFunction)} for a computed mapping or a non-enum semantic value. Neither construction
 * path chooses exact versus periodic-equivalent position realization; the final Plant target
 * resolver owns that policy.</p>
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
     * Start a complete fixed mapping for one enum-backed semantic command.
     *
     * <p>The initial value identifies the enum type and is not published until the returned
     * builder successfully builds the command. Map every enum value, including the initial one.
     * Different enum values may intentionally share one scalar target; semantic request identity
     * never depends on numeric uniqueness.</p>
     *
     * @param initialSemantic non-null initial enum request
     * @param <E> enum request type
     * @return a fresh mapping builder
     * @throws NullPointerException if {@code initialSemantic} is null
     */
    public static <E extends Enum<E>> EnumMappingBuilder<E> forEnum(E initialSemantic) {
        return new EnumMappingBuilder<>(
                Objects.requireNonNull(initialSemantic, "initialSemantic"));
    }

    /**
     * Builds one semantic command from a complete fixed enum-to-scalar table.
     *
     * <p>Rejected mapping answers do not change the retained table. An incomplete
     * {@link #build()} likewise leaves the builder open so the missing values can be supplied and
     * build retried. Once a complete build attempt begins, the builder is consumed: create a fresh
     * {@link SemanticScalarCommand#forEnum(Enum)} builder for another independently owned
     * command.</p>
     *
     * <p>This builder validates nulls, duplicate enum answers, finite scalar targets, and complete
     * enum coverage. Plant units, ranges, ordering, calibration, and exact-versus-equivalent target
     * resolution remain the owning mechanism's explicit responsibility.</p>
     *
     * @param <E> enum request type
     */
    public static final class EnumMappingBuilder<E extends Enum<E>> {
        private final E initialSemantic;
        private final Class<E> enumType;
        private final EnumMap<E, Double> commandTargets;
        private boolean buildAttempted;

        private EnumMappingBuilder(E initialSemantic) {
            this.initialSemantic = initialSemantic;
            this.enumType = initialSemantic.getDeclaringClass();
            this.commandTargets = new EnumMap<>(enumType);
        }

        /**
         * Map one enum request to its finite scalar command target.
         *
         * <p>Each enum value may be answered once. A rejected answer leaves this builder
         * unchanged; a duplicate never replaces the first accepted target.</p>
         *
         * @param semantic enum request to map
         * @param commandTarget corresponding finite target in the eventual Plant's units
         * @return this builder
         * @throws NullPointerException if {@code semantic} is null
         * @throws IllegalArgumentException if the value belongs to a different enum type or the
         * target is NaN or infinity
         * @throws IllegalStateException if this value is already mapped or build was attempted
         */
        public EnumMappingBuilder<E> map(E semantic, double commandTarget) {
            requireMutableForMap();
            E actualSemantic = Objects.requireNonNull(semantic, "semantic");
            if (actualSemantic.getDeclaringClass() != enumType) {
                throw new IllegalArgumentException(
                        "semantic must be a " + enumType.getSimpleName() + " value, got "
                                + actualSemantic.getDeclaringClass().getSimpleName() + "."
                                + actualSemantic.name());
            }
            if (commandTargets.containsKey(actualSemantic)) {
                throw new IllegalStateException(
                        "SemanticScalarCommand.forEnum(...) already maps semantic request "
                                + actualSemantic.name()
                                + "; each enum value may be mapped once");
            }
            if (!Double.isFinite(commandTarget)) {
                throw new IllegalArgumentException(
                        "commandTarget must be finite, got " + commandTarget
                                + " for semantic request " + actualSemantic.name());
            }
            commandTargets.put(actualSemantic, commandTarget);
            return this;
        }

        /**
         * Validate complete enum coverage and build one independently owned command.
         *
         * <p>An incomplete table reports every missing value in declaration order and remains
         * repairable. A complete build attempt consumes this builder and freezes a defensive copy
         * of its table inside the returned command.</p>
         *
         * @return a new semantic command holding the configured initial request
         * @throws IllegalStateException if values are missing or build was already attempted
         */
        public SemanticScalarCommand<E> build() {
            if (buildAttempted) {
                throw new IllegalStateException(
                        "SemanticScalarCommand.forEnum(...) build() has already been attempted; "
                                + "start a new builder");
            }

            EnumSet<E> missing = EnumSet.allOf(enumType);
            missing.removeAll(commandTargets.keySet());
            if (!missing.isEmpty()) {
                throw new IllegalStateException(
                        "SemanticScalarCommand.forEnum(" + initialSemantic.name()
                                + ") must map every " + enumType.getSimpleName()
                                + " value before build(); missing " + enumNames(missing));
            }

            final EnumMap<E, Double> frozenTargets = new EnumMap<>(commandTargets);
            buildAttempted = true;
            return new SemanticScalarCommand<>(initialSemantic, semantic -> {
                Double target = frozenTargets.get(semantic);
                if (target == null) {
                    throw new IllegalArgumentException(
                            "No command target is mapped for semantic request " + semantic);
                }
                return target;
            });
        }

        private void requireMutableForMap() {
            if (buildAttempted) {
                throw new IllegalStateException(
                        "map(...) cannot change this SemanticScalarCommand.forEnum(...) mapping "
                                + "after build() has been attempted; start a new builder");
            }
        }

        private static String enumNames(EnumSet<?> values) {
            StringBuilder out = new StringBuilder("[");
            boolean first = true;
            for (Enum<?> value : values) {
                if (!first) {
                    out.append(", ");
                }
                out.append(value.name());
                first = false;
            }
            return out.append(']').toString();
        }
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
