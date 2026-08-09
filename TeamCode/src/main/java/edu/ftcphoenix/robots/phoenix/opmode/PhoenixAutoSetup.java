package edu.ftcphoenix.robots.phoenix.opmode;

import java.util.Objects;

import edu.ftcphoenix.robots.phoenix.PhoenixReadiness;
import edu.ftcphoenix.robots.phoenix.autonomous.PhoenixAutoSpec;

/**
 * Immutable declaration of how one Phoenix autonomous entry obtains its setup.
 *
 * <p>Ordinary Phoenix Auto OpModes choose one of the two parallel factories below. Both feed the
 * same managed declaration path; the only difference is whether the supplied specification is
 * already final or remains editable through the standard INIT selector until FTC START.</p>
 */
public final class PhoenixAutoSetup {

    enum SelectionMode {
        FIXED,
        INIT_SELECTION
    }

    private final SelectionMode selectionMode;
    private final PhoenixAutoSpec initialSpec;
    private final PhoenixReadiness.AutoPurpose purpose;

    private PhoenixAutoSetup(SelectionMode selectionMode,
                             PhoenixAutoSpec initialSpec,
                             PhoenixReadiness.AutoPurpose purpose) {
        this.selectionMode = Objects.requireNonNull(selectionMode, "selectionMode");
        this.initialSpec = Objects.requireNonNull(initialSpec, "initialSpec");
        this.purpose = Objects.requireNonNull(purpose, "purpose");
    }

    /**
     * Declare an autonomous entry whose complete specification is known during construction.
     */
    public static PhoenixAutoSetup fromFixedSpec(
            PhoenixAutoSpec spec,
            PhoenixReadiness.AutoPurpose purpose
    ) {
        return new PhoenixAutoSetup(SelectionMode.FIXED, spec, purpose);
    }

    /**
     * Declare an autonomous entry whose standard INIT selector begins with {@code defaultSpec}.
     */
    public static PhoenixAutoSetup fromInitSelection(
            PhoenixAutoSpec defaultSpec,
            PhoenixReadiness.AutoPurpose purpose
    ) {
        return new PhoenixAutoSetup(SelectionMode.INIT_SELECTION, defaultSpec, purpose);
    }

    SelectionMode selectionMode() {
        return selectionMode;
    }

    PhoenixAutoSpec initialSpec() {
        return initialSpec;
    }

    PhoenixReadiness.AutoPurpose purpose() {
        return purpose;
    }
}
