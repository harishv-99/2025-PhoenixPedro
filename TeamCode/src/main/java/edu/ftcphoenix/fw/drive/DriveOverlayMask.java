package edu.ftcphoenix.fw.drive;

/**
 * Mask describing which components of a {@link DriveSignal} are controlled by an overlay.
 *
 * <p>Phoenix uses the term <b>overlay</b> for “a secondary drive behavior that can temporarily
 * take control of one or more drive degrees-of-freedom (DOFs)”. Common examples are:</p>
 *
 * <ul>
 *   <li>Auto-aim that overrides only {@link DriveSignal#omega} while the driver keeps translation.</li>
 *   <li>A nudge / micro-drive mode that overrides translation while keeping heading control manual.</li>
 *   <li>A pose lock that overrides both translation and rotation when enabled.</li>
 * </ul>
 *
 * <p>The mask is carried separately from the {@link DriveSignal} so an overlay can dynamically
 * decide when it is “confident enough” to take control. For example, a vision-based overlay may
 * return {@link #NONE} when the target is not visible, allowing manual control to pass through.</p>
 */
public final class DriveOverlayMask {

    /**
     * No components are overridden.
     */
    public static final DriveOverlayMask NONE = new DriveOverlayMask(false, false, false);

    /**
     * Translation only (axial + lateral).
     */
    public static final DriveOverlayMask TRANSLATION_ONLY = new DriveOverlayMask(true, true, false);

    /**
     * Rotation only (omega).
     */
    public static final DriveOverlayMask OMEGA_ONLY = new DriveOverlayMask(false, false, true);

    /**
     * Translation + rotation (all components).
     */
    public static final DriveOverlayMask ALL = new DriveOverlayMask(true, true, true);

    /**
     * True if {@link DriveSignal#axial} should be overridden.
     */
    public final boolean axial;

    /**
     * True if {@link DriveSignal#lateral} should be overridden.
     */
    public final boolean lateral;

    /**
     * True if {@link DriveSignal#omega} should be overridden.
     */
    public final boolean omega;

    /**
     * Create a new mask.
     *
     * <p>Most callers should use the predefined constants ({@link #NONE}, {@link #TRANSLATION_ONLY},
     * {@link #OMEGA_ONLY}, {@link #ALL}) rather than constructing masks directly.</p>
     */
    public DriveOverlayMask(boolean axial, boolean lateral, boolean omega) {
        this.axial = axial;
        this.lateral = lateral;
        this.omega = omega;
    }

    /**
     * Return true if this mask overrides translation (axial and/or lateral).
     */
    public boolean overridesTranslation() {
        return axial || lateral;
    }

    /**
     * Return true if this mask overrides omega (rotation).
     */
    public boolean overridesOmega() {
        return omega;
    }

    /**
     * Return a new mask with translation (axial+lateral) set to {@code enabled}.
     */
    public DriveOverlayMask withTranslation(boolean enabled) {
        return new DriveOverlayMask(enabled, enabled, omega);
    }

    /**
     * Return a new mask with omega set to {@code enabled}.
     */
    public DriveOverlayMask withOmega(boolean enabled) {
        return new DriveOverlayMask(axial, lateral, enabled);
    }

    /**
     * Return true if this mask overrides no components.
     */
    public boolean isNone() {
        return !axial && !lateral && !omega;
    }

    /**
     * Intersection (logical AND) of two masks.
     *
     * <p>This is used to combine:</p>
     * <ul>
     *   <li>a <b>requested</b> mask (what the caller wants to override), and</li>
     *   <li>a <b>dynamic</b> mask (what the overlay believes it can safely override right now).</li>
     * </ul>
     */
    public DriveOverlayMask intersect(DriveOverlayMask other) {
        if (other == null) {
            return this;
        }
        return new DriveOverlayMask(
                this.axial && other.axial,
                this.lateral && other.lateral,
                this.omega && other.omega
        );
    }

    /**
     * Union (logical OR) of two masks.
     */
    public DriveOverlayMask union(DriveOverlayMask other) {
        if (other == null) {
            return this;
        }
        return new DriveOverlayMask(
                this.axial || other.axial,
                this.lateral || other.lateral,
                this.omega || other.omega
        );
    }

    @Override
    public String toString() {
        return "DriveOverlayMask{" +
                "axial=" + axial +
                ", lateral=" + lateral +
                ", omega=" + omega +
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof DriveOverlayMask)) return false;
        DriveOverlayMask other = (DriveOverlayMask) o;
        return this.axial == other.axial
                && this.lateral == other.lateral
                && this.omega == other.omega;
    }

    @Override
    public int hashCode() {
        int bits = 7;
        bits = 31 * bits + (axial ? 1 : 0);
        bits = 31 * bits + (lateral ? 1 : 0);
        bits = 31 * bits + (omega ? 1 : 0);
        return bits;
    }
}
