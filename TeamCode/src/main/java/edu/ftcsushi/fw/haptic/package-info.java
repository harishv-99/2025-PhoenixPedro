/**
 * Device-neutral command seams for controller haptic feedback.
 *
 * <p>The composition root chooses one recipient per
 * {@link edu.ftcsushi.fw.haptic.HapticSink}; robot controls or a dedicated driver-feedback owner
 * owns cue mapping and repeat policy. Supervisors and services may supply robot status or policy;
 * FTC SDK conversion stays in {@code edu.ftcsushi.fw.ftc}.</p>
 */
package edu.ftcsushi.fw.haptic;
