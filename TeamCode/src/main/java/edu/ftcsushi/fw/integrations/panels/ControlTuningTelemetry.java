package edu.ftcsushi.fw.integrations.panels;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

/** Stable numeric Panels transport with explicit availability for every optional value. */
final class ControlTuningTelemetry {

    /**
     * Publish a finite numeric value plus {@code key.available}. When unavailable, the numeric zero
     * is presentation-only and must never be interpreted without its companion availability flag.
     */
    static void addOptionalNumber(Telemetry telemetry, String key, Double value) {
        boolean available = value != null && Double.isFinite(value);
        telemetry.addData(key, available ? value : 0.0);
        telemetry.addData(key + ".available", available ? 1.0 : 0.0);
    }

    /** Publish one controller-evidence snapshot with stable, schema-qualified keys. */
    static void addEvidence(Telemetry telemetry,
                            String prefix,
                            Map<String, Double> numeric,
                            Map<String, String> text) {
        for (Map.Entry<String, Double> entry : numeric.entrySet()) {
            addOptionalNumber(telemetry, prefix + entry.getKey(), entry.getValue());
        }
        for (Map.Entry<String, String> entry : text.entrySet()) {
            telemetry.addData(prefix + entry.getKey(), entry.getValue());
        }
    }

    private ControlTuningTelemetry() {
    }
}
