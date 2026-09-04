package edu.ftcsushi.fw.testing.ftc;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Proxy;

/** Supplied FTC test-boundary helpers that keep proxy plumbing out of teaching scenarios. */
public final class FtcTestTelemetry {

    private FtcTestTelemetry() {
        // Static test support.
    }

    /** Returns inert telemetry suitable for a managed test that does not inspect presentation. */
    public static Telemetry silent() {
        return (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class},
                (ignored, method, args) ->
                        method.getReturnType() == boolean.class ? false : null);
    }
}
