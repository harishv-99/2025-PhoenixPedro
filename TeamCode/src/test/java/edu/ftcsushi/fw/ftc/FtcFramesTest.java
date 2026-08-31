package edu.ftcsushi.fw.ftc;

import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Pins the source-breaking Sushi frame vocabulary without former-name aliases. */
public final class FtcFramesTest {

    @Test
    public void publicConversionsUseOnlyCurrentSushiNames() {
        Set<String> publicMethods = new HashSet<String>();
        for (Method method : FtcFrames.class.getDeclaredMethods()) {
            if (Modifier.isPublic(method.getModifiers())) {
                publicMethods.add(method.getName());
            }
        }

        assertTrue(publicMethods.containsAll(Arrays.asList(
                "toSushiFromFtcDetectionFrame",
                "toFtcDetectionFrameFromSushi",
                "toSushiFromFtcLocalizationCameraAxes",
                "toFtcLocalizationCameraAxesFromSushi",
                "toSushiFromFtcRobotAxes",
                "toFtcRobotAxesFromSushi",
                "sushiFromAprilTagRawCameraFrame",
                "aprilTagRawCameraFromSushiFrame",
                "sushiFromFtcDetectionFrame",
                "ftcDetectionFromSushiFrame"
        )));
        for (String method : publicMethods) {
            assertFalse("Former Phoenix frame alias remains public: " + method,
                    method.toLowerCase().contains("phoenix"));
        }
    }
}
