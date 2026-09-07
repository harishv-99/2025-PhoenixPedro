package edu.ftcsushi.fw.ftc.vision;

import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.VendorProductCalibrationIdentity;
import org.junit.Test;

import edu.ftcsushi.fw.core.geometry.Vec3;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

/** Production calibration math with authored synthetic metadata; no physical calibration claim. */
public final class WebcamColorCalibrationTest {

    @Test
    public void calibratedPixelAxesBecomeCameraForwardLeftUp() {
        WebcamColorCalibration calibration = WebcamColorCalibration.of(
                640, 480, 500, 400, 320, 240, new double[8]);
        assertTrue(calibration.isAvailable());
        Vec3 center = calibration.rayForPixel(320, 240);
        assertEquals(1.0, center.x, 0.0);
        assertEquals(0.0, center.y, 0.0);
        assertEquals(0.0, center.z, 0.0);
        Vec3 rightDown = calibration.rayForPixel(420, 280);
        assertEquals(-0.2, rightDown.y, 1e-12);
        assertEquals(-0.1, rightDown.z, 1e-12);
        Vec3 leftUp = calibration.rayForPixel(220, 200);
        assertEquals(0.2, leftUp.y, 1e-12);
        assertEquals(0.1, leftUp.z, 1e-12);
    }

    @Test
    public void undistortsRadialTangentialAndRationalTermsTogether() {
        double[] coefficients = {0.12, -0.04, 0.004, -0.003, 0.01, 0.02, -0.01, 0.003};
        WebcamColorCalibration calibration = WebcamColorCalibration.of(
                640, 480, 500, 500, 320, 240, coefficients);
        double x = 0.42;
        double y = -0.28;
        double[] distorted = distort(x, y, coefficients);
        Vec3 recovered = calibration.rayForPixel(320 + 500 * distorted[0],
                240 + 500 * distorted[1]);
        assertNotNull(recovered);
        assertEquals(-x, recovered.y, 0.0001);
        assertEquals(-y, recovered.z, 0.0001);
        assertTrue(Math.abs(recovered.y + distorted[0]) > 0.001);
    }

    @Test
    public void snapshotCopiesSdkCoefficientsAndChecksApplicableDimensions() {
        CameraCalibration sdk = sdkCalibration(false);
        WebcamColorCalibration snapshot = WebcamColorCalibration.snapshot(640, 480, sdk);
        assertTrue(snapshot.reason(), snapshot.isAvailable());
        sdk.focalLengthX = 1;
        sdk.distortionCoefficients[0] = 100;
        Vec3 ray = snapshot.rayForPixel(420, 240);
        assertNotNull(ray);
        assertEquals(-0.2, ray.y, 1e-12);
        assertFalse(WebcamColorCalibration.snapshot(320, 240, sdkCalibration(false)).isAvailable());
        assertFalse(WebcamColorCalibration.snapshot(640, 480, sdkCalibration(true)).isAvailable());
        assertFalse(WebcamColorCalibration.snapshot(640, 480, null).isAvailable());
    }

    @Test
    public void pureMathConstructionAlsoCopiesItsInputArray() {
        double[] coefficients = new double[8];
        WebcamColorCalibration calibration = WebcamColorCalibration.of(
                640, 480, 500, 500, 320, 240, coefficients);
        coefficients[0] = 100;
        assertEquals(-0.2, calibration.rayForPixel(420, 240).y, 1e-12);
    }

    @Test
    public void invalidMetadataNeverFallsBackToGuessedFovOrZeroDistortion() {
        for (double bad : new double[]{0.0, -1.0, Double.NaN, Double.POSITIVE_INFINITY}) {
            assertFalse(WebcamColorCalibration.of(640, 480, bad, 500, 320, 240,
                    new double[8]).isAvailable());
            assertFalse(WebcamColorCalibration.of(640, 480, 500, bad, 320, 240,
                    new double[8]).isAvailable());
        }
        assertFalse(WebcamColorCalibration.of(0, 480, 500, 500, 320, 240,
                new double[8]).isAvailable());
        assertFalse(WebcamColorCalibration.of(640, 480, 500, 500, 640, 240,
                new double[8]).isAvailable());
        assertFalse(WebcamColorCalibration.of(640, 480, 500, 500, 320, Double.NaN,
                new double[8]).isAvailable());
        assertFalse(WebcamColorCalibration.of(640, 480, 500, 500, 320, 240, null).isAvailable());
        assertFalse(WebcamColorCalibration.of(640, 480, 500, 500, 320, 240,
                new double[5]).isAvailable());
        double[] nonFinite = new double[8];
        nonFinite[6] = Double.NaN;
        assertFalse(WebcamColorCalibration.of(640, 480, 500, 500, 320, 240, nonFinite)
                .isAvailable());
    }

    @Test
    public void pixelsOutsideTheImageAndSingularInverseAreUnavailable() {
        WebcamColorCalibration calibration = WebcamColorCalibration.of(
                640, 480, 500, 500, 320, 240, new double[8]);
        assertNull(calibration.rayForPixel(-1, 240));
        assertNull(calibration.rayForPixel(640, 240));
        assertNull(calibration.rayForPixel(320, 480));
        assertNull(calibration.rayForPixel(Double.NaN, 240));
        double[] singular = {-25, 0, 0, 0, 0, 0, 0, 0};
        WebcamColorCalibration singularCalibration = WebcamColorCalibration.of(
                640, 480, 500, 500, 320, 240, singular);
        assertNull(singularCalibration.rayForPixel(420, 240));
    }

    /** Independent forward model generates test pixels from known undistorted directions. */
    private static double[] distort(double x, double y, double[] k) {
        double r2 = x * x + y * y;
        double radial = (1 + k[0] * r2 + k[1] * r2 * r2 + k[4] * r2 * r2 * r2)
                / (1 + k[5] * r2 + k[6] * r2 * r2 + k[7] * r2 * r2 * r2);
        return new double[]{
                x * radial + 2 * k[2] * x * y + k[3] * (r2 + 2 * x * x),
                y * radial + k[2] * (r2 + 2 * y * y) + 2 * k[3] * x * y
        };
    }

    private static CameraCalibration sdkCalibration(boolean fake) {
        return new CameraCalibration(new VendorProductCalibrationIdentity(1, 2),
                new Size(640, 480), 500, 500, 320, 240, new float[8], false, fake);
    }
}
