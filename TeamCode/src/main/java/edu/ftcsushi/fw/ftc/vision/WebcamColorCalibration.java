package edu.ftcsushi.fw.ftc.vision;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

import edu.ftcsushi.fw.core.geometry.Vec3;

/**
 * Immutable SDK intrinsics/distortion snapshot with bounded point undistortion. This validates
 * software metadata, not the physical accuracy of a camera calibration or its mounting pose.
 */
final class WebcamColorCalibration {

    private static final int MAX_ITERATIONS = 30;
    private static final double MAX_PIXEL_RESIDUAL = 0.05;

    final int width;
    final int height;
    private final double focalX;
    private final double focalY;
    private final double centerX;
    private final double centerY;
    private final double[] distortion;
    private final String unavailableReason;

    private WebcamColorCalibration(int width, int height, double focalX, double focalY,
                                   double centerX, double centerY, double[] distortion,
                                   String unavailableReason) {
        this.width = width;
        this.height = height;
        this.focalX = focalX;
        this.focalY = focalY;
        this.centerX = centerX;
        this.centerY = centerY;
        this.distortion = distortion.clone();
        this.unavailableReason = unavailableReason;
    }

    /** Snapshots mutable SDK arrays and checks calibration applicability to this callback image. */
    static WebcamColorCalibration snapshot(int width, int height, CameraCalibration calibration) {
        if (calibration == null || calibration.isFake() || calibration.getRemove()
                || calibration.getSize() == null) {
            return unavailable(width, height, "webcam calibration is absent, fake, or removed");
        }
        if (calibration.getSize().getWidth() != width
                || calibration.getSize().getHeight() != height) {
            return unavailable(width, height, "webcam calibration dimensions do not match the callback image");
        }
        float[] sdkDistortion = calibration.distortionCoefficients;
        if (sdkDistortion == null || sdkDistortion.length != 8) {
            return unavailable(width, height, "webcam calibration requires eight distortion coefficients");
        }
        double[] coefficients = new double[8];
        for (int i = 0; i < coefficients.length; i++) {
            coefficients[i] = sdkDistortion[i];
        }
        return of(width, height, calibration.focalLengthX, calibration.focalLengthY,
                calibration.principalPointX, calibration.principalPointY, coefficients);
    }

    /** Hardware-neutral validation seam used by the SDK snapshot and focused calibration tests. */
    static WebcamColorCalibration of(int width, int height, double focalX, double focalY,
                                     double centerX, double centerY, double[] distortion) {
        if (width <= 0 || height <= 0 || !Double.isFinite(focalX) || focalX <= 0.0
                || !Double.isFinite(focalY) || focalY <= 0.0
                || !Double.isFinite(centerX) || !Double.isFinite(centerY)
                || centerX < 0.0 || centerX >= width || centerY < 0.0 || centerY >= height) {
            return unavailable(width, height, "webcam calibration requires positive dimensions/focal lengths and an in-image principal point");
        }
        if (distortion == null || distortion.length != 8) {
            return unavailable(width, height, "webcam calibration requires eight distortion coefficients");
        }
        for (double coefficient : distortion) {
            if (!Double.isFinite(coefficient)) {
                return unavailable(width, height, "webcam distortion coefficients must all be finite");
            }
        }
        return new WebcamColorCalibration(width, height, focalX, focalY, centerX, centerY,
                distortion, "");
    }

    private static WebcamColorCalibration unavailable(int width, int height, String reason) {
        return new WebcamColorCalibration(width, height, 0.0, 0.0, 0.0, 0.0,
                new double[8], reason);
    }

    boolean isAvailable() {
        return unavailableReason.isEmpty();
    }

    String reason() {
        return unavailableReason;
    }

    /**
     * Converts a full-image, right/down-positive pixel to a camera-forward/left/up ray, or null
     * when the pixel or bounded inverse is unusable. Uses the SDK's eight-coefficient rational
     * radial/tangential model, then requires a forward-distortion residual below 0.05 pixel.
     * There is no FOV estimate or fallback which silently discards distortion.
     */
    Vec3 rayForPixel(double pixelX, double pixelY) {
        if (!isAvailable() || !Double.isFinite(pixelX) || !Double.isFinite(pixelY)
                || pixelX < 0.0 || pixelX >= width || pixelY < 0.0 || pixelY >= height) {
            return null;
        }
        double distortedX = (pixelX - centerX) / focalX;
        double distortedY = (pixelY - centerY) / focalY;
        double x = distortedX;
        double y = distortedY;
        for (int i = 0; i < MAX_ITERATIONS; i++) {
            double radius2 = x * x + y * y;
            double numerator = 1.0 + radius2 * (distortion[0]
                    + radius2 * (distortion[1] + radius2 * distortion[4]));
            double denominator = 1.0 + radius2 * (distortion[5]
                    + radius2 * (distortion[6] + radius2 * distortion[7]));
            if (!Double.isFinite(numerator) || !Double.isFinite(denominator)
                    || numerator <= 1e-12 || denominator <= 1e-12) {
                return null;
            }
            double deltaX = 2.0 * distortion[2] * x * y
                    + distortion[3] * (radius2 + 2.0 * x * x);
            double deltaY = distortion[2] * (radius2 + 2.0 * y * y)
                    + 2.0 * distortion[3] * x * y;
            double predictedX = x * numerator / denominator + deltaX;
            double predictedY = y * numerator / denominator + deltaY;
            double residual = Math.hypot((predictedX - distortedX) * focalX,
                    (predictedY - distortedY) * focalY);
            if (Double.isFinite(residual) && residual <= MAX_PIXEL_RESIDUAL) {
                return new Vec3(1.0, -x, -y);
            }
            x = (distortedX - deltaX) * denominator / numerator;
            y = (distortedY - deltaY) * denominator / numerator;
            if (!Double.isFinite(x) || !Double.isFinite(y)) {
                return null;
            }
        }
        return null;
    }
}
