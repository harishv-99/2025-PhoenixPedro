package edu.ftcsushi.fw.ftc.vision;

import android.graphics.Canvas;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.ftcsushi.fw.core.geometry.Vec3;

/**
 * Owner-private SDK color delegate with a fixed backing image and callback-leased native lifetime.
 * Construction allocates no native state. SDK-owned contours/ROI/scratch storage never escape;
 * the SDK has no supported disposal API for those objects, so their reclamation is not promised.
 */
final class FtcColorBlobProcessor implements VisionProcessor {

    /** One immutable bounded callback publication; no SDK/native objects are retained here. */
    static final class Frame {
        final long captureTimeNanos;
        final List<Vec3> rays;
        final String reason;
        private final List<Box> boxes;

        private Frame(long captureTimeNanos, List<Vec3> rays, List<Box> boxes, String reason) {
            this.captureTimeNanos = captureTimeNanos;
            this.rays = Collections.unmodifiableList(new ArrayList<>(rays));
            this.boxes = Collections.unmodifiableList(new ArrayList<>(boxes));
            this.reason = reason;
        }

        static Frame unavailable(long captureTimeNanos, String reason) {
            return new Frame(captureTimeNanos, Collections.emptyList(),
                    Collections.emptyList(), reason);
        }

        boolean isAvailable() {
            return reason.isEmpty();
        }
    }

    /** Copied preview points, rather than the SDK's retained contour/native drawing context. */
    private static final class Box {
        final float[] coordinates;

        Box(Point[] corners) {
            coordinates = new float[8];
            for (int i = 0; i < 4; i++) {
                coordinates[2 * i] = (float) corners[i].x;
                coordinates[2 * i + 1] = (float) corners[i].y;
            }
        }
    }

    /**
     * Tiny lifetime gate: native work never occurs under its lock. A terminal request closes new
     * admission immediately; exactly one terminalizer or final active callback owns disposal.
     * Unexpected overlapping SDK callbacks fail closed instead of racing the native delegate.
     */
    static final class CallbackLease {
        private boolean active;
        private boolean terminal;
        private boolean disposalClaimed;
        private Frame published = Frame.unavailable(Long.MIN_VALUE, "webcam color has no frame yet");

        synchronized boolean begin() {
            if (terminal) {
                return false;
            }
            if (active) {
                terminal = true;
                published = Frame.unavailable(Long.MIN_VALUE, "webcam color callbacks overlapped");
                return false;
            }
            active = true;
            return true;
        }

        synchronized void publish(Frame frame) {
            if (!terminal) {
                published = frame;
            }
        }

        synchronized Frame latest() {
            return published;
        }

        synchronized boolean isTerminal() {
            return terminal;
        }

        synchronized boolean terminalize(String reason) {
            if (!terminal) {
                terminal = true;
                published = Frame.unavailable(Long.MIN_VALUE, reason);
            }
            return claimDisposalIfQuiescent();
        }

        synchronized boolean finish() {
            if (!active) {
                throw new IllegalStateException("webcam color callback lease was not active");
            }
            active = false;
            return claimDisposalIfQuiescent();
        }

        private boolean claimDisposalIfQuiescent() {
            if (terminal && !active && !disposalClaimed) {
                disposalClaimed = true;
                return true;
            }
            return false;
        }
    }

    private final FtcFloorObjectVision.Config config;
    private final CallbackLease lease = new CallbackLease();
    private ColorBlobLocatorProcessor delegate;
    private Mat stableInput;
    private WebcamColorCalibration calibration;
    private int imageWidth;
    private int imageHeight;
    private int imageType = -1;

    FtcColorBlobProcessor(FtcFloorObjectVision.Config config) {
        this.config = config.validatedCopy("FtcColorBlobProcessor");
    }

    /** Lazy initialization runs on an admitted SDK callback, never in an owner constructor. */
    @Override
    public void init(int width, int height, CameraCalibration sdkCalibration) {
        if (!lease.begin()) {
            return;
        }
        try {
            if (imageWidth != 0 && (width != imageWidth || height != imageHeight)) {
                fail("webcam color image dimensions changed; construct a fresh camera owner");
                return;
            }
            imageWidth = width;
            imageHeight = height;
            calibration = WebcamColorCalibration.snapshot(width, height, sdkCalibration);
            if (!calibration.isAvailable()) {
                lease.publish(Frame.unavailable(Long.MIN_VALUE, calibration.reason()));
                return;
            }
            if (delegate == null) {
                delegate = new ColorBlobLocatorProcessor.Builder()
                        .setTargetColorRange(new ColorRange(ColorSpace.YCrCb,
                                new Scalar(config.minY, config.minCr, config.minCb),
                                new Scalar(config.maxY, config.maxCr, config.maxCb)))
                        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                        .setRoi(ImageRegion.entireFrame())
                        .setBlurSize(config.blurSizePixels)
                        .setErodeSize(0)
                        .setDilateSize(0)
                        .setDrawContours(false)
                        .setBoxFitColor(0)
                        .setCircleFitColor(0)
                        .build();
            }
            delegate.init(width, height, sdkCalibration);
            lease.publish(Frame.unavailable(Long.MIN_VALUE, "webcam color is waiting for a calibrated frame"));
        } catch (RuntimeException | LinkageError failure) {
            fail("webcam color initialization failed: " + failure.getClass().getSimpleName());
            throw failure;
        } finally {
            finishCallback();
        }
    }

    /** Copies the current borrowed input into one stable Mat before invoking the SDK delegate. */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (!lease.begin()) {
            return lease.latest();
        }
        try {
            if (calibration == null || !calibration.isAvailable() || delegate == null) {
                Frame missing = Frame.unavailable(captureTimeNanos, calibration == null
                        ? "webcam color has not been initialized" : calibration.reason());
                lease.publish(missing);
                return missing;
            }
            if (frame == null || frame.empty() || frame.cols() != imageWidth
                    || frame.rows() != imageHeight || (imageType != -1 && frame.type() != imageType)) {
                fail("webcam color input dimensions/type changed or became empty; construct a fresh camera owner");
                return lease.latest();
            }
            if (stableInput == null) {
                imageType = frame.type();
                stableInput = new Mat(imageHeight, imageWidth, imageType);
            }
            // SDK 11.1 aliases its first input ROI forever. Same shape/type copyTo preserves this
            // backing allocation across frames and streaming restarts; it must never be replaced.
            frame.copyTo(stableInput);
            delegate.processFrame(stableInput, captureTimeNanos);
            List<ColorBlobLocatorProcessor.Blob> blobs = delegate.getBlobs();
            List<Vec3> rays = new ArrayList<>();
            List<Box> boxes = new ArrayList<>();
            for (ColorBlobLocatorProcessor.Blob blob : blobs) {
                if (blob.getContourArea() < config.minContourAreaPixels) {
                    continue;
                }
                if (rays.size() == config.maxCandidates) {
                    Frame overflow = Frame.unavailable(captureTimeNanos,
                            "webcam color candidate count exceeds configured maxCandidates; narrow thresholding");
                    lease.publish(overflow);
                    return overflow;
                }
                RotatedRect box = blob.getBoxFit();
                Vec3 ray = box == null || box.center == null ? null
                        : calibration.rayForPixel(box.center.x, box.center.y);
                if (ray == null) {
                    Frame invalid = Frame.unavailable(captureTimeNanos,
                            "webcam color box center cannot be undistorted using this calibration");
                    lease.publish(invalid);
                    return invalid;
                }
                rays.add(ray);
                Point[] corners = new Point[4];
                box.points(corners);
                boxes.add(new Box(corners));
            }
            Frame copied = new Frame(captureTimeNanos, rays, boxes, "");
            lease.publish(copied);
            return copied;
        } catch (RuntimeException | LinkageError failure) {
            fail("webcam color processing failed: " + failure.getClass().getSimpleName());
            throw failure;
        } finally {
            finishCallback();
        }
    }

    /** Draws only copied Java coordinates; a queued draw never touches delegate/native storage. */
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (lease.isTerminal() || !(userContext instanceof Frame)) {
            return;
        }
        Frame copied = (Frame) userContext;
        Paint paint = new Paint();
        paint.setColor(0xff00ff00);
        paint.setStrokeWidth(Math.max(1.0f, scaleCanvasDensity * 2.0f));
        for (Box box : copied.boxes) {
            for (int i = 0; i < 4; i++) {
                int next = (i + 1) % 4;
                canvas.drawLine(box.coordinates[2 * i] * scaleBmpPxToCanvasPx,
                        box.coordinates[2 * i + 1] * scaleBmpPxToCanvasPx,
                        box.coordinates[2 * next] * scaleBmpPxToCanvasPx,
                        box.coordinates[2 * next + 1] * scaleBmpPxToCanvasPx, paint);
            }
        }
    }

    Frame latest() {
        return lease.latest();
    }

    boolean isTerminal() {
        return lease.isTerminal();
    }

    /**
     * Called for owner close or acquisition failure, even when portal close is asynchronous or
     * fails. Does not wait: an active native callback retains its lease and performs final cleanup.
     */
    void terminalize() {
        fail("webcam color owner is terminal; construct a fresh owner for another lifetime");
    }

    private void fail(String reason) {
        if (lease.terminalize(reason)) {
            disposeOwnedResources();
        }
    }

    private void finishCallback() {
        if (lease.finish()) {
            disposeOwnedResources();
        }
    }

    /** Releases only wrapper-owned references once, outside the admission/publication lock. */
    private void disposeOwnedResources() {
        Mat ownedInput = stableInput;
        stableInput = null;
        delegate = null;
        calibration = null;
        if (ownedInput != null) {
            ownedInput.release();
        }
    }
}
