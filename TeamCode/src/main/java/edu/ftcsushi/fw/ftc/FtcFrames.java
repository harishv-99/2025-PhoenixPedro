package edu.ftcsushi.fw.ftc;

import edu.ftcsushi.fw.core.geometry.Mat3;
import edu.ftcsushi.fw.core.geometry.Pose3d;
import edu.ftcsushi.fw.core.geometry.Vec3;

/**
 * Coordinate-frame conversion utilities for the FTC SDK.
 *
 * <h2>Design principle</h2>
 * <p>
 * Core Sushi framework code (outside {@code edu.ftcsushi.fw.ftc}) operates only in the
 * Sushi pose convention:
 * </p>
 * <ul>
 *   <li><b>+X</b> forward</li>
 *   <li><b>+Y</b> left</li>
 *   <li><b>+Z</b> up</li>
 * </ul>
 *
 * <p>
 * The FTC SDK uses multiple reference frames depending on the feature and API.
 * Sushi does not attempt to redefine those SDK frames globally. Instead, Sushi converts
 * SDK-provided poses <b>at the adapter boundary</b> so the rest of the framework remains consistent.
 * </p>
 *
 * <h2>FTC frames referenced here</h2>
 *
 * <h3>FTC AprilTag detection pose reference frame (VisionPortal / AprilTagProcessor detection values)</h3>
 * <p>
 * FTC docs describe detection pose values using:
 * </p>
 * <ul>
 *   <li><b>+X</b> right</li>
 *   <li><b>+Y</b> forward (outward from the camera lens)</li>
 *   <li><b>+Z</b> up</li>
 * </ul>
 *
 * <h3>FTC AprilTag Localization optical-camera axes</h3>
 * <p>
 * The AprilTag Localization tutorial defines these physical camera axes:
 * </p>
 * <ul>
 *   <li><b>+x</b> right</li>
 *   <li><b>+y</b> down</li>
 *   <li><b>+z</b> forward (from the camera’s perspective)</li>
 * </ul>
 * <p>The {@code AprilTagProcessor.Builder.setCameraPose(...)} position is <em>not</em> expressed
 * in those camera axes. Its position uses the FTC robot axes below, while its orientation rotates
 * these optical-camera axes into the robot frame. The webcam owner performs that two-sided
 * conversion explicitly; a simple one-frame basis change is not sufficient.</p>
 *
 * <h3>FTC robot axes (as described in the localization tutorial)</h3>
 * <ul>
 *   <li><b>+x</b> right</li>
 *   <li><b>+y</b> forward</li>
 *   <li><b>+z</b> up</li>
 * </ul>
 *
 * <h2>Rotation convention and an explicit assumption</h2>
 * <p>
 * FTC documentation states that Pitch, Roll, and Yaw are rotations about the X, Y, and Z axes
 * (right-hand rule). However, FTC docs do not precisely specify a composition order (e.g. ZYX vs XYZ).
 * </p>
 * <p>
 * Sushi {@link Pose3d} uses the {@link Mat3#fromYawPitchRoll(double, double, double)} mapping
 * defined by Sushi (see {@link Pose3d} Javadoc). This class assumes the incoming angles represent a
 * rotation that is compatible with that mapping when converting to/from matrices.
 * </p>
 *
 * <p><b>AprilTagDetection.ftcPose gotcha:</b>
 * The FTC SDK reports {@code pitch} as rotation about <b>+X</b> and {@code roll} as rotation about
 * <b>+Y</b>. {@code AprilTagProcessor.Builder.setOutputUnits(...)} controls their units; Sushi
 * configures that processor for radians. Sushi {@link Pose3d} also uses radians, but uses the more
 * common naming where roll is about +X and pitch is about +Y. Therefore, when converting the
 * configured processor's {@code det.ftcPose} into a {@link Pose3d}, preserve the angle values and
 * swap the pitch/roll fields (FTC pitch→Sushi roll, FTC roll→Sushi pitch). The webcam
 * AprilTag adapter owned by {@code FtcWebcamAprilTagVisionLane} applies the canonical conversion
 * before observations cross the FTC boundary.
 * </p>
 * <h2>How to use</h2>
 * <p>
 * Use these functions only inside the FTC adapter layer. Store transforms with explicit
 * from/to names such as {@code cameraToTagPose} or {@code robotToCameraPose}.
 * </p>
 */
public final class FtcFrames {

    private FtcFrames() {
        // static utility
    }

    // ---------------------------------------------------------------------------------------------
    // Sushi <-> FTC AprilTag detection pose reference frame (X right, Y forward, Z up)
    // ---------------------------------------------------------------------------------------------

    /**
     * Convert a pose expressed in the FTC AprilTag detection reference frame to Sushi framing.
     *
     * <p><b>FTC detection frame:</b> +X right, +Y forward (out of lens), +Z up.</p>
     * <p><b>Sushi frame:</b> +X forward, +Y left, +Z up.</p>
     *
     * @param ftcXRightYForwardZUpPose pose expressed in FTC detection frame axes
     * @return same physical pose, expressed in Sushi framing
     */
    public static Pose3d toSushiFromFtcDetectionFrame(Pose3d ftcXRightYForwardZUpPose) {
        return changeBasis(ftcXRightYForwardZUpPose, Basis.FTC_XRIGHT_YFWD_ZUP_TO_SUSHI);
    }

    /**
     * Convert a pose expressed in Sushi framing to the FTC AprilTag detection reference frame.
     *
     * @param sushiPose pose expressed in Sushi framing
     * @return same physical pose, expressed in FTC detection frame axes (+X right, +Y forward, +Z up)
     */
    public static Pose3d toFtcDetectionFrameFromSushi(Pose3d sushiPose) {
        return changeBasis(sushiPose, Basis.SUSHI_TO_FTC_XRIGHT_YFWD_ZUP);
    }

    // ---------------------------------------------------------------------------------------------
    // Sushi <-> FTC AprilTag Localization camera axes (X right, Y down, Z forward)
    // ---------------------------------------------------------------------------------------------

    /**
     * Convert a pose expressed in the FTC AprilTag Localization camera axes to Sushi framing.
     *
     * <p><b>FTC localization camera axes:</b> +X right, +Y down, +Z forward.</p>
     * <p><b>Sushi frame:</b> +X forward, +Y left, +Z up.</p>
     *
     * @param ftcXRightYDownZForwardPose pose expressed in FTC localization camera axes
     * @return same physical pose, expressed in Sushi framing
     */
    public static Pose3d toSushiFromFtcLocalizationCameraAxes(Pose3d ftcXRightYDownZForwardPose) {
        return changeBasis(ftcXRightYDownZForwardPose, Basis.FTC_LOC_CAM_TO_SUSHI);
    }

    /**
     * Convert a pose expressed in Sushi framing to the FTC AprilTag Localization camera axes.
     *
     * @param sushiPose pose expressed in Sushi framing
     * @return same physical pose, expressed in FTC localization camera axes (+X right, +Y down, +Z forward)
     */
    public static Pose3d toFtcLocalizationCameraAxesFromSushi(Pose3d sushiPose) {
        return changeBasis(sushiPose, Basis.SUSHI_TO_FTC_LOC_CAM);
    }

    // ---------------------------------------------------------------------------------------------
    // Sushi <-> FTC robot axes (as described in localization tutorial: X right, Y forward, Z up)
    // ---------------------------------------------------------------------------------------------

    /**
     * Convert a pose expressed in FTC robot axes (+X right, +Y forward, +Z up) to Sushi framing.
     *
     * <p>Note: This is the same axis convention as the FTC AprilTag detection reference frame.</p>
     *
     * @param ftcXRightYForwardZUpPose pose expressed in FTC robot axes
     * @return same physical pose, expressed in Sushi framing
     */
    public static Pose3d toSushiFromFtcRobotAxes(Pose3d ftcXRightYForwardZUpPose) {
        return changeBasis(ftcXRightYForwardZUpPose, Basis.FTC_XRIGHT_YFWD_ZUP_TO_SUSHI);
    }

    /**
     * Convert a pose expressed in Sushi framing to FTC robot axes (+X right, +Y forward, +Z up).
     *
     * @param sushiPose pose expressed in Sushi framing
     * @return same physical pose, expressed in FTC robot axes
     */
    public static Pose3d toFtcRobotAxesFromSushi(Pose3d sushiPose) {
        return changeBasis(sushiPose, Basis.SUSHI_TO_FTC_XRIGHT_YFWD_ZUP);
    }

    // ---------------------------------------------------------------------------------------------
    // Implementation
    // ---------------------------------------------------------------------------------------------

    private static Pose3d changeBasis(Pose3d in, Basis basis) {
        if (in == null) {
            return null;
        }

        // Translation: t_out = M * t_in
        Vec3 tIn = new Vec3(in.xInches, in.yInches, in.zInches);
        Vec3 tOut = basis.M.mul(tIn);

        // Rotation: R_out = M * R_in * M^-1
        Mat3 rIn = Mat3.fromYawPitchRoll(in.yawRad, in.pitchRad, in.rollRad);
        Mat3 rOut = basis.M.mul(rIn).mul(basis.Minv);
        Mat3.YawPitchRoll yprOut = Mat3.toYawPitchRoll(rOut);

        return new Pose3d(tOut.x, tOut.y, tOut.z, yprOut.yawRad, yprOut.pitchRad, yprOut.rollRad);
    }

    /**
     * Fixed basis changes used by the FTC adapters.
     *
     * <p>All of these are pure axis/basis changes (orthonormal matrices), so inverse = transpose.</p>
     */
    private enum Basis {

        /**
         * FTC (X right, Y forward, Z up) -> Sushi (X forward, Y left, Z up).
         *
         * <p>Mapping:</p>
         * <ul>
         *   <li>sushiX = ftcY</li>
         *   <li>sushiY = -ftcX</li>
         *   <li>sushiZ = ftcZ</li>
         * </ul>
         */
        FTC_XRIGHT_YFWD_ZUP_TO_SUSHI(new Mat3(
                0, 1, 0,
                -1, 0, 0,
                0, 0, 1
        )),

        /**
         * Sushi (X forward, Y left, Z up) -> FTC (X right, Y forward, Z up).
         */
        SUSHI_TO_FTC_XRIGHT_YFWD_ZUP(FTC_XRIGHT_YFWD_ZUP_TO_SUSHI.M.transpose()),

        /**
         * FTC localization camera axes (X right, Y down, Z forward) -> Sushi (X forward, Y left, Z up).
         *
         * <p>Mapping:</p>
         * <ul>
         *   <li>sushiX = ftcZ</li>
         *   <li>sushiY = -ftcX</li>
         *   <li>sushiZ = -ftcY</li>
         * </ul>
         */
        FTC_LOC_CAM_TO_SUSHI(new Mat3(
                0, 0, 1,
                -1, 0, 0,
                0, -1, 0
        )),

        /**
         * Sushi (X forward, Y left, Z up) -> FTC localization camera axes (X right, Y down, Z forward).
         */
        SUSHI_TO_FTC_LOC_CAM(FTC_LOC_CAM_TO_SUSHI.M.transpose());

        final Mat3 M;
        final Mat3 Minv;

        Basis(Mat3 M) {
            this.M = M;
            this.Minv = M.transpose();
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Public basis matrices (for adapters)
    // ---------------------------------------------------------------------------------------------

    /**
     * Rotation matrix that converts vectors expressed in the <b>FTC Localization / AprilTag raw
     * camera frame</b> into Sushi camera coordinates.
     *
     * <p><b>FTC/AprilTag raw camera axes</b> (OpenCV / AprilRobotics):
     * <ul>
     *   <li>+X: right</li>
     *   <li>+Y: down</li>
     *   <li>+Z: forward (out of the camera lens)</li>
     * </ul>
     *
     * <p><b>Sushi camera axes</b>:
     * <ul>
     *   <li>+X: forward</li>
     *   <li>+Y: left</li>
     *   <li>+Z: up</li>
     * </ul>
     */
    public static Mat3 sushiFromAprilTagRawCameraFrame() {
        return Basis.FTC_LOC_CAM_TO_SUSHI.M;
    }

    /**
     * Inverse of {@link #sushiFromAprilTagRawCameraFrame()}.
     */
    public static Mat3 aprilTagRawCameraFromSushiFrame() {
        return Basis.FTC_LOC_CAM_TO_SUSHI.Minv;
    }

    /**
     * Rotation matrix that converts vectors expressed in the FTC "ftcPose" AprilTag reference
     * frame (+X right, +Y forward, +Z up) into Sushi camera coordinates.
     */
    public static Mat3 sushiFromFtcDetectionFrame() {
        // FTC AprilTag detection frame uses the same axes as FTC robot axes:
        // +X right, +Y forward, +Z up.
        return Basis.FTC_XRIGHT_YFWD_ZUP_TO_SUSHI.M;
    }

    /**
     * Inverse of {@link #sushiFromFtcDetectionFrame()}.
     */
    public static Mat3 ftcDetectionFromSushiFrame() {
        return Basis.FTC_XRIGHT_YFWD_ZUP_TO_SUSHI.Minv;
    }
}
