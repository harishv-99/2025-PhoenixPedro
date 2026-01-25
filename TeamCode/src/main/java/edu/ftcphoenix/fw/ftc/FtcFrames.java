package edu.ftcphoenix.fw.ftc;

import edu.ftcphoenix.fw.core.geometry.Mat3;
import edu.ftcphoenix.fw.core.geometry.Pose3d;
import edu.ftcphoenix.fw.core.geometry.Vec3;

/**
 * Coordinate-frame conversion utilities for the FTC SDK.
 *
 * <h2>Design principle</h2>
 * <p>
 * Core Phoenix framework code (outside {@code edu.ftcphoenix.fw.adapters.ftc}) operates only in the
 * Phoenix pose convention:
 * </p>
 * <ul>
 *   <li><b>+X</b> forward</li>
 *   <li><b>+Y</b> left</li>
 *   <li><b>+Z</b> up</li>
 * </ul>
 *
 * <p>
 * The FTC SDK uses multiple reference frames depending on the feature and API.
 * Phoenix does not attempt to redefine those SDK frames globally. Instead, Phoenix converts
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
 * <h3>FTC AprilTag Localization camera axes (used for setCameraPose in the localization sample)</h3>
 * <p>
 * The AprilTag Localization tutorial defines a camera-axes convention for specifying camera placement:
 * </p>
 * <ul>
 *   <li><b>+x</b> right</li>
 *   <li><b>+y</b> down</li>
 *   <li><b>+z</b> forward (from the camera’s perspective)</li>
 * </ul>
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
 * Phoenix {@link Pose3d} uses the {@link Mat3#fromYawPitchRoll(double, double, double)} mapping
 * defined by Phoenix (see {@link Pose3d} Javadoc). This class assumes the incoming angles represent a
 * rotation that is compatible with that mapping when converting to/from matrices.
 * </p>
 *
 * <p><b>AprilTagDetection.ftcPose gotcha:</b>
 * The FTC SDK reports {@code pitch} as rotation about <b>+X</b> and {@code roll} as rotation about <b>+Y</b>,
 * and the sample OpModes treat these values as <b>degrees</b>. Phoenix {@link Pose3d} uses the more common
 * naming where roll is about +X and pitch is about +Y, and Phoenix angles are in <b>radians</b>.
 * Therefore, when converting {@code det.ftcPose} into a {@link Pose3d}, convert degrees→radians and swap
 * the pitch/roll fields (FTC pitch→Phoenix roll, FTC roll→Phoenix pitch). See {@link FtcVision} for the
 * canonical conversion.
 * </p>
 * <h2>How to use</h2>
 * <p>
 * Use these functions only inside the FTC adapter layer. Store outputs using Phoenix naming:
 * {@code pFromToTo}, for example {@code cameraToTagPose} or {@code robotToCameraPose}.
 * </p>
 */
public final class FtcFrames {

    private FtcFrames() {
        // static utility
    }

    // ---------------------------------------------------------------------------------------------
    // Phoenix <-> FTC AprilTag detection pose reference frame (X right, Y forward, Z up)
    // ---------------------------------------------------------------------------------------------

    /**
     * Convert a pose expressed in the FTC AprilTag detection reference frame to Phoenix framing.
     *
     * <p><b>FTC detection frame:</b> +X right, +Y forward (out of lens), +Z up.</p>
     * <p><b>Phoenix frame:</b> +X forward, +Y left, +Z up.</p>
     *
     * @param ftcXRightYForwardZUpPose pose expressed in FTC detection frame axes
     * @return same physical pose, expressed in Phoenix framing
     */
    public static Pose3d toPhoenixFromFtcDetectionFrame(Pose3d ftcXRightYForwardZUpPose) {
        return changeBasis(ftcXRightYForwardZUpPose, Basis.FTC_XRIGHT_YFWD_ZUP_TO_P);
    }

    /**
     * Convert a pose expressed in Phoenix framing to the FTC AprilTag detection reference frame.
     *
     * @param pPose pose expressed in Phoenix framing
     * @return same physical pose, expressed in FTC detection frame axes (+X right, +Y forward, +Z up)
     */
    public static Pose3d toFtcDetectionFrameFromPhoenix(Pose3d pPose) {
        return changeBasis(pPose, Basis.P_TO_FTC_XRIGHT_YFWD_ZUP);
    }

    // ---------------------------------------------------------------------------------------------
    // Phoenix <-> FTC AprilTag Localization camera axes (X right, Y down, Z forward)
    // ---------------------------------------------------------------------------------------------

    /**
     * Convert a pose expressed in the FTC AprilTag Localization camera axes to Phoenix framing.
     *
     * <p><b>FTC localization camera axes:</b> +X right, +Y down, +Z forward.</p>
     * <p><b>Phoenix frame:</b> +X forward, +Y left, +Z up.</p>
     *
     * @param ftcXRightYDownZForwardPose pose expressed in FTC localization camera axes
     * @return same physical pose, expressed in Phoenix framing
     */
    public static Pose3d toPhoenixFromFtcLocalizationCameraAxes(Pose3d ftcXRightYDownZForwardPose) {
        return changeBasis(ftcXRightYDownZForwardPose, Basis.FTC_LOC_CAM_TO_P);
    }

    /**
     * Convert a pose expressed in Phoenix framing to the FTC AprilTag Localization camera axes.
     *
     * @param pPose pose expressed in Phoenix framing
     * @return same physical pose, expressed in FTC localization camera axes (+X right, +Y down, +Z forward)
     */
    public static Pose3d toFtcLocalizationCameraAxesFromPhoenix(Pose3d pPose) {
        return changeBasis(pPose, Basis.P_TO_FTC_LOC_CAM);
    }

    // ---------------------------------------------------------------------------------------------
    // Phoenix <-> FTC robot axes (as described in localization tutorial: X right, Y forward, Z up)
    // ---------------------------------------------------------------------------------------------

    /**
     * Convert a pose expressed in FTC robot axes (+X right, +Y forward, +Z up) to Phoenix framing.
     *
     * <p>Note: This is the same axis convention as the FTC AprilTag detection reference frame.</p>
     *
     * @param ftcXRightYForwardZUpPose pose expressed in FTC robot axes
     * @return same physical pose, expressed in Phoenix framing
     */
    public static Pose3d toPhoenixFromFtcRobotAxes(Pose3d ftcXRightYForwardZUpPose) {
        return changeBasis(ftcXRightYForwardZUpPose, Basis.FTC_XRIGHT_YFWD_ZUP_TO_P);
    }

    /**
     * Convert a pose expressed in Phoenix framing to FTC robot axes (+X right, +Y forward, +Z up).
     *
     * @param pPose pose expressed in Phoenix framing
     * @return same physical pose, expressed in FTC robot axes
     */
    public static Pose3d toFtcRobotAxesFromPhoenix(Pose3d pPose) {
        return changeBasis(pPose, Basis.P_TO_FTC_XRIGHT_YFWD_ZUP);
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
         * FTC (X right, Y forward, Z up) -> Phoenix (X forward, Y left, Z up).
         *
         * <p>Mapping:</p>
         * <ul>
         *   <li>pX = ftcY</li>
         *   <li>pY = -ftcX</li>
         *   <li>pZ = ftcZ</li>
         * </ul>
         */
        FTC_XRIGHT_YFWD_ZUP_TO_P(new Mat3(
                0, 1, 0,
                -1, 0, 0,
                0, 0, 1
        )),

        /**
         * Phoenix (X forward, Y left, Z up) -> FTC (X right, Y forward, Z up).
         */
        P_TO_FTC_XRIGHT_YFWD_ZUP(FTC_XRIGHT_YFWD_ZUP_TO_P.M.transpose()),

        /**
         * FTC localization camera axes (X right, Y down, Z forward) -> Phoenix (X forward, Y left, Z up).
         *
         * <p>Mapping:</p>
         * <ul>
         *   <li>pX = ftcZ</li>
         *   <li>pY = -ftcX</li>
         *   <li>pZ = -ftcY</li>
         * </ul>
         */
        FTC_LOC_CAM_TO_P(new Mat3(
                0, 0, 1,
                -1, 0, 0,
                0, -1, 0
        )),

        /**
         * Phoenix (X forward, Y left, Z up) -> FTC localization camera axes (X right, Y down, Z forward).
         */
        P_TO_FTC_LOC_CAM(FTC_LOC_CAM_TO_P.M.transpose());

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
     * camera frame</b> into Phoenix camera coordinates.
     *
     * <p><b>FTC/AprilTag raw camera axes</b> (OpenCV / AprilRobotics):
     * <ul>
     *   <li>+X: right</li>
     *   <li>+Y: down</li>
     *   <li>+Z: forward (out of the camera lens)</li>
     * </ul>
     *
     * <p><b>Phoenix camera axes</b>:
     * <ul>
     *   <li>+X: forward</li>
     *   <li>+Y: left</li>
     *   <li>+Z: up</li>
     * </ul>
     */
    public static Mat3 phoenixFromAprilTagRawCameraFrame() {
        return Basis.FTC_LOC_CAM_TO_P.M;
    }

    /**
     * Inverse of {@link #phoenixFromAprilTagRawCameraFrame()}.
     */
    public static Mat3 aprilTagRawCameraFromPhoenixFrame() {
        return Basis.FTC_LOC_CAM_TO_P.Minv;
    }

    /**
     * Rotation matrix that converts vectors expressed in the FTC "ftcPose" AprilTag reference
     * frame (+X right, +Y forward, +Z up) into Phoenix camera coordinates.
     */
    public static Mat3 phoenixFromFtcDetectionFrame() {
        // FTC AprilTag detection frame uses the same axes as FTC robot axes:
        // +X right, +Y forward, +Z up.
        return Basis.FTC_XRIGHT_YFWD_ZUP_TO_P.M;
    }

    /**
     * Inverse of {@link #phoenixFromFtcDetectionFrame()}.
     */
    public static Mat3 ftcDetectionFromPhoenixFrame() {
        return Basis.FTC_XRIGHT_YFWD_ZUP_TO_P.Minv;
    }
}
