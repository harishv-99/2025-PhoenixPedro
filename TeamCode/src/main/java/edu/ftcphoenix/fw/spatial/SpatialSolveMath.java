package edu.ftcphoenix.fw.spatial;

import edu.ftcphoenix.fw.core.geometry.Pose2d;
import edu.ftcphoenix.fw.core.time.LoopTimestamp;

/**
 * Shared spatial relationship math used by query lanes and higher-level consumers.
 */
final class SpatialSolveMath {

    private SpatialSolveMath() {
        // utility holder
    }

    static TranslationSolution translationFromFieldPose(Pose2d fieldToRobot,
                                                        Pose2d robotToTranslationFrame,
                                                        Pose2d fieldToTargetPoint,
                                                        boolean hasRangeInches,
                                                        double rangeInches,
                                                        double quality,
                                                        LoopTimestamp timestamp) {
        Pose2d robotToTargetPoint = fieldToRobot.inverse().then(fieldToTargetPoint);
        return translationFromRobotPoint(robotToTranslationFrame,
                robotToTargetPoint,
                hasRangeInches,
                rangeInches,
                quality,
                timestamp);
    }

    static TranslationSolution translationFromRobotPoint(Pose2d robotToTranslationFrame,
                                                         Pose2d robotToTargetPoint,
                                                         boolean hasRangeInches,
                                                         double rangeInches,
                                                         double quality,
                                                         LoopTimestamp timestamp) {
        Pose2d translationFrameToTargetPoint = robotToTranslationFrame.inverse().then(robotToTargetPoint);
        return new TranslationSolution(robotToTargetPoint,
                translationFrameToTargetPoint,
                hasRangeInches,
                rangeInches,
                quality,
                timestamp);
    }

    static FacingSolution facingFromFieldHeading(Pose2d fieldToRobot,
                                                 Pose2d robotToFacingFrame,
                                                 double desiredFieldHeadingRad,
                                                 double quality,
                                                 LoopTimestamp timestamp) {
        Pose2d fieldToFacingFrame = fieldToRobot.then(robotToFacingFrame);
        return new FacingSolution(Pose2d.wrapToPi(desiredFieldHeadingRad - fieldToFacingFrame.headingRad),
                quality, timestamp);
    }

    static FacingSolution facingFromFieldPoint(Pose2d fieldToRobot,
                                               Pose2d robotToFacingFrame,
                                               Pose2d fieldToFacingPoint,
                                               double quality,
                                               LoopTimestamp timestamp) {
        Pose2d fieldToFacingFrame = fieldToRobot.then(robotToFacingFrame);
        Pose2d facingFrameToPoint = fieldToFacingFrame.inverse().then(fieldToFacingPoint);
        return new FacingSolution(Pose2d.wrapToPi(SpatialMath2d.bearingRadOfVector(
                facingFrameToPoint.xInches,
                facingFrameToPoint.yInches
        )), quality, timestamp);
    }

    static FacingSolution facingFromRobotHeading(Pose2d robotToFacingFrame,
                                                 double desiredRobotHeadingRad,
                                                 double quality,
                                                 LoopTimestamp timestamp) {
        return new FacingSolution(Pose2d.wrapToPi(desiredRobotHeadingRad - robotToFacingFrame.headingRad),
                quality, timestamp);
    }

    static FacingSolution facingFromRobotPoint(Pose2d robotToFacingFrame,
                                               Pose2d robotToTargetPoint,
                                               double quality,
                                               LoopTimestamp timestamp) {
        Pose2d facingFrameToPoint = robotToFacingFrame.inverse().then(robotToTargetPoint);
        return new FacingSolution(Pose2d.wrapToPi(SpatialMath2d.bearingRadOfVector(
                facingFrameToPoint.xInches,
                facingFrameToPoint.yInches
        )), quality, timestamp);
    }
}
