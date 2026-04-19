package edu.ftcphoenix.fw.spatial;

import edu.ftcphoenix.fw.core.geometry.Pose2d;

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
                                                        double ageSec) {
        return translationFromFieldPose(fieldToRobot, robotToTranslationFrame, fieldToTargetPoint,
                hasRangeInches, rangeInches, quality, ageSec, Double.NaN);
    }

    static TranslationSolution translationFromFieldPose(Pose2d fieldToRobot,
                                                        Pose2d robotToTranslationFrame,
                                                        Pose2d fieldToTargetPoint,
                                                        boolean hasRangeInches,
                                                        double rangeInches,
                                                        double quality,
                                                        double ageSec,
                                                        double timestampSec) {
        Pose2d robotToTargetPoint = fieldToRobot.inverse().then(fieldToTargetPoint);
        return translationFromRobotPoint(robotToTranslationFrame,
                robotToTargetPoint,
                hasRangeInches,
                rangeInches,
                quality,
                ageSec,
                timestampSec);
    }

    static TranslationSolution translationFromRobotPoint(Pose2d robotToTranslationFrame,
                                                         Pose2d robotToTargetPoint,
                                                         boolean hasRangeInches,
                                                         double rangeInches,
                                                         double quality,
                                                         double ageSec) {
        return translationFromRobotPoint(robotToTranslationFrame, robotToTargetPoint,
                hasRangeInches, rangeInches, quality, ageSec, Double.NaN);
    }

    static TranslationSolution translationFromRobotPoint(Pose2d robotToTranslationFrame,
                                                         Pose2d robotToTargetPoint,
                                                         boolean hasRangeInches,
                                                         double rangeInches,
                                                         double quality,
                                                         double ageSec,
                                                         double timestampSec) {
        Pose2d translationFrameToTargetPoint = robotToTranslationFrame.inverse().then(robotToTargetPoint);
        return new TranslationSolution(robotToTargetPoint,
                translationFrameToTargetPoint,
                hasRangeInches,
                rangeInches,
                quality,
                ageSec,
                timestampSec);
    }

    static FacingSolution facingFromFieldHeading(Pose2d fieldToRobot,
                                                 Pose2d robotToFacingFrame,
                                                 double desiredFieldHeadingRad,
                                                 double quality,
                                                 double ageSec) {
        return facingFromFieldHeading(fieldToRobot, robotToFacingFrame, desiredFieldHeadingRad,
                quality, ageSec, Double.NaN);
    }

    static FacingSolution facingFromFieldHeading(Pose2d fieldToRobot,
                                                 Pose2d robotToFacingFrame,
                                                 double desiredFieldHeadingRad,
                                                 double quality,
                                                 double ageSec,
                                                 double timestampSec) {
        Pose2d fieldToFacingFrame = fieldToRobot.then(robotToFacingFrame);
        return new FacingSolution(Pose2d.wrapToPi(desiredFieldHeadingRad - fieldToFacingFrame.headingRad),
                quality, ageSec, timestampSec);
    }

    static FacingSolution facingFromFieldPoint(Pose2d fieldToRobot,
                                               Pose2d robotToFacingFrame,
                                               Pose2d fieldToFacingPoint,
                                               double quality,
                                               double ageSec) {
        return facingFromFieldPoint(fieldToRobot, robotToFacingFrame, fieldToFacingPoint,
                quality, ageSec, Double.NaN);
    }

    static FacingSolution facingFromFieldPoint(Pose2d fieldToRobot,
                                               Pose2d robotToFacingFrame,
                                               Pose2d fieldToFacingPoint,
                                               double quality,
                                               double ageSec,
                                               double timestampSec) {
        Pose2d fieldToFacingFrame = fieldToRobot.then(robotToFacingFrame);
        Pose2d facingFrameToPoint = fieldToFacingFrame.inverse().then(fieldToFacingPoint);
        return new FacingSolution(Pose2d.wrapToPi(SpatialMath2d.bearingRadOfVector(
                facingFrameToPoint.xInches,
                facingFrameToPoint.yInches
        )), quality, ageSec, timestampSec);
    }

    static FacingSolution facingFromRobotHeading(Pose2d robotToFacingFrame,
                                                 double desiredRobotHeadingRad,
                                                 double quality,
                                                 double ageSec) {
        return facingFromRobotHeading(robotToFacingFrame, desiredRobotHeadingRad, quality, ageSec, Double.NaN);
    }

    static FacingSolution facingFromRobotHeading(Pose2d robotToFacingFrame,
                                                 double desiredRobotHeadingRad,
                                                 double quality,
                                                 double ageSec,
                                                 double timestampSec) {
        return new FacingSolution(Pose2d.wrapToPi(desiredRobotHeadingRad - robotToFacingFrame.headingRad),
                quality, ageSec, timestampSec);
    }

    static FacingSolution facingFromRobotPoint(Pose2d robotToFacingFrame,
                                               Pose2d robotToTargetPoint,
                                               double quality,
                                               double ageSec) {
        return facingFromRobotPoint(robotToFacingFrame, robotToTargetPoint, quality, ageSec, Double.NaN);
    }

    static FacingSolution facingFromRobotPoint(Pose2d robotToFacingFrame,
                                               Pose2d robotToTargetPoint,
                                               double quality,
                                               double ageSec,
                                               double timestampSec) {
        Pose2d facingFrameToPoint = robotToFacingFrame.inverse().then(robotToTargetPoint);
        return new FacingSolution(Pose2d.wrapToPi(SpatialMath2d.bearingRadOfVector(
                facingFrameToPoint.xInches,
                facingFrameToPoint.yInches
        )), quality, ageSec, timestampSec);
    }
}
