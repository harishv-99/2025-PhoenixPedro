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
        Pose2d robotToTargetPoint = fieldToRobot.inverse().then(fieldToTargetPoint);
        return translationFromRobotPoint(robotToTranslationFrame,
                robotToTargetPoint,
                hasRangeInches,
                rangeInches,
                quality,
                ageSec);
    }

    static TranslationSolution translationFromRobotPoint(Pose2d robotToTranslationFrame,
                                                         Pose2d robotToTargetPoint,
                                                         boolean hasRangeInches,
                                                         double rangeInches,
                                                         double quality,
                                                         double ageSec) {
        Pose2d translationFrameToTargetPoint = robotToTranslationFrame.inverse().then(robotToTargetPoint);
        return new TranslationSolution(robotToTargetPoint,
                translationFrameToTargetPoint,
                hasRangeInches,
                rangeInches,
                quality,
                ageSec);
    }

    static AimSolution aimFromFieldHeading(Pose2d fieldToRobot,
                                           Pose2d robotToAimFrame,
                                           double desiredFieldHeadingRad,
                                           double quality,
                                           double ageSec) {
        Pose2d fieldToAimFrame = fieldToRobot.then(robotToAimFrame);
        return new AimSolution(Pose2d.wrapToPi(desiredFieldHeadingRad - fieldToAimFrame.headingRad), quality, ageSec);
    }

    static AimSolution aimFromFieldPoint(Pose2d fieldToRobot,
                                         Pose2d robotToAimFrame,
                                         Pose2d fieldToAimPoint,
                                         double quality,
                                         double ageSec) {
        Pose2d fieldToAimFrame = fieldToRobot.then(robotToAimFrame);
        Pose2d aimFrameToPoint = fieldToAimFrame.inverse().then(fieldToAimPoint);
        return new AimSolution(Pose2d.wrapToPi(SpatialMath2d.bearingRadOfVector(
                aimFrameToPoint.xInches,
                aimFrameToPoint.yInches
        )), quality, ageSec);
    }

    static AimSolution aimFromRobotHeading(Pose2d robotToAimFrame,
                                           double desiredRobotHeadingRad,
                                           double quality,
                                           double ageSec) {
        return new AimSolution(Pose2d.wrapToPi(desiredRobotHeadingRad - robotToAimFrame.headingRad), quality, ageSec);
    }

    static AimSolution aimFromRobotPoint(Pose2d robotToAimFrame,
                                         Pose2d robotToTargetPoint,
                                         double quality,
                                         double ageSec) {
        Pose2d aimFrameToPoint = robotToAimFrame.inverse().then(robotToTargetPoint);
        return new AimSolution(Pose2d.wrapToPi(SpatialMath2d.bearingRadOfVector(
                aimFrameToPoint.xInches,
                aimFrameToPoint.yInches
        )), quality, ageSec);
    }
}
