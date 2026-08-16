package edu.ftcphoenix.fw.ftc.vision;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

/** Owner-private, canonical copy of one FTC AprilTag detector library. */
final class FtcAprilTagLibrarySnapshot {

    private static final double UNIT_QUATERNION_TOLERANCE = 1e-5;

    private final AprilTagLibrary library;
    private final boolean currentGame;
    private final String fieldName;

    private FtcAprilTagLibrarySnapshot(
            AprilTagLibrary library,
            boolean currentGame,
            String fieldName
    ) {
        this.library = library;
        this.currentGame = currentGame;
        this.fieldName = fieldName;
    }

    static FtcAprilTagLibrarySnapshot capture(AprilTagLibrary authored, String fieldName) {
        String checkedField = requireFieldName(fieldName);
        boolean currentGame = authored == null;
        AprilTagLibrary source = currentGame
                ? AprilTagGameDatabase.getCurrentGameTagLibrary()
                : authored;
        return new FtcAprilTagLibrarySnapshot(
                canonicalCopy(source, checkedField),
                currentGame,
                checkedField
        );
    }

    FtcAprilTagLibrarySnapshot recaptured() {
        return new FtcAprilTagLibrarySnapshot(
                canonicalCopy(library, fieldName),
                currentGame,
                fieldName
        );
    }

    AprilTagLibrary freshLibrary() {
        return canonicalCopy(library, fieldName);
    }

    boolean isCurrentGame() {
        return currentGame;
    }

    String provenance() {
        return currentGame ? "currentGame" : "custom";
    }

    private static AprilTagLibrary canonicalCopy(AprilTagLibrary source, String fieldName) {
        AprilTagLibrary checkedSource = Objects.requireNonNull(
                source,
                fieldName + (source == null && fieldName != null
                        ? " resolved to null"
                        : " must not be null")
        );
        AprilTagMetadata[] returned = checkedSource.getAllTags();
        if (returned == null) {
            throw new IllegalArgumentException(fieldName + ".getAllTags() must not return null");
        }
        AprilTagMetadata[] metadata = returned.clone();
        if (metadata.length == 0) {
            throw new IllegalArgumentException(fieldName + " must contain at least one tag");
        }

        Map<Integer, Integer> firstIndexById = new HashMap<Integer, Integer>();
        AprilTagLibrary.Builder builder = new AprilTagLibrary.Builder();
        for (int index = 0; index < metadata.length; index++) {
            String entryName = fieldName + ".metadata[" + index + "]";
            AprilTagMetadata entry = metadata[index];
            if (entry == null) {
                throw new IllegalArgumentException(entryName + " must not be null");
            }
            if (entry.id < 0) {
                throw new IllegalArgumentException(
                        entryName + ".id must be non-negative, got " + entry.id);
            }
            Integer firstIndex = firstIndexById.put(entry.id, index);
            if (firstIndex != null) {
                throw new IllegalArgumentException(
                        fieldName + " contains duplicate id " + entry.id
                                + " at metadata[" + firstIndex + "] and metadata[" + index + "]");
            }

            DistanceUnit unit = entry.distanceUnit;
            if (unit == null) {
                throw new IllegalArgumentException(entryName + ".distanceUnit must not be null");
            }
            double sizeInches = unit.toInches(entry.tagsize);
            if (!Double.isFinite(entry.tagsize)
                    || entry.tagsize == DistanceUnit.infinity
                    || entry.tagsize <= 0.0
                    || !Double.isFinite(sizeInches) || sizeInches <= 0.0) {
                throw new IllegalArgumentException(
                        entryName + ".tagsize must convert to finite inches and be > 0, got "
                                + entry.tagsize + " " + unit);
            }

            VectorF sourcePosition = entry.fieldPosition;
            if (sourcePosition == null) {
                throw new IllegalArgumentException(entryName + ".fieldPosition must not be null");
            }
            int positionLength;
            try {
                positionLength = sourcePosition.length();
            } catch (RuntimeException failure) {
                throw new IllegalArgumentException(
                        entryName + ".fieldPosition length could not be read", failure);
            }
            if (positionLength != 3) {
                throw new IllegalArgumentException(
                        entryName + ".fieldPosition must have exactly three components, got "
                                + positionLength);
            }
            float[] positionInches = new float[3];
            for (int component = 0; component < 3; component++) {
                double raw;
                try {
                    raw = sourcePosition.get(component);
                } catch (RuntimeException failure) {
                    throw new IllegalArgumentException(
                            entryName + ".fieldPosition[" + component + "] could not be read",
                            failure);
                }
                double converted = unit.toInches(raw);
                float represented = (float) converted;
                if (!Double.isFinite(raw) || !Double.isFinite(converted)
                        || !Float.isFinite(represented)) {
                    throw new IllegalArgumentException(
                            entryName + ".fieldPosition[" + component
                                    + "] must convert to a finite float number of inches, got "
                                    + raw + " " + unit);
                }
                positionInches[component] = represented;
            }

            Quaternion sourceOrientation = entry.fieldOrientation;
            if (sourceOrientation == null) {
                throw new IllegalArgumentException(entryName + ".fieldOrientation must not be null");
            }
            requireFinite(entryName + ".fieldOrientation.w", sourceOrientation.w);
            requireFinite(entryName + ".fieldOrientation.x", sourceOrientation.x);
            requireFinite(entryName + ".fieldOrientation.y", sourceOrientation.y);
            requireFinite(entryName + ".fieldOrientation.z", sourceOrientation.z);
            double magnitude = stableMagnitude(sourceOrientation);
            if (!Double.isFinite(magnitude)
                    || Math.abs(magnitude - 1.0) > UNIT_QUATERNION_TOLERANCE) {
                throw new IllegalArgumentException(
                        entryName + ".fieldOrientation must be normalized within 1e-5 of unit "
                                + "magnitude; normalize the authored quaternion (magnitude="
                                + magnitude + ")");
            }

            Quaternion orientation = new Quaternion(
                    sourceOrientation.w,
                    sourceOrientation.x,
                    sourceOrientation.y,
                    sourceOrientation.z,
                    sourceOrientation.acquisitionTime
            );
            builder.addTag(new AprilTagMetadata(
                    entry.id,
                    entry.name,
                    sizeInches,
                    new VectorF(positionInches),
                    DistanceUnit.INCH,
                    orientation
            ));
        }
        return builder.build();
    }

    private static double stableMagnitude(Quaternion value) {
        double max = Math.max(
                Math.max(Math.abs((double) value.w), Math.abs((double) value.x)),
                Math.max(Math.abs((double) value.y), Math.abs((double) value.z))
        );
        if (max == 0.0) {
            return 0.0;
        }
        double w = value.w / max;
        double x = value.x / max;
        double y = value.y / max;
        double z = value.z / max;
        return max * Math.sqrt(w * w + x * x + y * y + z * z);
    }

    private static void requireFinite(String fieldName, float value) {
        if (!Float.isFinite(value)) {
            throw new IllegalArgumentException(fieldName + " must be finite, got " + value);
        }
    }

    private static String requireFieldName(String fieldName) {
        String checked = Objects.requireNonNull(fieldName, "fieldName").trim();
        if (checked.isEmpty()) {
            throw new IllegalArgumentException("fieldName must not be blank");
        }
        return checked;
    }
}
