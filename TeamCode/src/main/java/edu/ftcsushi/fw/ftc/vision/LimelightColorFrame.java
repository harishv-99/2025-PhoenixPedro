package edu.ftcsushi.fw.ftc.vision;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.JsonPrimitive;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.ftcsushi.fw.core.geometry.Vec3;

/**
 * Immutable, bounded color geometry copied while the Limelight owner acquires one SDK result.
 * No SDK object, JSON tree, mutable corner list, pixel convention, or camera lifecycle escapes.
 */
final class LimelightColorFrame {

    static final int MAX_CANDIDATES = 64;
    private static final int MAX_JSON_CHARACTERS = 262144;

    private final boolean available;
    private final String reason;
    private final List<Vec3> rays;

    private LimelightColorFrame(boolean available, String reason, List<Vec3> rays) {
        this.available = available;
        this.reason = reason;
        this.rays = Collections.unmodifiableList(new ArrayList<>(rays));
    }

    /**
     * Copies the documented {@code LLResult.toString()} JSON representation, whose root is the
     * result object and whose color candidates are {@code Retro}. Missing fields are unavailable,
     * not zero. A malformed candidate rejects the whole frame instead of hiding ambiguity by
     * dropping that candidate. Candidate ordering is the vendor's ordering, not tracking identity.
     */
    static LimelightColorFrame parse(String json) {
        if (json == null || json.length() == 0 || json.length() > MAX_JSON_CHARACTERS
                || exceedsNestingBound(json)) {
            return unavailable("Limelight color JSON is absent or exceeds bounded size/nesting");
        }
        try {
            JsonElement parsed = new JsonParser().parse(json);
            if (!parsed.isJsonObject()) {
                return unavailable("Limelight color JSON must contain one result object");
            }
            JsonObject result = parsed.getAsJsonObject();
            JsonElement pipelineType = result.get("pTYPE");
            if (pipelineType == null || !pipelineType.isJsonPrimitive()
                    || !pipelineType.getAsJsonPrimitive().isString()
                    || !"color".equalsIgnoreCase(pipelineType.getAsString())) {
                return unavailable("Limelight result must explicitly report pTYPE=color");
            }
            JsonElement candidates = result.get("Retro");
            if (candidates == null || !candidates.isJsonArray()) {
                return unavailable("Limelight color result must contain the Retro array; absent is not empty");
            }
            JsonArray array = candidates.getAsJsonArray();
            if (array.size() > MAX_CANDIDATES) {
                return unavailable("Limelight color candidate count exceeds " + MAX_CANDIDATES);
            }
            List<Vec3> rays = new ArrayList<>(array.size());
            for (int i = 0; i < array.size(); i++) {
                JsonElement candidate = array.get(i);
                if (!candidate.isJsonObject()) {
                    return unavailable("Limelight color candidate " + i + " is not an object");
                }
                double rightDeg = requiredAngle(candidate.getAsJsonObject(), "tx_nocross");
                double upDeg = requiredAngle(candidate.getAsJsonObject(), "ty_nocross");
                if (!Double.isFinite(rightDeg) || !Double.isFinite(upDeg)) {
                    return unavailable("Limelight color candidate " + i
                            + " requires finite numeric tx_nocross and ty_nocross within (-90, 90) degrees");
                }
                // Independent image-plane angles: horizontal right/up become camera left/up.
                // Neither angle is a spherical azimuth/elevation pair or robot-relative bearing.
                rays.add(new Vec3(1.0, -Math.tan(Math.toRadians(rightDeg)),
                        Math.tan(Math.toRadians(upDeg))));
            }
            return new LimelightColorFrame(true, "", rays);
        } catch (RuntimeException invalidJson) {
            return unavailable("Limelight color JSON cannot be decoded: "
                    + invalidJson.getClass().getSimpleName());
        }
    }

    /** Bounds the recursive vendor JSON parser, ignoring brackets escaped inside JSON strings. */
    private static boolean exceedsNestingBound(String json) {
        int depth = 0;
        boolean inString = false;
        boolean escaped = false;
        for (int i = 0; i < json.length(); i++) {
            char character = json.charAt(i);
            if (inString) {
                if (escaped) escaped = false;
                else if (character == '\\') escaped = true;
                else if (character == '"') inString = false;
            } else if (character == '"') {
                inString = true;
            } else if (character == '[' || character == '{') {
                if (++depth > 64) return true;
            } else if (character == ']' || character == '}') {
                depth--;
                if (depth < 0) return true;
            }
        }
        return false;
    }

    /** Returns an explicit missing/invalid frame, never an observed empty frame. */
    static LimelightColorFrame unavailable(String reason) {
        return new LimelightColorFrame(false, reason, Collections.emptyList());
    }

    /** Requires a JSON number, rather than accepting strings, null, booleans, or getter defaults. */
    private static double requiredAngle(JsonObject candidate, String key) {
        JsonElement field = candidate.get(key);
        if (field == null || !field.isJsonPrimitive()) {
            return Double.NaN;
        }
        JsonPrimitive primitive = field.getAsJsonPrimitive();
        if (!primitive.isNumber()) {
            return Double.NaN;
        }
        double value = primitive.getAsDouble();
        return Double.isFinite(value) && value > -90.0 && value < 90.0
                ? value : Double.NaN;
    }

    boolean isAvailable() {
        return available;
    }

    String reason() {
        return reason;
    }

    /** Unmodifiable list of immutable camera-forward/left/up direction vectors. */
    List<Vec3> rays() {
        return rays;
    }
}
