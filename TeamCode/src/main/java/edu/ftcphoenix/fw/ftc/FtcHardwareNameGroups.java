package edu.ftcphoenix.fw.ftc;

import java.util.List;
import java.util.Objects;

/**
 * Keeps FTC SDK hardware-name identity checks consistent across private grouped owners.
 *
 * <p>The FTC SDK trims a configured name before its case-sensitive lookup. Group validation uses
 * that same key so two group members cannot resolve the same configured entry merely because one
 * spelling includes surrounding whitespace. This helper validates configuration strings only; it
 * does not prove physical device identity or track ownership across separately constructed
 * objects.</p>
 */
final class FtcHardwareNameGroups {

    private FtcHardwareNameGroups() {
        // package-private utility
    }

    /**
     * Require one name to be nonblank and distinct from earlier members of the same command group.
     *
     * <p>The caller owns null/direction precedence and adds {@code rawName} to
     * {@code earlierRawNames} only after this method returns. Rejection therefore cannot mutate the
     * caller's group.</p>
     */
    static void requireNewMember(String owner,
                                 String memberLabel,
                                 String rawName,
                                 List<String> earlierRawNames) {
        Objects.requireNonNull(owner, "owner");
        Objects.requireNonNull(memberLabel, "memberLabel");
        Objects.requireNonNull(earlierRawNames, "earlierRawNames");
        if (rawName == null) {
            throw new IllegalArgumentException(
                    owner + " " + memberLabel
                            + " requires a configured hardware name");
        }

        String effectiveName = sdkIdentityKey(rawName);
        if (effectiveName.isEmpty()) {
            throw new IllegalArgumentException(
                    owner + " " + memberLabel + " hardware name is blank; "
                            + "use a non-blank FTC Robot Configuration name");
        }

        for (int i = 0; i < earlierRawNames.size(); i++) {
            String earlierRawName = earlierRawNames.get(i);
            if (effectiveName.equals(sdkIdentityKey(earlierRawName))) {
                throw new IllegalArgumentException(
                        owner + " " + memberLabel + " hardware name \""
                                + rawName + "\" resolves to FTC name \""
                                + effectiveName + "\", already used by member "
                                + (i + 1) + " (\"" + earlierRawName + "\"); "
                                + "use a distinct configured hardware name");
            }
        }
    }

    /**
     * Return whether two non-null strings select the same case-sensitive FTC configured name.
     */
    static boolean sameConfiguredName(String firstRawName, String secondRawName) {
        return sdkIdentityKey(firstRawName).equals(sdkIdentityKey(secondRawName));
    }

    /**
     * Return the FTC SDK-equivalent identity key while preserving the caller's raw lookup string.
     */
    private static String sdkIdentityKey(String rawName) {
        return Objects.requireNonNull(rawName, "rawName").trim();
    }
}
