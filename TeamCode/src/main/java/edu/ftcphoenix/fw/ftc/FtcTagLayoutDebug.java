package edu.ftcphoenix.fw.ftc;

import edu.ftcphoenix.fw.core.debug.DebugSink;
import edu.ftcphoenix.fw.field.SubsetTagLayout;
import edu.ftcphoenix.fw.field.TagLayout;

/**
 * FTC-boundary debug helpers for {@link TagLayout} instances.
 *
 * <p>The core {@link TagLayout} contract stays intentionally small: it answers "where is tag X in
 * the field frame?". Maintainers and testers sometimes need richer introspection, though,
 * especially when a season includes detectable tags that are <em>not</em> safe to treat as fixed
 * field landmarks. This helper emits a compact summary without forcing robot code to duplicate that
 * logic.</p>
 *
 * <p>This utility lives in {@code fw.ftc} on purpose because its richest summaries depend on
 * FTC-backed layouts such as {@link FtcGameTagLayout}. That keeps the student-facing field package
 * free of FTC-boundary knowledge.</p>
 */
public final class FtcTagLayoutDebug {

    private FtcTagLayoutDebug() {
        // utility holder
    }

    /**
     * Emits a compact summary of {@code layout} into {@code dbg}.
     *
     * <p>The summary always includes:</p>
     * <ul>
     *   <li>layout type</li>
     *   <li>tag count</li>
     *   <li>included IDs</li>
     * </ul>
     *
     * <p>Known framework layouts add richer information where available. For example,
     * {@link FtcGameTagLayout} also prints the source FTC library IDs and the IDs intentionally
     * excluded from the fixed-field layout.</p>
     */
    public static void dumpSummary(TagLayout layout, DebugSink dbg, String prefix) {
        if (dbg == null) {
            return;
        }
        String p = (prefix == null || prefix.isEmpty()) ? "tagLayout" : prefix;

        if (layout == null) {
            dbg.addData(p + ".present", false);
            return;
        }

        dbg.addData(p + ".present", true)
                .addData(p + ".type", layout.getClass().getSimpleName())
                .addData(p + ".count", layout.ids().size())
                .addData(p + ".ids", layout.ids().toString());

        if (layout instanceof FtcGameTagLayout) {
            dumpFtcGameTagLayoutSummary((FtcGameTagLayout) layout, dbg, p);
            return;
        }

        if (layout instanceof SubsetTagLayout) {
            dumpSubsetSummary((SubsetTagLayout) layout, dbg, p);
        }
    }

    private static void dumpFtcGameTagLayoutSummary(FtcGameTagLayout layout,
                                                    DebugSink dbg,
                                                    String prefix) {
        dbg.addData(prefix + ".sourceDescription", layout.sourceDescription())
                .addData(prefix + ".sourceCount", layout.libraryIds().size())
                .addData(prefix + ".sourceIds", layout.libraryIds().toString())
                .addData(prefix + ".excludedIds", layout.excludedIds().toString())
                .addData(prefix + ".hasOfficialGamePolicy", layout.hasOfficialGamePolicy());

        if (layout.hasOfficialGamePolicy()) {
            dbg.addData(prefix + ".officialGameIds", layout.officialGameIds().toString())
                    .addData(prefix + ".nonFixedGameIds", layout.nonFixedGameIds().toString())
                    .addData(prefix + ".ancillaryLibraryIds", layout.ancillaryLibraryIds().toString());
        }
    }

    private static void dumpSubsetSummary(SubsetTagLayout layout,
                                          DebugSink dbg,
                                          String prefix) {
        dbg.addData(prefix + ".description", layout.description())
                .addData(prefix + ".baseType", layout.base().getClass().getSimpleName())
                .addData(prefix + ".baseCount", layout.base().ids().size());

        dumpSummary(layout.base(), dbg, prefix + ".base");
    }
}
