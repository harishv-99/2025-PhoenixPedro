package edu.ftcsushi.fw.ftc;

import com.qualcomm.hardware.lynx.LynxModule;

/** Package-private SDK seam for deterministic bulk-cache lifecycle tests. */
interface FtcBulkCachingHub {

    LynxModule.BulkCachingMode readMode();

    void setMode(LynxModule.BulkCachingMode mode);

    void clearCache();
}
