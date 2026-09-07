package edu.ftcsushi.fw.ftc.vision;

import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

/** Native-free lifecycle checks of the production callback admission/disposal gate. */
public final class FtcColorBlobProcessorTest {

    @Test
    public void constructionAndTerminalizationRequireNoNativeLibrary() {
        FtcColorBlobProcessor processor = new FtcColorBlobProcessor(
                FtcFloorObjectVision.Config.defaults());
        assertFalse(processor.latest().isAvailable());
        processor.terminalize();
        processor.terminalize();
        assertTrue(processor.isTerminal());
        processor.init(640, 480, null);
        assertSame(processor.latest(), processor.processFrame(null, 1));
        assertTrue(processor.isTerminal());
    }

    @Test
    public void terminalizationDefersDisposalUntilActiveNativeLeaseFinishes() {
        FtcColorBlobProcessor.CallbackLease lease = new FtcColorBlobProcessor.CallbackLease();
        assertTrue(lease.begin());
        assertFalse(lease.terminalize("owner closed"));
        assertFalse(lease.begin());
        lease.publish(FtcColorBlobProcessor.Frame.unavailable(42, "late callback"));
        assertTrue(lease.latest().reason.contains("owner closed"));
        assertTrue(lease.finish());
        assertFalse(lease.terminalize("repeated close"));
        assertFalse(lease.begin());
    }

    @Test
    public void idleTerminalizationClaimsDisposalExactlyOnce() {
        FtcColorBlobProcessor.CallbackLease lease = new FtcColorBlobProcessor.CallbackLease();
        assertTrue(lease.terminalize("construction failed"));
        assertFalse(lease.terminalize("close also failed"));
        assertFalse(lease.begin());
        assertTrue(lease.latest().reason.contains("construction failed"));
    }

    @Test
    public void normalCallbacksRetainTheirResourcesAndCanRepeat() {
        FtcColorBlobProcessor.CallbackLease lease = new FtcColorBlobProcessor.CallbackLease();
        for (int i = 0; i < 3; i++) {
            assertTrue(lease.begin());
            assertFalse(lease.finish());
        }
        assertTrue(lease.terminalize("closed"));
    }

    @Test
    public void unexpectedOverlappingCallbacksFailClosedWithoutEarlyDisposal() {
        FtcColorBlobProcessor.CallbackLease lease = new FtcColorBlobProcessor.CallbackLease();
        assertTrue(lease.begin());
        assertFalse(lease.begin());
        assertTrue(lease.isTerminal());
        assertFalse(lease.terminalize("also closed"));
        assertTrue(lease.finish());
        assertFalse(lease.terminalize("repeated close"));
        assertTrue(lease.latest().reason.contains("overlapped"));
    }
}
