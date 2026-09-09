package edu.ftcsushi.fw.ftc;

import org.junit.Test;

import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.lang.reflect.Modifier;
import java.lang.reflect.Proxy;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import fi.iki.elonen.NanoHTTPD;

import static org.junit.Assert.*;

/** Socket-free tests of the same frozen store and response handlers registered with the SDK. */
public final class FtcResultDownloadsTest {
    @Test public void pageAndAttachmentPreserveUtf8AndUseReadOnlySecurityHeaders() throws Exception {
        FtcResultDownloads.Store store = online();
        FtcResultDownloads.Session session = store.openSession();
        String original = "{\"value\":1.2345678901234567,\"unit\":\"µ\",\"note\":\"😀\"}\n";
        assertTrue(session.publish("trial-1.jsonl", original));
        String token = token(session.url());
        assertTrue(session.url().startsWith("http://robot.test:8080/sushi/results?result="));
        assertFalse(token.contains("trial-1"));

        NanoHTTPD.Response page = get(store, token, false);
        assertEquals(200, status(page));
        security(page);
        String html = read(page);
        assertTrue(html.contains("lang=\"en\""));
        assertTrue(html.contains("/sushi/results/download?result=" + token));
        assertTrue(html.contains("Never delay an emergency stop"));
        assertFalse(html.contains("<script"));

        NanoHTTPD.Response attachment = get(store, token, true);
        assertEquals(200, status(attachment));
        security(attachment);
        assertEquals("attachment; filename=\"trial-1.jsonl\"",
                attachment.getHeader("Content-Disposition"));
        assertEquals(original, read(attachment));
    }

    @Test public void unavailableAndRevokedHostsNeverPublishOrReviveAReplacement() throws Exception {
        FtcResultDownloads.Store store = new FtcResultDownloads.Store();
        FtcResultDownloads.Session unavailable = store.openSession();
        assertFalse(unavailable.publish("test.txt", "not retained"));
        assertNull(unavailable.url());
        store.setBaseUrl("(unavailable)");
        assertFalse(unavailable.publish("test.txt", "not retained"));
        store.setBaseUrl("http://robot.test:8080/");
        assertTrue(unavailable.publish("test.txt", "first"));
        String old = token(unavailable.url());
        FtcResultDownloads.Session replacement = store.openSession();
        assertFalse(unavailable.publish("test.txt", "late"));
        assertNull(unavailable.url());
        assertEquals(404, status(get(store, old, true)));
        assertTrue(replacement.publish("new.txt", "current"));
        String current = replacement.url();
        unavailable.clear();
        unavailable.close();
        assertEquals(current, replacement.url());
        assertEquals("current", read(get(store, token(current), true)));
        replacement.close();
        replacement.close();
        assertNull(replacement.url());
        assertFalse(replacement.publish("late.txt", "late"));
        assertEquals(404, status(get(store, token(current), false)));
    }

    @Test public void replacementAndClearInvalidateBothOldRoutesWithoutChangingTheirContents()
            throws Exception {
        FtcResultDownloads.Store store = online();
        FtcResultDownloads.Session session = store.openSession();
        assertTrue(session.publish("one.txt", "one"));
        String first = token(session.url());
        assertTrue(session.publish("two.txt", "two"));
        String second = token(session.url());
        assertNotEquals(first, second);
        assertEquals(404, status(get(store, first, false)));
        assertEquals(404, status(get(store, first, true)));
        assertEquals("two", read(get(store, second, true)));
        session.clear();
        session.clear();
        assertNull(session.url());
        assertEquals(404, status(get(store, second, false)));
        assertEquals(404, status(get(store, second, true)));
    }

    @Test public void rejectedNamesOrTextPreserveTheLastGoodResult() throws Exception {
        FtcResultDownloads.Store store = online();
        FtcResultDownloads.Session session = store.openSession();
        assertTrue(session.publish("good.jsonl", "good"));
        String good = session.url();
        String[] invalid = {null, "", ".hidden", "../trial", "a..b", "a/b", "a\\b",
                "a\r\nInjected: yes", "a\"b", "a<b", "a?b", "a:", "µ.txt", "a.", repeat('a', 97)};
        for (String filename : invalid) {
            assertThrows("filename " + filename, IllegalArgumentException.class,
                    () -> session.publish(filename, "bad"));
            assertEquals(good, session.url());
        }
        for (String malformed : new String[]{null, "\uD800", "\uDC00", "a\uD800b"}) {
            assertThrows(IllegalArgumentException.class, () -> session.publish("ok.txt", malformed));
            assertEquals(good, session.url());
        }
        assertThrows(IllegalArgumentException.class,
                () -> session.publish("too-big.txt", repeat('a', FtcResultDownloads.MAX_BYTES + 1)));
        assertEquals(good, session.url());
        assertEquals("good", read(get(store, token(good), true)));
    }

    @Test public void quotaCountsEncodedBytesIncludingSupplementaryUnicode() throws Exception {
        FtcResultDownloads.Store store = online();
        FtcResultDownloads.Session session = store.openSession();
        String exact = repeat('a', FtcResultDownloads.MAX_BYTES - 4) + "😀";
        assertEquals(FtcResultDownloads.MAX_BYTES, exact.getBytes(StandardCharsets.UTF_8).length);
        assertTrue(session.publish(repeat('a', 96), exact));
        String url = session.url();
        assertThrows(IllegalArgumentException.class, () -> session.publish("over.txt", exact + "a"));
        assertEquals(url, session.url());
        assertEquals(exact, read(get(store, token(url), true)));
        String twoByte = repeat('µ', FtcResultDownloads.MAX_BYTES / 2);
        assertTrue(session.publish("two-byte.txt", twoByte));
        assertThrows(IllegalArgumentException.class,
                () -> session.publish("over.txt", twoByte + "µ"));
        assertEquals(twoByte, read(get(store, token(session.url()), true)));
    }

    @Test public void oneInFlightTransferIsNonWaitingAndEofOrCloseReleasesItsPermit()
            throws Exception {
        FtcResultDownloads.Store store = online();
        FtcResultDownloads.Session session = store.openSession();
        session.publish("first.txt", "abc");
        NanoHTTPD.Response first = get(store, token(session.url()), true);
        InputStream oldStream = first.getData();
        NanoHTTPD.Response busy = get(store, token(session.url()), true);
        assertEquals(503, status(busy));
        assertEquals("1", busy.getHeader("Retry-After"));
        assertEquals('a', oldStream.read());
        assertEquals('b', oldStream.read());
        assertEquals('c', oldStream.read()); // Exhaustion releases even before a further read.
        NanoHTTPD.Response second = get(store, token(session.url()), true);
        assertEquals(200, status(second));
        oldStream.close(); // Cannot release the second transfer's permit.
        assertEquals(503, status(get(store, token(session.url()), true)));
        second.close();
        NanoHTTPD.Response third = get(store, token(session.url()), true);
        assertEquals(200, status(third));
        third.close();
        first.close();
        assertEquals("abc", read(get(store, token(session.url()), true)));
    }

    @Test public void admittedOldBytesSurviveClearStopAndNewHostWithoutServingNewData()
            throws Exception {
        FtcResultDownloads.Store store = online();
        FtcResultDownloads.Session old = store.openSession();
        old.publish("old.txt", "frozen old bytes");
        String oldToken = token(old.url());
        NanoHTTPD.Response admitted = get(store, oldToken, true);
        old.clear();
        old.close();
        FtcResultDownloads.Session next = store.openSession();
        next.publish("new.txt", "new bytes");
        assertEquals(503, status(get(store, token(next.url()), true)));
        assertEquals("frozen old bytes", read(admitted));
        assertEquals(404, status(get(store, oldToken, true)));
        assertEquals("new bytes", read(get(store, token(next.url()), true)));
    }

    @Test public void emptyAttachmentsAndRejectedRequestsDoNotLeakTheTransferPermit()
            throws Exception {
        FtcResultDownloads.Store store = online();
        FtcResultDownloads.Session session = store.openSession();
        session.publish("empty.txt", "");
        for (int i = 0; i < 3; i++) {
            assertEquals(404, status(get(store, "unknown", true)));
            assertEquals("", read(get(store, token(session.url()), true)));
        }
        for (NanoHTTPD.Method method : NanoHTTPD.Method.values()) {
            if (method == NanoHTTPD.Method.GET) continue;
            for (boolean attachment : new boolean[]{false, true}) {
                NanoHTTPD.Response response = store.respond(request(method,
                        Collections.singletonMap("result", Collections.singletonList(token(session.url())))),
                        attachment);
                assertEquals(405, status(response));
                assertEquals("GET", response.getHeader("Allow"));
            }
        }
        assertEquals(404, status(store.respond(request(NanoHTTPD.Method.GET, null), true)));
        assertEquals(404, status(store.respond(request(NanoHTTPD.Method.GET,
                Collections.singletonMap("result", Arrays.asList("a", "b"))), true)));
        assertEquals(404, status(store.respond(request(NanoHTTPD.Method.GET,
                Collections.singletonMap("file", Collections.singletonList("../secret"))), true)));
        assertEquals("", read(get(store, token(session.url()), true)));
    }

    @Test public void lifecyclePublicationDoesNotWaitForAnHttpReader() throws Exception {
        FtcResultDownloads.Store store = online();
        FtcResultDownloads.Session session = store.openSession();
        session.publish("first.txt", "original");
        NanoHTTPD.Response held = get(store, token(session.url()), true);
        CountDownLatch done = new CountDownLatch(1);
        AtomicReference<Throwable> failed = new AtomicReference<>();
        Thread lifecycle = new Thread(() -> {
            try {
                for (int i = 0; i < 200; i++) {
                    session.clear();
                    assertTrue(session.publish("next.txt", "value-" + i));
                }
                session.close();
            } catch (Throwable failure) {
                failed.set(failure);
            } finally {
                done.countDown();
            }
        });
        lifecycle.start();
        assertTrue("Publication must not wait for a retained HTTP stream", done.await(2, TimeUnit.SECONDS));
        lifecycle.join(2000);
        assertNull(failed.get());
        assertEquals("original", read(held));
        assertNull(session.url());
    }

    @Test public void registrarIsTheOnlyPublicImplementationMethodAndNoServerFactoryIsPublic()
            throws Exception {
        for (java.lang.reflect.Constructor<?> constructor : FtcResultDownloads.class.getDeclaredConstructors()) {
            assertTrue(Modifier.isPrivate(constructor.getModifiers()));
        }
        int publicMethods = 0;
        for (java.lang.reflect.Method method : FtcResultDownloads.class.getDeclaredMethods()) {
            if (!Modifier.isPublic(method.getModifiers())) continue;
            publicMethods++;
            assertEquals("register", method.getName());
            assertTrue(Modifier.isStatic(method.getModifiers()));
            assertNotNull(method.getAnnotation(
                    org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar.class));
        }
        assertEquals(1, publicMethods);
        assertEquals(3, ResultDownloads.class.getDeclaredMethods().length);
    }

    private static FtcResultDownloads.Store online() {
        FtcResultDownloads.Store store = new FtcResultDownloads.Store();
        store.setBaseUrl("http://robot.test:8080");
        return store;
    }

    private static String repeat(char value, int count) {
        char[] chars = new char[count];
        Arrays.fill(chars, value);
        return new String(chars);
    }

    private static String token(String url) {
        return url.substring(url.indexOf("?result=") + "?result=".length());
    }

    private static NanoHTTPD.Response get(FtcResultDownloads.Store store, String token,
                                         boolean attachment) {
        return store.respond(request(NanoHTTPD.Method.GET,
                Collections.singletonMap("result", Collections.singletonList(token))), attachment);
    }

    private static NanoHTTPD.IHTTPSession request(NanoHTTPD.Method method,
                                                 Map<String, List<String>> parameters) {
        return (NanoHTTPD.IHTTPSession) Proxy.newProxyInstance(NanoHTTPD.IHTTPSession.class.getClassLoader(),
                new Class<?>[]{NanoHTTPD.IHTTPSession.class}, (proxy, called, args) -> {
                    if (called.getName().equals("getMethod")) return method;
                    if (called.getName().equals("getParameters")) return parameters;
                    throw new AssertionError("HTTP must not inspect anything else: " + called);
                });
    }

    private static int status(NanoHTTPD.Response response) {
        return response.getStatus().getRequestStatus();
    }

    private static String read(NanoHTTPD.Response response) throws Exception {
        ByteArrayOutputStream output = new ByteArrayOutputStream();
        try (NanoHTTPD.Response closing = response; InputStream input = response.getData()) {
            byte[] buffer = new byte[1024];
            int count;
            while ((count = input.read(buffer)) != -1) output.write(buffer, 0, count);
        }
        return new String(output.toByteArray(), StandardCharsets.UTF_8);
    }

    private static void security(NanoHTTPD.Response response) {
        assertEquals("no-store", response.getHeader("Cache-Control"));
        assertEquals("no-cache", response.getHeader("Pragma"));
        assertEquals("nosniff", response.getHeader("X-Content-Type-Options"));
        assertEquals("no-referrer", response.getHeader("Referrer-Policy"));
        assertTrue(response.getHeader("Content-Security-Policy").contains("default-src 'none'"));
        assertNull(response.getHeader("Access-Control-Allow-Origin"));
    }
}
