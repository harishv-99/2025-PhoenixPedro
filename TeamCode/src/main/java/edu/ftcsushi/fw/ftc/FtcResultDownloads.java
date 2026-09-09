package edu.ftcsushi.fw.ftc;

import android.content.Context;

import com.qualcomm.robotcore.util.WebHandlerManager;
import com.qualcomm.robotcore.util.WebServer;

import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;

import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Map;
import java.util.UUID;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import fi.iki.elonen.NanoHTTPD;

/**
 * Automatically discovered FTC web-server edge for {@link ResultDownloads}.
 *
 * <p>The registrar is public solely for SDK discovery. Tester hosts privately own publication;
 * robot code does not register routes or construct a server. The two GET handlers serve a small
 * read-only page and a frozen text attachment, never live objects or a controller file.</p>
 */
public final class FtcResultDownloads {
    static final String PAGE_PATH = "/sushi/results";
    static final String DOWNLOAD_PATH = "/sushi/results/download";
    static final int MAX_BYTES = 512 * 1024;
    static final int MAX_FILENAME_LENGTH = 96;

    private static final Store STORE = new Store();
    private static volatile WebServer registeredServer;

    private FtcResultDownloads() { }

    /**
     * Register fixed read-only routes on the existing SDK server. Called by FTC SDK discovery,
     * not by an OpMode. The stable SDK server reference supplies its advertised URL at tester
     * INIT; connection information and credentials are never retained.
     *
     * @param context SDK registration context; not retained
     * @param manager existing SDK web handler manager
     */
    @WebHandlerRegistrar
    public static void register(Context context, WebHandlerManager manager) {
        manager.register(PAGE_PATH, session -> STORE.respond(session, false));
        manager.register(DOWNLOAD_PATH, session -> STORE.respond(session, true));
        // Registration can precede server start; its advertised address may still be unavailable.
        registeredServer = manager.getWebServer();
    }

    /** Begin the one current host lifetime without opening a socket or polling hardware. */
    static Session openSession() {
        String address = null;
        WebServer server = registeredServer;
        if (server != null) {
            try {
                // Lifecycle thread only, never an HTTP handler or a retained WebInfo/credential.
                address = server.getConnectionInformation().getServerUrl();
            } catch (RuntimeException unavailable) {
                // Optional evidence transport must not prevent a tester from owning cleanup.
            }
        }
        STORE.setBaseUrl(address);
        return STORE.openSession();
    }

    /** Package-private deterministic transport seam; no socket, clock, device, or telemetry owner. */
    static final class Store {
        private final AtomicReference<Session> active = new AtomicReference<>();
        private final AtomicBoolean transferInProgress = new AtomicBoolean();
        private volatile String baseUrl;

        void setBaseUrl(String value) {
            if (value == null || !(value.startsWith("http://") || value.startsWith("https://"))) {
                baseUrl = null;
                return;
            }
            baseUrl = value.endsWith("/") ? value.substring(0, value.length() - 1) : value;
        }

        Session openSession() {
            Session next = new Session(this);
            Session previous = active.getAndSet(next);
            if (previous != null) previous.close();
            return next;
        }

        /** Return one current immutable record; a caller may retain it only for an admitted GET. */
        private Payload find(String token) {
            Session session = active.get();
            if (session == null) return null;
            Payload payload = session.payload.get();
            return payload != Payload.CLOSED && payload.bytes != null && payload.token.equals(token)
                    && active.get() == session ? payload : null;
        }

        NanoHTTPD.Response respond(NanoHTTPD.IHTTPSession request, boolean attachment) {
            if (request.getMethod() != NanoHTTPD.Method.GET) {
                NanoHTTPD.Response response = text(NanoHTTPD.Response.Status.METHOD_NOT_ALLOWED,
                        "Only GET is supported; downloads cannot change robot behavior.");
                response.addHeader("Allow", "GET");
                return response;
            }
            Map<String, List<String>> parameters = request.getParameters();
            List<String> tokens = parameters == null ? null : parameters.get("result");
            String token = tokens != null && tokens.size() == 1 ? tokens.get(0) : null;
            if (token == null || token.length() > 96 || parameters.size() != 1) return missing();
            if (!attachment) {
                Payload payload = find(token);
                if (payload == null) return missing();
                String page = "<!doctype html><html lang=\"en\"><meta charset=\"utf-8\">"
                        + "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
                        + "<title>Sushi result download</title><h1>Download the frozen result</h1>"
                        + "<p>" + payload.filename + " (" + payload.bytes.length + " UTF-8 bytes)</p>"
                        + "<p><a href=\"" + DOWNLOAD_PATH + "?result=" + payload.token
                        + "\">Download result</a></p>"
                        + "<p>Save the file on your laptop before another trial, BACK, or FTC STOP. "
                        + "Never delay an emergency stop to save a result.</p>"
                        + "<p>This page cannot start a trial, move hardware, or change configuration. "
                        + "A download is recorded software evidence, not physical acceptance.</p></html>";
                return headers(NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK,
                        "text/html; charset=utf-8", page));
            }
            if (!transferInProgress.compareAndSet(false, true)) {
                NanoHTTPD.Response response = text(NanoHTTPD.Response.Status.SERVICE_UNAVAILABLE,
                        "Another result download is in progress. Retry after it finishes.");
                response.addHeader("Retry-After", "1");
                return response;
            }
            try {
                Payload payload = find(token);
                if (payload == null) {
                    transferInProgress.set(false);
                    return missing();
                }
                // Admission precedes a later clear/STOP. Only these immutable bytes survive it.
                InputStream stream = new TransferStream(payload.bytes, transferInProgress);
                NanoHTTPD.Response response = NanoHTTPD.newFixedLengthResponse(
                        NanoHTTPD.Response.Status.OK, "text/plain; charset=utf-8",
                        stream, payload.bytes.length);
                response.addHeader("Content-Disposition",
                        "attachment; filename=\"" + payload.filename + "\"");
                return headers(response);
            } catch (RuntimeException failure) {
                transferInProgress.set(false);
                throw failure;
            }
        }
    }

    /** One host's revocable slot. Publication and revocation linearize through one atomic state. */
    static final class Session implements ResultDownloads {
        private final Store store;
        private final String sessionToken = UUID.randomUUID().toString();
        private final AtomicReference<Payload> payload = new AtomicReference<>(Payload.empty());
        private long nextResult;

        private Session(Store store) {
            this.store = store;
        }

        @Override public boolean publish(String filename, String frozenUtf8Text) {
            Payload before = payload.get();
            if (before == Payload.CLOSED || store.active.get() != this || store.baseUrl == null) {
                return false;
            }
            validateFilename(filename);
            byte[] bytes = encodeBounded(frozenUtf8Text);
            Payload next = new Payload(filename, bytes, sessionToken + "-" + (++nextResult));
            // Encoding never holds a lock shared with HTTP. A revoked/replaced state cannot revive.
            return store.active.get() == this && payload.compareAndSet(before, next);
        }

        @Override public String url() {
            Payload current = payload.get();
            String address = store.baseUrl;
            return current == Payload.CLOSED || current.bytes == null || store.active.get() != this
                    || address == null ? null : address + PAGE_PATH + "?result=" + current.token;
        }

        @Override public void clear() {
            Payload current = payload.get();
            // A new empty identity also invalidates a publication that began before this clear.
            Payload cleared = Payload.empty();
            while (current != Payload.CLOSED && !payload.compareAndSet(current, cleared)) {
                current = payload.get();
            }
        }

        /** Terminal, idempotent revocation; cannot clear a replacement host's slot. */
        void close() {
            payload.set(Payload.CLOSED);
            store.active.compareAndSet(this, null);
        }
    }

    /** Private immutable bytes; no API exposes the array for mutation. */
    private static final class Payload {
        private static final Payload CLOSED = new Payload("", null, "");
        private final String filename;
        private final byte[] bytes;
        private final String token;

        private Payload(String filename, byte[] bytes, String token) {
            this.filename = filename;
            this.bytes = bytes;
            this.token = token;
        }

        private static Payload empty() {
            return new Payload("", null, "");
        }
    }

    /** One admitted transfer; releases both its bytes and permit on exhaustion or close. */
    private static final class TransferStream extends InputStream {
        private byte[] bytes;
        private int position;
        private final AtomicBoolean permit;
        private boolean released;

        private TransferStream(byte[] bytes, AtomicBoolean permit) {
            this.bytes = bytes;
            this.permit = permit;
        }

        @Override public synchronized int read() {
            if (bytes == null || position == bytes.length) {
                release();
                return -1;
            }
            int value = bytes[position++] & 0xff;
            if (position == bytes.length) release();
            return value;
        }

        @Override public synchronized int read(byte[] destination, int offset, int length) {
            if (destination == null) throw new NullPointerException("destination");
            if (offset < 0 || length < 0 || length > destination.length - offset) {
                throw new IndexOutOfBoundsException();
            }
            if (length == 0) return 0;
            if (bytes == null || position == bytes.length) {
                release();
                return -1;
            }
            int copied = Math.min(length, bytes.length - position);
            System.arraycopy(bytes, position, destination, offset, copied);
            position += copied;
            if (position == bytes.length) release();
            return copied;
        }

        @Override public synchronized int available() {
            return bytes == null ? 0 : bytes.length - position;
        }

        @Override public synchronized void close() {
            release();
        }

        private void release() {
            bytes = null;
            if (!released) {
                released = true;
                permit.set(false);
            }
        }
    }

    private static NanoHTTPD.Response missing() {
        return text(NanoHTTPD.Response.Status.NOT_FOUND,
                "This result is unavailable. Use the current tester's displayed download link.");
    }

    private static NanoHTTPD.Response text(NanoHTTPD.Response.Status status, String text) {
        return headers(NanoHTTPD.newFixedLengthResponse(status, "text/plain; charset=utf-8", text));
    }

    private static NanoHTTPD.Response headers(NanoHTTPD.Response response) {
        response.addHeader("Cache-Control", "no-store");
        response.addHeader("Pragma", "no-cache");
        response.addHeader("X-Content-Type-Options", "nosniff");
        response.addHeader("Referrer-Policy", "no-referrer");
        response.addHeader("Content-Security-Policy",
                "default-src 'none'; base-uri 'none'; form-action 'none'; frame-ancestors 'none'");
        return response;
    }

    private static void validateFilename(String filename) {
        if (filename == null || filename.length() == 0 || filename.length() > MAX_FILENAME_LENGTH
                || !filename.matches("[A-Za-z0-9][A-Za-z0-9._-]*")
                || filename.contains("..") || filename.endsWith(".")) {
            throw new IllegalArgumentException("Download filename must be 1-96 ASCII letters, digits, "
                    + "dots, hyphens, or underscores; start with a letter/digit, omit '..', and "
                    + "do not end with a dot. Supply a filename, not a path.");
        }
    }

    /** Reject oversized or malformed Unicode before allocating its bounded UTF-8 representation. */
    private static byte[] encodeBounded(String text) {
        if (text == null) throw new IllegalArgumentException("Frozen result text must not be null");
        if (text.length() > MAX_BYTES) throw tooLarge();
        int bytes = 0;
        for (int i = 0; i < text.length(); i++) {
            char value = text.charAt(i);
            if (value < 0x80) bytes++;
            else if (value < 0x800) bytes += 2;
            else if (Character.isHighSurrogate(value)) {
                if (i + 1 == text.length() || !Character.isLowSurrogate(text.charAt(++i))) {
                    throw new IllegalArgumentException("Frozen result text contains malformed Unicode");
                }
                bytes += 4;
            } else if (Character.isLowSurrogate(value)) {
                throw new IllegalArgumentException("Frozen result text contains malformed Unicode");
            } else bytes += 3;
            if (bytes > MAX_BYTES) throw tooLarge();
        }
        return text.getBytes(StandardCharsets.UTF_8);
    }

    private static IllegalArgumentException tooLarge() {
        return new IllegalArgumentException("Frozen result exceeds the 512 KiB UTF-8 download limit");
    }
}
