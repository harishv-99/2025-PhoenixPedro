package edu.ftcsushi.fw.docs;

import edu.ftcsushi.fw.core.control.DebounceBoolean;
import edu.ftcsushi.fw.testing.ManualLoopClock;

import org.junit.Test;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NamedNodeMap;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import javax.xml.parsers.DocumentBuilderFactory;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Structural and factual safeguards; these checks do not establish student comprehension. */
public final class DocumentationConceptsTest {

    private static final String FRAMEWORK_PATH = "TeamCode/src/main/java/edu/ftcsushi/fw";
    private static final String SVG_NAMESPACE = "http://www.w3.org/2000/svg";
    private static final double EPSILON = 1e-8;
    private static final Pattern MERMAID = Pattern.compile(
            "(?m)^[ \\t]*```mermaid[ \\t]*\\r?\\n([\\s\\S]*?)^[ \\t]*```[ \\t]*\\r?$");

    @Test
    public void noviceContractPreservesFirstMeaningAndFocusedFoundation() throws Exception {
        String principles = prose(read("Framework Principles.md"));
        String maintainers = prose(read("docs/maintainers/Maintainer Notes.md"));

        requirePattern("Starting knowledge is limited", principles,
                "Basic Java flows and FTC basics.{0,50}only assumed starting knowledge");
        requirePattern("Purpose precedes terminology", principles,
                "robot problem before the term.{0,100}small example.{0,40}expected result");
        requirePattern("A term's name is not its explanation", principles,
                "Naming a concept is not explaining it");
        requirePattern("A later lookup cannot repair first use", principles,
                "later definition.{0,80}cannot repair an unexplained first pass");
        requirePattern("The required course has a focused endpoint", principles,
                "common foundation complete at simple TeleOp and timed Auto");
        requirePattern("Optional branches teach their own foundations", principles,
                "feedback.{0,80}localization foundations at the start of the optional path");
        requirePattern("Reading is a complete learning route", principles,
                "Keep reading, running, and authoring distinct");
        requirePattern("Advanced pages still explain concepts", principles,
                "Advanced does not mean unexplained");
        requirePattern("Automated checks have an explicit evidence limit", principles,
                "structural tests support that review.{0,50}does not prove comprehension");

        requirePattern("Maintainers review independent First-pass entry points", maintainers,
                "Check linked First passes as independent entry points");
        requirePattern("Required reading cannot form a cycle", maintainers,
                "Required prerequisites must be acyclic");
        requirePattern("The review records first meaning and use", maintainers,
                "concept checklist.{0,170}first explanation.{0,30}first required use");
        requirePattern("Diagrams supplement text without a new prerequisite", maintainers,
                "no renderer or interactive prerequisite");
    }

    @Test
    public void switchFirstPassExplainsItsConceptsBeforeTheFirstJavaExcerpt() throws Exception {
        String page = read("docs/build/Read a Switch.md");
        Matcher firstJava = Pattern.compile("(?m)^```java(?:[^\\r\\n]*)$").matcher(page);
        assertTrue("The switch lesson must retain its small Java excerpt", firstJava.find());
        String beforeCode = prose(page.substring(0, firstJava.start()));

        // Check small explanatory relationships, not a frozen paragraph or a glossary mention.
        requirePattern("HIGH and LOW have electrical meaning before code", beforeCode,
                "electrical levels.{0,40}HIGH.{0,20}LOW");
        requirePattern("Polarity maps the electrical level to pressed", beforeCode,
                "LOW for pressed.{0,40}active-low polarity");
        requirePattern("LOW is explicitly mapped to a Java meaning", beforeCode,
                "maps LOW to rawPressed\\s*=\\s*true");
        requirePattern("Contact flicker explains the physical problem", beforeCode,
                "physical switch.{0,100}(?:flicker|bounce)");
        requirePattern("Debouncing explains how filtering helps", beforeCode,
                "Debounc\\w*.{0,30}filters.{0,40}brief changes.{0,60}accepting a new state");
        requirePattern("rawPressed means the latest reading", beforeCode,
                "rawPressed.{0,40}latest.{0,30}reading");
        requirePattern("pressed means the filtered result", beforeCode,
                "pressed.{0,30}filtered result");
        requirePattern("A source explains how a value is obtained", beforeCode,
                "source.{0,60}obtain a value when asked");
        requirePattern("Status means saved observations", beforeCode,
                "reads both and saves their results in status");
    }

    @Test
    public void allGuideMermaidDiagramsHaveAccessibleTitlesAndDescriptions() throws Exception {
        List<Path> pages;
        try (Stream<Path> paths = Files.walk(frameworkRoot())) {
            pages = paths.filter(Files::isRegularFile)
                    .filter(path -> path.toString().endsWith(".md"))
                    .collect(Collectors.toList());
        }
        int diagrams = 0;
        for (Path page : pages) {
            Matcher fences = MERMAID.matcher(read(page));
            while (fences.find()) {
                diagrams++;
                String diagram = fences.group(1);
                requirePattern(page + ": Mermaid title", diagram,
                        "(?m)^[ \\t]*accTitle[ \\t]*:[ \\t]*\\S[^\\r\\n]*");
                requirePattern(page + ": Mermaid description", diagram,
                        "(?m)^[ \\t]*accDescr[ \\t]*(?::[ \\t]*\\S[^\\r\\n]*"
                                + "|\\{\\s*\\S[\\s\\S]*?\\})");
            }
        }
        assertTrue("The maintained guides must contain their teaching diagrams", diagrams > 0);
    }

    @Test
    public void conceptSvgsAreAccessibleAndSelfContained() throws Exception {
        for (String name : Arrays.asList("debounce-samples.svg", "field-relative-frames.svg",
                "control-camera-frames.svg")) {
            Path path = frameworkRoot().resolve("docs/assets/diagrams/" + name);
            Document document = svg(path);
            Element root = document.getDocumentElement();
            assertEquals(name, "svg", root.getLocalName());
            assertEquals(name, SVG_NAMESPACE, root.getNamespaceURI());
            String[] bounds = root.getAttribute("viewBox").trim().split("[ ,]+");
            assertEquals(name + ": viewBox has four coordinates", 4, bounds.length);
            for (String bound : bounds) {
                assertTrue(name + ": finite viewBox", Double.isFinite(Double.parseDouble(bound)));
            }
            assertTrue(name + ": positive viewBox width", Double.parseDouble(bounds[2]) > 0);
            assertTrue(name + ": positive viewBox height", Double.parseDouble(bounds[3]) > 0);
            assertEquals(name + ": image semantics", "img", root.getAttribute("role"));
            List<String> labels = Arrays.asList(root.getAttribute("aria-labelledby").split("\\s+"));
            for (String tag : Arrays.asList("title", "desc")) {
                NodeList matches = root.getElementsByTagNameNS(SVG_NAMESPACE, tag);
                assertEquals(name + ": one accessible " + tag, 1, matches.getLength());
                Element label = (Element) matches.item(0);
                assertFalse(name + ": nonempty " + tag, label.getTextContent().trim().isEmpty());
                assertFalse(name + ": labeled " + tag, label.getAttribute("id").isEmpty());
                assertTrue(name + ": accessible association for " + tag,
                        labels.contains(label.getAttribute("id")));
            }
            assertSelfContained(path, document);
        }
    }

    @Test
    public void debounceChartMatchesTheLessonAndRealSampledFilter() throws Exception {
        String lesson = read("docs/build/Read a Switch.md");
        Document document = svg(frameworkRoot().resolve("docs/assets/diagrams/debounce-samples.svg"));
        List<Sample> raw = samples(document, "circle");
        List<Sample> accepted = samples(document, "rect");
        List<Sample> tableRaw = new ArrayList<>();
        List<Sample> tableAccepted = new ArrayList<>();
        Matcher rows = Pattern.compile("\\|\\s*(?:START|Loop),\\s*([0-9.]+)\\s*\\|"
                + "\\s*(HIGH|LOW)\\s*\\|\\s*true\\s*\\|\\s*(true|false)\\s*\\|"
                + "\\s*(true|false)\\s*\\|").matcher(lesson);
        while (rows.find()) {
            double time = Double.parseDouble(rows.group(1));
            boolean value = Boolean.parseBoolean(rows.group(3));
            assertEquals("The table's active-low mapping at " + time,
                    "LOW".equals(rows.group(2)), value);
            tableRaw.add(new Sample(time, value, 0, 0));
            tableAccepted.add(new Sample(time, Boolean.parseBoolean(rows.group(4)), 0, 0));
        }
        assertEquals("Seven actual observations, separate from unknown INIT", 7, tableRaw.size());
        assertEquals("One raw marker for every observation", tableRaw.size(), raw.size());
        assertEquals("One accepted marker for every observation", raw.size(), accepted.size());
        assertEquals("START begins with zero elapsed time", 0, raw.get(0).timeSec, EPSILON);

        DebounceBoolean filter = DebounceBoolean.onAfterOffAfter(
                configDouble(lesson, "pressedDebounceSec"),
                configDouble(lesson, "releasedDebounceSec"));
        ManualLoopClock time = new ManualLoopClock();
        for (int i = 0; i < raw.size(); i++) {
            Sample input = raw.get(i);
            Sample output = accepted.get(i);
            assertEquals("Raw/table sample time " + i, tableRaw.get(i).timeSec,
                    input.timeSec, EPSILON);
            assertEquals("Accepted/table sample time " + i, tableAccepted.get(i).timeSec,
                    output.timeSec, EPSILON);
            assertEquals("Raw/table state " + i, tableRaw.get(i).value, input.value);
            assertEquals("Accepted/table state " + i, tableAccepted.get(i).value, output.value);
            if (i > 0) {
                double elapsedSec = input.timeSec - raw.get(i - 1).timeSec;
                assertTrue("Sample times increase", elapsedSec > 0);
                time.nextCycle(elapsedSec);
            }
            assertEquals("Actual sampled debounce state at " + input.timeSec,
                    filter.update(time.clock(), input.value), output.value);
            assertEquals("Raw and accepted markers share their sample x at " + input.timeSec,
                    input.x, output.x, EPSILON);
        }
        assertLinearTimeAxis(raw);
        assertLinearTimeAxis(accepted);
        assertBooleanRow(raw);
        assertBooleanRow(accepted);
    }

    private static void assertLinearTimeAxis(List<Sample> samples) {
        Sample first = samples.get(0);
        Sample last = samples.get(samples.size() - 1);
        assertTrue("Time advances left to right", last.x > first.x);
        for (Sample sample : samples) {
            assertEquals("Marker spacing represents elapsed time at " + sample.timeSec,
                    (sample.timeSec - first.timeSec) / (last.timeSec - first.timeSec),
                    (sample.x - first.x) / (last.x - first.x), EPSILON);
        }
    }

    private static void assertBooleanRow(List<Sample> samples) {
        double trueY = Double.NaN;
        double falseY = Double.NaN;
        for (Sample sample : samples) {
            if (sample.value) {
                if (Double.isNaN(trueY)) trueY = sample.y;
                assertEquals("All true markers use the row's true level", trueY, sample.y, EPSILON);
            } else {
                if (Double.isNaN(falseY)) falseY = sample.y;
                assertEquals("All false markers use the row's false level", falseY, sample.y, EPSILON);
            }
        }
        assertTrue("Both boolean levels exist, with true above false", trueY < falseY);
    }

    private static List<Sample> samples(Document document, String tag) {
        List<Sample> result = new ArrayList<>();
        NodeList elements = document.getElementsByTagNameNS(SVG_NAMESPACE, tag);
        for (int i = 0; i < elements.getLength(); i++) {
            Element marker = (Element) elements.item(i);
            if (!marker.hasAttribute("data-time-sec")) continue;
            String value = marker.getAttribute("data-value");
            assertTrue("Marker state is explicit", "true".equals(value) || "false".equals(value));
            for (Node node = marker; node instanceof Element; node = node.getParentNode()) {
                assertFalse("Sample coordinates must be in SVG user space; extend the test parser"
                                + " if a transform is introduced",
                        ((Element) node).hasAttribute("transform"));
            }
            boolean circle = "circle".equals(tag);
            double x = number(marker, circle ? "cx" : "x");
            double y = number(marker, circle ? "cy" : "y");
            if (!circle) {
                x += number(marker, "width") / 2;
                y += number(marker, "height") / 2;
            }
            result.add(new Sample(number(marker, "data-time-sec"),
                    Boolean.parseBoolean(value), x, y));
        }
        result.sort(Comparator.comparingDouble(sample -> sample.timeSec));
        return result;
    }

    private static void assertSelfContained(Path path, Document document) throws Exception {
        String source = read(path);
        assertFalse(path + ": no external stylesheet", source.contains("<?xml-stylesheet"));
        assertFalse(path + ": no stylesheet import", source.toLowerCase(Locale.ROOT).contains("@import"));
        Matcher urls = Pattern.compile("url\\(\\s*['\"]?([^\\s)'\"]+)", Pattern.CASE_INSENSITIVE)
                .matcher(source);
        while (urls.find()) {
            assertTrue(path + ": SVG references only local fragments", urls.group(1).startsWith("#"));
        }
        NodeList elements = document.getElementsByTagName("*");
        for (int i = 0; i < elements.getLength(); i++) {
            Element element = (Element) elements.item(i);
            assertFalse(path + ": no embedded executable or dependent asset",
                    Arrays.asList("script", "foreignobject", "image", "iframe", "object", "embed")
                            .contains(element.getLocalName().toLowerCase(Locale.ROOT)));
            NamedNodeMap attributes = element.getAttributes();
            for (int j = 0; j < attributes.getLength(); j++) {
                Node attribute = attributes.item(j);
                String name = attribute.getLocalName().toLowerCase(Locale.ROOT);
                assertFalse(path + ": no event handlers", name.startsWith("on"));
                if ("href".equals(name) || "src".equals(name)) {
                    assertTrue(path + ": no external asset reference",
                            attribute.getNodeValue().startsWith("#"));
                }
            }
        }
    }

    private static Document svg(Path path) throws Exception {
        DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
        factory.setNamespaceAware(true);
        factory.setFeature("http://apache.org/xml/features/disallow-doctype-decl", true);
        factory.setFeature("http://xml.org/sax/features/external-general-entities", false);
        factory.setFeature("http://xml.org/sax/features/external-parameter-entities", false);
        // Android's compile-time XMLConstants omits these standard JAXP property constants.
        // The host-JVM parser still honors the property URIs; retain both restrictions.
        factory.setAttribute("http://javax.xml.XMLConstants/property/accessExternalDTD", "");
        factory.setAttribute("http://javax.xml.XMLConstants/property/accessExternalSchema", "");
        factory.setXIncludeAware(false);
        factory.setExpandEntityReferences(false);
        try (InputStream input = Files.newInputStream(path)) {
            return factory.newDocumentBuilder().parse(input);
        }
    }

    private static double configDouble(String lesson, String name) {
        Matcher assignment = Pattern.compile("\\." + Pattern.quote(name)
                + "\\s*=\\s*([0-9.]+)\\s*;").matcher(lesson);
        assertTrue("The switch lesson exposes its effective " + name, assignment.find());
        return Double.parseDouble(assignment.group(1));
    }

    private static double number(Element element, String attribute) {
        double value = Double.parseDouble(element.getAttribute(attribute));
        assertTrue(attribute + " is finite", Double.isFinite(value));
        return value;
    }

    private static void requirePattern(String message, String text, String expression) {
        assertTrue(message, Pattern.compile(expression).matcher(text).find());
    }

    private static String prose(String text) {
        return text.replace("`", "").replace("**", "").replaceAll("\\s+", " ");
    }

    private static Path frameworkRoot() {
        return DocumentationLinksTest.MarkdownIntegrity.findRepositoryRoot(
                Paths.get(System.getProperty("user.dir"))).resolve(FRAMEWORK_PATH);
    }

    private static String read(String path) throws Exception {
        return read(frameworkRoot().resolve(path));
    }

    private static String read(Path path) throws Exception {
        return new String(Files.readAllBytes(path), StandardCharsets.UTF_8);
    }

    private static final class Sample {
        final double timeSec;
        final boolean value;
        final double x;
        final double y;

        Sample(double timeSec, boolean value, double x, double y) {
            this.timeSec = timeSec;
            this.value = value;
            this.x = x;
            this.y = y;
        }
    }
}
