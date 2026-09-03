#!/usr/bin/env python3
"""Verify authored Sushi API links against the generated local Javadoc artifact."""

from __future__ import annotations

import argparse
from html.parser import HTMLParser
from pathlib import Path
import re
import sys
import tomllib
from urllib.parse import unquote_to_bytes, urljoin, urlsplit


MAINTAINED_SOURCE_ROOT = (
    "https://github.com/harishv-99/2025-PhoenixPedro/blob/master/"
)
_MALFORMED_PERCENT = re.compile(r"%(?![0-9A-Fa-f]{2})")


class _JavadocInspector(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.is_type_page = False
        self.member_ids: set[str] = set()

    def handle_starttag(
        self, tag: str, attrs: list[tuple[str, str | None]]
    ) -> None:
        attributes = dict(attrs)
        classes = (attributes.get("class") or "").split()
        if tag == "body" and "class-declaration-page" in classes:
            self.is_type_page = True
        member_id = attributes.get("id")
        if tag == "section" and "detail" in classes and member_id is not None:
            self.member_ids.add(member_id)


def _line_number(text: str, offset: int) -> int:
    return text.count("\n", 0, offset) + 1


def _inspect_javadoc(
    path: Path, cache: dict[Path, _JavadocInspector]
) -> _JavadocInspector:
    if path not in cache:
        parser = _JavadocInspector()
        parser.feed(path.read_text(encoding="utf-8"))
        cache[path] = parser
    return cache[path]


def _strict_unquote(value: str) -> str:
    if _MALFORMED_PERCENT.search(value):
        raise ValueError("malformed percent escape")
    return unquote_to_bytes(value).decode("utf-8", errors="strict")


def _file_inventory(root: Path, pattern: str) -> dict[str, Path]:
    return {
        path.relative_to(root).as_posix(): path
        for path in root.rglob(pattern)
        if path.is_file() and path.stat().st_size > 0
    }


def verify(repository_root: Path) -> list[str]:
    config_path = repository_root / "zensical.toml"
    with config_path.open("rb") as config_file:
        project = tomllib.load(config_file)["project"]

    docs_root = (repository_root / project["docs_dir"]).resolve()
    site_root = (repository_root / project["site_dir"]).resolve()
    generated_api_root = (site_root / "api").resolve()
    published_api_root = urljoin(project["site_url"].rstrip("/") + "/", "api/")
    published_parts = urlsplit(published_api_root)
    api_link_pattern = re.compile(
        r"\]\(<(" + re.escape(published_api_root) + r"[^>\s]*)>\)"
    )
    source_link_pattern = re.compile(
        r"\[(?:Complete source:\s*)?`([^`]+)`\]\(<("
        + re.escape(MAINTAINED_SOURCE_ROOT)
        + r"[^>\s]+)>\)"
    )
    javadoc_cache: dict[Path, _JavadocInspector] = {}
    api_inventory = {
        relative: path
        for relative, path in _file_inventory(generated_api_root, "*.html").items()
        if (
            relative.startswith("edu/ftcsushi/fw/")
            or relative.startswith("edu/ftcsushi/robots/examples/")
        )
        and _inspect_javadoc(path, javadoc_cache).is_type_page
    }
    source_root = repository_root / "TeamCode" / "src"
    source_inventory = {
        "TeamCode/src/" + relative: path
        for relative, path in _file_inventory(source_root, "*.java").items()
    }

    failures: list[str] = []
    api_link_count = 0
    source_link_count = 0
    linked_pages: set[Path] = set()
    for markdown_path in sorted(docs_root.rglob("*.md")):
        markdown = markdown_path.read_text(encoding="utf-8")
        api_matches = list(api_link_pattern.finditer(markdown))
        source_matches = list(source_link_pattern.finditer(markdown))
        if markdown.count(published_api_root) != len(api_matches):
            failures.append(
                f"{markdown_path.relative_to(repository_root)}: every published API URL "
                "must be an inline Markdown link with an angle-bracket destination"
            )
        if markdown.count(MAINTAINED_SOURCE_ROOT) != len(source_matches):
            failures.append(
                f"{markdown_path.relative_to(repository_root)}: every maintained source URL "
                "must be an inline Markdown link with an angle-bracket destination"
            )

        for match in api_matches:
            api_link_count += 1
            linked_pages.add(markdown_path)
            url = match.group(1)
            parts = urlsplit(url)
            location = (
                f"{markdown_path.relative_to(repository_root)}:"
                f"{_line_number(markdown, match.start())}"
            )
            if parts.scheme != published_parts.scheme or parts.netloc != published_parts.netloc:
                failures.append(f"{location}: API link uses the wrong published host: {url}")
                continue
            if parts.query:
                failures.append(f"{location}: API link must not contain a query: {url}")
                continue
            if not parts.path.startswith(published_parts.path):
                failures.append(f"{location}: API link escapes the published API path: {url}")
                continue

            try:
                relative_target = _strict_unquote(
                    parts.path[len(published_parts.path) :]
                )
                fragment = _strict_unquote(parts.fragment)
            except (UnicodeDecodeError, ValueError) as error:
                failures.append(f"{location}: invalid API URL encoding ({error}): {url}")
                continue
            if relative_target in ("", "index.html"):
                generated_index = generated_api_root / "index.html"
                if fragment:
                    failures.append(
                        f"{location}: the API search entry link must not contain a fragment: {url}"
                    )
                elif not generated_index.is_file() or generated_index.stat().st_size == 0:
                    failures.append(
                        f"{location}: generated API search entry is missing or empty: {url}"
                    )
                continue
            generated_target = api_inventory.get(relative_target)
            if generated_target is None:
                failures.append(
                    f"{location}: generated API class target is missing, empty, "
                    f"wrong-case, or outside a published package: {url}"
                )
                continue

            if (
                fragment
                and fragment
                not in _inspect_javadoc(generated_target, javadoc_cache).member_ids
            ):
                failures.append(
                    f"{location}: canonical Javadoc member-detail fragment does not exist: {url}"
                )

        for match in source_matches:
            source_link_count += 1
            linked_pages.add(markdown_path)
            label = match.group(1)
            url = match.group(2)
            parts = urlsplit(url)
            location = (
                f"{markdown_path.relative_to(repository_root)}:"
                f"{_line_number(markdown, match.start())}"
            )
            if parts.query or parts.fragment:
                failures.append(
                    f"{location}: maintained source links must target one complete file: {url}"
                )
                continue
            try:
                source_path = _strict_unquote(url[len(MAINTAINED_SOURCE_ROOT) :])
            except (UnicodeDecodeError, ValueError) as error:
                failures.append(f"{location}: invalid source URL encoding ({error}): {url}")
                continue
            if source_path.startswith(
                "TeamCode/src/main/java/edu/ftcsushi/robots/examples/"
            ) and not label.endswith(".java"):
                relative_api_path = (
                    source_path.removeprefix("TeamCode/src/main/java/")
                    .removesuffix(".java")
                    + ".html"
                )
                if relative_api_path in api_inventory:
                    failures.append(
                        f"{location}: public maintained example class names must link to "
                        f"generated API documentation; reserve source links for explicit "
                        f".java files and package-private types: {url}"
                    )
            if source_path not in source_inventory:
                failures.append(
                    f"{location}: maintained Java source is missing, empty, or wrong-case: {url}"
                )

    if api_link_count == 0:
        failures.append(f"{docs_root.relative_to(repository_root)}: no published API links found")
    if not failures:
        print(
            f"Verified {api_link_count} generated API links and "
            f"{source_link_count} maintained source links across "
            f"{len(linked_pages)} Markdown pages."
        )
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repository-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="repository root containing zensical.toml",
    )
    arguments = parser.parse_args()
    failures = verify(arguments.repository_root.resolve())
    if failures:
        for failure in failures:
            print(failure, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
