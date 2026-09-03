#!/usr/bin/env python3
"""Verify the generated Sushi guide-search integration and area metadata."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys
from urllib.parse import unquote, urlsplit


GUIDE_AREAS = {
    "Get Started",
    "Learn",
    "Build",
    "Test & Tune",
    "Advanced",
    "Reference",
}

ENHANCER_MARKERS = (
    'const searchLabel = "Search all guides"',
    'headerLabel.setAttribute("aria-label", searchLabel)',
    'headerLabel.setAttribute("title", searchLabel)',
    'search.setAttribute("aria-label", searchLabel)',
    'trigger.setAttribute("aria-label", searchLabel)',
    'element.shadowRoot',
    'input[role="combobox"]',
    'controls.insertAdjacentElement("afterend", help)',
    'root.host.setAttribute("aria-hidden", "true")',
    'root.host.removeAttribute("aria-hidden")',
    'savedTabIndexes.set(control',
    'control.tabIndex = -1',
    'savedTabIndexes.delete(control)',
    'list.setAttribute("role", "group")',
    'item.setAttribute("role", "button")',
    'item.setAttribute("aria-pressed"',
    'filterBaseClasses.get(root)',
    'item.addEventListener("keydown"',
    'event.key === "Enter"',
    'event.key === " "',
    'event.stopPropagation()',
    'item.click()',
)

BUNDLE_MARKERS = (
    'attachShadow({mode:"open"})',
    'role:"combobox"',
    'children:"Filters"',
    'children:"Tags"',
    '.n.l{opacity:0;pointer-events:none;',
    'class:qt(Ot.item,{[Ot.active]:Su(r.node.value)})',
)


def verify(repository_root: Path) -> list[str]:
    site_root = repository_root / "build" / "docs-site"
    index_path = site_root / "index.html"
    search_path = site_root / "search.json"
    failures: list[str] = []

    if not index_path.is_file() or index_path.stat().st_size == 0:
        return [f"{index_path}: generated site index is missing or empty"]
    if not search_path.is_file() or search_path.stat().st_size == 0:
        return [f"{search_path}: generated search index is missing or empty"]

    index = index_path.read_text(encoding="utf-8")
    bundle_match = re.search(
        r'<script\s+src="([^"]*assets/javascripts/bundle\.[^"]+\.min\.js)"></script>',
        index,
    )
    if bundle_match is None:
        failures.append(f"{index_path}: cannot locate Zensical's generated search bundle")
    else:
        enhancer_offset = index.find(ENHANCER_MARKERS[0])
        if enhancer_offset < bundle_match.end():
            failures.append(
                f"{index_path}: the guide-search enhancer must run after Zensical's bundle"
            )

        bundle_url = urlsplit(bundle_match.group(1))
        bundle_path = (site_root / unquote(bundle_url.path).lstrip("./")).resolve()
        if not bundle_path.is_relative_to(site_root.resolve()):
            failures.append(f"{index_path}: generated bundle escapes the site root")
        elif not bundle_path.is_file() or bundle_path.stat().st_size == 0:
            failures.append(f"{bundle_path}: generated search bundle is missing or empty")
        else:
            bundle = bundle_path.read_text(encoding="utf-8")
            for marker in BUNDLE_MARKERS:
                if marker not in bundle:
                    failures.append(
                        f"{bundle_path}: pinned search component no longer exposes {marker}"
                    )

    for marker in ENHANCER_MARKERS:
        if marker not in index:
            failures.append(f"{index_path}: generated search enhancer is missing {marker}")

    try:
        search = json.loads(search_path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError) as error:
        failures.append(f"{search_path}: cannot parse generated search index ({error})")
        return failures

    items = search.get("items")
    if not isinstance(items, list) or not items:
        failures.append(f"{search_path}: generated search index has no items")
        return failures

    found_areas: set[str] = set()
    for index, item in enumerate(items):
        tags = item.get("tags")
        path = item.get("path")
        if not isinstance(tags, list) or len(tags) != 1 or tags[0] not in GUIDE_AREAS:
            failures.append(
                f"{search_path}: item {index} must have one guide-area tag, found {tags!r}"
            )
            continue
        found_areas.add(tags[0])
        if not isinstance(path, list) or not path or path[0] != tags[0]:
            failures.append(
                f"{search_path}: item {index} tag {tags[0]!r} disagrees with path {path!r}"
            )

    if found_areas != GUIDE_AREAS:
        failures.append(
            f"{search_path}: expected guide areas {sorted(GUIDE_AREAS)}, "
            f"found {sorted(found_areas)}"
        )

    if not failures:
        print(
            f"Verified the open-ShadowRoot search enhancer and {len(items)} indexed "
            f"sections across all six guide areas."
        )
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repository-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="repository root containing build/docs-site",
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
