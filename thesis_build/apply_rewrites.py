#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Apply paragraph-index rewrites to a thesis docx without changing styles/structure."""
from __future__ import annotations
import argparse
import json
from copy import deepcopy
from pathlib import Path

from docx import Document
from docx.oxml.ns import qn


def set_paragraph_text(paragraph, text: str) -> None:
    """Replace visible text while keeping paragraph style and first-run formatting."""
    if paragraph.text == text:
        return
    runs = paragraph.runs
    if not runs:
        paragraph.add_run(text)
        return
    # Keep first run formatting; clear the rest.
    first = runs[0]
    first.text = text
    for run in runs[1:]:
        run.text = ""


def apply(doc_path: Path, mapping_path: Path, out_path: Path) -> None:
    mapping = json.loads(mapping_path.read_text(encoding="utf-8"))
    # mapping: {"59": "new text", ...} or [{"i":59,"text":"..."}]
    if isinstance(mapping, list):
        mp = {int(item["i"]): item["text"] for item in mapping}
    else:
        mp = {int(k): v for k, v in mapping.items()}

    doc = Document(str(doc_path))
    changed = 0
    missing = []
    for i, p in enumerate(doc.paragraphs):
        if i not in mp:
            continue
        new = mp[i]
        if new is None:
            continue
        if p.text == new:
            continue
        # Do not rewrite headings / captions / toc by accident unless explicitly mapped.
        set_paragraph_text(p, new)
        changed += 1
    out_path.parent.mkdir(parents=True, exist_ok=True)
    doc.save(str(out_path))
    print(f"applied={changed} mapped={len(mp)} out={out_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--doc", required=True)
    ap.add_argument("--map", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    apply(Path(args.doc), Path(args.map), Path(args.out))


if __name__ == "__main__":
    main()
