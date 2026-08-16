#!/usr/bin/env python3
"""
Convert Markdown to Native Word Equations DOCX using Pandoc
============================================================
Uses Pandoc to convert Markdown (.md) to Word (.docx).
Pandoc natively converts LaTeX math formulas ($...$ and $$...$$)
into native Microsoft Word Open XML Equation Objects (w:oMath / OMML)!
"""

import subprocess
from pathlib import Path


def convert_file_with_pandoc(md_path: Path, docx_path: Path):
    """Run pandoc to convert markdown to native docx with native OMML equations."""
    # Command to run pandoc
    cmd = [
        "pandoc",
        str(md_path),
        "-o",
        str(docx_path),
        "--from=markdown+tex_math_dollars+yaml_metadata_block",
        "--to=docx",
    ]

    try:
        subprocess.run(cmd, capture_output=True, text=True, check=True)
        print(f"  [PANDOC OK] {md_path.name} -> {docx_path.name}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"  [ERROR] Pandoc failed for {md_path.name}: {e.stderr}")
        return False
    except FileNotFoundError:
        print("  [ERROR] Pandoc command not found. Please ensure pandoc is installed.")
        return False


def main():
    project_root = Path(__file__).resolve().parents[1]
    docs_dir = project_root / "docs"

    print("=" * 65)
    print("  Convert Markdown to DOCX via Pandoc (Native Word OMML Equations)")
    print("=" * 65)

    md_files = sorted(list(docs_dir.glob("*.md")) + list(project_root.glob("*.md")))

    if not md_files:
        print("[FAIL] No Markdown (.md) files found")
        return

    success_count = 0
    for md_file in md_files:
        docx_file = md_file.with_suffix(".docx")
        if convert_file_with_pandoc(md_file, docx_file):
            success_count += 1

    print("\n" + "=" * 65)
    print(f"  [DONE] Conversion Complete: {success_count}/{len(md_files)} files converted!")
    print("=" * 65)


if __name__ == "__main__":
    main()
