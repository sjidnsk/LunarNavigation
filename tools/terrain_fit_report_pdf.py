#!/usr/bin/env python3
"""Typeset a local Chinese Markdown report as one A4 PDF.

Run with the bundled Python containing reportlab. Imports, font loading and
image reading are deferred until after argument parsing, so --help and parser
checks do not interfere with a running benchmark. This script performs no
network requests and never deletes files.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from datetime import date
from html import escape
from pathlib import Path
import re
from typing import Iterable
from urllib.parse import unquote, urlparse


@dataclass
class Block:
    kind: str
    text: str = ""
    level: int = 0
    rows: list[list[str]] = field(default_factory=list)
    target: str = ""
    marker: str = ""


def table_cells(line: str) -> list[str]:
    """Split pipes, preserving escaped pipes and pipes inside inline code."""
    cells: list[str] = []
    cell: list[str] = []
    fence = ""
    i = 0
    stripped = line.strip()
    if stripped.startswith("|"):
        stripped = stripped[1:]
    if stripped.endswith("|") and not stripped.endswith(r"\|"):
        stripped = stripped[:-1]
    while i < len(stripped):
        char = stripped[i]
        if char == "\\" and i + 1 < len(stripped) and stripped[i + 1] == "|":
            cell.append("|")
            i += 2
            continue
        if char == "`":
            stop = i + 1
            while stop < len(stripped) and stripped[stop] == "`":
                stop += 1
            run = stripped[i:stop]
            if not fence:
                fence = run
            elif run == fence:
                fence = ""
            cell.append(run)
            i = stop
            continue
        if char == "|" and not fence:
            cells.append("".join(cell).strip())
            cell = []
        else:
            cell.append(char)
        i += 1
    cells.append("".join(cell).strip())
    return cells


def is_table_separator(line: str) -> bool:
    cells = table_cells(line)
    return bool(cells) and all(re.fullmatch(r":?-{3,}:?", cell) for cell in cells)


HEADING = re.compile(r"^\s{0,3}(#{1,6})\s+(.+?)\s*#*\s*$")
LIST_ITEM = re.compile(r"^(\s*)([-+*]|\d+[.)])\s+(.+)$")
FENCE = re.compile(r"^\s{0,3}(`{3,}|~{3,})(.*)$")
IMAGE = re.compile(r"^\s*!\[([^]]*)\]\((.+)\)\s*$")


def clean_target(target: str) -> str:
    target = target.strip()
    if target.startswith("<"):
        end = target.find(">")
        if end >= 0:
            return target[1:end]
    # Optional Markdown title after a URL/path. Spaces in local paths are
    # otherwise preserved instead of being treated as a separator.
    return re.sub(r'\s+["\'][^"\']*["\']\s*$', "", target).strip()


def parse_markdown(source: str) -> list[Block]:
    """Parse common report blocks without a third-party Markdown dependency."""
    lines = source.replace("\r\n", "\n").replace("\r", "\n").splitlines()
    blocks: list[Block] = []
    i = 0

    def starts_block(index: int) -> bool:
        line = lines[index]
        if not line.strip():
            return True
        if HEADING.match(line) or LIST_ITEM.match(line) or FENCE.match(line) or IMAGE.match(line):
            return True
        return index + 1 < len(lines) and "|" in line and is_table_separator(lines[index + 1])

    while i < len(lines):
        line = lines[i]
        if not line.strip():
            i += 1
            continue
        fenced = FENCE.match(line)
        if fenced:
            fence, language = fenced.groups()
            content: list[str] = []
            i += 1
            closing = re.compile(r"^\s{0,3}" + re.escape(fence[0]) + "{" + str(len(fence)) + r",}\s*$")
            while i < len(lines) and not closing.match(lines[i]):
                content.append(lines[i].expandtabs(2))
                i += 1
            if i < len(lines):
                i += 1
            blocks.append(Block("code", "\n".join(content), marker=language.strip()))
            continue
        heading = HEADING.match(line)
        if heading:
            blocks.append(Block("heading", heading.group(2), len(heading.group(1))))
            i += 1
            continue
        picture = IMAGE.match(line)
        if picture:
            blocks.append(Block("image", picture.group(1), target=clean_target(picture.group(2))))
            i += 1
            continue
        if i + 1 < len(lines) and "|" in line and is_table_separator(lines[i + 1]):
            rows = [table_cells(line)]
            i += 2
            while i < len(lines) and lines[i].strip() and "|" in lines[i]:
                rows.append(table_cells(lines[i]))
                i += 1
            column_count = max(map(len, rows))
            rows = [row + [""] * (column_count - len(row)) for row in rows]
            blocks.append(Block("table", rows=rows))
            continue
        item = LIST_ITEM.match(line)
        if item:
            indentation, marker, first = item.groups()
            content = [first]
            i += 1
            while i < len(lines) and not starts_block(i):
                content.append(lines[i].strip())
                i += 1
            blocks.append(Block("list", " ".join(content),
                                level=min(3, len(indentation.expandtabs(2)) // 2), marker=marker))
            continue
        if re.fullmatch(r"\s*([-*_])\1{2,}\s*", line):
            blocks.append(Block("rule"))
            i += 1
            continue
        def paragraph_line(value: str) -> str:
            cleaned = value.lstrip("> ") if value.lstrip().startswith(">") else value.strip()
            return cleaned.rstrip() + ("\n" if value.endswith("  ") else "")

        content = [paragraph_line(line)]
        i += 1
        while i < len(lines) and not starts_block(i):
            content.append(paragraph_line(lines[i]))
            i += 1
        joined = "".join(value + ("" if value.endswith("\n") else " ") for value in content).rstrip()
        blocks.append(Block("paragraph", joined))
    return blocks


def uses_latin_font(char: str) -> bool:
    value = ord(char)
    # DejaVu also covers Greek, Cyrillic and common math/punctuation. Keep CJK
    # ideographs and CJK punctuation in the embedded fallback font.
    return value < 0x0530 or 0x2000 <= value <= 0x2BFF


def fontify(text: str, *, mono: bool = False, preserve_space: bool = False) -> str:
    """Mix an embedded Latin font with the embedded Chinese fallback font."""
    if not text:
        return ""
    runs: list[str] = []
    start = 0
    latin = uses_latin_font(text[0])
    for index in range(1, len(text) + 1):
        if index < len(text) and uses_latin_font(text[index]) == latin:
            continue
        body = escape(text[start:index], quote=False)
        if preserve_space:
            body = body.replace(" ", "&#160;")
        face = "LatinMono" if latin and mono else "Latin" if latin else "CJK"
        runs.append(f'<font name="{face}">{body}</font>')
        start = index
        if index < len(text):
            latin = uses_latin_font(text[index])
    return "".join(runs)


def inline_markdown(text: str) -> str:
    """Escape source and retain simple emphasis, code and clickable web links."""
    output: list[str] = []
    plain: list[str] = []

    def flush() -> None:
        if plain:
            output.append(fontify("".join(plain)))
            plain.clear()

    i = 0
    while i < len(text):
        if text[i] == "\\" and i + 1 < len(text):
            plain.append(text[i + 1])
            i += 2
            continue
        if text[i] == "\n":
            flush()
            output.append("<br/>")
            i += 1
            continue
        if text[i] == "`":
            stop = i + 1
            while stop < len(text) and text[stop] == "`":
                stop += 1
            fence = text[i:stop]
            end = text.find(fence, stop)
            if end >= 0:
                flush()
                output.append('<font color="#234D70">' + fontify(text[stop:end], mono=True) + "</font>")
                i = end + len(fence)
                continue
        if text.startswith("**", i):
            end = text.find("**", i + 2)
            if end >= 0:
                flush()
                output.append("<b>" + inline_markdown(text[i + 2:end]) + "</b>")
                i = end + 2
                continue
        if text[i] == "[":
            close = text.find("](", i + 1)
            if close >= 0:
                stop, depth = close + 2, 1
                while stop < len(text) and depth:
                    if text[stop] == "(" and (stop == 0 or text[stop - 1] != "\\"):
                        depth += 1
                    elif text[stop] == ")" and (stop == 0 or text[stop - 1] != "\\"):
                        depth -= 1
                    stop += 1
                if depth == 0:
                    label = inline_markdown(text[i + 1:close])
                    target = clean_target(text[close + 2:stop - 1])
                    flush()
                    if urlparse(target).scheme in {"http", "https", "mailto"}:
                        output.append(f'<link href="{escape(target, quote=True)}" color="#245D88">{label}</link>')
                    else:
                        # Local source paths are useful in Markdown but would
                        # create fragile, very long file links in the PDF.
                        output.append(label)
                    i = stop
                    continue
        if text[i] == "<":
            stop = text.find(">", i + 1)
            if stop >= 0 and urlparse(text[i + 1:stop]).scheme in {"http", "https"}:
                flush()
                target = text[i + 1:stop]
                output.append(f'<link href="{escape(target, quote=True)}" color="#245D88">{fontify(target)}</link>')
                i = stop + 1
                continue
        plain.append(text[i])
        i += 1
    flush()
    return "".join(output)


def plain_text(text: str) -> str:
    text = re.sub(r"\[([^]]+)\]\([^\n]*?\)", r"\1", text)
    return text.replace("`", "").replace("**", "").strip()


def resolve_image(target: str, markdown_path: Path, asset_root: Path | None) -> Path:
    parsed = urlparse(target)
    if parsed.scheme in {"http", "https"}:
        raise ValueError(f"Remote image is unsupported (no network): {target}")
    original = Path(unquote(parsed.path if parsed.scheme == "file" else target))
    candidates: list[Path] = []
    if asset_root:
        if not original.is_absolute():
            candidates.append(asset_root / original)
        # Keep a recognizable chart subdirectory when mapping a durable path
        # to a temporary experiment copy, then try the flat basename mapping.
        for index, part in enumerate(original.parts):
            if part.lower() in {"figures", "charts", "plots", "images", "assets"}:
                candidates.append(asset_root.joinpath(*original.parts[index:]))
        candidates.extend([asset_root / original.name,
                           asset_root / "figures" / original.name,
                           asset_root / "charts" / original.name,
                           asset_root / "plots" / original.name])
    candidates.append(original if original.is_absolute() else markdown_path.parent / original)
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    if asset_root:
        matches = list(asset_root.rglob(original.name))
        matches = [match for match in matches if match.is_file()]
        if len(matches) == 1:
            return matches[0].resolve()
        if len(matches) > 1:
            raise ValueError(f"Ambiguous image basename under --asset-root: {original.name}")
    raise FileNotFoundError(f"Image not found: {target}; --asset-root={asset_root}")


def visual_length(text: str) -> float:
    return sum(2. if ord(char) > 0x024F else 1. for char in plain_text(text))


def column_widths(rows: list[list[str]], available: float) -> list[float]:
    count = len(rows[0])
    desired: list[float] = []
    for col in range(count):
        lengths = sorted(visual_length(row[col]) for row in rows)
        median = lengths[len(lengths) // 2]
        header = visual_length(rows[0][col])
        desired.append(max(12., min(60., max(header, .7 * median))))
    minimum = min(43., available / count * .58)
    remainder = available - count * minimum
    total = sum(desired)
    return [minimum + remainder * weight / total for weight in desired]


def build_pdf(markdown_path: Path, output_path: Path, asset_root: Path | None,
              title_override: str | None, subtitle: str, report_date: str) -> None:
    # These imports and all asset reads deliberately occur only during an
    # explicitly requested PDF build, never --help or pure parser validation.
    from reportlab.lib import colors
    from reportlab.lib.enums import TA_CENTER, TA_LEFT
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.styles import ParagraphStyle
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.ttfonts import TTFont
    from reportlab.lib.utils import ImageReader
    from reportlab.platypus import (BaseDocTemplate, Frame, HRFlowable, Image,
                                   KeepTogether, LongTable, PageBreak, PageTemplate,
                                   Paragraph, Spacer, TableStyle)
    from reportlab.platypus.tableofcontents import TableOfContents

    chinese_font = Path("/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf")
    if not chinese_font.is_file():
        raise FileNotFoundError(f"Required Chinese font not found: {chinese_font}")
    latin_root = Path("/usr/share/fonts/truetype/dejavu")
    for name, filename in [("CJK", str(chinese_font)), ("Latin", "DejaVuSans.ttf"),
                           ("LatinBold", "DejaVuSans-Bold.ttf"),
                           ("LatinItalic", "DejaVuSans-Oblique.ttf"),
                           ("LatinMono", "DejaVuSansMono.ttf")]:
        font_path = Path(filename) if name == "CJK" else latin_root / filename
        if not font_path.is_file():
            raise FileNotFoundError(f"Required font not found: {font_path}")
        pdfmetrics.registerFont(TTFont(name, str(font_path)))
    pdfmetrics.registerFontFamily("CJK", normal="CJK", bold="CJK", italic="CJK", boldItalic="CJK")
    pdfmetrics.registerFontFamily("Latin", normal="Latin", bold="LatinBold",
                                  italic="LatinItalic", boldItalic="LatinBold")
    pdfmetrics.registerFontFamily("LatinMono", normal="LatinMono", bold="LatinMono",
                                  italic="LatinMono", boldItalic="LatinMono")

    blocks = parse_markdown(markdown_path.read_text(encoding="utf-8"))
    first_heading = next((block for block in blocks if block.kind == "heading"), None)
    title = title_override or (plain_text(first_heading.text) if first_heading else markdown_path.stem)
    blue = colors.HexColor("#254E70")
    dark = colors.HexColor("#263647")
    muted = colors.HexColor("#65778A")
    pale = colors.HexColor("#F0F5F9")
    grid = colors.HexColor("#D5DFE8")
    body = ParagraphStyle("Body", fontName="CJK", fontSize=10, leading=15,
                          textColor=dark, alignment=TA_LEFT, wordWrap="CJK",
                          splitLongWords=True, spaceAfter=7, allowWidows=0, allowOrphans=0)
    caption = ParagraphStyle("Caption", parent=body, fontSize=8.3, leading=12,
                             textColor=muted, alignment=TA_CENTER, spaceBefore=5, spaceAfter=12)
    headings = {
        1: ParagraphStyle("Heading1", parent=body, fontSize=16, leading=24,
                          textColor=blue, spaceBefore=17, spaceAfter=9, keepWithNext=True),
        2: ParagraphStyle("Heading2", parent=body, fontSize=13.2, leading=20,
                          textColor=blue, spaceBefore=15, spaceAfter=8, keepWithNext=True),
        3: ParagraphStyle("Heading3", parent=body, fontSize=11.3, leading=17,
                          textColor=blue, spaceBefore=12, spaceAfter=6, keepWithNext=True),
    }
    code_style = ParagraphStyle("Code", parent=body, fontSize=7.8, leading=11.6,
                               spaceAfter=0, textColor=dark)

    class ReportDocument(BaseDocTemplate):
        def beforeDocument(self) -> None:
            self._outline_level = -1

        def afterFlowable(self, flowable) -> None:
            if not hasattr(flowable, "_toc_level"):
                return
            anchor = flowable._toc_anchor
            label = flowable.getPlainText()
            self.canv.bookmarkPage(anchor)
            outline_level = min(flowable._toc_level, self._outline_level + 1)
            self.canv.addOutlineEntry(label, anchor, level=outline_level, closed=False)
            self._outline_level = outline_level
            self.notify("TOCEntry", (flowable._toc_level, fontify(label), self.page, anchor))

    page_width, page_height = A4
    document = ReportDocument(str(output_path), pagesize=A4, leftMargin=45, rightMargin=45,
                              topMargin=52, bottomMargin=42, title=title,
                              author="", subject="可信度机制拟合方法对比分析")

    def page_decoration(canvas, doc) -> None:
        canvas.saveState()
        if doc.page > 1:
            canvas.setStrokeColor(grid)
            canvas.line(45, page_height - 36, page_width - 45, page_height - 36)
            label = title
            def header_width(value: str) -> float:
                return sum(pdfmetrics.stringWidth(char, "Latin" if uses_latin_font(char) else "CJK", 7.3)
                           for char in value)
            while header_width(label) > doc.width and label:
                label = label[:-1]
            canvas.setFillColor(muted)
            header_style = ParagraphStyle("PageHeader", parent=body, fontSize=7.3,
                                          leading=10, textColor=muted, spaceAfter=0)
            header = Paragraph(fontify(label), header_style)
            header.wrap(doc.width, 12)
            header.drawOn(canvas, 45, page_height - 29)
        canvas.setStrokeColor(grid)
        canvas.line(45, 34, page_width - 45, 34)
        canvas.setFillColor(muted)
        canvas.setFont("Latin", 8)
        canvas.drawCentredString(page_width / 2, 20, str(doc.page))
        canvas.restoreState()

    frame = Frame(document.leftMargin, document.bottomMargin, document.width, document.height,
                  leftPadding=0, rightPadding=0, topPadding=0, bottomPadding=0)
    document.addPageTemplates([PageTemplate(id="Report", frames=[frame], onPage=page_decoration)])
    cover_title = ParagraphStyle("CoverTitle", parent=body, fontSize=23, leading=34,
                                 textColor=blue, spaceAfter=16)
    cover_small = ParagraphStyle("CoverSmall", parent=body, fontSize=10, leading=16,
                                 textColor=muted, spaceAfter=12)
    story: list = [Spacer(1, 92),
                   Paragraph(fontify("可信度机制 / 开发与验证"), cover_small),
                   HRFlowable(width="100%", thickness=2, color=blue), Spacer(1, 27),
                   Paragraph(fontify(title), cover_title),
                   Paragraph(fontify(subtitle), cover_small), Spacer(1, 64),
                   Paragraph(fontify(f"报告日期：{report_date}"), cover_small),
                   Paragraph(fontify(f"来源：{markdown_path.name}"), cover_small), PageBreak()]
    story.append(Paragraph(fontify("目录"), headings[1]))
    toc = TableOfContents()
    toc.levelStyles = [
        ParagraphStyle(f"TOC{level}", parent=body, fontName="Latin", fontSize=9 if level == 0 else 8.5,
                       leading=12 if level == 0 else 11, leftIndent=level*15,
                       firstLineIndent=0, spaceBefore=3 if level == 0 else 1,
                       spaceAfter=0,
                       textColor=blue if level == 0 else dark)
        for level in range(3)
    ]
    toc.dotsMinLevel = 0
    story.extend([toc, PageBreak()])
    skipped_cover_heading = False
    heading_count = 0

    for block in blocks:
        if block.kind == "heading":
            if not skipped_cover_heading and block is first_heading and block.level == 1:
                skipped_cover_heading = True
                continue
            level = max(1, min(3, block.level - 1)) if block.level > 1 else 1
            paragraph = Paragraph(inline_markdown(block.text), headings[level])
            paragraph._toc_level = level - 1
            paragraph._toc_anchor = f"section_{heading_count}"
            heading_count += 1
            story.append(paragraph)
        elif block.kind == "paragraph":
            story.append(Paragraph(inline_markdown(block.text), body))
        elif block.kind == "list":
            style = ParagraphStyle(f"List{block.level}", parent=body,
                                   leftIndent=17 + block.level*10,
                                   firstLineIndent=0, bulletIndent=block.level*10,
                                   bulletFontName="Latin", bulletFontSize=9, spaceAfter=5)
            marker = block.marker if block.marker[0].isdigit() else "-"
            story.append(Paragraph(inline_markdown(block.text), style, bulletText=marker))
        elif block.kind == "rule":
            story.extend([Spacer(1, 7), HRFlowable(width="100%", color=grid), Spacer(1, 8)])
        elif block.kind == "table":
            count = len(block.rows[0])
            font_size = 8.4 if count <= 6 else 7.6 if count <= 10 else 7.0
            cell_style = ParagraphStyle("TableCell", parent=body, fontSize=font_size,
                                        leading=font_size*1.48, spaceAfter=0)
            head_style = ParagraphStyle("TableHead", parent=cell_style, textColor=blue)
            data = [[Paragraph(inline_markdown(cell), head_style if index == 0 else cell_style)
                     for cell in row] for index, row in enumerate(block.rows)]
            kwargs = dict(colWidths=column_widths(block.rows, document.width), repeatRows=1,
                          hAlign="LEFT", splitByRow=1)
            try:
                table = LongTable(data, splitInRow=1, **kwargs)
            except TypeError:
                table = LongTable(data, **kwargs)
            table.setStyle(TableStyle([
                ("BACKGROUND", (0, 0), (-1, 0), pale),
                ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#FAFCFD")]),
                ("LINEBELOW", (0, 0), (-1, 0), .7, grid),
                ("LINEBELOW", (0, 1), (-1, -1), .3, grid),
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("LEFTPADDING", (0, 0), (-1, -1), 5),
                ("RIGHTPADDING", (0, 0), (-1, -1), 5),
                ("TOPPADDING", (0, 0), (-1, -1), 6),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
            ]))
            # Keep compact comparison tables together so a single method does
            # not end up alone after a page break; long tables keep headers.
            if story and hasattr(story[-1], "_toc_level") and len(data) <= 12:
                # A heading directly followed by a short table should not
                # remain alone on the previous page when the table moves.
                heading = story.pop()
                story.extend([KeepTogether([heading, table]), Spacer(1, 11)])
            else:
                story.extend([KeepTogether([table]) if len(data) <= 8 else table,
                              Spacer(1, 11)])
        elif block.kind == "code":
            lines = block.text.split("\n")
            data = [[Paragraph(fontify(line or " ", mono=True, preserve_space=True), code_style)]
                    for line in lines]
            table = LongTable(data, colWidths=[document.width], hAlign="LEFT", splitByRow=1)
            table.setStyle(TableStyle([
                ("BACKGROUND", (0, 0), (-1, -1), pale),
                ("LEFTPADDING", (0, 0), (-1, -1), 8),
                ("RIGHTPADDING", (0, 0), (-1, -1), 8),
                ("TOPPADDING", (0, 0), (-1, -1), 3),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
            ]))
            if block.marker:
                story.append(Paragraph(fontify(block.marker), cover_small))
            story.extend([table, Spacer(1, 10)])
        elif block.kind == "image":
            image_path = resolve_image(block.target, markdown_path, asset_root)
            if image_path.suffix.lower() not in {".png", ".jpg", ".jpeg", ".gif", ".webp", ".bmp"}:
                raise ValueError(f"Only raster figures are supported: {image_path}")
            width, height = ImageReader(str(image_path)).getSize()
            scale = min(document.width / width, document.height*.63 / height)
            figure = Image(str(image_path), width=width*scale, height=height*scale, lazy=2)
            figure.hAlign = "CENTER"
            figures = [figure]
            if block.text:
                figures.append(Paragraph(inline_markdown(block.text), caption))
            else:
                figures.append(Spacer(1, 12))
            story.append(KeepTogether(figures))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    document.multiBuild(story, maxPasses=10)


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Convert a local Chinese Markdown terrain-fit report to one A4 PDF.",
        epilog="No network or deletion. HTTP(S) links remain clickable; local source links "
               "are rendered as labels. Supports ATX headings, paragraphs, lists, pipe tables, "
               "raster figures and fenced code. Requires the bundled Python with reportlab.")
    parser.add_argument("markdown", type=Path, help="Input UTF-8 Markdown report")
    parser.add_argument("output", type=Path, help="Output PDF path (existing files require --overwrite)")
    parser.add_argument("--asset-root", type=Path,
                        help="Map figure paths to this local chart-copy directory, preserving "
                             "figures/charts/plots suffixes or resolving a unique basename")
    parser.add_argument("--title", help="Override the first Markdown heading as the cover title")
    parser.add_argument("--subtitle", default="地表拟合、可信度与规划结果对比",
                        help="Cover subtitle")
    parser.add_argument("--date", default=date.today().isoformat(), help="Date printed on cover")
    parser.add_argument("--overwrite", action="store_true", help="Explicitly allow overwriting output PDF")
    arguments = parser.parse_args(list(argv) if argv is not None else None)
    markdown = arguments.markdown.expanduser().resolve()
    output = arguments.output.expanduser().resolve()
    if not markdown.is_file():
        parser.error(f"Input Markdown does not exist: {markdown}")
    if output == markdown:
        parser.error("Input Markdown and output PDF paths must differ")
    if output.exists() and not arguments.overwrite:
        parser.error(f"Output already exists; pass --overwrite to replace it: {output}")
    if output.suffix.lower() != ".pdf":
        parser.error("Output path must end with .pdf")
    asset_root = arguments.asset_root.expanduser().resolve() if arguments.asset_root else None
    if asset_root and not asset_root.is_dir():
        parser.error(f"--asset-root is not a directory: {asset_root}")
    try:
        build_pdf(markdown, output, asset_root, arguments.title,
                  arguments.subtitle, arguments.date)
    except (ValueError, FileNotFoundError, ImportError) as error:
        parser.error(str(error))
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
