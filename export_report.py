#!/usr/bin/env python3
"""Export docs/report.md to HTML, Word (.docx) and PDF.

    python3 src/line_follower/scripts/export_report.py

Writes docs/export/report.html (images embedded), report.docx and report.pdf.
Needs LibreOffice (`soffice`) for .docx/.pdf. The Markdown converter is deliberately small: it supports
exactly what report.md uses (headings, paragraphs, **bold**, `code`, links, images, tables, lists,
code blocks, block quotes and horizontal rules).
"""
import base64
import html
import os
import re
import shutil
import struct
import subprocess
import sys

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
DOCS = os.path.join(PKG_DIR, 'docs')
OUT = os.path.join(DOCS, 'export')
IMG_WIDTH_CM = 16.0     # images are scaled to the text width of an A4 page
IMG_MAX_HEIGHT_CM = 15.0

CSS = """
@page { size: 21cm 29.7cm; margin: 2cm 2.5cm; }
body { font-family: 'DejaVu Sans', Arial, sans-serif; font-size: 10.5pt; line-height: 1.35; }
h1 { font-size: 18pt; } h2 { font-size: 14pt; margin-top: 18pt; } h3 { font-size: 12pt; }
table { border-collapse: collapse; margin: 6pt 0; }
th, td { border: 1px solid #9b9a95; padding: 3pt 5pt; font-size: 9pt; vertical-align: top; }
th { background: #eeeeec; }
code, pre { font-family: 'DejaVu Sans Mono', monospace; font-size: 8.5pt; }
pre { background: #f4f4f2; padding: 6pt; }
blockquote { background: #fff4d6; padding: 4pt 8pt; margin: 6pt 0; }
p.caption { font-size: 9pt; color: #52514e; text-align: center; margin-top: 0; }
"""


def inline(text):
    """Inline Markdown -> HTML (escape first, then add tags)."""
    codes = []

    def keep_code(m):
        codes.append(html.escape(m.group(1)))
        return f'\x00{len(codes) - 1}\x00'
    text = re.sub(r'`([^`]+)`', keep_code, text)
    text = html.escape(text, quote=False)
    text = re.sub(r'\*\*(.+?)\*\*', r'<b>\1</b>', text)
    text = re.sub(r'(?<![\w*])\*([^*\s][^*]*?)\*(?![\w*])', r'<i>\1</i>', text)
    text = re.sub(r'\[([^\]]+)\]\(([^)]+)\)', r'<a href="\2">\1</a>', text)
    text = text.replace('\\|', '|')
    return re.sub(r'\x00(\d+)\x00', lambda m: f'<code>{codes[int(m.group(1))]}</code>', text)


def image(alt, path):
    full = os.path.join(DOCS, path)
    raw = open(full, 'rb').read()
    w_px, h_px = struct.unpack('>II', raw[16:24])          # PNG IHDR width/height
    w_cm = IMG_WIDTH_CM
    h_cm = w_cm * h_px / w_px
    if h_cm > IMG_MAX_HEIGHT_CM:                            # tall image: fit the page height instead
        w_cm, h_cm = w_cm * IMG_MAX_HEIGHT_CM / h_cm, IMG_MAX_HEIGHT_CM
    data = base64.b64encode(raw).decode()
    return (f'<p style="text-align:center"><img src="data:image/png;base64,{data}" alt="{html.escape(alt)}" '
            f'width="{w_cm * 96 / 2.54:.0f}" height="{h_cm * 96 / 2.54:.0f}" '
            f'style="width:{w_cm:.2f}cm;height:{h_cm:.2f}cm"></p>\n<p class="caption">{inline(alt)}</p>')


def split_row(line):
    cells = re.split(r'(?<!\\)\|', line.strip().strip('|'))
    return [c.strip() for c in cells]


def convert(md):
    lines = md.splitlines()
    out, i = [], 0
    while i < len(lines):
        line = lines[i]
        s = line.strip()
        if s.startswith('```'):                                        # code block
            i += 1
            buf = []
            while i < len(lines) and not lines[i].strip().startswith('```'):
                buf.append(lines[i])
                i += 1
            out.append('<pre>' + html.escape('\n'.join(buf)) + '</pre>')
            i += 1
            continue
        m = re.match(r'^(#{1,4})\s+(.*)', s)
        if m:                                                           # heading
            n = len(m.group(1))
            out.append(f'<h{n}>{inline(m.group(2))}</h{n}>')
            i += 1
            continue
        if s == '---':
            out.append('<hr>')
            i += 1
            continue
        m = re.match(r'^!\[([^\]]*)\]\(([^)]+)\)$', s)
        if m:                                                           # image
            out.append(image(m.group(1), m.group(2)))
            i += 1
            continue
        if s.startswith('|') and i + 1 < len(lines) and re.match(r'^\|[\s:|-]+\|$', lines[i + 1].strip()):
            head = split_row(s)                                         # table
            rows = []
            i += 2
            while i < len(lines) and lines[i].strip().startswith('|'):
                rows.append(split_row(lines[i]))
                i += 1
            t = ['<table><tr>' + ''.join(f'<th>{inline(c)}</th>' for c in head) + '</tr>']
            for r in rows:
                t.append('<tr>' + ''.join(f'<td>{inline(c)}</td>' for c in r) + '</tr>')
            out.append('\n'.join(t) + '</table>')
            continue
        if s.startswith('>'):                                           # block quote
            buf = []
            while i < len(lines) and lines[i].strip().startswith('>'):
                buf.append(lines[i].strip()[1:].strip())
                i += 1
            out.append('<blockquote>' + inline(' '.join(buf)) + '</blockquote>')
            continue
        m = re.match(r'^(\d+\.|-)\s+(.*)', s)
        if m and not line.startswith(' '):                              # list (with indented continuation)
            tag = 'ol' if m.group(1)[0].isdigit() else 'ul'
            items = []
            while i < len(lines):
                m = re.match(r'^(\d+\.|-)\s+(.*)', lines[i])
                if m:
                    items.append([m.group(2).strip()])
                elif lines[i].startswith(' ') and lines[i].strip() and items:
                    items[-1].append(lines[i].strip())
                elif lines[i].strip() == '' and i + 1 < len(lines) and lines[i + 1].startswith('   ') \
                        and not re.match(r'^\s*(\d+\.|-)\s', lines[i + 1]) and items:
                    items[-1].append('<br>')
                else:
                    break
                i += 1
            li = []
            for parts in items:
                txt = ' '.join(p for p in parts if p != '<br>')
                li.append(f'<li>{inline(txt)}</li>')
            out.append(f'<{tag}>' + ''.join(li) + f'</{tag}>')
            continue
        if s == '':
            i += 1
            continue
        buf = []                                                        # paragraph
        while i < len(lines) and lines[i].strip() and not re.match(r'^(#|\||```|!\[|>|---$|(\d+\.|-)\s)',
                                                                   lines[i].strip()):
            buf.append(lines[i].strip())
            i += 1
        out.append('<p>' + inline(' '.join(buf)) + '</p>')
    return '\n'.join(out)


def main():
    os.makedirs(OUT, exist_ok=True)
    md = open(os.path.join(DOCS, 'report.md')).read()
    title = re.search(r'^#\s+(.*)', md, re.M).group(1)
    page = (f'<!DOCTYPE html><html><head><meta charset="utf-8"><title>{html.escape(title)}</title>'
            f'<style>{CSS}</style></head><body>\n{convert(md)}\n</body></html>')
    html_path = os.path.join(OUT, 'report.html')
    open(html_path, 'w').write(page)
    print('Wrote', html_path)

    soffice = shutil.which('soffice') or shutil.which('libreoffice')
    if not soffice:
        sys.exit('LibreOffice not found: only the HTML was written')
    # Open the HTML in Writer (not Writer/Web) so images are embedded, then save as .docx and .pdf
    for fmt in ('docx:MS Word 2007 XML', 'pdf:writer_pdf_Export'):
        subprocess.run([soffice, '--headless', '--infilter=HTML (StarWriter)', '--convert-to', fmt,
                        '--outdir', OUT, html_path], check=True, capture_output=True)
    for f in ('report.docx', 'report.pdf'):
        p = os.path.join(OUT, f)
        print('Wrote', p, f'({os.path.getsize(p) / 1e6:.2f} MB)' if os.path.exists(p) else '(MISSING)')


if __name__ == '__main__':
    main()
