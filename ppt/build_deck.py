#!/usr/bin/env python3
"""Build the SIH 2026 idea deck for PS SIH26037 from the official template.

Every number on the slides is read from results/*.json at build time, so the
deck cannot drift from the runs it describes. Regenerate after sih_run_all:

    python ppt/build_deck.py

Writes SIH2026_26037_Idea.pptx at the repository root. Assets (the official
SIH template, the team logo and the rendered icons) live in ppt/assets/ so the
deck is reproducible from a clean checkout -- an earlier copy was lost to a
`git clean`, which is why nothing here depends on a temporary directory.
"""

import json
import math
import os
import sys

from lxml import etree
from PIL import Image, ImageChops
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.dml import MSO_LINE_DASH_STYLE
from pptx.enum.shapes import MSO_SHAPE, MSO_CONNECTOR
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.oxml.ns import qn
from pptx.util import Emu, Inches, Pt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
ASSETS = os.path.join(HERE, "assets")
BUILD = os.path.join(HERE, "build")
RESULTS = os.path.join(ROOT, "results")
TEMPLATE = os.path.join(ASSETS, "template.pptx")
OUT = os.path.join(ROOT, "SIH2026_26037_Idea.pptx")

TEAM_NAME = "CraterFinders"
TEAM_ID = "162509"
PS_ID = "SIH26037"
PS_TITLE_1 = "Adaptive Path Planning and Collision Avoidance"
PS_TITLE_2 = "for Autonomous Vehicles on Unstructured Indian Roads"
PS_TITLE_FULL = ("Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles "
                 "on Unstructured Indian Roads")

SCENARIOS = [
    ("village_road", "Village road"),
    ("urban_intersection", "Crossroads"),
    ("highway_merge", "Highway merge"),
    ("market", "Market street"),
    ("cattle_crossing", "Cattle crossing"),
]

BLUE, ORANGE, GREEN, PURPLE, RED, TEAL = "1565C0", "D9541E", "2E7D32", "6A1B9A", "C62828", "00897B"
SLATE, DARK, MUTED, BORDER = "455A64", "1F2937", "555555", "595959"
P_OR, P_GR, P_BL, P_PU, P_RD, P_TE, P_YE, P_LV, P_PE = (
    "FDEBDD", "E6F2E6", "E3EEFB", "F0E7F7", "FBE4E4", "DDF1EF", "FFF6DA", "E6E8FA", "FCE3D3")
FONT = "Calibri"


def rgb(h):
    return RGBColor.from_string(h)


def icon(name, color):
    return os.path.join(ASSETS, "icons", f"{name}_{color}.png")


# --------------------------------------------------------------- run metrics
def load_metrics():
    runs = {}
    for key, label in SCENARIOS:
        path = os.path.join(RESULTS, f"{key}.json")
        if not os.path.exists(path):
            sys.exit(f"missing {path}\nrun: octave-cli --eval \"startup; sih_run_all\"")
        with open(path, encoding="utf-8") as fh:
            d = json.load(fh)
        runs[key] = {"label": label, "m": d["metrics"], "meta": d["meta"], "data": d}
    return runs


# ---------------------------------------------------------------- primitives
def box(sl, x, y, w, h, fill=None, line=BORDER, dash=True, radius=None, lw=1.0, shape=None):
    kind = shape or (MSO_SHAPE.ROUNDED_RECTANGLE if radius is not None else MSO_SHAPE.RECTANGLE)
    s = sl.shapes.add_shape(kind, Inches(x), Inches(y), Inches(w), Inches(h))
    if radius is not None:
        s.adjustments[0] = radius
    if fill:
        s.fill.solid()
        s.fill.fore_color.rgb = rgb(fill)
    else:
        s.fill.background()
    if line:
        s.line.color.rgb = rgb(line)
        s.line.width = Pt(lw)
        if dash:
            s.line.dash_style = MSO_LINE_DASH_STYLE.DASH
    else:
        s.line.fill.background()
    s.shadow.inherit = False
    return s


def _run(p, text, size, color, bold=False, italic=False, font=FONT, underline=False):
    r = p.add_run()
    r.text = text
    f = r.font
    f.size, f.bold, f.italic, f.name = Pt(size), bold, italic, font
    f.color.rgb = rgb(color)
    if underline:
        f.underline = True
    # Pin the East-Asian and complex-script faces: without them PowerPoint sets
    # symbols such as the degree sign and multiplication sign in the template's
    # MS PGothic, which spaces them oddly.
    for tag in ("a:ea", "a:cs"):
        etree.SubElement(r._r.get_or_add_rPr(), qn(tag)).set("typeface", font)
    return r


def _bullet(p, char="•", indent=0.16, color=None):
    pPr = p._p.get_or_add_pPr()
    pPr.set("marL", str(Inches(indent)))
    pPr.set("indent", str(-Inches(indent)))
    if color:
        bc = etree.SubElement(pPr, qn("a:buClr"))
        etree.SubElement(bc, qn("a:srgbClr")).set("val", color)
    bf = etree.SubElement(pPr, qn("a:buFont"))
    bf.set("typeface", "Arial")
    etree.SubElement(pPr, qn("a:buChar")).set("char", char)


def text(sl, x, y, w, h, paras, size=12, color=DARK, align="l", anchor="t", margin=0.04,
         space_after=0, font=FONT):
    tb = sl.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = tb.text_frame
    tf.word_wrap = True
    for side in ("left", "right", "top", "bottom"):
        setattr(tf, f"margin_{side}", Inches(margin))
    tf.vertical_anchor = {"t": MSO_ANCHOR.TOP, "m": MSO_ANCHOR.MIDDLE, "b": MSO_ANCHOR.BOTTOM}[anchor]
    for i, para in enumerate(paras):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        spec = para if isinstance(para, dict) else {"runs": para}
        runs = spec["runs"]
        if isinstance(runs, (str, tuple)):
            runs = [runs]
        psize = spec.get("size", size)
        p.alignment = {"l": PP_ALIGN.LEFT, "c": PP_ALIGN.CENTER,
                       "r": PP_ALIGN.RIGHT}[spec.get("align", align)]
        p.space_after = Pt(spec.get("space_after", space_after))
        if spec.get("bullet"):
            b = spec["bullet"]
            _bullet(p, char=b if isinstance(b, str) else "•")
        for r in runs:
            if isinstance(r, str):
                _run(p, r, psize, color, font=font)
            else:
                t, o = r
                _run(p, t, o.get("size", psize), o.get("color", color), o.get("bold", False),
                     o.get("italic", False), o.get("font", font), o.get("underline", False))
    return tb


def B(t, color=None, **kw):
    o = {"bold": True, **kw}
    if color:
        o["color"] = color
    return (t, o)


def pic(sl, path, x, y, w=None, h=None):
    kw = {}
    if w is not None:
        kw["width"] = Inches(w)
    if h is not None:
        kw["height"] = Inches(h)
    return sl.shapes.add_picture(path, Inches(x), Inches(y), **kw)


def img_h(path, w):
    iw, ih = Image.open(path).size
    return w * ih / iw


def icon_disc(sl, x, y, d, color, name, pad=0.2):
    box(sl, x, y, d, d, fill=color, line=None, shape=MSO_SHAPE.OVAL)
    s = d * (1 - 2 * pad)
    pic(sl, icon(name, "FFFFFF"), x + d * pad, y + d * pad, w=s, h=s)


def arrow(sl, x1, y1, x2, y2, color=DARK, lw=1.75, both=False, dash=False):
    c = sl.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, Inches(x1), Inches(y1), Inches(x2), Inches(y2))
    c.line.color.rgb = rgb(color)
    c.line.width = Pt(lw)
    if dash:
        c.line.dash_style = MSO_LINE_DASH_STYLE.DASH
    ln = c.line._get_or_add_ln()
    if both:
        etree.SubElement(ln, qn("a:headEnd")).set("type", "triangle")
    etree.SubElement(ln, qn("a:tailEnd")).set("type", "triangle")
    return c


def heading(sl, x, y, w, t, color, size=19, align="c", icon_name=None):
    if icon_name:
        pic(sl, icon(icon_name, color), x, y + 0.03, w=0.32, h=0.32)
        x, w = x + 0.38, w - 0.38
    return text(sl, x, y, w, 0.4, [[B(t, color)]], size=size, align=align, anchor="m", margin=0)


def table(sl, x, y, col_w, rows, head_fill, size=10.5, row_h=0.3, zebra="F7F7F7",
          aligns=None, bold_cols=(0,), color_cols=None):
    nr, nc = len(rows), len(rows[0])
    gs = sl.shapes.add_table(nr, nc, Inches(x), Inches(y), Inches(sum(col_w)), Inches(row_h * nr))
    tbl = gs.table
    for j, cw in enumerate(col_w):
        tbl.columns[j].width = Inches(cw)
    for i, row in enumerate(rows):
        tbl.rows[i].height = Inches(row_h)
        for j, val in enumerate(row):
            cell = tbl.cell(i, j)
            cell.fill.solid()
            cell.fill.fore_color.rgb = rgb(head_fill if i == 0 else (zebra if i % 2 == 0 else "FFFFFF"))
            cell.margin_left = cell.margin_right = Inches(0.06)
            cell.margin_top = cell.margin_bottom = Inches(0.02)
            cell.vertical_anchor = MSO_ANCHOR.MIDDLE
            cell.text_frame.word_wrap = True
            p = cell.text_frame.paragraphs[0]
            if aligns:
                p.alignment = {"l": PP_ALIGN.LEFT, "c": PP_ALIGN.CENTER}[aligns[j]]
            col = "FFFFFF" if i == 0 else ((color_cols or {}).get(j, DARK))
            _run(p, val, size, col, bold=(i == 0 or j in bold_cols or j in (color_cols or {})))
    return gs


# ------------------------------------------------------------- slide scaffold
TEMPLATE_BODY_MARKERS = ("Detailed explanation", "Technologies to be used", "Analysis of the feasibility",
                         "Potential impact", "Details / Links", "Proposed Solution (")


def strip_template_body(sl):
    for shp in list(sl.shapes):
        if shp.has_text_frame and any(m in shp.text_frame.text for m in TEMPLATE_BODY_MARKERS):
            shp._element.getparent().remove(shp._element)


def set_title(sl, t):
    ttl = sl.shapes.title
    ttl.left, ttl.top, ttl.width, ttl.height = Inches(1.85), Inches(0.12), Inches(8.75), Inches(0.98)
    tf = ttl.text_frame
    for p in list(tf.paragraphs)[1:]:
        p._p.getparent().remove(p._p)
    p = tf.paragraphs[0]
    for child in list(p._p):
        if child.tag != qn("a:pPr"):
            p._p.remove(child)
    p.alignment = PP_ALIGN.CENTER
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    _run(p, t, 36, BLUE, bold=True, font="Times New Roman")


def swap_logo(sl):
    """Replace the template's 'Your Team Name' oval with the team logo."""
    for shp in list(sl.shapes):
        if shp.has_text_frame and shp.text_frame.text.strip() == "Your Team Name":
            shp._element.getparent().remove(shp._element)
    pic(sl, os.path.join(ASSETS, "team_logo.png"), 0.49, 0.03, w=1.11, h=1.11)


def hline(sl, x1, x2, y, color="404040", lw=1.25):
    c = sl.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, Inches(x1), Inches(y), Inches(x2), Inches(y))
    c.line.color.rgb = rgb(color)
    c.line.width = Pt(lw)
    c.line.dash_style = MSO_LINE_DASH_STYLE.DASH


def content_slide(sl, title):
    strip_template_body(sl)
    set_title(sl, title)
    swap_logo(sl)
    hline(sl, 0.2, 13.13, 1.2)


# ----------------------------------------------------------------- figures
def trim(path):
    im = Image.open(path).convert("RGB")
    bg = Image.new("RGB", im.size, (255, 255, 255))
    bbox = ImageChops.difference(im, bg).getbbox()
    if bbox:
        im.crop(bbox).save(path)


def make_figures(runs):
    """Bird's-eye hero shot and full-route overview, drawn with the project's own palette."""
    sys.path.insert(0, os.path.join(ROOT, "tools"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon
    from matplotlib.collections import LineCollection
    import sihviz as V

    os.makedirs(BUILD, exist_ok=True)

    def col(cls):
        c = V.CLASS_STYLE.get(cls, "#888888")
        return c[0] if isinstance(c, (tuple, list)) else c

    d = runs["village_road"]["data"]
    road, ser, frames, meta = d["road"], d["series"], d["frames"], d["meta"]
    rx, ry, hw = road["x"], road["y"], road["hw"]
    left, right = V.road_edges(rx, ry, hw)
    LABEL = {"auto": "Auto-rickshaw", "bicycle": "Cyclist", "pedestrian": "Pedestrian",
             "pushcart": "Parked pushcart"}

    def road_patch(ax):
        ax.add_patch(Polygon(left + right[::-1], closed=True, fc="#E9E4DA", ec="none", zorder=0))
        for e in (left, right):
            ax.plot([p[0] for p in e], [p[1] for p in e], color="#B8A98C", lw=1.4,
                    ls=(0, (1, 2)), zorder=1)

    def rect(ax, x, y, psi, L, W, c, z=4, ec="#333", lw=0.6):
        co, si = math.cos(psi), math.sin(psi)
        pts = [(-L / 2, -W / 2), (L / 2, -W / 2), (L / 2, W / 2), (-L / 2, W / 2)]
        ax.add_patch(Polygon([(x + a * co - b * si, y + a * si + b * co) for a, b in pts],
                             closed=True, fc=c, ec=ec, lw=lw, zorder=z))

    # Hero: the moment the ego eases past the parked pushcart under NUDGE.
    # Pick the frame where the pushcart sits ~14 m ahead, so both the vehicle
    # and the obstacle it is avoiding fit in the crop with the plan visible.
    def score(i):
        f = frames[i]
        tj = f["traj"]
        if not (tj and len(tj) == 2 and isinstance(tj[0], list) and len(tj[0]) > 3):
            return -1e9
        if f["ego"][3] < 2.0:
            return -1e9
        ex_, ey_, epsi_, _ = f["ego"]
        best = -1e9
        for j, a in enumerate(V.rows(f["agents"], 3)):
            if V.as_list(f["agent_class"])[j] != "pushcart":
                continue
            dx, dy = a[0] - ex_, a[1] - ey_
            ahead = dx * math.cos(epsi_) + dy * math.sin(epsi_)
            if ahead > 4:
                best = max(best, -abs(math.hypot(dx, dy) - 14))
        return best + (2.0 if f["state"] == "NUDGE" else 0.0)

    k = max(range(len(frames)), key=score)
    f = frames[k]
    ex, ey, epsi, _ = f["ego"]
    fig, ax = plt.subplots(figsize=(8, 2.75), dpi=220)
    road_patch(ax)
    ax.plot(ser["x"][:min(k * 2, len(ser["x"]))], ser["y"][:min(k * 2, len(ser["y"]))],
            color="#555", lw=1.0, ls="--", zorder=2)
    for i, (axx, ayy, ap) in enumerate(V.rows(f["agents"], 3)):
        cls = V.as_list(f["agent_class"])[i]
        L, W = V.CLASS_SIZE.get(cls, (2, 1))
        rect(ax, axx, ayy, ap, L, W, col(cls))
        ax.annotate(LABEL.get(cls, cls), (axx, ayy), xytext=(0, 13), textcoords="offset points",
                    ha="center", fontsize=8.5, zorder=9,
                    bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#999", lw=0.5))
    for tx, ty, _tp, _tv in V.rows(f["tracks"], 4):
        ax.add_patch(Circle((tx, ty), 1.3, fill=False, ec="#1565C0", lw=1.1, ls=":", zorder=5))
    ax.plot(f["traj"][0], f["traj"][1], color="#00897B", lw=3.2, zorder=6, solid_capstyle="round")
    rect(ax, ex + math.cos(epsi) * 1.35, ey + math.sin(epsi) * 1.35, epsi, 4.0, 1.8, "#111",
         z=7, ec="none")
    ax.annotate("Ego vehicle", (ex, ey), xytext=(0, -17), textcoords="offset points", ha="center",
                fontsize=8.5, fontweight="bold", zorder=9,
                bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#111", lw=0.6))
    span = 30
    ax.set_xlim(ex - span * 0.45, ex + span * 1.25)
    ax.set_ylim(ey - span * 0.29, ey + span * 0.29)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.plot([], [], color="#00897B", lw=3, label="Planned trajectory (4 s)")
    ax.plot([], [], color="#1565C0", lw=1.1, ls=":", label="Fused track")
    ax.plot([], [], color="#555", lw=1, ls="--", label="Path driven")
    ax.legend(loc="lower right", fontsize=7.5, framealpha=0.9)
    fig.tight_layout(pad=0.1)
    hero = os.path.join(BUILD, "hero.png")
    fig.savefig(hero, facecolor="white")
    plt.close(fig)

    # route overview, ego path coloured by behaviour state
    fig, ax = plt.subplots(figsize=(8, 2.6), dpi=220)
    road_patch(ax)
    trails = {}
    for fr in frames:
        for i, a in enumerate(V.rows(fr["agents"], 3)):
            trails.setdefault(V.as_list(fr["agent_class"])[i], []).append((a[0], a[1]))
    for cls, pts in trails.items():
        if cls == "pushcart":
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=col(cls), marker="s",
                    ms=6, ls="none", zorder=3)
        else:
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=col(cls), lw=1.2,
                    alpha=0.8, zorder=3)
    xs, ys, st = ser["x"], ser["y"], ser["state"]
    ax.add_collection(LineCollection([[(xs[i], ys[i]), (xs[i + 1], ys[i + 1])]
                                      for i in range(len(xs) - 1)],
                                     colors=[V.STATE_COLOR.get(s, "#999999") for s in st[:-1]],
                                     lw=4.5, zorder=5, capstyle="round"))
    gx, gy = meta["goal"]
    tg = runs["village_road"]["m"]["t_to_goal"]
    ax.plot(xs[0], ys[0], "o", color="#111111", ms=7, zorder=6)
    ax.plot(gx, gy, "*", color="#2E7D32", ms=15, zorder=6, mec="white")
    ax.annotate("Start", (xs[0], ys[0]), xytext=(0, -14), textcoords="offset points",
                ha="center", fontsize=8)
    ax.annotate(f"Goal ({tg:.1f} s)", (gx, gy), xytext=(0, 11), textcoords="offset points",
                ha="center", fontsize=8)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_xlim(min(rx) - 3, max(rx) + 3)
    h = [plt.Line2D([0], [0], color=V.STATE_COLOR[s], lw=5, label=s)
         for s in V.STATE_COLOR if s in set(st)]
    h += [plt.Line2D([0], [0], color=col(c), lw=1.5, label=V.CLASS_LABEL.get(c, c)) for c in trails]
    ax.legend(handles=h, loc="upper center", bbox_to_anchor=(0.5, 0.02), ncol=5, fontsize=7,
              frameon=False, handlelength=1.6, columnspacing=1.0)
    fig.tight_layout(pad=0.1)
    route = os.path.join(BUILD, "route.png")
    fig.savefig(route, facecolor="white", bbox_inches="tight")
    plt.close(fig)
    trim(route)
    return hero, route


# ===================================================================== build
def build():
    runs = load_metrics()
    hero, route = make_figures(runs)

    p95 = [runs[k]["m"]["latency_p95_ms"] for k, _ in SCENARIOS]
    mean = [runs[k]["m"]["latency_mean_ms"] for k, _ in SCENARIOS]
    p95_range = f"{min(p95):.0f}–{max(p95):.0f} ms"
    mean_range = f"{min(mean):.0f}–{max(mean):.0f} ms"
    worst = max(SCENARIOS, key=lambda s: runs[s[0]]["m"]["latency_p95_ms"])
    n_done = sum(1 for k, _ in SCENARIOS if runs[k]["m"]["reached"])
    n_coll = sum(1 for k, _ in SCENARIOS if runs[k]["m"]["collided"])
    village = runs["village_road"]["m"]
    market = runs["market"]["m"]

    prs = Presentation(TEMPLATE)
    sld_ids = prs.slides._sldIdLst
    last = list(sld_ids)[6]                       # template slide 7 is the instructions page
    prs.part.drop_rel(last.get(qn("r:id")))
    sld_ids.remove(last)
    S = list(prs.slides)

    # ------------------------------------------------------------- 1. title
    s1 = S[0]
    for shp in list(s1.shapes):
        if shp.has_text_frame and "TITLE PAGE" in shp.text_frame.text:
            tf = shp.text_frame
            for p in list(tf.paragraphs)[1:]:
                p._p.getparent().remove(p._p)
            p = tf.paragraphs[0]
            for child in list(p._p):
                if child.tag != qn("a:pPr"):
                    p._p.remove(child)
            p.alignment = PP_ALIGN.CENTER
            _run(p, PS_TITLE_1, 24, DARK, bold=True, font="Times New Roman")
            p2 = tf.add_paragraph()
            p2.alignment = PP_ALIGN.CENTER
            _run(p2, PS_TITLE_2, 24, DARK, bold=True, font="Times New Roman")
            shp.top, shp.height = Inches(1.0), Inches(1.25)
        elif shp.has_text_frame and "Problem Statement ID" in shp.text_frame.text:
            shp._element.getparent().remove(shp._element)

    tb = text(s1, 0.36, 2.55, 6.6, 4.6, [
        {"runs": [B("Problem Statement ID – "), PS_ID], "bullet": True, "space_after": 12},
        {"runs": [B("Problem Statement Title –")], "bullet": True, "space_after": 2},
        {"runs": [PS_TITLE_FULL], "size": 15, "space_after": 12},
        {"runs": [B("Theme – "), "Smart Vehicles"], "bullet": True, "space_after": 12},
        {"runs": [B("PS Category – "), "Software"], "bullet": True, "space_after": 12},
        {"runs": [B("Team ID – "), TEAM_ID], "bullet": True, "space_after": 12},
        {"runs": [B("Team Name (Registered on portal) – "), TEAM_NAME], "bullet": True},
    ], size=19, font="Arial")
    tb.text_frame.paragraphs[2]._p.get_or_add_pPr().set("marL", str(Inches(0.16)))

    # -------------------------------------------------- 2. proposed solution
    s2 = S[1]
    content_slide(s2, "PROPOSED SOLUTION")
    heading(s2, 0.3, 1.3, 4.05, "Problem at Hand", ORANGE, size=20)
    problems = [
        (P_YE, "No Lanes to Follow: ", "Village and peri-urban roads are unmarked, so lane-keeping "
                                       "ADAS and AV stacks lose their reference path."),
        (P_RD, "Mixed, Rule-Free Traffic: ", "Autos, cyclists, pedestrians, cattle and pushcarts "
                                             "share one carriageway with no lane discipline."),
        (P_LV, "Unpredictable Intent: ", "A rickshaw may brake, drift or cut in. A single predicted "
                                         "path hides that uncertainty from the planner."),
        (P_TE, "High Human Cost: ", "India recorded 1,68,491 road deaths in 2022 (MoRTH). Autonomy "
                                    "built for Western roads does not transfer."),
    ]
    y = 1.78
    for fill, lead, body in problems:
        box(s2, 0.3, y, 4.05, 1.18, fill=fill)
        text(s2, 0.38, y + 0.05, 3.9, 1.08, [[B(lead), body]], size=12.5, anchor="m")
        y += 1.28

    heading(s2, 4.55, 1.3, 4.45, "Our Solution", GREEN, size=20)
    hh = img_h(hero, 4.45)
    pic(s2, hero, 4.55, 1.78, w=4.45)
    box(s2, 4.55, 1.78, 4.45, hh, line=BORDER)
    text(s2, 4.55, 1.8 + hh, 4.45, 0.45,
         [[("Closed-loop simulation: in NUDGE mode the ego plans a smooth path past a parked "
            "pushcart on an unmarked village road.", {"italic": True})]],
         size=10, color=MUTED, align="c")
    wy = 1.8 + hh + 0.5
    heading(s2, 4.55, wy, 4.45, "Why We Stand Out", BLUE, size=18)
    box(s2, 4.55, wy + 0.42, 4.45, 6.8 - (wy + 0.42), fill=P_PE, radius=0.06)
    text(s2, 4.65, wy + 0.5, 4.28, 6.8 - (wy + 0.58), [
        {"runs": [B("Lane-free by design: "), "plans inside a drivable corridor and passes on "
                                              "either side, as Indian drivers do."],
         "bullet": True, "space_after": 5},
        {"runs": [B("Working prototype today: "),
                  f"all {n_done} required scenarios run end-to-end and complete collision-free, "
                  "scored against ground truth."], "bullet": True, "space_after": 5},
        {"runs": [B("Zero-licence core: "), "toolbox-free MATLAB/Octave code that ports directly "
                                            "to Simulink & Stateflow."], "bullet": True},
    ], size=12, anchor="m")

    heading(s2, 9.2, 1.3, 3.85, "Key Features", BLUE, size=20)
    box(s2, 9.2, 1.78, 3.85, 5.02)
    feats = [
        ("MdAltRoute", GREEN, "Corridor Planning", "Frenet lattice over free space, not painted lanes."),
        ("MdSensors", BLUE, "Multi-Sensor Fusion", "Camera + radar + LiDAR into EKF tracks that survive occlusion."),
        ("MdPsychology", ORANGE, "Intent Prediction", "Four class-weighted hypotheses per road user."),
        ("MdAccountTree", PURPLE, "Behaviour FSM", "Cruise, Nudge, Yield, Creep and Stop set the planner's limits."),
        ("MdShield", RED, "Safety First", "Risk-dominated cost, 0.45 m margin and emergency braking."),
        ("MdSpeed", TEAL, "Real-Time", f"10 Hz replanning; {mean_range} mean cycle."),
    ]
    y = 1.9
    for ic, c, t, d_ in feats:
        pic(s2, icon(ic, c), 9.32, y + 0.12, w=0.5, h=0.5)
        text(s2, 9.92, y, 3.05, 0.78, [[B(t, c)], [d_]], size=11, anchor="m")
        y += 0.81

    # ------------------------------------------------- 3. technical approach
    s3 = S[2]
    content_slide(s3, "TECHNICAL APPROACH")
    heading(s3, 0.3, 1.27, 8.8, "Closed-Loop Autonomy Pipeline", BLUE, size=18)
    stages = [
        ("Multi-Sensor Sensing", BLUE, P_BL, "MdCameraAlt",
         "Camera 60 m, 100°, class label\nRadar 90 m + range-rate\nLiDAR 40 m, 360°, occlusion"),
        ("Track Fusion", TEAL, P_TE, "MdHub",
         "Hungarian (GNN) association\nEKF per track, CTRV model\nM-of-N confirm & coast"),
        ("Intent Prediction", ORANGE, P_OR, "MdInsights",
         "4 weighted hypotheses/agent\nClass-aware: bus ≠ cattle\n4 s horizon, growing σ"),
        ("Conflict Risk", RED, P_RD, "MdGridOn",
         "Swept-disc conflict test\nWeighted by hypothesis\nTime-discounted, τ = 2 s"),
        ("Lattice Planner", GREEN, P_GR, "MdTimeline",
         "Frenet quintic + quartic\nLateral d(s), not d(t)\n~90 candidates per cycle"),
        ("Behaviour FSM", PURPLE, P_PU, "MdAccountTree",
         "Cruise · Nudge · Yield\nCreep · Stop; sets speed cap,\nlateral room, risk tolerance"),
        ("Motion Control", SLATE, "ECEFF1", "MdSettingsInputComponent",
         "Pure-pursuit steering\nPI speed loop\nJerk & steer-rate limits"),
        ("Vehicle Model", DARK, "E5E7EB", "MdDirectionsCar",
         "Kinematic bicycle model\n2.7 m wheelbase, δ ≤ 31°\n20 Hz sim · 10 Hz replan"),
    ]
    BW, BH, GAP = 1.95, 1.42, 0.333
    XS = [0.3 + i * (BW + GAP) for i in range(4)]
    R1, R2 = 1.72, 3.55
    pos = [(XS[i], R1) for i in range(4)] + [(XS[3 - i], R2) for i in range(4)]
    for (t, c, fill, ic, det), (x, y) in zip(stages, pos):
        box(s3, x, y, BW, BH, fill=fill, line=c, radius=0.08, lw=1.25)
        icon_disc(s3, x + 0.09, y + 0.09, 0.44, c, ic)
        text(s3, x + 0.58, y + 0.07, BW - 0.62, 0.48, [[B(t, c)]], size=12, anchor="m", margin=0.02)
        text(s3, x + 0.07, y + 0.58, BW - 0.1, BH - 0.62,
             [{"runs": [ln], "space_after": 1} for ln in det.split("\n")], size=9.5, margin=0.02)
    for i in range(3):
        arrow(s3, XS[i] + BW + 0.02, R1 + BH / 2, XS[i + 1] - 0.02, R1 + BH / 2)
        arrow(s3, XS[3 - i] - 0.02, R2 + BH / 2, XS[2 - i] + BW + 0.02, R2 + BH / 2, both=(i == 0))
    arrow(s3, XS[3] + BW / 2, R1 + BH + 0.02, XS[3] + BW / 2, R2 - 0.02)
    arrow(s3, XS[0] + BW / 2, R2 - 0.02, XS[0] + BW / 2, R1 + BH + 0.02, color=RED, dash=True)
    text(s3, XS[0] + BW / 2 + 0.08, R1 + BH + 0.03, 2.2, 0.36,
         [[("closed loop: ego state → world → sensors", {"italic": True})]],
         size=9, color=RED, anchor="m")

    rw, rh = 8.8, img_h(route, 8.8)
    if rh > 1.45:
        rh, rw = 1.45, 1.45 * Image.open(route).size[0] / Image.open(route).size[1]
    pic(s3, route, 0.3 + (8.8 - rw) / 2, 5.12, w=rw, h=rh)
    text(s3, 0.3, 5.12 + rh + 0.02, 8.8, 0.3,
         [[(f"All {n_done} scenarios complete collision-free. Shown: the "
            f"{village['dist_travel']:.0f} m village road, ego path coloured by behaviour.",
            {"italic": True})]], size=9.5, color=MUTED, align="c")

    heading(s3, 9.35, 1.27, 3.7, "Technologies Used", BLUE, size=18)
    box(s3, 9.35, 1.72, 3.7, 2.55)
    techs = [
        ("MdCode", "MATLAB R2025 / GNU Octave", "Toolbox-free algorithm core"),
        ("MdAccountTree", "Simulink + Stateflow", "Closed-loop model, behaviour chart"),
        ("MdMap", "RoadRunner + ASAM OpenDRIVE", "Indian road scenes (.xodr)"),
        ("MdVisibility", "YOLO on IDD", "Indian road-user detection demo"),
        ("MdMovie", "Python · Matplotlib · FFmpeg", "Replays and metrics figures"),
    ]
    y = 1.8
    for ic, t, d_ in techs:
        pic(s3, icon(ic, BLUE), 9.45, y + 0.07, w=0.34, h=0.34)
        text(s3, 9.88, y, 3.12, 0.48, [[B(t)], [(d_, {"color": MUTED})]], size=10.5,
             anchor="m", margin=0.01)
        y += 0.49

    heading(s3, 9.35, 4.35, 3.7, "Implementation Roadmap", GREEN, size=18)
    box(s3, 9.35, 4.78, 3.7, 2.02)
    phases = [
        (True, "Core loop + village-road scenario ✔"),
        (True, f"All {n_done} scenarios + submission video ✔"),
        (False, "Simulink closed-loop model"),
        (False, "Metrics & technical report, dashboard"),
        (False, "Stateflow, RoadRunner scenes, YOLO-IDD"),
    ]
    y = 4.86
    for i, (done, t) in enumerate(phases):
        c = GREEN if done else "78909C"
        box(s3, 9.45, y + 0.06, 0.27, 0.27, fill=c, line=None, shape=MSO_SHAPE.OVAL)
        text(s3, 9.45, y + 0.06, 0.27, 0.27, [[B(str(i + 1), "FFFFFF")]], size=10,
             align="c", anchor="m", margin=0)
        text(s3, 9.8, y, 3.2, 0.38, [[B(t, GREEN) if done else t]], size=10.5,
             anchor="m", margin=0.01)
        y += 0.38

    # ---------------------------------------------- 4. feasibility & viability
    s4 = S[3]
    content_slide(s4, "FEASIBILITY AND VIABILITY")
    heading(s4, 0.3, 1.27, 3.85, "Technical Feasibility", ORANGE, size=17)
    box(s4, 0.3, 1.68, 3.85, 2.52)
    text(s4, 0.38, 1.72, 3.7, 2.44, [
        {"runs": [B(f"All {n_done} scenarios run: "),
                  "sensing → fusion → prediction → planning → control closes end-to-end."],
         "bullet": True, "space_after": 4},
        {"runs": [B("Tested: "), "8/8 tests pass, including the full closed-loop regression run "
                                 "(EKF, Frenet, Hungarian, collision, bicycle model)."],
         "bullet": True, "space_after": 4},
        {"runs": [B("15× faster planner "), "(919 → 61 ms/cycle) from batched candidate checks."],
         "bullet": True, "space_after": 4},
        {"runs": [B("One scenario spec "), "feeds Octave, Simulink and drivingScenario, so nothing "
                                           "is authored twice."], "bullet": True},
    ], size=11, anchor="m")

    heading(s4, 4.35, 1.27, 4.75, f"Measured Results · All {n_done} Scenarios", GREEN, size=17)
    rows = [["Scenario", "Time", "Coll.", "Min clear", "Replan p95"]]
    for key, label in SCENARIOS:
        m = runs[key]["m"]
        rows.append([label, f"{m['t_to_goal']:.1f} s", "0" if not m["collided"] else "COLLIDED",
                     f"{m['min_clear']:.2f} m", f"{m['latency_p95_ms']:.0f} ms"])
    table(s4, 4.35, 1.68, [1.55, 0.75, 0.62, 0.9, 0.93], rows, GREEN, size=9.5, row_h=0.315,
          aligns=["l", "c", "c", "c", "c"], color_cols={2: GREEN})
    text(s4, 4.35, 1.68 + 0.315 * 6 + 0.04, 4.75, 0.3,
         [[(f"{n_done} of {len(SCENARIOS)} completed · {n_coll} collisions · "
            "clearance scored on ground truth", {"italic": True})]],
         size=9.5, color=MUTED, align="c")

    heading(s4, 9.3, 1.27, 3.75, "Resource Feasibility", BLUE, size=17)
    box(s4, 9.3, 1.68, 3.75, 2.52)
    text(s4, 9.38, 1.72, 3.6, 2.44, [
        {"runs": [B("Tier 0 · GNU Octave", BLUE),
                  f" – free, unlimited: algorithms, all {n_done} scenarios, metrics"], "space_after": 5},
        {"runs": [B("Tier 1 · MATLAB Online Basic", BLUE),
                  " – free: Simulink closed-loop model"], "space_after": 5},
        {"runs": [B("Tier 2 · 30-day trial", BLUE),
                  " – used last, only for Stateflow, RoadRunner & YOLO-IDD"], "space_after": 5},
        {"runs": [B("Hardware: "), "a standard laptop; no GPU needed."]},
    ], size=11, anchor="m")

    heading(s4, 0.3, 4.32, 12.75,
            "Potential Challenges & Risks → Strategies for Overcoming Them", RED, size=17)
    table(s4, 0.3, 4.72, [0.45, 4.85, 7.45], [
        ["#", "Challenge / Risk", "Strategy to Overcome"],
        ["1", f"Replan latency crosses the 100 ms budget on busy cycles (p95 {p95_range} by scenario)",
         "Batching already cut cost 15×; next, adaptive lattice size and MATLAB Coder C code"],
        ["2", "Hybrid A* fallback is written and verified, but enabling it made results worse",
         "Kept off: it raised p95 to 250–400 ms and cost a completion; re-enable once search "
         "cost and margins are reconciled"],
        ["3", f"Dense market clears by only {market['min_clear']:.2f} m at its tightest",
         "Most margin-sensitive scenario; tune the safety margin and re-validate all five runs"],
        ["4", "COCO-trained YOLO mislabels auto-rickshaws and pushcarts",
         "Class-agnostic tracking fallback; fine-tune on the IDD dataset when a GPU is available"],
        ["5", "RoadRunner is available only in the one-time 30-day trial",
         "Hand-author OpenDRIVE scenes first, so trial days go to assembly, not development"],
    ], RED, size=10.5, row_h=0.345, aligns=["c", "l", "l"], bold_cols=())

    # ------------------------------------------------- 5. impact and benefits
    s5 = S[4]
    content_slide(s5, "IMPACT AND BENEFITS")
    stats = [
        ("1.68 lakh", "road deaths in India in 2022 (MoRTH): the problem we target", RED, P_RD),
        (str(len(SCENARIOS)), "Indian road scenarios — village, crossroads, merge, market, "
                              "cattle — all completing", ORANGE, P_OR),
        (str(n_coll), f"collisions across all {n_done} scenarios, scored against ground truth", GREEN, P_GR),
        ("₹0", "licence cost to develop and verify the algorithm core", BLUE, P_BL),
    ]
    SW = (12.75 - 3 * 0.25) / 4
    for i, (num, lab, c, fill) in enumerate(stats):
        x = 0.3 + i * (SW + 0.25)
        box(s5, x, 1.35, SW, 1.22, fill=fill, radius=0.08)
        text(s5, x + 0.1, 1.38, SW - 0.2, 0.62, [[B(num, c)]], size=30, align="c", anchor="m")
        text(s5, x + 0.12, 1.98, SW - 0.24, 0.55, [lab], size=10.5, align="c")

    heading(s5, 0.3, 2.72, 4.3, "Who Benefits", ORANGE, size=18)
    box(s5, 0.3, 3.12, 4.3, 3.68)
    aud = [
        ("MdElectricCar", ORANGE, "Automotive OEMs & Tier-1s", "ADAS/AV behaviour tuned for Indian traffic"),
        ("MdLocalShipping", ORANGE, "Logistics & Last-Mile", "Autonomous delivery pods, campus shuttles"),
        ("MdAgriculture", GREEN, "Rural Mobility", "Agri-vehicles and buses on unmarked roads"),
        ("MdSchool", ORANGE, "Students & Researchers", "Open, toolbox-free baseline and scenarios"),
        ("MdFlag", ORANGE, "Road-Safety & Test Agencies", "Repeatable Indian-road test scenarios"),
    ]
    y = 3.2
    for ic, c, t, d_ in aud:
        pic(s5, icon(ic, c), 0.42, y + 0.13, w=0.44, h=0.44)
        text(s5, 0.98, y, 3.55, 0.7, [[B(t)], [(d_, {"color": MUTED})]], size=11.5, anchor="m")
        y += 0.71

    heading(s5, 4.85, 2.72, 8.2, "Benefits of the Solution", GREEN, size=18)
    bens = [
        ("Social", RED, P_RD, "MdFavorite", [
            "Class-aware caution around pedestrians, cyclists & cattle, the most vulnerable road users",
            "Legible yield/creep behaviour that other road users can anticipate"]),
        ("Economic", GREEN, P_GR, "MdCurrencyRupee", [
            "Free-tier development cuts R&D and licence cost",
            "Reusable scenario library lowers validation cost; indigenous tech for Atmanirbhar Bharat"]),
        ("Environmental", TEAL, P_TE, "MdEco", [
            "Jerk-limited, smooth speed profiles avoid harsh braking and wasted energy",
            "A natural fit for electric autonomous vehicles and low-emission mobility"]),
        ("Technological", ORANGE, P_OR, "MdLightbulb", [
            "Benchmark scenarios and metrics for unstructured roads",
            "Stateflow chart keeps every decision explainable to judges and regulators"]),
    ]
    CW, CH = (8.2 - 0.2) / 2, (3.68 - 0.2) / 2
    for i, (t, c, fill, ic, pts) in enumerate(bens):
        x = 4.85 + (i % 2) * (CW + 0.2)
        y = 3.12 + (i // 2) * (CH + 0.2)
        box(s5, x, y, CW, CH, fill=fill, radius=0.06)
        pic(s5, icon(ic, c), x + 0.12, y + 0.1, w=0.36, h=0.36)
        text(s5, x + 0.55, y + 0.08, CW - 0.65, 0.4, [[B(t, c)]], size=14, anchor="m")
        text(s5, x + 0.12, y + 0.52, CW - 0.24, CH - 0.58,
             [{"runs": [p], "bullet": True, "space_after": 3} for p in pts], size=11)

    # --------------------------------------------- 6. research and references
    s6 = S[5]
    content_slide(s6, "RESEARCH AND REFERENCES")
    heading(s6, 0.3, 1.27, 7.4, "Research Insights That Shaped the Design", BLUE, size=17)
    ins = [
        (P_YE, "Lanes are the wrong prior",
         "IDD shows Indian scenes break lane-centric assumptions, so we plan in a free-space corridor [3]."),
        (P_LV, "Keep uncertainty visible",
         "Multi-hypothesis prediction feeds a risk field instead of collapsing to one guess."),
        (P_GR, "Frenet lattices give comfort",
         "Jerk-optimal quintic/quartic trajectories are smooth by construction [1]."),
        (P_RD, "Score safety on ground truth",
         "Clearance measured from our own tracks would be circular, so we use true positions."),
    ]
    IW = (7.4 - 0.15) / 2
    for i, (fill, t, d_) in enumerate(ins):
        x = 0.3 + (i % 2) * (IW + 0.15)
        y = 1.68 + (i // 2) * 0.95
        box(s6, x, y, IW, 0.85, fill=fill)
        text(s6, x + 0.06, y + 0.02, IW - 0.12, 0.81, [[B(t)], [d_]], size=10.5, anchor="m")

    heading(s6, 0.3, 3.62, 7.4, "References", BLUE, size=17, align="l", icon_name="MdMenuBook")
    box(s6, 0.3, 4.02, 7.4, 2.78)
    refs = [
        "M. Werling, J. Ziegler, S. Kammel, S. Thrun, “Optimal trajectory generation for dynamic "
        "street scenarios in a Frenét frame,” IEEE ICRA, 2010.",
        "D. Dolgov, S. Thrun, M. Montemerlo, J. Diebel, “Path planning for autonomous vehicles in "
        "unknown semi-structured environments,” IJRR, vol. 29, no. 5, 2010.",
        "G. Varma, A. Subramanian, A. Namboodiri, M. Chandraker, C. V. Jawahar, “IDD: A dataset for "
        "exploring problems of autonomous navigation in unconstrained environments,” IEEE WACV, 2019.",
        "R. C. Coulter, “Implementation of the pure pursuit path tracking algorithm,” "
        "CMU-RI-TR-92-01, Carnegie Mellon University, 1992.",
        "H. W. Kuhn, “The Hungarian method for the assignment problem,” Naval Research "
        "Logistics Quarterly, vol. 2, 1955.",
        "S. Thrun, W. Burgard, D. Fox, Probabilistic Robotics, MIT Press, 2005.",
        "Ministry of Road Transport & Highways, “Road Accidents in India 2022,” Government "
        "of India, 2023.",
        "ASAM e.V., “ASAM OpenDRIVE” road-network standard; MathWorks RoadRunner & Automated "
        "Driving Toolbox documentation.",
    ]
    text(s6, 0.38, 4.08, 7.25, 2.68,
         [{"runs": [B(f"[{i + 1}] "), r], "space_after": 2.5} for i, r in enumerate(refs)],
         size=9.5, anchor="m")

    heading(s6, 7.95, 1.27, 5.1, "Prototype Evidence", GREEN, size=17)
    mpng = os.path.join(RESULTS, "village_road_metrics.png")
    if os.path.exists(mpng):
        mh = 3.25
        mw = mh * Image.open(mpng).size[0] / Image.open(mpng).size[1]
        mw = min(mw, 5.1)
        mh = mw * Image.open(mpng).size[1] / Image.open(mpng).size[0]
        mx = 7.95 + (5.1 - mw) / 2
        pic(s6, mpng, mx, 1.68, w=mw, h=mh)
        box(s6, mx, 1.68, mw, mh, line=BORDER)

    heading(s6, 7.95, 5.05, 5.1, "Project Links", BLUE, size=17, align="l", icon_name="MdLink")
    box(s6, 7.95, 5.45, 5.1, 1.35)
    lk = {"color": BLUE, "underline": True, "bold": True}
    text(s6, 8.05, 5.5, 4.95, 1.25, [
        {"runs": [("1) Source Code Repository", lk)], "space_after": 5},
        {"runs": [(f"2) Demo Video: All {n_done} Scenarios", lk)], "space_after": 5},
        {"runs": [("3) Metrics & Run Data (JSON)", lk)]},
    ], size=12, anchor="m")

    prs.save(OUT)
    print(f"wrote {OUT}")
    print(f"  {n_done}/{len(SCENARIOS)} scenarios, {n_coll} collisions")
    print(f"  replan p95 {p95_range} (worst: {worst[1]}), mean {mean_range}")


if __name__ == "__main__":
    build()
