#!/usr/bin/env python3
"""按黑龙江科技大学官方模板样式体系重建论文格式。"""
from __future__ import annotations

import re
import shutil
from copy import deepcopy
from pathlib import Path

from docx import Document
from docx.oxml.ns import qn
from lxml import etree

SRC = Path("/home/jinyicheng/auto_navigation/thesis_build/矿井未知环境无人机自主探索技术研究_学术重写版.docx")
TPL = Path("/home/jinyicheng/auto_navigation/thesis_build/format_ref/official_template.docx")
OUT = Path("/home/jinyicheng/auto_navigation/thesis_build/矿井未知环境无人机自主探索技术研究_官方格式版.docx")

EN_TITLE_LINES = [
    "Research on Autonomous Exploration Technology",
    "of Unmanned Aerial Vehicles",
    "in Unknown Mine Environments",
]

# 一级标题但不自动编号、需居中
UNNUMBERED_L1 = {
    "摘  要",
    "摘 要",
    "摘要",
    "Abstract",
    "目  录",
    "目 录",
    "目录",
    "致  谢",
    "致 谢",
    "致谢",
    "变量注释表",
    "参考文献",
    "作者简介",
    "作者简历",
}

# 一级标题文本规范化：去掉手工章号
L1_NUM_RE = re.compile(r"^\s*\d+\s+")
# 二/三级：去掉 1.2 / 4.2.1 / A.1 / 1.1.2研究意义 等前缀（编号后可无空格）
L23_NUM_RE = re.compile(r"^\s*(?:[A-Za-z]\.)?\d+(?:\.\d+){0,3}(?:\s+|(?=[\u4e00-\u9fff]))")


def ensure_pPr(p):
    pPr = p._p.find(qn("w:pPr"))
    if pPr is None:
        pPr = etree.SubElement(p._p, qn("w:pPr"))
        p._p.insert(0, pPr)
    return pPr


def set_pstyle(p, style_id: str):
    pPr = ensure_pPr(p)
    pStyle = pPr.find(qn("w:pStyle"))
    if pStyle is None:
        pStyle = etree.Element(qn("w:pStyle"))
        pPr.insert(0, pStyle)
    pStyle.set(qn("w:val"), style_id)


def set_num(p, ilvl: int | None, num_id: int | None):
    """设置或清除段落编号。num_id=0 表示取消编号。"""
    pPr = ensure_pPr(p)
    numPr = pPr.find(qn("w:numPr"))
    if num_id is None:
        if numPr is not None:
            pPr.remove(numPr)
        return
    if numPr is None:
        numPr = etree.Element(qn("w:numPr"))
        # insert after pStyle if present
        pStyle = pPr.find(qn("w:pStyle"))
        idx = list(pPr).index(pStyle) + 1 if pStyle is not None else 0
        pPr.insert(idx, numPr)
    # clear children
    for child in list(numPr):
        numPr.remove(child)
    if ilvl is not None:
        ilvl_el = etree.SubElement(numPr, qn("w:ilvl"))
        ilvl_el.set(qn("w:val"), str(ilvl))
    nid_el = etree.SubElement(numPr, qn("w:numId"))
    nid_el.set(qn("w:val"), str(num_id))


def set_jc(p, val: str | None):
    pPr = ensure_pPr(p)
    jc = pPr.find(qn("w:jc"))
    if val is None:
        if jc is not None:
            pPr.remove(jc)
        return
    if jc is None:
        jc = etree.SubElement(pPr, qn("w:jc"))
    jc.set(qn("w:val"), val)


def set_page_break_before(p, enabled: bool):
    pPr = ensure_pPr(p)
    pb = pPr.find(qn("w:pageBreakBefore"))
    if enabled:
        if pb is None:
            pb = etree.Element(qn("w:pageBreakBefore"))
            # after pStyle/numPr preferably
            pPr.append(pb)
        # presence alone means true; remove val=0 if any
        if qn("w:val") in pb.attrib:
            del pb.attrib[qn("w:val")]
    else:
        if pb is not None:
            pPr.remove(pb)


def replace_paragraph_text(p, text: str, keep_first_run_rPr: bool = True):
    """用单 run 替换段落全部文本，尽量保留第一个 run 的 rPr。"""
    runs = p.runs
    rPr = None
    if runs and keep_first_run_rPr:
        old = runs[0]._r.find(qn("w:rPr"))
        if old is not None:
            rPr = deepcopy(old)
    # remove all runs
    for child in list(p._p):
        if child.tag == qn("w:r"):
            p._p.remove(child)
    r = etree.SubElement(p._p, qn("w:r"))
    if rPr is not None:
        r.insert(0, rPr)
    t = etree.SubElement(r, qn("w:t"))
    if text.startswith(" ") or text.endswith(" ") or "  " in text:
        t.set("{http://www.w3.org/XML/1998/namespace}space", "preserve")
    t.text = text


def strip_heading_number(text: str, level: int) -> str:
    t = (text or "").strip()
    if not t:
        return t
    if level == 1:
        # 附录 A  系统启动... → 附录 A  系统启动...（保留）
        if t.startswith("附录"):
            return t
        return L1_NUM_RE.sub("", t).strip()
    # level 2/3
    # 已无编号的标题（如“研究背景与意义”）保持
    m = L23_NUM_RE.match(t)
    if m:
        return t[m.end() :].strip()
    return t


def normalize_l1_key(text: str) -> str:
    t = strip_heading_number(text, 1)
    # 统一空白
    t = re.sub(r"\s+", " ", t).strip()
    # 恢复双空格常见标题
    mapping = {
        "摘要": "摘  要",
        "摘 要": "摘  要",
        "目录": "目  录",
        "目 录": "目  录",
        "致谢": "致  谢",
        "致 谢": "致  谢",
    }
    return mapping.get(t, t)


def fix_body_style(doc: Document, tpl: Document):
    """用模板的学位论文正文样式定义覆盖当前损坏定义（保留 styleId）。"""
    cur_style = doc.styles["学位论文正文"]
    tpl_style = tpl.styles["学位论文正文"]
    cur_el = cur_style.element
    tpl_el = tpl_style.element

    # 保留当前 styleId / name / basedOn 目标，重写 pPr/rPr
    for tag in ("w:pPr", "w:rPr"):
        old = cur_el.find(qn(tag))
        if old is not None:
            cur_el.remove(old)
        src = tpl_el.find(qn(tag))
        if src is not None:
            cur_el.append(deepcopy(src))

    # basedOn 指向当前 Normal 的 styleId（a4 或 1）
    # 尝试找到 Normal
    normal = doc.styles["Normal"]
    based = cur_el.find(qn("w:basedOn"))
    if based is None:
        based = etree.Element(qn("w:basedOn"))
        # after name
        name_el = cur_el.find(qn("w:name"))
        idx = list(cur_el).index(name_el) + 1 if name_el is not None else 0
        cur_el.insert(idx, based)
    based.set(qn("w:val"), normal.style_id)


def fix_heading_styles(doc: Document):
    """一级标题：左对齐、小二黑体；不要默认居中。"""
    s = doc.styles["学位论文一级标题"]
    el = s.element
    pPr = el.find(qn("w:pPr"))
    if pPr is None:
        pPr = etree.SubElement(el, qn("w:pPr"))
    jc = pPr.find(qn("w:jc"))
    if jc is not None:
        pPr.remove(jc)  # 章标题左对齐；特殊节单独居中
    # 确保字号 36
    rPr = el.find(qn("w:rPr"))
    if rPr is None:
        rPr = etree.SubElement(el, qn("w:rPr"))
    sz = rPr.find(qn("w:sz"))
    if sz is None:
        sz = etree.SubElement(rPr, qn("w:sz"))
    sz.set(qn("w:val"), "36")
    szCs = rPr.find(qn("w:szCs"))
    if szCs is None:
        szCs = etree.SubElement(rPr, qn("w:szCs"))
    szCs.set(qn("w:val"), "36")
    rFonts = rPr.find(qn("w:rFonts"))
    if rFonts is None:
        rFonts = etree.Element(qn("w:rFonts"))
        rPr.insert(0, rFonts)
    rFonts.set(qn("w:ascii"), "黑体")
    rFonts.set(qn("w:hAnsi"), "黑体")
    rFonts.set(qn("w:eastAsia"), "黑体")


def style_ids(doc: Document):
    return {
        "l1": doc.styles["学位论文一级标题"].style_id,
        "l2": doc.styles["学位论文二级标题"].style_id,
        "l3": doc.styles["学位论文三级标题"].style_id,
        "body": doc.styles["学位论文正文"].style_id,
        "fig": doc.styles["图题"].style_id,
        "tab": doc.styles["表题"].style_id,
    }


def thesis_num_id(doc: Document) -> int:
    """学位论文标题样式绑定的 numId。"""
    el = doc.styles["学位论文一级标题"].element
    pPr = el.find(qn("w:pPr"))
    numPr = pPr.find(qn("w:numPr")) if pPr is not None else None
    if numPr is None:
        return 11
    nid = numPr.find(qn("w:numId"))
    return int(nid.get(qn("w:val"))) if nid is not None else 11


def fix_cover_english(doc: Document):
    """替换封面残留的水合物英文题。"""
    for i, p in enumerate(doc.paragraphs):
        t = (p.text or "").strip()
        if "Thermodynamics" in t or "Gas Hydrate" in t or "Surfactan" in t:
            # 找出连续英文题段落并替换
            # 封面出现两处（外封+内封），各三行
            pass
    # 更稳妥：按索引替换已知坏段落
    # 外封 6,7,8；内封 22,23,24（以当前稿为准，运行时再探测）
    bad_idxs = []
    for i, p in enumerate(doc.paragraphs):
        t = p.text or ""
        if any(k in t for k in ("Thermodynamics", "Gas Hydrate", "Surfactan", "Surfactant")):
            bad_idxs.append(i)
    # 分组为连续块
    blocks = []
    cur = []
    for i in bad_idxs:
        if not cur or i == cur[-1] + 1:
            cur.append(i)
        else:
            blocks.append(cur)
            cur = [i]
    if cur:
        blocks.append(cur)
    for block in blocks:
        # 用三行英文题覆盖；若块更长/更短，按块长度适配
        for j, idx in enumerate(block):
            line = EN_TITLE_LINES[j] if j < len(EN_TITLE_LINES) else ""
            if line:
                replace_paragraph_text(doc.paragraphs[idx], line)


def is_formula_like(text: str) -> bool:
    t = (text or "").strip()
    if not t:
        return False
    if re.search(r"（\s*\d+-\d+\s*）|\(\s*\d+-\d+\s*\)", t):
        return True
    if re.match(r"^[WGθρ].*=", t):
        return True
    if "→" in t and re.search(r"\d+-\d+", t):
        return True
    return False


def main():
    shutil.copy2(SRC, OUT)
    doc = Document(str(OUT))
    tpl = Document(str(TPL))

    fix_body_style(doc, tpl)
    fix_heading_styles(doc)
    ids = style_ids(doc)
    num_id = thesis_num_id(doc)
    print(f"styleIds={ids}, thesis_numId={num_id}")

    fix_cover_english(doc)

    # 映射标题
    mapped = {"h1": 0, "h2": 0, "h3": 0, "unnum": 0}
    for p in doc.paragraphs:
        style_name = p.style.name
        text = (p.text or "").strip()

        if style_name == "Heading 1" or (
            style_name == "学位论文一级标题" and text
        ):
            # 统一处理
            key = normalize_l1_key(text)
            is_appendix = key.startswith("附录")
            is_unnumbered = key in UNNUMBERED_L1 or is_appendix

            clean = key if is_unnumbered or is_appendix else strip_heading_number(text, 1)
            # 附录保留原文本（含 A）
            if is_appendix:
                clean = text.strip()
                # 去掉可能的全角多余空格，保留“附录 A”
                clean = re.sub(r"^附录\s*", "附录 ", clean)

            set_pstyle(p, ids["l1"])
            if clean != text:
                replace_paragraph_text(p, clean)

            if is_unnumbered:
                set_num(p, 0, 0)  # 取消自动编号
                set_jc(p, "center")
                mapped["unnum"] += 1
            else:
                # 使用样式默认编号；清除段落级覆盖，让样式生效
                set_num(p, None, None)
                set_jc(p, None)  # 左对齐
                mapped["h1"] += 1

            # 每章/主要一级节另起页
            set_page_break_before(p, True)
            continue

        if style_name == "Heading 2":
            clean = strip_heading_number(text, 2)
            set_pstyle(p, ids["l2"])
            set_num(p, None, None)
            set_jc(p, None)
            set_page_break_before(p, False)
            if clean != text:
                replace_paragraph_text(p, clean)
            mapped["h2"] += 1
            continue

        if style_name == "Heading 3" or style_name == "学位论文三级标题":
            clean = strip_heading_number(text, 3)
            set_pstyle(p, ids["l3"])
            set_num(p, None, None)
            set_jc(p, None)
            set_page_break_before(p, False)
            if clean != text:
                replace_paragraph_text(p, clean)
            mapped["h3"] += 1
            continue

        # 公式行：取消正文首行缩进观感（保持 Normal，居中常见）
        if style_name in ("Normal", "学位论文正文") and is_formula_like(text):
            # 公式保持 Normal，居中更接近模板示例
            set_pstyle(p, doc.styles["Normal"].style_id)
            set_jc(p, "center")
            pPr = ensure_pPr(p)
            ind = pPr.find(qn("w:ind"))
            if ind is None:
                ind = etree.SubElement(pPr, qn("w:ind"))
            ind.set(qn("w:firstLine"), "0")
            ind.set(qn("w:firstLineChars"), "0")

        # 空 Normal 夹在图题附近可保留

    # 致谢标题若仍是 Normal，升为一级无编号
    for p in doc.paragraphs:
        t = (p.text or "").strip()
        if t in ("致  谢", "致 谢", "致谢") and p.style.name not in (
            "学位论文一级标题",
        ):
            set_pstyle(p, ids["l1"])
            set_num(p, 0, 0)
            set_jc(p, "center")
            set_page_break_before(p, True)
            if t != "致  谢":
                replace_paragraph_text(p, "致  谢")

    # 附录二级：手工 A.n，取消自动编号
    appendix_idxs = [
        i
        for i, p in enumerate(doc.paragraphs)
        if p.style.name == "学位论文一级标题" and "附录" in (p.text or "")
    ]
    for astart in appendix_idxs:
        aend = len(doc.paragraphs)
        for j in range(astart + 1, len(doc.paragraphs)):
            if doc.paragraphs[j].style.name == "学位论文一级标题":
                aend = j
                break
        a_idx = 1
        for j in range(astart + 1, aend):
            p = doc.paragraphs[j]
            if p.style.name != "学位论文二级标题":
                continue
            t = (p.text or "").strip()
            t2 = re.sub(r"^A\.\d+\s*", "", t)
            replace_paragraph_text(p, f"A.{a_idx} {t2}".strip())
            set_num(p, 1, 0)
            a_idx += 1

    # 一级标题样式加粗；正文/图题/表题/参考文献硬性属性
    l1 = doc.styles["学位论文一级标题"].element
    rPr = l1.find(qn("w:rPr"))
    if rPr is None:
        rPr = etree.SubElement(l1, qn("w:rPr"))
    if rPr.find(qn("w:b")) is None:
        etree.SubElement(rPr, qn("w:b"))
    if rPr.find(qn("w:bCs")) is None:
        etree.SubElement(rPr, qn("w:bCs"))

    body = doc.styles["学位论文正文"].element
    pPr = body.find(qn("w:pPr"))
    if pPr is None:
        pPr = etree.SubElement(body, qn("w:pPr"))
    spacing = pPr.find(qn("w:spacing"))
    if spacing is None:
        spacing = etree.SubElement(pPr, qn("w:spacing"))
    spacing.set(qn("w:line"), "400")
    spacing.set(qn("w:lineRule"), "exact")
    ind = pPr.find(qn("w:ind"))
    if ind is None:
        ind = etree.SubElement(pPr, qn("w:ind"))
    ind.set(qn("w:firstLine"), "480")
    ind.set(qn("w:firstLineChars"), "200")

    for name in ("图题", "表题"):
        el = doc.styles[name].element
        rPr = el.find(qn("w:rPr"))
        if rPr is None:
            rPr = etree.SubElement(el, qn("w:rPr"))
        rf = rPr.find(qn("w:rFonts"))
        if rf is None:
            rf = etree.Element(qn("w:rFonts"))
            rPr.insert(0, rf)
        rf.set(qn("w:ascii"), "Times New Roman")
        rf.set(qn("w:hAnsi"), "Times New Roman")
        rf.set(qn("w:eastAsia"), "宋体")
        sz = rPr.find(qn("w:sz"))
        if sz is None:
            sz = etree.SubElement(rPr, qn("w:sz"))
        sz.set(qn("w:val"), "21")

    try:
        ref = doc.styles["参考文献"].element
        pPr = ref.find(qn("w:pPr"))
        if pPr is None:
            pPr = etree.SubElement(ref, qn("w:pPr"))
        spacing = pPr.find(qn("w:spacing"))
        if spacing is None:
            spacing = etree.SubElement(pPr, qn("w:spacing"))
        spacing.set(qn("w:line"), "400")
        spacing.set(qn("w:lineRule"), "exact")
        ind = pPr.find(qn("w:ind"))
        if ind is None:
            ind = etree.SubElement(pPr, qn("w:ind"))
        ind.set(qn("w:left"), "480")
        ind.set(qn("w:hanging"), "480")
        rPr = ref.find(qn("w:rPr"))
        if rPr is None:
            rPr = etree.SubElement(ref, qn("w:rPr"))
        sz = rPr.find(qn("w:sz"))
        if sz is None:
            sz = etree.SubElement(rPr, qn("w:sz"))
        sz.set(qn("w:val"), "21")
    except KeyError:
        pass

    # 关键词行：保证“关键词：”加粗
    for p in doc.paragraphs:
        t = p.text or ""
        if t.startswith("关键词") or t.startswith("Keywords"):
            # 拆成 label + content
            if "：" in t:
                label, content = t.split("：", 1)
                label = label + "："
            elif ":" in t:
                label, content = t.split(":", 1)
                label = label + ":"
                content = content.lstrip()
            else:
                continue
            # 重建 runs
            for child in list(p._p):
                if child.tag == qn("w:r"):
                    p._p.remove(child)
            # label bold
            r1 = etree.SubElement(p._p, qn("w:r"))
            rPr1 = etree.SubElement(r1, qn("w:rPr"))
            etree.SubElement(rPr1, qn("w:b"))
            t1 = etree.SubElement(r1, qn("w:t"))
            t1.text = label
            r2 = etree.SubElement(p._p, qn("w:r"))
            t2 = etree.SubElement(r2, qn("w:t"))
            if content.startswith(" "):
                t2.set("{http://www.w3.org/XML/1998/namespace}space", "preserve")
            t2.text = content
            # 关键词段用正文样式
            set_pstyle(p, ids["body"])

    doc.save(str(OUT))
    print(f"saved: {OUT}")
    print(f"mapped: {mapped}")

    # 自检
    doc2 = Document(str(OUT))
    from collections import Counter

    c = Counter(p.style.name for p in doc2.paragraphs)
    print("style counts:")
    for k, v in c.most_common(20):
        print(f"  {v:4d}  {k}")
    print("remaining Heading*:", sum(v for k, v in c.items() if k.startswith("Heading")))
    print("sample L1:")
    for i, p in enumerate(doc2.paragraphs):
        if p.style.name == "学位论文一级标题":
            pPr = p._p.find(qn("w:pPr"))
            num = None
            jc = None
            pb = False
            if pPr is not None:
                numPr = pPr.find(qn("w:numPr"))
                if numPr is not None:
                    nid = numPr.find(qn("w:numId"))
                    num = nid.get(qn("w:val")) if nid is not None else None
                jc_el = pPr.find(qn("w:jc"))
                jc = jc_el.get(qn("w:val")) if jc_el is not None else None
                pb = pPr.find(qn("w:pageBreakBefore")) is not None
            print(f"  {i}|num={num}|jc={jc}|pb={pb}|{p.text}")
    print("cover EN check:")
    for i, p in enumerate(doc2.paragraphs[:30]):
        if "Thermodynamics" in (p.text or "") or "Research on Autonomous" in (p.text or "") or "Mine" in (p.text or ""):
            print(i, p.text)


if __name__ == "__main__":
    main()
