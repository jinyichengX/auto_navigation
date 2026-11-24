#!/usr/bin/env python3
"""以官方排版模板为骨架，迁入学术重写正文，严格对齐模板样式用法。"""
from __future__ import annotations

import re
import shutil
from copy import deepcopy
from pathlib import Path

from docx import Document
from docx.oxml.ns import qn
from lxml import etree

TPL = Path("/home/jinyicheng/auto_navigation/thesis_build/format_ref/official_template.docx")
SRC = Path("/home/jinyicheng/auto_navigation/thesis_build/矿井未知环境无人机自主探索技术研究_学术重写版.docx")
OUT = Path("/home/jinyicheng/auto_navigation/thesis_build/矿井未知环境无人机自主探索技术研究_官方格式版.docx")

CN_TITLE = "矿井未知环境无人机自主探索技术研究"
EN_TITLE_LINES = [
    "Research on Autonomous Exploration Technology",
    "of Unmanned Aerial Vehicles",
    "in Unknown Mine Environments",
]
DATE_CN = "二〇二六年六月"

FRONT_TITLES = {
    "致  谢",
    "致 谢",
    "致谢",
    "摘  要",
    "摘 要",
    "摘要",
    "Abstract",
    "目  录",
    "目 录",
    "目录",
    "变量注释表",
}
UNNUM_CHAPTERS = {"参考文献", "作者简介", "作者简历"}
L1_NUM_RE = re.compile(r"^\s*\d+\s+")
L23_NUM_RE = re.compile(r"^\s*(?:[A-Za-z]\.)?\d+(?:\.\d+){0,3}(?:\s+|(?=[\u4e00-\u9fff]))")


def ensure_rPr(r):
    rPr = r.find(qn("w:rPr"))
    if rPr is None:
        rPr = etree.Element(qn("w:rPr"))
        r.insert(0, rPr)
    return rPr


def set_run_font(r, *, eastAsia=None, ascii_font=None, size=None, bold=None):
    rPr = ensure_rPr(r)
    rf = rPr.find(qn("w:rFonts"))
    if rf is None:
        rf = etree.Element(qn("w:rFonts"))
        rPr.insert(0, rf)
    if eastAsia:
        rf.set(qn("w:eastAsia"), eastAsia)
    if ascii_font:
        rf.set(qn("w:ascii"), ascii_font)
        rf.set(qn("w:hAnsi"), ascii_font)
    if size is not None:
        sz = rPr.find(qn("w:sz"))
        if sz is None:
            sz = etree.SubElement(rPr, qn("w:sz"))
        sz.set(qn("w:val"), str(size))
        szCs = rPr.find(qn("w:szCs"))
        if szCs is None:
            szCs = etree.SubElement(rPr, qn("w:szCs"))
        szCs.set(qn("w:val"), str(size))
    if bold is True:
        if rPr.find(qn("w:b")) is None:
            etree.SubElement(rPr, qn("w:b"))
        if rPr.find(qn("w:bCs")) is None:
            etree.SubElement(rPr, qn("w:bCs"))
    elif bold is False:
        for tag in ("w:b", "w:bCs"):
            el = rPr.find(qn(tag))
            if el is not None:
                rPr.remove(el)


def clear_runs(p_el):
    for child in list(p_el):
        if child.tag == qn("w:r"):
            p_el.remove(child)


def set_paragraph_text(p_el, text: str, *, eastAsia=None, ascii_font=None, size=None, bold=None):
    clear_runs(p_el)
    r = etree.SubElement(p_el, qn("w:r"))
    set_run_font(r, eastAsia=eastAsia, ascii_font=ascii_font, size=size, bold=bold)
    t = etree.SubElement(r, qn("w:t"))
    if text[:1].isspace() or text[-1:].isspace():
        t.set("{http://www.w3.org/XML/1998/namespace}space", "preserve")
    t.text = text


def ensure_pPr(p_el):
    pPr = p_el.find(qn("w:pPr"))
    if pPr is None:
        pPr = etree.Element(qn("w:pPr"))
        p_el.insert(0, pPr)
    return pPr


def set_pstyle(p_el, style_id: str):
    pPr = ensure_pPr(p_el)
    pStyle = pPr.find(qn("w:pStyle"))
    if pStyle is None:
        pStyle = etree.Element(qn("w:pStyle"))
        pPr.insert(0, pStyle)
    pStyle.set(qn("w:val"), style_id)


def set_jc(p_el, val: str | None):
    pPr = ensure_pPr(p_el)
    jc = pPr.find(qn("w:jc"))
    if val is None:
        if jc is not None:
            pPr.remove(jc)
        return
    if jc is None:
        jc = etree.SubElement(pPr, qn("w:jc"))
    jc.set(qn("w:val"), val)


def set_page_break(p_el, enabled: bool):
    pPr = ensure_pPr(p_el)
    pb = pPr.find(qn("w:pageBreakBefore"))
    if enabled:
        if pb is None:
            etree.SubElement(pPr, qn("w:pageBreakBefore"))
    elif pb is not None:
        pPr.remove(pb)


def set_num_override(p_el, num_id: int | None, ilvl: int | None = 0):
    """num_id=0 取消编号；None 表示清除段落级覆盖，继承样式。"""
    pPr = ensure_pPr(p_el)
    numPr = pPr.find(qn("w:numPr"))
    if num_id is None:
        if numPr is not None:
            pPr.remove(numPr)
        return
    if numPr is None:
        numPr = etree.Element(qn("w:numPr"))
        pStyle = pPr.find(qn("w:pStyle"))
        idx = list(pPr).index(pStyle) + 1 if pStyle is not None else 0
        pPr.insert(idx, numPr)
    for c in list(numPr):
        numPr.remove(c)
    if ilvl is not None:
        il = etree.SubElement(numPr, qn("w:ilvl"))
        il.set(qn("w:val"), str(ilvl))
    nid = etree.SubElement(numPr, qn("w:numId"))
    nid.set(qn("w:val"), str(num_id))


def normalize_front_title(text: str) -> str:
    t = re.sub(r"\s+", " ", (text or "").strip())
    mapping = {
        "摘要": "摘  要",
        "摘 要": "摘  要",
        "致谢": "致  谢",
        "致 谢": "致  谢",
        "目录": "目  录",
        "目 录": "目  录",
    }
    return mapping.get(t, text.strip() if text.strip() in FRONT_TITLES else mapping.get(t, t))


def strip_l1(text: str) -> str:
    t = (text or "").strip()
    if t.startswith("附录"):
        return t
    return L1_NUM_RE.sub("", t).strip()


def strip_l23(text: str) -> str:
    t = (text or "").strip()
    return L23_NUM_RE.sub("", t).strip()


def make_p(style_id: str, text: str = "", *, jc=None, page_break=False, num_id=None, ilvl=0,
           eastAsia=None, ascii_font=None, size=None, bold=None) -> etree._Element:
    p = etree.Element(qn("w:p"))
    set_pstyle(p, style_id)
    if jc is not None:
        set_jc(p, jc)
    if page_break:
        set_page_break(p, True)
    if num_id is not None:
        set_num_override(p, num_id, ilvl)
    if text:
        set_paragraph_text(p, text, eastAsia=eastAsia, ascii_font=ascii_font, size=size, bold=bold)
    return p


def keyword_paragraph(style_id: str, label: str, content: str) -> etree._Element:
    p = etree.Element(qn("w:p"))
    set_pstyle(p, style_id)
    r1 = etree.SubElement(p, qn("w:r"))
    set_run_font(r1, eastAsia="宋体", ascii_font="Times New Roman", size=24, bold=True)
    t1 = etree.SubElement(r1, qn("w:t"))
    t1.text = label
    r2 = etree.SubElement(p, qn("w:r"))
    set_run_font(r2, eastAsia="宋体", ascii_font="Times New Roman", size=24, bold=False)
    t2 = etree.SubElement(r2, qn("w:t"))
    if content[:1].isspace():
        t2.set("{http://www.w3.org/XML/1998/namespace}space", "preserve")
    t2.text = content
    return p


def has_drawing(el) -> bool:
    xml = etree.tostring(el)
    return b"w:drawing" in xml or b"w:pict" in xml or b"v:imagedata" in xml


def clone_with_media(src_el, rel_map: dict[str, str]):
    """深拷贝段落/表格，并重写 rId。"""
    el = deepcopy(src_el)
    for blip in el.findall(".//{http://schemas.openxmlformats.org/drawingml/2006/main}blip"):
        embed = blip.get(qn("r:embed"))
        if embed in rel_map:
            blip.set(qn("r:embed"), rel_map[embed])
    for imagedata in el.findall(".//{urn:schemas-microsoft-com:vml}imagedata"):
        rid = imagedata.get(qn("r:id"))
        if rid in rel_map:
            imagedata.set(qn("r:id"), rel_map[rid])
    return el


def collect_image_rels(doc: Document) -> dict[str, str]:
    """rid -> target path relative to word/"""
    rels = {}
    for rel in doc.part.rels.values():
        if "image" in rel.reltype:
            rels[rel.rId] = rel.target_ref
    return rels


def copy_media_to_template(tpl_doc: Document, src_doc: Document) -> dict[str, str]:
    """把 SRC 图片复制进模板，返回 src_rId -> new_rId。"""
    from io import BytesIO
    from docx.opc.constants import RELATIONSHIP_TYPE as RT
    from docx.opc.packuri import PackURI
    from docx.parts.image import ImagePart
    from docx.image.image import Image

    mapping = {}
    # 已有 media 最大编号
    existing = []
    for rel in tpl_doc.part.rels.values():
        if "image" in rel.reltype:
            existing.append(rel.target_ref)
    next_i = 1
    for rel in src_doc.part.rels.values():
        if "image" not in rel.reltype:
            continue
        blob = rel.target_part.blob
        content_type = rel.target_part.content_type
        ext = {
            "image/png": ".png",
            "image/jpeg": ".jpeg",
            "image/jpg": ".jpg",
            "image/gif": ".gif",
            "image/x-emf": ".emf",
            "image/x-wmf": ".wmf",
        }.get(content_type, ".bin")
        # 优先走标准 API；JPEG APP13 等异常则手工挂载
        try:
            rId, _img = tpl_doc.part.get_or_add_image(BytesIO(blob))
            mapping[rel.rId] = rId
            continue
        except Exception:
            pass
        while True:
            name = f"media/image{next_i}{ext}"
            next_i += 1
            if name not in existing:
                break
        partname = PackURI(f"/word/{name}")
        image_part = ImagePart(partname, content_type, blob, None)
        rId = tpl_doc.part.relate_to(image_part, RT.IMAGE)
        existing.append(name)
        mapping[rel.rId] = rId
    return mapping


def replace_cover_titles(body, paras):
    """替换模板封面中英文题目。"""
    # 外封中文题：模板 paras 3,4（两行）；我们改成一行+空行或两行拆分
    # 内封：12,13
    # 英文：5-7 与 14-16
    # 用文本匹配更稳
    cn_idxs = []
    en_idxs = []
    for i, p in enumerate(paras):
        t = (p.text or "").strip()
        if "瓦斯水合物" in t or "表面活性剂影响研究" in t:
            cn_idxs.append(i)
        if "Thermodynamics" in t or "Gas Hydrate" in t or "Surfactan" in t:
            en_idxs.append(i)

    # 中文题：每个连续块替换为 CN_TITLE 放第一行，其余清空
    def blocks(idxs):
        out, cur = [], []
        for i in idxs:
            if not cur or i == cur[-1] + 1:
                cur.append(i)
            else:
                out.append(cur)
                cur = [i]
        if cur:
            out.append(cur)
        return out

    for block in blocks(cn_idxs):
        set_paragraph_text(paras[block[0]]._p, CN_TITLE, eastAsia="黑体", ascii_font="Times New Roman", size=28, bold=True)
        set_jc(paras[block[0]]._p, "center")
        for i in block[1:]:
            clear_runs(paras[i]._p)

    for block in blocks(en_idxs):
        for j, i in enumerate(block):
            line = EN_TITLE_LINES[j] if j < len(EN_TITLE_LINES) else ""
            if line:
                set_paragraph_text(paras[i]._p, line, eastAsia="Times New Roman", ascii_font="Times New Roman", size=28, bold=False)
                set_jc(paras[i]._p, "center")
            else:
                clear_runs(paras[i]._p)

    # 原创性声明中的书名号题目
    for p in paras:
        t = p.text or ""
        if "所呈交的学位论文" in t and "《" in t:
            # 重建整段，保留声明文案并插入题目
            new = t
            # 模板常是《                 》
            new = re.sub(r"《.*?》", f"《{CN_TITLE}》", new, count=1)
            # 若跨段：本段只有前半
            if "《" in t and "》" not in t:
                # 清到书名号后，后面段落处理
                set_paragraph_text(p._p, f"本人郑重声明：所呈交的学位论文《{CN_TITLE}", eastAsia="楷体", ascii_font="Times New Roman", size=24)
            else:
                set_paragraph_text(p._p, new, eastAsia="楷体", ascii_font="Times New Roman", size=24)


def find_ack_index(paras) -> int:
    for i, p in enumerate(paras):
        if (p.text or "").strip() in ("致  谢", "致 谢", "致谢"):
            return i
    raise RuntimeError("模板中未找到致谢标题")


def build_content_elements(src: Document, style_ids: dict, img_map: dict[str, str]):
    """从 SRC 致谢开始构造要插入模板的元素列表。"""
    elements = []
    body = src.element.body
    children = [c for c in body if c.tag in (qn("w:p"), qn("w:tbl"))]

    # map paragraph element -> paragraph object for style/text
    pmap = {p._p: p for p in src.paragraphs}

    started = False
    appendix_mode = False
    appendix_sec_i = 0
    in_front_matter = True  # 致谢/摘要/目录区：用 Normal/Indent，不用学位论文正文

    for child in children:
        if child.tag == qn("w:tbl"):
            if not started:
                continue
            elements.append(clone_with_media(child, img_map))
            continue

        p = pmap.get(child)
        if p is None:
            continue
        text = (p.text or "").strip()
        style = p.style.name

        if not started:
            if text in ("致  谢", "致 谢", "致谢"):
                started = True
            else:
                continue

        # 跳过旧目录条目（保留标题，目录留给 Word 更新域；先放占位提示）
        if style.startswith("toc"):
            continue

        # 前端标题：致谢/摘要/Abstract/目录/变量注释表
        if text in FRONT_TITLES or normalize_front_title(text) in ("致  谢", "摘  要", "Abstract", "目  录", "变量注释表"):
            title = normalize_front_title(text)
            if title == "致谢":
                title = "致  谢"
            el = make_p(
                style_ids["plain"],
                title,
                jc="center",
                page_break=True,
                eastAsia="黑体",
                ascii_font="Times New Roman",
                size=36,
                bold=True,
            )
            # 取消可能继承的编号
            set_num_override(el, 0, 0)
            elements.append(el)
            if title == "目  录":
                # 放一条提示，用户在 Word 中更新目录域
                tip = make_p(
                    style_ids["normal"],
                    "（请在 Word 中：引用 → 目录 → 自动目录，或右键已有目录“更新域”。）",
                    eastAsia="宋体",
                    ascii_font="Times New Roman",
                    size=24,
                )
                elements.append(tip)
            appendix_mode = False
            continue

        # 章标题
        if style in ("Heading 1", "学位论文一级标题"):
            in_front_matter = False
            raw = text
            if raw.startswith("附录") or raw in UNNUM_CHAPTERS:
                title = raw if raw.startswith("附录") else raw
                el = make_p(
                    style_ids["l1"],
                    title,
                    jc="center" if raw in UNNUM_CHAPTERS else None,
                    page_break=True,
                    num_id=0,
                    ilvl=0,
                    eastAsia="黑体",
                    ascii_font="Times New Roman",
                    size=36,
                    bold=True,
                )
                elements.append(el)
                appendix_mode = raw.startswith("附录")
                appendix_sec_i = 0
                continue

            title = strip_l1(raw)
            el = make_p(
                style_ids["l1"],
                title,
                page_break=True,
                # 继承样式编号
                eastAsia="黑体",
                ascii_font="Times New Roman",
                size=36,
                bold=True,
            )
            # 确保不取消编号
            set_num_override(el, None)
            elements.append(el)
            appendix_mode = False
            continue

        # 二级
        if style in ("Heading 2", "学位论文二级标题"):
            if appendix_mode:
                appendix_sec_i += 1
                title = f"A.{appendix_sec_i} {strip_l23(text)}"
                el = make_p(
                    style_ids["l2"],
                    title,
                    num_id=0,
                    ilvl=1,
                    eastAsia="黑体",
                    ascii_font="Times New Roman",
                    size=30,
                    bold=True,
                )
            else:
                title = strip_l23(text)
                el = make_p(
                    style_ids["l2"],
                    title,
                    eastAsia="黑体",
                    ascii_font="Times New Roman",
                    size=30,
                    bold=True,
                )
                set_num_override(el, None)
            elements.append(el)
            continue

        # 三级
        if style in ("Heading 3", "学位论文三级标题"):
            title = strip_l23(text)
            el = make_p(
                style_ids["l3"],
                title,
                eastAsia="黑体",
                ascii_font="Times New Roman",
                size=28,
                bold=True,
            )
            set_num_override(el, None)
            elements.append(el)
            continue

        # 图题/表题：按模板用 Normal + 五号居中
        if style in ("图题", "表题") or (
            text.startswith("图 ") or text.startswith("表 ") or text.startswith("图") and re.match(r"^图\s*\d", text) or re.match(r"^表\s*\d", text)
        ):
            if style in ("图题", "表题") or re.match(r"^[图表]\s*\d", text):
                el = make_p(
                    style_ids["normal"],
                    text,
                    jc="center",
                    eastAsia="宋体",
                    ascii_font="Times New Roman",
                    size=21,
                    bold=False,
                )
                elements.append(el)
                continue

        # 关键词
        if text.startswith("关键词") or text.startswith("Keywords"):
            if "：" in text:
                label, content = text.split("：", 1)
                label += "："
            elif ":" in text:
                label, content = text.split(":", 1)
                label = label + ": "
                content = content.lstrip()
            else:
                label, content = text, ""
            elements.append(keyword_paragraph(style_ids["normal"], label, content))
            continue

        # 参考文献条目
        if style == "参考文献":
            el = clone_with_media(child, img_map)
            set_pstyle(el, style_ids.get("ref", style_ids["normal"]))
            elements.append(el)
            continue

        # 正文 / 其他：若含图则整段克隆；否则按正文样式重建文本
        if has_drawing(child) or style == "Normal" and has_drawing(child):
            el = clone_with_media(child, img_map)
            # 图片段保持居中
            set_jc(el, "center")
            elements.append(el)
            continue

        if not text:
            # 保留少量空行（致谢后等），避免完全压扁
            el = make_p(style_ids["normal"], "")
            elements.append(el)
            continue

        # 公式行粗判：居中 Normal
        if re.search(r"（\s*\d+-\d+\s*）|\(\s*\d+-\d+\s*\)", text) or (re.match(r"^[A-Za-zθρWG].*=", text) and len(text) < 120):
            el = make_p(
                style_ids["normal"],
                text,
                jc="center",
                eastAsia="宋体",
                ascii_font="Times New Roman",
                size=24,
            )
            elements.append(el)
            continue

        # 前置区用 Body Text Indent 2/Normal；章节正文用 学位论文正文
        use_body = style_ids["indent2"] if in_front_matter else style_ids["body"]
        el = make_p(
            use_body,
            text,
            eastAsia="宋体",
            ascii_font="Times New Roman",
            size=24,
        )
        elements.append(el)

    return elements


def extract_section_break_paragraphs(tpl_doc: Document):
    """从官方模板提取关键分节段落（含 sectPr）。

    Word 中 sectPr 定义的是“到该段落为止”的节属性：
    - 盲审封面末/摘要前：封面区结束
    - 绪论前：摘要/目录区（大写罗马页码）结束，正文节开始
    """
    body = tpl_doc.element.body
    children = list(body)
    sect_paras = []
    for child in children:
        if child.tag != qn("w:p"):
            continue
        if child.find(".//" + qn("w:sectPr")) is None:
            continue
        texts = "".join(t.text or "" for t in child.findall(".//" + qn("w:t"))).strip()
        sect_paras.append((child, texts))
    return sect_paras


def make_section_break_para(style_id: str, sect_pr_src) -> etree._Element:
    p = etree.Element(qn("w:p"))
    pPr = etree.SubElement(p, qn("w:pPr"))
    pStyle = etree.SubElement(pPr, qn("w:pStyle"))
    pStyle.set(qn("w:val"), style_id)
    pPr.append(deepcopy(sect_pr_src))
    return p


def main():
    shutil.copy2(TPL, OUT)
    tpl = Document(str(OUT))
    src = Document(str(SRC))

    style_ids = {
        "plain": tpl.styles["Plain Text"].style_id,
        "normal": tpl.styles["Normal"].style_id,
        "indent2": tpl.styles["Body Text Indent 2"].style_id,
        "l1": tpl.styles["学位论文一级标题"].style_id,
        "l2": tpl.styles["学位论文二级标题"].style_id,
        "l3": tpl.styles["学位论文三级标题"].style_id,
        "body": tpl.styles["学位论文正文"].style_id,
        "web": tpl.styles["Normal (Web)"].style_id,
    }
    try:
        style_ids["ref"] = tpl.styles["参考文献"].style_id
    except KeyError:
        style_ids["ref"] = style_ids["normal"]

    print("style_ids", style_ids)

    # 先提取模板分节，再改封面/删内容
    sect_paras = extract_section_break_paragraphs(tpl)
    if len(sect_paras) < 2:
        raise RuntimeError(f"模板分节不足: {len(sect_paras)}")
    # 第1个分节：摘要前；第2个：绪论前（罗马页码节结束）
    sect_before_abstract = sect_paras[0][0].find(".//" + qn("w:sectPr"))
    sect_before_chapter = sect_paras[1][0].find(".//" + qn("w:sectPr"))
    print("captured section breaks", len(sect_paras))

    # 1) 封面题目
    replace_cover_titles(tpl.element.body, tpl.paragraphs)

    # 2) 复制图片
    img_map = copy_media_to_template(tpl, src)
    print("copied images", len(img_map))

    # 3) 删除模板致谢及之后全部内容（保留最终 body sectPr）
    body = tpl.element.body
    ack_i = find_ack_index(tpl.paragraphs)
    ack_el = tpl.paragraphs[ack_i]._p
    deleting = False
    for child in list(body):
        if child is ack_el:
            deleting = True
        if not deleting:
            continue
        if child.tag == qn("w:sectPr"):
            continue
        body.remove(child)

    # 4) 构造正文，并在关键位置插入分节
    raw_elements = build_content_elements(src, style_ids, img_map)
    elements = []
    inserted_abs_break = False
    inserted_ch_break = False
    for el in raw_elements:
        texts = "".join(t.text or "" for t in el.findall(".//" + qn("w:t"))).strip()
        # 摘要前插入封面区分节
        if not inserted_abs_break and texts == "摘  要":
            elements.append(make_section_break_para(style_ids["normal"], sect_before_abstract))
            inserted_abs_break = True
        # 第一章前插入前置区（罗马页码）分节
        if (
            not inserted_ch_break
            and el.find(qn("w:pPr")) is not None
            and el.find(qn("w:pPr")).find(qn("w:pStyle")) is not None
            and el.find(qn("w:pPr")).find(qn("w:pStyle")).get(qn("w:val")) == style_ids["l1"]
            and texts == "绪论"
        ):
            elements.append(make_section_break_para(style_ids["normal"], sect_before_chapter))
            inserted_ch_break = True
        elements.append(el)

    if not inserted_abs_break or not inserted_ch_break:
        raise RuntimeError(
            f"分节插入失败 abs={inserted_abs_break} chapter={inserted_ch_break}"
        )

    sect = body.find(qn("w:sectPr"))
    for el in elements:
        if sect is not None:
            sect.addprevious(el)
        else:
            body.append(el)

    # 5) 正文节页眉改用 STYLEREF 章节页眉（模板绪论章使用的 header5/6）
    # 模板末尾 body sectPr 指向空页眉，必须替换。
    children_tpl = list(
        etree.fromstring(
            __import__("zipfile").ZipFile(TPL).read("word/document.xml")
        ).find(qn("w:body"))
    )
    # 模板第三个分节（绪论章结束处）携带正确正文页眉页脚
    sect_idxs = [
        i
        for i, c in enumerate(children_tpl)
        if c.find(".//" + qn("w:sectPr")) is not None
    ]
    chapter_sect = deepcopy(children_tpl[sect_idxs[2]].find(".//" + qn("w:sectPr")))
    final = body.find(qn("w:sectPr"))
    for tag in ("w:headerReference", "w:footerReference", "w:pgNumType", "w:titlePg"):
        for el in list(final.findall(qn(tag))):
            final.remove(el)
    insert_at = 0
    for el in chapter_sect:
        if el.tag in (
            qn("w:headerReference"),
            qn("w:footerReference"),
            qn("w:pgNumType"),
            qn("w:titlePg"),
        ):
            final.insert(insert_at, deepcopy(el))
            insert_at += 1

    tpl.save(str(OUT))
    print("saved", OUT)

    # 自检
    doc = Document(str(OUT))
    from collections import Counter

    c = Counter(p.style.name for p in doc.paragraphs)
    print("para", len(doc.paragraphs), "tables", len(doc.tables), "sections", len(doc.sections))
    print("styles top:")
    for k, v in c.most_common(15):
        print(f"  {v:4d} {k}")
    print("Heading left", sum(v for k, v in c.items() if k.startswith("Heading")))
    print("front titles:")
    for p in doc.paragraphs:
        t = (p.text or "").strip()
        if t in ("致  谢", "摘  要", "Abstract", "目  录", "变量注释表", "参考文献", "作者简介") or t.startswith("附录"):
            print(" ", p.style.name, repr(t))
        if p.style.name == "学位论文一级标题":
            print("  L1", repr(t))
    # cover
    print("cover sample:")
    for i, p in enumerate(doc.paragraphs[:20]):
        t = (p.text or "").strip()
        if t:
            print(i, t[:60])


if __name__ == "__main__":
    main()
