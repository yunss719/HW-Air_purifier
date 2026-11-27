#!/usr/bin/env python3
"""
Bambu Lab A1 mini용 초소형 DIY 공기청정기 (3단 테이퍼 도브테일 슬라이드-락)
순수 Python 3 바이너리 STL 생성기 + 매니폴드 검증 + 3D 프리뷰 렌더러

생성 파일:
  - cad/01_intake_base.stl        (120 x 120 x 38 mm + 수레일 2.2 mm)
  - cad/02_hepa_module.stl        (120 x 120 x 35 mm + 수레일 2.2 mm)
  - cad/03_carbon_fan_module.stl  (120 x 120 x 42 mm, 코너 M4 너트 포켓 4개)
  - cad/04_fan_grille.stl         (120 x 120 x 3 mm, M4 관통 홀 4개)
"""

import math
import os
import struct
from collections import defaultdict
from PIL import Image, ImageDraw

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.normpath(os.path.join(BASE_DIR, "..", "stl"))
PREVIEW_PATH = os.path.normpath(os.path.join(BASE_DIR, "..", "..", "docs", "images", "preview_exploded.png"))
os.makedirs(OUT_DIR, exist_ok=True)
os.makedirs(os.path.dirname(PREVIEW_PATH), exist_ok=True)

# ============================================================================
# 설계 파라미터 (mm)
# ============================================================================
W = 120.0
H_W = W / 2.0        # 60.0
WALL = 3.2
INNER = W - 2 * WALL # 113.6
H_IN = INNER / 2.0   # 56.8

H_INTAKE = 38.0
H_HEPA   = 35.0
H_CARBON = 42.0
H_GRILLE = 3.0

# 테이퍼 도브테일 슬라이드 파라미터
# 슬라이드 방향: -X(앞/좌측) -> +X(뒤/우측 스토퍼)
RAIL_YC    = 54.5    # 좌/우(±Y) 레일 중심 Y 좌표
DT_H       = 2.2     # 수레일 높이 (45도 오버행)
DT_GH      = 2.55    # 암레일 홈 깊이 (+0.35mm 상하 여유)
DT_NECK_F  = 2.6     # 앞쪽(-X) 수레일 목 폭
DT_NECK_B  = 3.0     # 뒤쪽(+X) 수레일 목 폭 (테이퍼)
DT_CLR_F   = 0.30    # 진입부 편측 공차
DT_CLR_B   = 0.12    # 끝단 압착 편측 공차
DT_X0      = -54.0   # 수레일 시작 X
DT_X1      =  53.0   # 수레일 끝 X (뒤쪽 스토퍼 여유)
GROOVE_X1  =  54.0   # 암레일 끝 X (뒤쪽 스토퍼 벽 6mm)

RIM_BOT_H  = 5.5     # 하단 슬라이드 림 높이
RIM_Y_IN   = 48.0    # 하단 좌/우 림 내측 경계 (내부 폭 96mm)
RIM_X_IN   = 52.0    # 하단 앞/뒤 림 내측 경계 (내부 폭 104mm)

# 120mm 팬 & M4 너트 포켓 파라미터
FAN_PITCH  = 105.0   # ±52.5 mm
FAN_HX     = FAN_PITCH / 2.0
M4_R       = 2.2     # Ø4.4 mm
NUT_W      = 7.4     # M4 너트 폭(7.0) + 0.4
NUT_H      = 3.6     # M4 너트 두께(3.2) + 0.4
NUT_Z0     = H_CARBON - 8.0  # 34.0 mm (상단 림에서 8mm 아래 -> M4x35~40mm 호환)


# ============================================================================
# 기본 메쉬 빌더 (Watertight B-Rep Primitives)
# ============================================================================
class Mesh:
    def __init__(self):
        self.tris = []  # list of ((x1,y1,z1), (x2,y2,z2), (x3,y3,z3))

    def add_tri(self, p1, p2, p3):
        # 퇴화 삼각형 방지
        ux, uy, uz = p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]
        vx, vy, vz = p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2]
        nx = uy*vz - uz*vy
        ny = uz*vx - ux*vz
        nz = ux*vy - uy*vx
        if nx*nx + ny*ny + nz*nz > 1e-12:
            self.tris.append((p1, p2, p3))

    def add_quad(self, p1, p2, p3, p4):
        """CCW 외향 기준 사각형 (p1->p2->p3->p4)"""
        self.add_tri(p1, p2, p3)
        self.add_tri(p1, p3, p4)

    def add_box(self, x0, x1, y0, y1, z0, z1):
        """축 정렬 직육면체 (x0<x1, y0<y1, z0<z1)"""
        if x1 <= x0 or y1 <= y0 or z1 <= z0:
            return
        v000 = (x0, y0, z0); v100 = (x1, y0, z0)
        v110 = (x1, y1, z0); v010 = (x0, y1, z0)
        v001 = (x0, y0, z1); v101 = (x1, y0, z1)
        v111 = (x1, y1, z1); v011 = (x0, y1, z1)
        # -Z (bottom)
        self.add_quad(v000, v010, v110, v100)
        # +Z (top)
        self.add_quad(v001, v101, v111, v011)
        # -Y (front)
        self.add_quad(v000, v100, v101, v001)
        # +Y (back)
        self.add_quad(v110, v010, v011, v111)
        # -X (left)
        self.add_quad(v010, v000, v001, v011)
        # +X (right)
        self.add_quad(v100, v110, v111, v101)

    def add_loft_x(self, x0, poly0_yz, x1, poly1_yz):
        """
        X축 방향으로 두 2D 다각형 단면(poly_yz: [(y,z), ...] CCW in Y-Z plane)을 연결한
        완벽한 닫힌 프리즘(테이퍼/경사 지원).
        """
        n = len(poly0_yz)
        assert n == len(poly1_yz) and n >= 3
        p0 = [(x0, y, z) for (y, z) in poly0_yz]
        p1 = [(x1, y, z) for (y, z) in poly1_yz]
        # -X cap (reverse winding so normal points -X)
        for i in range(1, n - 1):
            self.add_tri(p0[0], p0[i+1], p0[i])
        # +X cap (normal points +X)
        for i in range(1, n - 1):
            self.add_tri(p1[0], p1[i], p1[i+1])
        # side quads
        for i in range(n):
            j = (i + 1) % n
            self.add_quad(p0[i], p0[j], p1[j], p1[i])

    def add_loft_y(self, y0, poly0_xz, y1, poly1_xz):
        """
        Y축 방향으로 두 2D 다각형 단면(poly_xz: [(x,z), ...] CCW in X-Z plane)을 연결.
        주의: (X, Z) CCW의 외적은 -Y 방향이므로 cap 와인딩 조정.
        """
        n = len(poly0_xz)
        assert n == len(poly1_xz) and n >= 3
        p0 = [(x, y0, z) for (x, z) in poly0_xz]
        p1 = [(x, y1, z) for (x, z) in poly1_xz]
        # -Y cap (X x Z = -Y, so CCW in XZ points -Y!)
        for i in range(1, n - 1):
            self.add_tri(p0[0], p0[i], p0[i+1])
        # +Y cap
        for i in range(1, n - 1):
            self.add_tri(p1[0], p1[i+1], p1[i])
        # side quads
        for i in range(n):
            j = (i + 1) % n
            self.add_quad(p0[i], p1[i], p1[j], p0[j])

    def add_annular_ring(self, cx, cy, r_in, r_out, z0, z1, segs=64):
        """Z축 원형 링(도넛형 원통)"""
        for i in range(segs):
            a0 = 2.0 * math.pi * i / segs
            a1 = 2.0 * math.pi * (i + 1) / segs
            c0, s0 = math.cos(a0), math.sin(a0)
            c1, s1 = math.cos(a1), math.sin(a1)
            # outer
            o0b = (cx + r_out*c0, cy + r_out*s0, z0)
            o1b = (cx + r_out*c1, cy + r_out*s1, z0)
            o0t = (cx + r_out*c0, cy + r_out*s0, z1)
            o1t = (cx + r_out*c1, cy + r_out*s1, z1)
            # inner
            i0b = (cx + r_in*c0, cy + r_in*s0, z0)
            i1b = (cx + r_in*c1, cy + r_in*s1, z0)
            i0t = (cx + r_in*c0, cy + r_in*s0, z1)
            i1t = (cx + r_in*c1, cy + r_in*s1, z1)
            # outer wall
            self.add_quad(o0b, o1b, o1t, o0t)
            # inner wall
            self.add_quad(i1b, i0b, i0t, i1t)
            # top
            self.add_quad(i0t, o0t, o1t, i1t)
            # bottom
            self.add_quad(o0b, i0b, i1b, o1b)

    def add_cylinder(self, cx, cy, r, z0, z1, segs=48):
        """꽉 찬 원기둥"""
        cb = (cx, cy, z0)
        ct = (cx, cy, z1)
        for i in range(segs):
            a0 = 2.0 * math.pi * i / segs
            a1 = 2.0 * math.pi * (i + 1) / segs
            p0b = (cx + r*math.cos(a0), cy + r*math.sin(a0), z0)
            p1b = (cx + r*math.cos(a1), cy + r*math.sin(a1), z0)
            p0t = (cx + r*math.cos(a0), cy + r*math.sin(a0), z1)
            p1t = (cx + r*math.cos(a1), cy + r*math.sin(a1), z1)
            self.add_quad(p0b, p1b, p1t, p0t)
            self.add_tri(ct, p0t, p1t)
            self.add_tri(cb, p1b, p0b)

    def add_square_with_circle_hole(self, cx, cy, x0, x1, y0, y1, r_hole, z0, z1, segs=32):
        """
        직사각형 [x0,x1] x [y0,y1] 내부에 중심 (cx,cy), 반경 r_hole 원형 구멍이 뚫린 플레이트.
        원 둘레를 4분면으로 나누어 직사각형 4개 코너와 매핑 -> 100% 깔끔한 매니폴드!
        """
        assert segs % 4 == 0
        q = segs // 4
        # 각도 0 ~ 2pi 를 4개 변에 투영
        outer_pts = []
        inner_pts = []
        for i in range(segs):
            ang = 2.0 * math.pi * i / segs
            ca, sa = math.cos(ang), math.sin(ang)
            inner_pts.append((cx + r_hole * ca, cy + r_hole * sa))
            # 레이 (cx,cy) + t*(ca,sa) 가 사각형 경계와 만나는 최소 양수 t
            ts = []
            if ca > 1e-9:  ts.append((x1 - cx) / ca)
            if ca < -1e-9: ts.append((x0 - cx) / ca)
            if sa > 1e-9:  ts.append((y1 - cy) / sa)
            if sa < -1e-9: ts.append((y0 - cy) / sa)
            t = min(ts)
            outer_pts.append((cx + t * ca, cy + t * sa))

        for i in range(segs):
            j = (i + 1) % segs
            ox0, oy0 = outer_pts[i]; ox1, oy1 = outer_pts[j]
            ix0, iy0 = inner_pts[i]; ix1, iy1 = inner_pts[j]
            o0b = (ox0, oy0, z0); o1b = (ox1, oy1, z0)
            o0t = (ox0, oy0, z1); o1t = (ox1, oy1, z1)
            i0b = (ix0, iy0, z0); i1b = (ix1, iy1, z0)
            i0t = (ix0, iy0, z1); i1t = (ix1, iy1, z1)
            self.add_quad(o0b, o1b, o1t, o0t)
            self.add_quad(i1b, i0b, i0t, i1t)
            self.add_quad(i0t, o0t, o1t, i1t)
            self.add_quad(o0b, i0b, i1b, o1b)

    def add_hollow_rect(self, x0, x1, y0, y1, ix0, ix1, iy0, iy1, z0, z1):
        """사각 테두리 벽체 (4개 블록으로 깔끔하게 구성)"""
        self.add_box(x0, x1, y0, iy0, z0, z1)   # -Y wall
        self.add_box(x0, x1, iy1, y1, z0, z1)   # +Y wall
        self.add_box(x0, ix0, iy0, iy1, z0, z1) # -X wall
        self.add_box(ix1, x1, iy0, iy1, z0, z1) # +X wall

    def signed_volume(self):
        vol = 0.0
        for p1, p2, p3 in self.tris:
            vol += (
                p1[0] * (p2[1]*p3[2] - p2[2]*p3[1]) +
                p1[1] * (p2[2]*p3[0] - p2[0]*p3[2]) +
                p1[2] * (p2[0]*p3[1] - p2[1]*p3[0])
            ) / 6.0
        return vol

    def bbox(self):
        xs = [p[0] for t in self.tris for p in t]
        ys = [p[1] for t in self.tris for p in t]
        zs = [p[2] for t in self.tris for p in t]
        return (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))

    def save_stl(self, filepath, header_text="A1 mini DIY Air Purifier"):
        with open(filepath, "wb") as f:
            hdr = header_text.encode("ascii")[:80].ljust(80, b"\x00")
            f.write(hdr)
            f.write(struct.pack("<I", len(self.tris)))
            for p1, p2, p3 in self.tris:
                ux, uy, uz = p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]
                vx, vy, vz = p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2]
                nx = uy*vz - uz*vy
                ny = uz*vx - ux*vz
                nz = ux*vy - uy*vx
                nl = math.sqrt(nx*nx + ny*ny + nz*nz)
                if nl > 0:
                    nx /= nl; ny /= nl; nz /= nl
                f.write(struct.pack(
                    "<12fH",
                    nx, ny, nz,
                    p1[0], p1[1], p1[2],
                    p2[0], p2[1], p2[2],
                    p3[0], p3[1], p3[2],
                    0
                ))


# ============================================================================
# 공통 기구 요소: 테이퍼 도브테일 수레일(Male) & 암레일(Female)
# ============================================================================
def add_male_dovetails(mesh: Mesh, z_top: float):
    """모듈 상단 좌/우(±Y)에 45도 테이퍼 도브테일 수레일(Male) 추가"""
    for sgn in (-1.0, 1.0):
        yc = sgn * RAIL_YC
        nf = DT_NECK_F / 2.0
        nb = DT_NECK_B / 2.0
        hf = nf + DT_H
        hb = nb + DT_H
        # Y-Z 평면 CCW: (y_bottom_left, z_bot) -> (y_bottom_right, z_bot) -> (y_top_right, z_top) -> (y_top_left, z_top)
        poly0 = [
            (yc - nf, z_top),
            (yc + nf, z_top),
            (yc + hf, z_top + DT_H),
            (yc - hf, z_top + DT_H),
        ]
        poly1 = [
            (yc - nb, z_top),
            (yc + nb, z_top),
            (yc + hb, z_top + DT_H),
            (yc - hb, z_top + DT_H),
        ]
        mesh.add_loft_x(DT_X0, poly0, DT_X1, poly1)


def add_top_rail_shelf(mesh: Mesh, z_top: float):
    """
    좌/우(±Y) 상단 수레일을 지지하는 45도 챔퍼 선반 (서포트 0%).
    앞/뒤(±X) 벽은 두께 3.2mm 그대로 유지해 HEPA 필터 상단 삽입 공간을 101 x 113.6 mm 확보!
    """
    y_in_top = 50.5
    shelf_h  = 3.0
    chamf_h  = H_IN - y_in_top  # 56.8 - 50.5 = 6.3 mm -> 정확히 45도!
    z_mid    = z_top - shelf_h
    z_bot    = z_mid - chamf_h

    for sgn in (-1.0, 1.0):
        if sgn > 0:
            # +Y 측: y_in_top ~ H_IN (외벽 안쪽 추가 선반)
            poly = [
                (y_in_top, z_mid),
                (H_IN,     z_bot),
                (H_IN,     z_top),
                (y_in_top, z_top),
            ]
        else:
            # -Y 측
            poly = [
                (-H_IN,     z_bot),
                (-y_in_top, z_mid),
                (-y_in_top, z_top),
                (-H_IN,     z_top),
            ]
        mesh.add_loft_x(-H_IN, poly, H_IN, poly)


def add_bottom_female_slide_rim(mesh: Mesh):
    """
    모듈 하단(z = 0 ~ RIM_BOT_H)에 좌/우 테이퍼 도브테일 암레일 홈(Female)이
    정밀하게 파인 바닥 림(HEPA 필터 받침 턱 겸용)을 생성.
    - x = -60 ~ GROOVE_X1(+54): 암레일 홈 개방 (진입부 여유 0.30mm -> 끝단 압착 0.12mm)
    - x = GROOVE_X1(+54) ~ +60: 완전히 막힌 뒤쪽 스토퍼 벽!
    """
    # 1) 뒤쪽 스토퍼 벽 (+X: GROOVE_X1 ~ +H_W, 전체 폭)
    mesh.add_box(GROOVE_X1, H_W, -H_W, H_W, 0.0, RIM_BOT_H)
    # 2) 뒤쪽 받침 턱 연장 (+X: RIM_X_IN ~ GROOVE_X1, 중앙부)
    mesh.add_box(RIM_X_IN, GROOVE_X1, -RIM_Y_IN, RIM_Y_IN, 0.0, RIM_BOT_H)
    # 3) 앞쪽 받침 턱 (-X: -H_W ~ -RIM_X_IN, 중앙부)
    mesh.add_box(-H_W, -RIM_X_IN, -RIM_Y_IN, RIM_Y_IN, 0.0, RIM_BOT_H)

    # 4) 좌/우(±Y) 슬라이드 레일 본체 (x = -H_W ~ GROOVE_X1)
    # 홈의 천장 위쪽(z = DT_GH ~ RIM_BOT_H)은 꽉 찬 직육면체로 연결
    for sgn in (-1.0, 1.0):
        yc = sgn * RAIL_YC
        y_lo = RIM_Y_IN if sgn > 0 else -H_W
        y_hi = H_W      if sgn > 0 else -RIM_Y_IN

        # 천장 위 솔리드층 (z = DT_GH ~ RIM_BOT_H)
        mesh.add_box(-H_W, GROOVE_X1, y_lo, y_hi, DT_GH, RIM_BOT_H)

        # 홈 양옆 벽 (z = 0 ~ DT_GH): 테이퍼 45도 홈의 좌측 벽과 우측 벽을 loft_x로 정확히 생성!
        nf = DT_NECK_B / 2.0 + DT_CLR_F
        nb = DT_NECK_B / 2.0 + DT_CLR_B
        hf = nf + DT_GH
        hb = nb + DT_GH

        # y_lo 쪽 벽: y_lo ~ (yc - neck/head)
        poly0_a = [(y_lo, 0.0), (yc - nf, 0.0), (yc - hf, DT_GH), (y_lo, DT_GH)]
        poly1_a = [(y_lo, 0.0), (yc - nb, 0.0), (yc - hb, DT_GH), (y_lo, DT_GH)]
        mesh.add_loft_x(-H_W, poly0_a, GROOVE_X1, poly1_a)

        # y_hi 쪽 벽: (yc + neck/head) ~ y_hi
        poly0_b = [(yc + nf, 0.0), (y_hi, 0.0), (y_hi, DT_GH), (yc + hf, DT_GH)]
        poly1_b = [(yc + nb, 0.0), (y_hi, 0.0), (y_hi, DT_GH), (yc + hb, DT_GH)]
        mesh.add_loft_x(-H_W, poly0_b, GROOVE_X1, poly1_b)


# ============================================================================
# 파트 1: 하단 ㄷ자 반절 집중 흡기 모듈 (01_intake_base.stl)
# ============================================================================
def build_intake_base() -> Mesh:
    m = Mesh()
    floor_z = 2.4
    vent_z0 = 4.0
    vent_z1 = 29.0  # 높이 25mm 대형 ㄷ자 개구부

    # 1) 바닥판 (z = 0 ~ 2.4)
    m.add_box(-H_W, H_W, -H_W, H_W, 0.0, floor_z)

    # 2) 하단 테두리 벽 (z = 2.4 ~ 4.0)
    m.add_hollow_rect(-H_W, H_W, -H_W, H_W, -H_IN, H_IN, -H_IN, H_IN, floor_z, vent_z0)

    # 3) 흡기구 구간 (z = 4.0 ~ 29.0) — ㄷ자 반절 집중 흡기!
    #    - 우측(+X, 후면): 100% 밀폐
    m.add_box(H_IN, H_W, -H_W, H_W, vent_z0, vent_z1)
    #    - 상/하(±Y) 측면 뒤쪽 50% (x = 0 ~ H_IN): 100% 밀폐
    m.add_box(0.0, H_IN,  H_IN, H_W, vent_z0, vent_z1)
    m.add_box(0.0, H_IN, -H_W, -H_IN, vent_z0, vent_z1)
    #    - 좌측 앞 코너 기둥 2개 (x = -H_W ~ -52, ±Y 코너 보강)
    m.add_box(-H_W, -52.0,  H_IN, H_W, vent_z0, vent_z1)
    m.add_box(-H_W, -52.0, -H_W, -H_IN, vent_z0, vent_z1)
    m.add_box(-H_W, -H_IN,  52.0, H_IN, vent_z0, vent_z1)
    m.add_box(-H_W, -H_IN, -H_IN, -52.0, vent_z0, vent_z1)

    #    - 좌측(-X, 프린터 방향 100% 개방) 수직 보호 리브
    for y in (-40.0, -26.7, -13.3, 0.0, 13.3, 26.7, 40.0):
        m.add_box(-H_W, -H_IN, y - 1.0, y + 1.0, vent_z0, vent_z1)

    #    - 상/하(±Y, 앞쪽 50% 개방) 수직 보호 리브
    for x in (-40.0, -26.7, -13.3):
        m.add_box(x - 1.0, x + 1.0,  H_IN, H_W, vent_z0, vent_z1)
        m.add_box(x - 1.0, x + 1.0, -H_W, -H_IN, vent_z0, vent_z1)

    # 4) 내부 경사 배플 (+X 뒤쪽에서 -X 앞쪽으로 부드럽게 유도)
    #    X-Z 단면 (CCW): (-18, 2.4) -> (56.8, 2.4) -> (56.8, 22.0) -> (-18, 2.8)
    baffle_xz = [(-18.0, floor_z), (H_IN, floor_z), (H_IN, 22.0), (-18.0, floor_z + 0.4)]
    m.add_loft_y(-H_IN, baffle_xz, H_IN, baffle_xz)

    # 5) 상단 벽체 (z = 29.0 ~ 38.0) + 45도 지지 선반 + 수레일
    m.add_hollow_rect(-H_W, H_W, -H_W, H_W, -H_IN, H_IN, -H_IN, H_IN, vent_z1, H_INTAKE)
    add_top_rail_shelf(m, H_INTAKE)
    add_male_dovetails(m, H_INTAKE)
    return m


# ============================================================================
# 파트 2: 중단 HEPA 필터 모듈 (02_hepa_module.stl)
# ============================================================================
def build_hepa_module() -> Mesh:
    m = Mesh()
    # 1) 하단 슬라이드 림 (암레일 홈 + 뒤쪽 스토퍼 + HEPA 받침 턱)
    add_bottom_female_slide_rim(m)
    # 2) 중단~상단 4면 외벽 (z = 5.5 ~ 35.0 -> 내부 113.6 x 113.6 mm)
    m.add_hollow_rect(-H_W, H_W, -H_W, H_W, -H_IN, H_IN, -H_IN, H_IN, RIM_BOT_H, H_HEPA)
    # 3) 상단 45도 지지 선반 (좌/우만 -> 상단 개구부 101 x 113.6 mm) + 수레일
    add_top_rail_shelf(m, H_HEPA)
    add_male_dovetails(m, H_HEPA)
    return m


# ============================================================================
# 파트 3: 상단 활성탄 + M4 너트 포켓 팬 마운트 모듈 (03_carbon_fan_module.stl)
# ============================================================================
def build_carbon_fan_module() -> Mesh:
    m = Mesh()
    # 1) 하단 슬라이드 림 (암레일 홈 + 뒤쪽 스토퍼)
    add_bottom_female_slide_rim(m)

    # 2) 타공 바닥판 (z = 3.2 ~ 5.5, 두께 2.3mm, 슬롯 폭 2.4mm / 리브 2.2mm)
    #    3~4mm 펠릿 활성탄이 빠지지 않으면서 공기 저항 최소화
    mz0, mz1 = 3.2, RIM_BOT_H
    # X방향 가로 지지 빔 5개
    for y in (-40.0, -20.0, 0.0, 20.0, 40.0):
        m.add_box(-RIM_X_IN, RIM_X_IN, y - 1.2, y + 1.2, mz0, mz1)
    # Y방향 세로 리브 (간격 4.6mm -> 슬롯 폭 2.4mm)
    x = -RIM_X_IN + 2.4
    while x < RIM_X_IN - 2.2:
        m.add_box(x, x + 2.2, -RIM_Y_IN, RIM_Y_IN, mz0, mz1)
        x += 4.6

    # 3) 중단 외벽 (z = 5.5 ~ 42.0) — 단, ±Y 측면 너트 포켓 슬롯(z = 34.0 ~ 37.6)
    #    및 +X 뒷면 케이블 노치(z = 38.5 ~ 42.0) 구간은 분할하여 정확히 비움!
    nut_z1 = NUT_Z0 + NUT_H  # 37.6

    # -X 앞면 벽 (전체 밀폐)
    m.add_box(-H_W, -H_IN, -H_W, H_W, RIM_BOT_H, H_CARBON)

    # +X 뒷면 벽 (상단 중앙 y = -3 ~ +3, z = 38.5 ~ 42.0 케이블 인출 노치 제외)
    m.add_box(H_IN, H_W, -H_W, H_W, RIM_BOT_H, 38.5)
    m.add_box(H_IN, H_W, -H_W, -3.0, 38.5, H_CARBON)
    m.add_box(H_IN, H_W,  3.0,  H_W, 38.5, H_CARBON)

    # ±Y 측면 벽 (z = 5.5 ~ 34.0 아래쪽, z = 37.6 ~ 42.0 위쪽)
    for y0, y1 in ((-H_W, -H_IN), (H_IN, H_W)):
        m.add_box(-H_IN, H_IN, y0, y1, RIM_BOT_H, NUT_Z0)
        m.add_box(-H_IN, H_IN, y0, y1, nut_z1, H_CARBON)
        # z = NUT_Z0 ~ nut_z1 구간: 너트 슬롯 (±52.5 ± 3.7)을 제외한 3개 구간만 벽체 생성
        s1_lo, s1_hi = -FAN_HX - NUT_W/2.0, -FAN_HX + NUT_W/2.0  # -56.2 ~ -48.8
        s2_lo, s2_hi =  FAN_HX - NUT_W/2.0,  FAN_HX + NUT_W/2.0  #  48.8 ~  56.2
        m.add_box(-H_IN, s1_lo, y0, y1, NUT_Z0, nut_z1)
        m.add_box(s1_hi, s2_lo, y0, y1, NUT_Z0, nut_z1)
        m.add_box(s2_hi,  H_IN, y0, y1, NUT_Z0, nut_z1)

    # 4) 코너 4곳: M4 너트 포켓 기둥 (표준 105x105mm = ±52.5mm, 서포트 0%!)
    #    각 기둥 범위: 코너에서 안쪽으로 14mm x 14mm (외벽 내측 ±56.8 ~ ±44.5)
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            cx, cy = sx * FAN_HX, sy * FAN_HX
            px0 = -H_IN if sx < 0 else 44.5
            px1 = -44.5 if sx < 0 else H_IN
            py0 = -H_IN if sy < 0 else 44.5
            py1 = -44.5 if sy < 0 else H_IN

            # 4-1) 하단 45도 지지 챔퍼 (z = 16.0 ~ 28.0, 서포트 0%)
            #      Y축으로 좁아지는 45도 쐐기
            if sy > 0:
                ch_yz = [(py0, 28.0), (H_IN, 15.7), (H_IN, 28.0)]
            else:
                ch_yz = [(-H_IN, 15.7), (py1, 28.0), (-H_IN, 28.0)]
            m.add_loft_x(px0, ch_yz, px1, ch_yz)

            # 4-2) 기둥 하부 (z = 28.0 ~ 34.0, M4 볼트 통과 여유 구멍 Ø4.4mm 포함 -> 40mm 볼트 호환!)
            m.add_square_with_circle_hole(cx, cy, px0, px1, py0, py1, M4_R, 28.0, NUT_Z0, segs=32)

            # 4-3) 기둥 중단 (z = 34.0 ~ 37.6, M4 육각 너트 포켓 슬롯 개방!)
            #      슬롯 범위: X = [cx - NUT_W/2, cx + NUT_W/2], Y = 외벽쪽 ~ 너트 안쪽 끝
            nx0, nx1 = cx - NUT_W/2.0, cx + NUT_W/2.0
            ny_in = (cy - NUT_W/2.0) if sy > 0 else (cy + NUT_W/2.0)
            # 슬롯 좌/우 기둥 벽
            m.add_box(px0, nx0, py0, py1, NUT_Z0, nut_z1)
            m.add_box(nx1, px1, py0, py1, NUT_Z0, nut_z1)
            # 슬롯 뒤쪽(하우징 안쪽) 막힘 벽
            if sy > 0:
                m.add_box(nx0, nx1, py0, ny_in, NUT_Z0, nut_z1)
            else:
                m.add_box(nx0, nx1, ny_in, py1, NUT_Z0, nut_z1)

            # 4-4) 기둥 상부 (z = 37.6 ~ 42.0, 두께 4.4mm, M4 볼트 관통 구멍 Ø4.4mm)
            m.add_square_with_circle_hole(cx, cy, px0, px1, py0, py1, M4_R, nut_z1, H_CARBON, segs=32)

    return m


# ============================================================================
# 파트 4: 상단 120mm 팬 보호 그릴 (04_fan_grille.stl)
# ============================================================================
def build_fan_grille() -> Mesh:
    m = Mesh()
    z0, z1 = 0.0, H_GRILLE
    r_open = 55.5  # 중앙 원형 개구부 반경 (Ø111mm)

    # 1) 외곽 프레임 (120 x 120, 중앙 Ø111mm 개구부)
    #    코너 4곳에 M4 관통 구멍이 있어야 하므로, 중앙 90x90 사각 영역과
    #    네 모서리 15x15 블록 + 4개 변 스트립으로 분할해 완벽한 매니폴드 구성!
    #    잠깐: 원 반경 55.5mm > 45mm 이므로, 사각형 전체에 원형 구멍을 뚫으면
    #    코너 영역은 (±52.5, ±52.5)에서 원 중심까지 거리 = sqrt(52.5^2*2) = 74.2mm >> 55.5mm!
    #    따라서 코너 4개 블록 [±44.5, ±60] x [±44.5, ±60] 은 원과 전혀 안 겹친다!
    #    (44.5^2 + 44.5^2 = 3960 -> sqrt = 62.9mm > 55.5mm!)
    #    중앙 링 프레임 + 4개 코너 M4 홀 블록으로 나누면 100% 깔끔!

    # 1-1) 중앙 원형 링 (r_in = 55.5, r_out = 60.0)
    m.add_annular_ring(0.0, 0.0, r_open, H_W, z0, z1, segs=72)

    # 1-2) 120 x 120 외곽 사각 테두리 프레임 (폭 4.5mm -> 4개 코너와 링이 완벽히 일체화)
    m.add_box(-44.5, 44.5,  55.5,  H_W, z0, z1)  # +Y 변
    m.add_box(-44.5, 44.5, -H_W, -55.5, z0, z1)  # -Y 변
    m.add_box( 55.5,  H_W, -44.5, 44.5, z0, z1)  # +X 변
    m.add_box(-H_W, -55.5, -44.5, 44.5, z0, z1)  # -X 변

    # 1-3) 4개 코너 M4 볼트 홀 블록 ([±44.5 ~ ±60] x [±44.5 ~ ±60])
    #      + 45도 대각선 리브로 중앙 원형 링(r=55.5)과 코너 블록 사이를 빈틈없이 연결!
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            cx, cy = sx * FAN_HX, sy * FAN_HX
            bx0 = -H_W if sx < 0 else 44.5
            bx1 = -44.5 if sx < 0 else H_W
            by0 = -H_W if sy < 0 else 44.5
            by1 = -44.5 if sy < 0 else H_W
            m.add_square_with_circle_hole(cx, cy, bx0, bx1, by0, by1, M4_R, z0, z1, segs=32)

    # 2) 중앙 허브 원판 (Ø32mm)
    m.add_cylinder(0.0, 0.0, 16.0, z0, z1, segs=48)

    # 3) 동심원 보호 링 4개 (Ø50, 68, 86, 102 mm, 폭 1.8mm)
    for d in (50.0, 68.0, 86.0, 102.0):
        r = d / 2.0
        m.add_annular_ring(0.0, 0.0, r - 0.9, r + 0.9, z0, z1, segs=64)

    # 4) 8방향 방사형 지지 살 (폭 2.4mm, 대각선 45/135도는 코너 블록까지 연장해 강성 극대화)
    rib_w = 1.2
    for deg in (0, 45, 90, 135):
        rib_len = 66.0 if deg in (45, 135) else (r_open + 1.5)
        rad = math.radians(deg)
        ca, sa = math.cos(rad), math.sin(rad)
        pts = [
            (-rib_len*ca + rib_w*sa, -rib_len*sa - rib_w*ca),
            ( rib_len*ca + rib_w*sa,  rib_len*sa - rib_w*ca),
            ( rib_len*ca - rib_w*sa,  rib_len*sa + rib_w*ca),
            (-rib_len*ca - rib_w*sa, -rib_len*sa + rib_w*ca),
        ]
        pb = [(x, y, z0) for x, y in pts]
        pt = [(x, y, z1) for x, y in pts]
        m.add_quad(pb[0], pb[3], pb[2], pb[1])
        m.add_quad(pt[0], pt[1], pt[2], pt[3])
        for i in range(4):
            j = (i + 1) % 4
            m.add_quad(pb[i], pb[j], pt[j], pt[i])

    return m


# ============================================================================
# PIL 3D 프리뷰 렌더러 (생성된 실제 STL 메쉬 검증용)
# ============================================================================
def render_preview(parts_with_offsets, out_png, img_w=1200, img_h=900, elev_deg=26, azim_deg=40):
    """Painter's algorithm + Lambert shading으로 실제 STL 메쉬를 PNG로 렌더링 (위에서 내려다보는 정면 사시도)"""
    el = math.radians(elev_deg)
    az = math.radians(azim_deg)
    ca, sa = math.cos(az), math.sin(az)
    ce, se = math.cos(el), math.sin(el)

    def proj(p):
        x, y, z = p
        # Z축 회전 후 위에서 elev_deg 각도로 내려다보는 투영 (화면 +Y가 아래쪽)
        rx = x * ca - y * sa
        ry = x * sa + y * ca
        rz = z
        sx = rx
        sy = -ry * se - rz * ce
        depth = ry * ce - rz * se
        return sx, sy, depth

    draw_list = []
    all_sx, all_sy = [], []
    light = (-0.50, -0.40, 0.76)
    ll = math.sqrt(sum(c*c for c in light))
    light = tuple(c/ll for c in light)

    for mesh, (ox, oy, oz), base_rgb in parts_with_offsets:
        for p1, p2, p3 in mesh.tris:
            q1 = (p1[0]+ox, p1[1]+oy, p1[2]+oz)
            q2 = (p2[0]+ox, p2[1]+oy, p2[2]+oz)
            q3 = (p3[0]+ox, p3[1]+oy, p3[2]+oz)
            ux, uy, uz = q2[0]-q1[0], q2[1]-q1[1], q2[2]-q1[2]
            vx, vy, vz = q3[0]-q1[0], q3[1]-q1[1], q3[2]-q1[2]
            nx, ny, nz = uy*vz - uz*vy, uz*vx - ux*vz, ux*vy - uy*vx
            nl = math.sqrt(nx*nx + ny*ny + nz*nz)
            if nl < 1e-9:
                continue
            nx /= nl; ny /= nl; nz /= nl

            s1 = proj(q1); s2 = proj(q2); s3 = proj(q3)
            # Backface culling in screen space
            area2 = (s2[0]-s1[0])*(s3[1]-s1[1]) - (s2[1]-s1[1])*(s3[0]-s1[0])
            if area2 >= 0:
                continue

            depth = (s1[2] + s2[2] + s3[2]) / 3.0
            ndotl = max(0.0, nx*light[0] + ny*light[1] + nz*light[2])
            shade = 0.38 + 0.62 * ndotl
            col = tuple(min(255, int(c * shade)) for c in base_rgb)
            draw_list.append((depth, s1, s2, s3, col))
            all_sx.extend((s1[0], s2[0], s3[0]))
            all_sy.extend((s1[1], s2[1], s3[1]))

    min_x, max_x = min(all_sx), max(all_sx)
    min_y, max_y = min(all_sy), max(all_sy)
    scale = min((img_w * 0.84) / max(1e-6, max_x - min_x),
                (img_h * 0.84) / max(1e-6, max_y - min_y))
    cx_off = img_w / 2.0 - ((min_x + max_x) / 2.0) * scale
    cy_off = img_h / 2.0 - ((min_y + max_y) / 2.0) * scale

    img = Image.new("RGB", (img_w, img_h), (248, 249, 251))
    draw = ImageDraw.Draw(img)
    # 원근 깊이(큰 depth가 뒤쪽) 정렬
    draw_list.sort(key=lambda item: -item[0])

    for _, s1, s2, s3, col in draw_list:
        pts = [
            (s1[0]*scale + cx_off, s1[1]*scale + cy_off),
            (s2[0]*scale + cx_off, s2[1]*scale + cy_off),
            (s3[0]*scale + cx_off, s3[1]*scale + cy_off),
        ]
        draw.polygon(pts, fill=col)

    img.save(out_png)


def main():
    builders = [
        ("01_intake_base.stl",       build_intake_base,       (195, 205, 215)),
        ("02_hepa_module.stl",       build_hepa_module,       (175, 195, 212)),
        ("03_carbon_fan_module.stl", build_carbon_fan_module, (155, 180, 205)),
        ("04_fan_grille.stl",        build_fan_grille,        (130, 145, 160)),
    ]

    meshes = {}
    print("=== STL 생성 및 검증 결과 ===")
    for fname, fn, _ in builders:
        mesh = fn()
        meshes[fname] = mesh
        fpath = os.path.join(OUT_DIR, fname)
        mesh.save_stl(fpath, f"A1 mini DIY Air Purifier - {fname}")
        bb = mesh.bbox()
        vol = mesh.signed_volume()
        fsize = os.path.getsize(fpath)
        print(
            f"[OK] {fname:26s} | tris={len(mesh.tris):5d} | "
            f"size=({bb[1]-bb[0]:.1f} x {bb[3]-bb[2]:.1f} x {bb[5]-bb[4]:.1f} mm) | "
            f"vol={vol/1000.0:.2f} cm3 | {fsize/1024:.1f} KB"
        )

    # 분해도 프리뷰 렌더링 (좌측 -X ㄷ자 흡기구 + 상단 도브테일 레일/너트포켓이 위에서 잘 보이도록)
    render_preview(
        [
            (meshes["01_intake_base.stl"],       (0, 0, 0),   (195, 205, 215)),
            (meshes["02_hepa_module.stl"],       (0, 0, 56),  (175, 195, 212)),
            (meshes["03_carbon_fan_module.stl"], (0, 0, 108), (155, 180, 205)),
            (meshes["04_fan_grille.stl"],        (0, 0, 175), (120, 132, 145)),
        ],
        PREVIEW_PATH,
        img_w=1000, img_h=1100, elev_deg=26, azim_deg=135
    )
    print(f"[OK] {PREVIEW_PATH} 생성 완료")


if __name__ == "__main__":
    main()
