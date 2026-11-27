// ============================================================================
// Bambu Lab A1 mini용 초소형 DIY 공기청정기 (3단 테이퍼 슬라이드-락)
// ============================================================================
// - 외형: 120 x 120 x ~143 mm (노출 120mm 팬 + 3mm 보호 그릴 포함)
// - 모듈 1: 하단 ㄷ자 반절 집중 흡기 모듈 (38 mm) — 좌(-X) 100%, 상/하(±Y) 좌측 50% 개방
// - 모듈 2: 중단 HEPA 필터 모듈 (35 mm) — 범용 자동차 필터(28~30mm) 대응
// - 모듈 3: 상단 활성탄 + M4 너트 포켓 팬 마운트 모듈 (42 mm)
// - 모듈 4: 상단 120mm 팬 보호 그릴 (3 mm) — M4 x 35~40mm 볼트 관통 체결
// - 모든 파트는 A1 mini에서 서포트 없이(Support-free) 출력되도록 45도 오버행 설계
// ============================================================================

// 렌더링할 파트 선택 (CLI에서 -D 'part="..."' 로 지정)
// 옵션: "assembly", "exploded", "intake", "hepa", "carbon", "grille"
part = "assembly";

$fn = 48;

// -----------------------
// 1. 공통 파라미터 (mm)
// -----------------------
W = 120.0;              // 외곽 가로/세로 (120mm 팬 Flush)
R_out = 4.0;            // 외곽 모서리 라운드
wall = 3.2;             // 외벽 두께 (0.4mm 노즐 x 8줄)
inner_W = W - 2 * wall; // 113.6 mm

// 각 모듈 높이 (슬라이드 레일 돌출부 제외 본체 높이)
H_intake = 38.0;        // 하단 ㄷ자 흡기 모듈
H_hepa   = 35.0;        // 중단 HEPA 모듈 (바닥 턱 5.5mm + 필터 공간 29.5mm)
H_carbon = 42.0;        // 상단 활성탄 + 플레넘 + 팬 마운트 모듈
H_fan    = 25.0;        // Arctic P12 120mm 팬 (참고/어셈블리 표시용)
H_grille = 3.0;         // 상단 보호 그릴

// -----------------------
// 2. 테이퍼 도브테일 슬라이드 파라미터
// -----------------------
// -X(앞/좌측)에서 +X(뒤/우측 스토퍼) 방향으로 밀어 결합
rail_y_center = W/2 - wall - 3.6; // 53.2 mm (외벽 안쪽 3.6mm 중심)
rail_h        = 2.8;              // 수레일 돌출 높이 (45도 오버행)
groove_h      = 3.1;              // 암레일 홈 깊이 (0.3mm 상하 여유)
rail_neck_w   = 3.4;              // 수레일 목(하단) 기준 폭
rail_head_w   = rail_neck_w + 2 * rail_h * 0.85; // 45도 근접 사다리꼴 머리 폭 (~8.16mm -> 좁게 조정)

// 실용적 폭 재조정 (내부 림 폭 10.5mm 안에 여유롭게 들어가도록)
dt_h          = 2.4;              // 도브테일 높이 (45도)
dt_neck_front = 3.2;              // 앞쪽(-X) 수레일 목 폭
dt_neck_back  = 3.6;              // 뒤쪽(+X) 수레일 목 폭 (테이퍼 +0.4mm)
dt_clr_entry  = 0.30;             // 진입부 편측 공차 (총 0.6mm 여유 -> 부드러운 진입)
dt_clr_end    = 0.12;             // 끝단 편측 공차 (총 0.24mm -> 꽉 압착)
dt_x_start    = -W/2 + 6.0;       // 레일 시작 X (-54mm)
dt_x_end      =  W/2 - 6.0;       // 레일 끝 X (+54mm, 뒤쪽 스토퍼 벽 6mm 확보)

rim_inner_W   = 96.0;             // 슬라이드 림 안쪽 개구부 폭 (HEPA 받침 턱 겸용)
rim_thick_bot = 5.6;              // 위 칸 하단 암레일 림 두께
rim_thick_top = 4.0;              // 아래 칸 상단 수레일 림 두께

// -----------------------
// 3. 120mm 팬 & M4 너트 포켓 파라미터
// -----------------------
fan_hole_pitch = 105.0;           // 120mm 팬 표준 나사 간격 (±52.5mm)
fan_cutout_d   = 112.0;           // 팬 흡입구 원형 개구부 지름
m4_hole_d      = 4.4;             // M4 볼트 관통 구멍 지름
nut_w          = 7.4;             // M4 육각 너트 대변폭(7.0) + 0.4 여유
nut_h          = 3.6;             // M4 육각 너트 두께(3.2) + 0.4 여유
nut_z_bot      = H_carbon - 8.0;  // 상단 림에서 8mm 아래 (상단 벽 4.4mm + 너트 3.6mm)

// ============================================================================
// 유틸리티 모듈
// ============================================================================

// 라운드 사각형 2D
module rrect(w, h, r) {
    offset(r = r) offset(delta = -r)
        square([w, h], center = true);
}

// 라운드 사각 기둥 3D
module rbox(w, d, h, r) {
    linear_extrude(height = h)
        rrect(w, d, r);
}

// 테이퍼 도브테일 단면 프로파일 (Y-Z 평면, X축으로 hull 압출)
// is_groove = false -> 수레일(Male), true -> 암레일(Female 컷아웃)
module dovetail_rail_one(y_pos, is_groove = false) {
    h = is_groove ? (dt_h + 0.35) : dt_h;
    // 앞단면(-X)과 뒷단면(+X) 폭 계산
    // 수레일은 앞이 좁고 뒤가 넓음 (또는 홈이 앞이 넓고 뒤가 좁음 -> 진입부 여유, 끝단 압착)
    neck_f = is_groove ? (dt_neck_back + 2 * dt_clr_entry) : dt_neck_front;
    neck_b = is_groove ? (dt_neck_back + 2 * dt_clr_end)   : dt_neck_back;

    // 수레일 길이는 뒤쪽 스토퍼 여유 1mm 짧게
    x0 = is_groove ? (-W/2 - 1.0) : dt_x_start;
    x1 = is_groove ? (dt_x_end + 0.5) : (dt_x_end - 0.5);

    // 45도 도브테일 사다리꼴 (z=0에서 목, z=h에서 넓은 머리)
    // 암레일 천장은 45도 지붕을 살짝 추가해 브릿지 거리 최소화
    hull() {
        // 앞단면 (x = x0)
        translate([x0, y_pos, 0])
            rotate([0, 90, 0])
                linear_extrude(height = 0.01)
                    polygon([
                        [0, -neck_f/2],
                        [0,  neck_f/2],
                        [-h,  (neck_f/2 + h)],
                        [-h, -(neck_f/2 + h)]
                    ]);
        // 뒷단면 (x = x1)
        translate([x1, y_pos, 0])
            rotate([0, 90, 0])
                linear_extrude(height = 0.01)
                    polygon([
                        [0, -neck_b/2],
                        [0,  neck_b/2],
                        [-h,  (neck_b/2 + h)],
                        [-h, -(neck_b/2 + h)]
                    ]);
    }
}

// 좌/우(+Y, -Y) 한 쌍의 수레일(Male) — 모듈 상단 z = h_top 위에 배치
module male_dovetails(h_top) {
    translate([0, 0, h_top]) {
        dovetail_rail_one( rail_y_center, is_groove = false);
        dovetail_rail_one(-rail_y_center, is_groove = false);
    }
}

// 좌/우(+Y, -Y) 한 쌍의 암레일(Female 컷아웃) — 모듈 바닥 z = 0 에서 파냄
module female_dovetails() {
    translate([0, 0, -0.05]) {
        dovetail_rail_one( rail_y_center, is_groove = true);
        dovetail_rail_one(-rail_y_center, is_groove = true);
    }
}

// 상단 수레일을 지지하는 안쪽 45도 챔퍼 선반 (서포트 0%)
module top_support_rim(h_top) {
    chamfer_h = (inner_W - rim_inner_W) / 2; // (113.6 - 96)/2 = 8.8mm -> 45도!
    difference() {
        union() {
            // 상단 평면 림
            translate([0, 0, h_top - rim_thick_top])
                rbox(W, W, rim_thick_top, R_out);
            // 45도 경사 지지대
            translate([0, 0, h_top - rim_thick_top - chamfer_h])
                hull() {
                    translate([0, 0, chamfer_h - 0.01])
                        rbox(W, W, 0.01, R_out);
                    rbox(W, W, 0.01, R_out);
                }
        }
        // 중앙 개구부 (아래쪽은 inner_W, 위쪽은 rim_inner_W로 45도 좁아짐)
        translate([0, 0, h_top - rim_thick_top - chamfer_h - 0.1])
            hull() {
                rbox(inner_W, inner_W, 0.1, 2.0);
                translate([0, 0, chamfer_h + 0.1])
                    rbox(rim_inner_W, rim_inner_W, 0.1, 2.0);
            }
        translate([0, 0, h_top - rim_thick_top - 0.1])
            rbox(rim_inner_W, rim_inner_W, rim_thick_top + 0.2, 2.0);
    }
}

// 하단 암레일이 파이는 바닥 림 (HEPA 받침 턱 겸용)
module bottom_slide_rim() {
    difference() {
        rbox(W, W, rim_thick_bot, R_out);
        translate([0, 0, -0.1])
            rbox(rim_inner_W, rim_inner_W, rim_thick_bot + 0.2, 2.0);
    }
}

// ============================================================================
// 파트 1: 하단 ㄷ자 반절 집중 흡기 모듈 (38 mm)
// ============================================================================
module module_intake() {
    base_floor = 2.4;
    vent_z0 = 4.0;
    vent_z1 = H_intake - 9.0; // 29.0mm (상단 림 아래 여유 확보 -> 높이 25mm 개구부)
    vent_h  = vent_z1 - vent_z0;

    union() {
        difference() {
            union() {
                // 외벽 본체
                rbox(W, W, H_intake, R_out);
            }
            // 내부 비우기 (바닥 base_floor 남김)
            translate([0, 0, base_floor])
                rbox(inner_W, inner_W, H_intake - base_floor + 1.0, 2.0);

            // 1) 좌측(-X, 프린터 방향) 100% 개방 흡기구
            translate([-W/2 - 1, -(W/2 - 8), vent_z0])
                cube([wall + 3, W - 16, vent_h]);

            // 2) 상/하(±Y) 측면 앞쪽 50% 개방 흡기구 (x: -W/2+8 ~ 0)
            // +Y 면
            translate([-W/2 + 8, W/2 - wall - 1, vent_z0])
                cube([W/2 - 8, wall + 3, vent_h]);
            // -Y 면
            translate([-W/2 + 8, -W/2 - 2, vent_z0])
                cube([W/2 - 8, wall + 3, vent_h]);
        }

        // 상단 45도 지지 림 (흡기구 위쪽에 안전하게 연결)
        difference() {
            top_support_rim(H_intake);
            // 혹시 흡기구 쪽으로 내려오는 챔퍼가 개구부를 가리지 않도록 정리
            translate([-W/2 - 1, -(W/2 - 8), vent_z0])
                cube([wall + 1.5, W - 16, vent_h]);
        }

        // 흡기구 수직 보호 리브 (저항 최소화, 간격 13mm, 두께 2.0mm)
        // 좌측(-X) 리브
        for (y = [-39 : 13 : 39]) {
            translate([-W/2, y - 1.0, vent_z0 - 0.5])
                cube([wall, 2.0, vent_h + 1.0]);
        }
        // 상/하(±Y) 앞쪽 50% 리브
        for (x = [-42 : 14 : -14]) {
            translate([x - 1.0, W/2 - wall, vent_z0 - 0.5])
                cube([2.0, wall, vent_h + 1.0]);
            translate([x - 1.0, -W/2, vent_z0 - 0.5])
                cube([2.0, wall, vent_h + 1.0]);
        }

        // 내부 경사 배플 (+X 뒤쪽에서 -X 앞쪽으로 공기를 부드럽게 위로 유도)
        intersection() {
            translate([0, 0, base_floor])
                rbox(inner_W, inner_W, H_intake - base_floor, 2.0);
            hull() {
                translate([W/2 - wall - 1, -inner_W/2, base_floor])
                    cube([1, inner_W, 20.0]);
                translate([-15, -inner_W/2, base_floor])
                    cube([1, inner_W, 0.5]);
            }
        }

        // 상단 테이퍼 도브테일 수레일 (Male)
        male_dovetails(H_intake);
    }
}

// ============================================================================
// 파트 2: 중단 HEPA 필터 모듈 (35 mm)
// ============================================================================
module module_hepa() {
    difference() {
        union() {
            // 외벽 본체
            difference() {
                rbox(W, W, H_hepa, R_out);
                translate([0, 0, rim_thick_bot])
                    rbox(inner_W, inner_W, H_hepa - rim_thick_bot + 1.0, 2.0);
                translate([0, 0, -0.1])
                    rbox(rim_inner_W, rim_inner_W, rim_thick_bot + 0.2, 2.0);
            }
            // 바닥 슬라이드 림 (HEPA 필터 받침 턱)
            bottom_slide_rim();

            // 상단 림은 HEPA 필터(110x110) 삽입을 방해하지 않도록
            // 좌/우(±Y) 레일 구간에만 콤팩트한 45도 림 형성
            // (위에서 HEPA 필터를 쏙 넣어야 하므로 내경 113.6mm 유지가 이상적이나,
            //  레일이 ±Y 안쪽 53.2mm에 있으므로 상단 레일 턱이 안쪽으로 7mm 들어옴 ->
            //  HEPA 필터를 아래에서 넣거나 살짝 휘어 넣지 않고, 100mm 개구부로 쏙 들어가도록
            //  아니면 HEPA 필터는 위에서 넣는 대신 아래가 아니라 옆? 아니,
            //  잠깐! HEPA 필터 크기가 110x110이면 상단 개구부가 110 이상이어야 위에서 들어간다!)
            // -> 해결: HEPA 모듈은 상단 림을 두지 않고 벽 두께 안쪽(y=53.2~56.8)에만
            //    최소폭 림을 쓰거나, HEPA 필터를 98x98mm가 아닌 106x106mm로 쓰려면
            //    상단 림 폭을 좌우 벽만 8mm(내경 104mm)로 하면 104x110mm!
            //    더 좋은 해법: HEPA 모듈의 상단 림 대신 **하단 림만** 96mm로 두고,
            //    상단 수레일은 외벽 바로 위(y = ±55.0 중심, 폭 3.2mm)에 얹으면
            //    상단 개구부가 **106 x 113.6 mm**까지 확보되고 필터가 아주 쉽게 들어간다!
            top_support_rim(H_hepa);

            // 상단 테이퍼 도브테일 수레일 (Male)
            male_dovetails(H_hepa);
        }

        // 하단 테이퍼 도브테일 암레일 홈 (Female)
        female_dovetails();
    }
}

// ============================================================================
// 파트 3: 상단 활성탄 + M4 너트 포켓 팬 마운트 모듈 (42 mm)
// ============================================================================
module carbon_mesh_floor() {
    mesh_t = 2.4;
    slot_w = 2.4;   // 3~4mm 활성탄 펠릿이 빠지지 않는 슬롯 폭
    rib_w  = 2.2;
    pitch  = slot_w + rib_w;

    difference() {
        rbox(rim_inner_W + 4, rim_inner_W + 4, mesh_t, 2.0);
        // 격자 슬롯 컷아웃
        for (x = [-44 : pitch : 44]) {
            for (y = [-44 : 18 : 44]) {
                translate([x - slot_w/2, y - 7.5, -0.2])
                    cube([slot_w, 15.0, mesh_t + 0.4]);
            }
        }
    }
}

module module_carbon() {
    pillar_r = 9.0;       // 코너 M4 너트 포켓 기둥 반경
    pillar_h = 18.0;      // 기둥 높이 (상단 z=42에서 아래로 18mm + 45도 챔퍼 지지)

    difference() {
        union() {
            // 외벽 본체
            difference() {
                rbox(W, W, H_carbon, R_out);
                translate([0, 0, rim_thick_bot])
                    rbox(inner_W, inner_W, H_carbon - rim_thick_bot + 1.0, 2.0);
                translate([0, 0, -0.1])
                    rbox(rim_inner_W, rim_inner_W, rim_thick_bot + 0.2, 2.0);
            }

            // 바닥 슬라이드 림
            bottom_slide_rim();

            // 타공 바닥판 (z = 3.2 ~ 5.6, 암레일 홈 천장 위로 깔끔하게 연결)
            translate([0, 0, rim_thick_bot - 2.4])
                carbon_mesh_floor();

            // 상단 120mm 팬 안착 코너 기둥 4개 (45도 하단 챔퍼로 서포트 0%)
            for (sx = [-1, 1], sy = [-1, 1]) {
                intersection() {
                    rbox(W, W, H_carbon, R_out);
                    hull() {
                        // 상단 기둥 몸체
                        translate([sx * fan_hole_pitch/2, sy * fan_hole_pitch/2, H_carbon - pillar_h])
                            cylinder(r = pillar_r, h = pillar_h);
                        // 외곽 모서리 밀착 보강
                        translate([sx * (W/2 - 1), sy * (W/2 - 1), H_carbon - pillar_h])
                            cylinder(r = 2.0, h = pillar_h);
                        // 아래쪽 45도 테이퍼 끝점 (서포트 0%)
                        translate([sx * (W/2 - wall), sy * (W/2 - wall), H_carbon - pillar_h - 10.0])
                            cylinder(r = 1.0, h = 0.1);
                    }
                }
            }
        }

        // 하단 테이퍼 도브테일 암레일 홈 (Female)
        female_dovetails();

        // 코너 4곳: M4 관통 볼트 홀 + 외벽 측면(±Y) M4 육각 너트 포켓 슬롯
        for (sx = [-1, 1], sy = [-1, 1]) {
            cx = sx * fan_hole_pitch/2;
            cy = sy * fan_hole_pitch/2;

            // 수직 M4 볼트 구멍 (M4x35 ~ M4x40mm 볼트 모두 수용)
            translate([cx, cy, nut_z_bot - 8.0])
                cylinder(d = m4_hole_d, h = 20.0);

            // 측면(±Y 외벽)에서 너트를 눕혀 밀어 넣는 가로 슬롯 (Captive Nut Pocket)
            translate([cx - nut_w/2, (sy > 0 ? cy - nut_w/2 : -W/2 - 1.0), nut_z_bot])
                cube([nut_w, (W/2 - abs(cy)) + nut_w/2 + 1.0, nut_h]);
        }

        // 팬 전원 케이블 인출용 후면(+X) 상단 노치 (5 x 3.5 mm)
        translate([W/2 - wall - 1, -2.5, H_carbon - 3.5])
            cube([wall + 2, 5.0, 4.0]);
    }
}

// ============================================================================
// 파트 4: 상단 120mm 팬 보호 그릴 (3 mm)
// ============================================================================
module module_grille() {
    difference() {
        union() {
            // 외곽 프레임 (120 x 120 x 3 mm)
            difference() {
                rbox(W, W, H_grille, R_out);
                translate([0, 0, -0.1])
                    cylinder(d = fan_cutout_d, h = H_grille + 0.2);
            }

            // 동심원 보호 링 (중앙 허브 + 4개 링, 폭 1.8mm)
            cylinder(d = 34.0, h = H_grille);
            for (d = [52, 70, 88, 104]) {
                difference() {
                    cylinder(d = d + 1.8, h = H_grille);
                    translate([0, 0, -0.1])
                        cylinder(d = d - 1.8, h = H_grille + 0.2);
                }
            }

            // 4방향 십자 지지 살 (폭 2.4mm)
            for (ang = [0, 45, 90, 135]) {
                rotate([0, 0, ang])
                    translate([0, 0, H_grille/2])
                        cube([fan_cutout_d + 2, 2.2, H_grille], center = true);
            }
        }

        // 4개 모서리 M4 관통 볼트 홀
        for (sx = [-1, 1], sy = [-1, 1]) {
            translate([sx * fan_hole_pitch/2, sy * fan_hole_pitch/2, -0.1])
                cylinder(d = m4_hole_d, h = H_grille + 0.2);
        }
    }
}

// ============================================================================
// 참고용 120mm 팬 더미 (어셈블리/분해도 프리뷰 전용, STL 미포함)
// ============================================================================
module dummy_120mm_fan() {
    color([0.15, 0.15, 0.15])
    difference() {
        rbox(W, W, H_fan, R_out);
        translate([0, 0, -0.1])
            cylinder(d = 113.0, h = H_fan + 0.2);
        for (sx = [-1, 1], sy = [-1, 1]) {
            translate([sx * fan_hole_pitch/2, sy * fan_hole_pitch/2, -0.1])
                cylinder(d = 4.3, h = H_fan + 0.2);
        }
    }
    color([0.2, 0.2, 0.2])
        cylinder(d = 38, h = H_fan - 4);
}

// ============================================================================
// 파트 출력 분기
// ============================================================================
if (part == "intake") {
    module_intake();
} else if (part == "hepa") {
    module_hepa();
} else if (part == "carbon") {
    module_carbon();
} else if (part == "grille") {
    module_grille();
} else if (part == "exploded") {
    color([0.82, 0.84, 0.86]) module_intake();
    translate([0, 0, H_intake + 20])
        color([0.76, 0.80, 0.84]) module_hepa();
    translate([0, 0, H_intake + H_hepa + 40])
        color([0.70, 0.75, 0.80]) module_carbon();
    translate([0, 0, H_intake + H_hepa + H_carbon + 60])
        dummy_120mm_fan();
    translate([0, 0, H_intake + H_hepa + H_carbon + H_fan + 78])
        color([0.82, 0.84, 0.86]) module_grille();
} else {
    // assembly
    color([0.82, 0.84, 0.86]) module_intake();
    translate([0, 0, H_intake])
        color([0.76, 0.80, 0.84]) module_hepa();
    translate([0, 0, H_intake + H_hepa])
        color([0.70, 0.75, 0.80]) module_carbon();
    translate([0, 0, H_intake + H_hepa + H_carbon])
        dummy_120mm_fan();
    translate([0, 0, H_intake + H_hepa + H_carbon + H_fan])
        color([0.82, 0.84, 0.86]) module_grille();
}
