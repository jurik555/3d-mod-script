"""Параметрический корпус спасательного дрона на раме F450 (CadQuery).

Запуск:  python drone_case.py
Результат: папка out/ с STEP (для Onshape) и STL (для печати) по каждой детали
и общий STEP сборки.

Оси: X вперёд (по стрелке Pixhawk), Y влево, Z вверх. Все размеры в мм.
Параметры с пометкой "ИЗМЕРИТЬ" взяты из спецификаций или угаданы.
Проверьте их штангенциркулем до печати.
"""

import math
from pathlib import Path

import cadquery as cq

# ---------------------------------------------------------------------------
# Печать: Elegoo Neptune 4, PETG
# ---------------------------------------------------------------------------
BED_X, BED_Y, BED_Z = 225, 225, 265  # рабочая область принтера
FIT = 0.3            # зазор на сторону для посадки компонентов (PETG)
WALL = 2.4           # стенка корпуса (6 периметров при сопле 0.4)
INSERT_D = 4.2       # отверстие под термовставку M3 (ИЗМЕРИТЬ вставку)
INSERT_H = 5.0       # глубина под термовставку
M3_CLEAR = 3.4       # сквозное отверстие под винт M3

# ---------------------------------------------------------------------------
# Компоненты (Д x Ш x В)
# ---------------------------------------------------------------------------
PIXHAWK = (81.5, 50.0, 15.5)       # Pixhawk 2.4.8 (ИЗМЕРИТЬ, клоны отличаются)
# Виброразвязка из комплекта: две пластины на резиновых шарах.
# Нижняя пластина крепится к основанию, Pixhawk клеится на верхнюю.
DAMPER_PLATE = (100.0, 70.0, 1.5)  # нижняя (большая) пластина (ИЗМЕРИТЬ)
DAMPER_HOLES = (90.0, 60.0)        # шаг крепёжных отверстий нижней пластины (ИЗМЕРИТЬ)
DAMPER_BALL_H = 12.0               # высота резиновых шаров (ИЗМЕРИТЬ)
DAMPER_TOP_T = 1.5                 # верхняя пластина
DAMPER_BOSS_H = 3.0                # бобышки под нижнюю пластину, чтобы шары не касались основания
PIXHAWK_TAPE = 1.0                 # двусторонний скотч или 3M-пена под Pixhawk
PIXHAWK_DAMPER_H = (DAMPER_BOSS_H + DAMPER_PLATE[2] + DAMPER_BALL_H
                    + DAMPER_TOP_T + PIXHAWK_TAPE)  # от основания до низа Pixhawk
OPI = (65.0, 32.0, 1.6)            # Orange Pi Zero 3W, плата (ИЗМЕРИТЬ)
OPI_HOLES = (58.0, 25.0)           # шаг отверстий платы (ИЗМЕРИТЬ)
OPI_TOP_H = 14.0                   # высота деталей над платой с радиатором (ИЗМЕРИТЬ)
OPI_STANDOFF_H = 5.0               # высота стоек под платой
RX = (47.0, 26.2, 15.0)            # FlySky FS-iA6B без антенн
FLOW = (21.6, 16.0, 6.5)           # MicoAir MTF-02P
BATTERY = (133.0, 43.0, 34.0)      # Gens Ace G-Tech 6S 2500 (6S, ЗАМЕНИТЬ на размеры своего 4S)

# ---------------------------------------------------------------------------
# Рама F450: крепление к центральным плитам (ИЗМЕРИТЬ свою раму)
# ---------------------------------------------------------------------------
FRAME_HOLES_TOP = [(-25, -25), (-25, 25), (25, -25), (25, 25)]   # верхняя плита
FRAME_HOLES_BOTTOM = [(-30, -20), (-30, 20), (30, -20), (30, 20)]  # нижняя плита

# ---------------------------------------------------------------------------
# Компоновка верхнего отсека (центры компонентов в плоскости XY)
# ---------------------------------------------------------------------------
GAP = 4.0                                   # промежуток между компонентами
PIXHAWK_POS = (0.0, 0.0)                    # Pixhawk в центре рамы (ЦТ)
FC_ZONE = (max(PIXHAWK[0], DAMPER_PLATE[0]), max(PIXHAWK[1], DAMPER_PLATE[1]))
RX_POS = (FC_ZONE[0] / 2 + GAP + RX[1] / 2, 0.0)        # приёмник спереди, поперёк
OPI_POS = (-(FC_ZONE[0] / 2 + GAP + OPI[1] / 2), 0.0)   # Orange Pi сзади, поперёк

BASE_T = 3.0            # толщина основания
POCKET_WALL = 1.6       # бортик вокруг приёмника
POCKET_H = 5.0          # высота бортика
CORNER_R = 6.0          # радиус скругления углов корпуса
BOSS_D = 9.0            # бобышки под винты крышки
CABLE_SLOT = (30.0, 12.0)  # прорезь под провода вниз к PDB

BASE_L = FC_ZONE[0] + RX[1] + OPI[1] + 2 * GAP + 2 * (WALL + 8)
BASE_W = max(FC_ZONE[1], OPI[0], RX[0]) + 2 * (WALL + 8)

COVER_INNER_H = max(
    PIXHAWK_DAMPER_H + PIXHAWK[2],
    OPI_STANDOFF_H + OPI[2] + OPI_TOP_H,
    RX[2],
) + 6.0                 # запас на провода сверху
COVER_TOP_T = 2.0
# Стойка GPS M8N из комплекта ставится на крышку
GPS_HOLDER_HOLES = (27.0, 27.0)  # шаг отверстий основания стойки (ИЗМЕРИТЬ)
GPS_HOLDER_POS = (-20.0, 0.0)
GPS_CABLE_D = 8.0                # отверстие под кабель GPS рядом со стойкой
GPS_PAD_T = 3.0                  # утолщение крышки под стойкой изнутри
ANTENNA_D = 4.0         # отверстия под антенны приёмника
VENT_W, VENT_L, VENT_N = 2.5, 22.0, 6   # вентиляционные прорези над Orange Pi
USB_CUT = (14.0, 9.0)   # окно под USB Pixhawk (Ш x В) в левой стенке (ИЗМЕРИТЬ)
USB_X = -PIXHAWK[0] / 2 + 15.0          # положение USB вдоль X (ИЗМЕРИТЬ)
SAFETY_D = 12.0         # отверстие под кнопку safety (ИЗМЕРИТЬ корпус кнопки)
SAFETY_POS = (25.0, 0.0)
BUZZER_POS = (25.0, -22.0)  # отверстия для звука пищалки, сама пищалка внутри
BUZZER_HOLE_D, BUZZER_N = 2.0, 7

# ---------------------------------------------------------------------------
# Ложемент батареи под нижней плитой
# ---------------------------------------------------------------------------
BATT_WALL = 2.4
BATT_FLOOR = 2.4
BATT_STRAP_W = 22.0     # ширина ремня-липучки
BATT_STRAP_T = 3.0
BATT_SLIDE = 20.0       # запас хода батареи вдоль X для балансировки ЦТ
BATT_GUIDE_H = 8.0      # высота боковых направляющих

# ---------------------------------------------------------------------------
# Кронштейн датчика потока (крепится к нижней плите рамы)
# ---------------------------------------------------------------------------
FLOW_BRACKET_T = 2.0
FLOW_WINDOW = (14.0, 10.0)  # окно под оптику и лидар (ИЗМЕРИТЬ)

OUT_DIR = Path(__file__).parent / "out"


def corner_points(length, width, inset):
    dx, dy = length / 2 - inset, width / 2 - inset
    return [(-dx, -dy), (-dx, dy), (dx, -dy), (dx, dy)]


def make_base():
    """Основание верхнего отсека: крепится к верхней плите F450."""
    base = (
        cq.Workplane("XY")
        .box(BASE_L, BASE_W, BASE_T, centered=(True, True, False))
        .edges("|Z").fillet(CORNER_R)
    )

    # Бобышки с термовставками под нижнюю пластину виброразвязки Pixhawk
    damper = corner_points(DAMPER_HOLES[0] + 2 * BOSS_D, DAMPER_HOLES[1] + 2 * BOSS_D, BOSS_D)
    damper = [(PIXHAWK_POS[0] + x, PIXHAWK_POS[1] + y) for x, y in damper]
    base = base.union(
        cq.Workplane("XY").workplane(offset=BASE_T)
        .pushPoints(damper).circle(BOSS_D / 2 - 1).extrude(DAMPER_BOSS_H)
    )
    base = base.cut(
        cq.Workplane("XY").workplane(offset=BASE_T + DAMPER_BOSS_H - min(INSERT_H, DAMPER_BOSS_H + BASE_T - 1))
        .pushPoints(damper).circle(INSERT_D / 2)
        .extrude(min(INSERT_H, DAMPER_BOSS_H + BASE_T - 1))
    )

    # Бортик-карман под приёмник (на липучке)
    for (x, y), (l, w), rot in (
        (RX_POS, RX[:2], 90),
    ):
        il, iw = l + 2 * FIT, w + 2 * FIT
        ring = (
            cq.Workplane("XY").workplane(offset=BASE_T)
            .rect(il + 2 * POCKET_WALL, iw + 2 * POCKET_WALL)
            .rect(il, iw)
            .extrude(POCKET_H)
            .rotate((0, 0, 0), (0, 0, 1), rot)
            .translate((x, y, 0))
        )
        base = base.union(ring)

    # Стойки под Orange Pi с отверстиями под саморез M2.5
    hx, hy = OPI_HOLES[0] / 2, OPI_HOLES[1] / 2
    opi_holes = [(OPI_POS[0] + y, OPI_POS[1] + x)  # плата повёрнута поперёк
                 for x, y in ((-hx, -hy), (-hx, hy), (hx, -hy), (hx, hy))]
    base = base.union(
        cq.Workplane("XY").workplane(offset=BASE_T)
        .pushPoints(opi_holes).circle(3.0).extrude(OPI_STANDOFF_H)
    )
    base = base.cut(
        cq.Workplane("XY").pushPoints(opi_holes).circle(1.1)
        .extrude(BASE_T + OPI_STANDOFF_H)
    )

    # Бобышки под термовставки для винтов крышки
    bosses = corner_points(BASE_L, BASE_W, WALL + BOSS_D / 2 + FIT)
    base = base.union(
        cq.Workplane("XY").workplane(offset=BASE_T)
        .pushPoints(bosses).circle(BOSS_D / 2).extrude(INSERT_H + 1)
    )
    base = base.cut(
        cq.Workplane("XY").workplane(offset=BASE_T + 1)
        .pushPoints(bosses).circle(INSERT_D / 2).extrude(INSERT_H)
    )

    # Крепление к раме и прорезь под провода
    base = base.cut(
        cq.Workplane("XY").pushPoints(FRAME_HOLES_TOP).circle(M3_CLEAR / 2)
        .extrude(BASE_T)
    )
    base = base.cut(
        cq.Workplane("XY")
        .center(PIXHAWK_POS[0] - FC_ZONE[0] / 2 - GAP / 2, 0)
        .slot2D(CABLE_SLOT[0], CABLE_SLOT[1], angle=90)
        .extrude(BASE_T)
    )
    return base


def make_cover():
    """Крышка верхнего отсека. Печатать верхом вниз."""
    outer_h = COVER_INNER_H + COVER_TOP_T
    cover = (
        cq.Workplane("XY")
        .box(BASE_L, BASE_W, outer_h, centered=(True, True, False))
        .edges("|Z").fillet(CORNER_R)
        .edges(">Z").fillet(2.0)
    )
    cavity = (
        cq.Workplane("XY")
        .box(BASE_L - 2 * WALL, BASE_W - 2 * WALL, COVER_INNER_H,
             centered=(True, True, False))
        .edges("|Z").fillet(CORNER_R - WALL)
    )
    cover = cover.cut(cavity)

    # Колонны под винты, совпадают с бобышками основания
    bosses = corner_points(BASE_L, BASE_W, WALL + BOSS_D / 2 + FIT)
    col_z0 = INSERT_H + 1
    cover = cover.union(
        cq.Workplane("XY").workplane(offset=col_z0)
        .pushPoints(bosses).circle(BOSS_D / 2)
        .extrude(COVER_INNER_H - col_z0)
    )
    cover = cover.cut(
        cq.Workplane("XY").pushPoints(bosses).circle(M3_CLEAR / 2).extrude(outer_h)
    )
    cover = cover.cut(  # утопленная головка винта
        cq.Workplane("XY").workplane(offset=outer_h - 3.0)
        .pushPoints(bosses).circle(3.2).extrude(3.0)
    )

    top = cq.Workplane("XY").workplane(offset=COVER_INNER_H)

    # Стойка GPS: утолщение изнутри, 4 отверстия M3 и отверстие под кабель
    gps = [(GPS_HOLDER_POS[0] + x, GPS_HOLDER_POS[1] + y)
           for x, y in corner_points(GPS_HOLDER_HOLES[0] * 2, GPS_HOLDER_HOLES[1] * 2,
                                     GPS_HOLDER_HOLES[0] / 2)]
    pad = GPS_HOLDER_HOLES[0] + 12
    cover = cover.union(
        cq.Workplane("XY").workplane(offset=COVER_INNER_H - GPS_PAD_T)
        .center(*GPS_HOLDER_POS).rect(pad, pad).extrude(GPS_PAD_T)
    )
    cover = cover.cut(
        cq.Workplane("XY").workplane(offset=COVER_INNER_H - GPS_PAD_T)
        .pushPoints(gps).circle(M3_CLEAR / 2).extrude(GPS_PAD_T + COVER_TOP_T)
    )
    cover = cover.cut(
        cq.Workplane("XY").workplane(offset=COVER_INNER_H - GPS_PAD_T)
        .center(GPS_HOLDER_POS[0] - pad / 2 - GPS_CABLE_D / 2 - 1, GPS_HOLDER_POS[1])
        .circle(GPS_CABLE_D / 2).extrude(GPS_PAD_T + COVER_TOP_T)
    )

    # Кнопка safety, пищалка, антенны приёмника
    buzzer = [(BUZZER_POS[0] + 4 * math.cos(2 * math.pi * i / (BUZZER_N - 1)),
               BUZZER_POS[1] + 4 * math.sin(2 * math.pi * i / (BUZZER_N - 1)))
              for i in range(BUZZER_N - 1)] + [BUZZER_POS]
    cover = cover.cut(top.pushPoints(buzzer).circle(BUZZER_HOLE_D / 2)
                      .extrude(COVER_TOP_T))
    cover = cover.cut(top.center(*SAFETY_POS).circle(SAFETY_D / 2)
                      .extrude(COVER_TOP_T))
    ant = [(RX_POS[0], RX_POS[1] - RX[0] / 2 + 6), (RX_POS[0], RX_POS[1] + RX[0] / 2 - 6)]
    cover = cover.cut(top.pushPoints(ant).circle(ANTENNA_D / 2).extrude(COVER_TOP_T))

    # Вентиляция над Orange Pi
    pitch = VENT_W * 2.2
    vents = [(OPI_POS[0], (i - (VENT_N - 1) / 2) * pitch) for i in range(VENT_N)]
    cover = cover.cut(
        top.pushPoints(vents).slot2D(VENT_L, VENT_W, angle=0).extrude(COVER_TOP_T)
    )
    # Боковые вентиляционные прорези в задней стенке
    rear = (
        cq.Workplane("YZ").workplane(offset=-BASE_L / 2)
        .pushPoints([((i - 2) * 8.0, COVER_INNER_H / 2) for i in range(5)])
        .slot2D(COVER_INNER_H * 0.6, 3.0, angle=90)
        .extrude(WALL)
    )
    cover = cover.cut(rear)

    # Окно под USB Pixhawk в левой стенке
    usb_z = PIXHAWK_DAMPER_H + PIXHAWK[2] / 2 - BASE_T
    usb = (
        cq.Workplane("XZ").workplane(offset=-BASE_W / 2)
        .center(USB_X, max(usb_z, USB_CUT[1] / 2))
        .rect(*USB_CUT)
        .extrude(-WALL)
    )
    cover = cover.cut(usb)
    return cover


def make_battery_tray():
    """Ложемент батареи: пластина крепится под нижнюю плиту F450,
    батарея прижимается к ней ремнём-липучкой снизу.
    Печать пластиной вниз, направляющие вверх."""
    il = BATTERY[0] + BATT_SLIDE + 2 * FIT
    iw = BATTERY[1] + 2 * FIT
    ol, ow = il + 2 * BATT_WALL, iw + 2 * BATT_WALL

    tray = (
        cq.Workplane("XY")
        .box(ol, ow, BATT_FLOOR + BATT_GUIDE_H, centered=(True, True, False))
        .edges("|Z").fillet(3.0)
    )
    tray = tray.cut(
        cq.Workplane("XY").workplane(offset=BATT_FLOOR)
        .box(il, iw, BATT_GUIDE_H, centered=(True, True, False))
    )
    # Торцы открыты: батарея сдвигается вдоль X для балансировки ЦТ
    for sx in (-1, 1):
        tray = tray.cut(
            cq.Workplane("XY").workplane(offset=BATT_FLOOR)
            .center(sx * (ol / 2 - BATT_WALL / 2), 0)
            .box(BATT_WALL, iw, BATT_GUIDE_H, centered=(True, True, False))
        )

    # Крепление к нижней плите рамы (головки винтов утоплены)
    tray = tray.cut(
        cq.Workplane("XY").pushPoints(FRAME_HOLES_BOTTOM).circle(M3_CLEAR / 2)
        .extrude(BATT_FLOOR)
    )
    tray = tray.cut(
        cq.Workplane("XY").workplane(offset=BATT_FLOOR - 1.2)
        .pushPoints(FRAME_HOLES_BOTTOM).circle(3.2).extrude(1.2)
    )

    # Прорези под ремень-липучку
    for x in (-BATTERY[0] / 4, BATTERY[0] / 4):
        tray = tray.cut(
            cq.Workplane("XY")
            .pushPoints([(x, -iw / 2 + BATT_STRAP_T / 2 + 1),
                         (x, iw / 2 - BATT_STRAP_T / 2 - 1)])
            .rect(BATT_STRAP_W, BATT_STRAP_T)
            .extrude(BATT_FLOOR)
        )
    # Облегчение пластины между креплениями
    tray = tray.cut(
        cq.Workplane("XY")
        .slot2D(BATTERY[0] / 4, 10).extrude(BATT_FLOOR)
    )
    return tray


def make_flow_bracket():
    """Кронштейн MTF-02P: карман с окном вниз, крепится к нижней плите рамы
    рядом с батареей, чтобы ничто не закрывало датчику вид вниз."""
    il, iw = FLOW[0] + 2 * FIT, FLOW[1] + 2 * FIT
    ol, ow = il + 2 * 2.0, iw + 2 * 2.0
    h = FLOW_BRACKET_T + FLOW[2]
    bracket = (
        cq.Workplane("XY").box(ol, ow, h, centered=(True, True, False))
        .edges("|Z").fillet(1.5)
    )
    bracket = bracket.cut(
        cq.Workplane("XY").workplane(offset=FLOW_BRACKET_T)
        .box(il, iw, FLOW[2], centered=(True, True, False))
    )
    bracket = bracket.cut(
        cq.Workplane("XY").rect(*FLOW_WINDOW).extrude(FLOW_BRACKET_T)
    )
    # Ушки под винты M3
    for sx in (-1, 1):
        ear = (
            cq.Workplane("XY").center(sx * (ol / 2 + 4), 0)
            .rect(8, ow).extrude(FLOW_BRACKET_T)
            .faces(">Z").workplane().circle(M3_CLEAR / 2).cutThruAll()
        )
        bracket = bracket.union(ear)
    return bracket


def check_fits_bed(name, shape):
    bb = shape.val().BoundingBox()
    size = sorted((bb.xlen, bb.ylen))
    bed = sorted((BED_X, BED_Y))
    ok = size[0] <= bed[0] and size[1] <= bed[1] and bb.zlen <= BED_Z
    status = "OK" if ok else "НЕ ВЛЕЗАЕТ НА СТОЛ"
    print(f"{name:14s} {bb.xlen:6.1f} x {bb.ylen:6.1f} x {bb.zlen:5.1f} мм  {status}")
    return ok


def main():
    OUT_DIR.mkdir(exist_ok=True)
    parts = {
        "base": make_base(),
        "cover": make_cover(),
        "battery_tray": make_battery_tray(),
        "flow_bracket": make_flow_bracket(),
    }
    for name, shape in parts.items():
        check_fits_bed(name, shape)
        cq.exporters.export(shape, str(OUT_DIR / f"{name}.step"))
        cq.exporters.export(shape, str(OUT_DIR / f"{name}.stl"),
                            tolerance=0.05, angularTolerance=0.1)

    # Сборка для проверки в Onshape: крышка на основании
    asm = cq.Assembly(name="drone_case")
    asm.add(parts["base"], name="base", color=cq.Color("gray"))
    asm.add(parts["cover"], name="cover", color=cq.Color(0.2, 0.4, 0.8, 0.5),
            loc=cq.Location((0, 0, BASE_T)))
    asm.export(str(OUT_DIR / "assembly_top.step"))
    print(f"Файлы сохранены в {OUT_DIR}")


if __name__ == "__main__":
    main()
