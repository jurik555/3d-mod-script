"""Параметрическая рама-корпус спасательного дрона (CadQuery).

Своя рама вместо F450: вся электроника, провода и батарея внутри, ESC в лучах.
Запуск:  python drone_frame.py
Результат: папка out/ с STEP (для Onshape) и STL (для печати) по каждой детали
и общий STEP сборки с макетами компонентов.

Оси: X вперёд (по стрелке Pixhawk), Y влево, Z вверх. Все размеры в мм.
Z = 0 — нижняя плоскость палубы (верх лучей).
Параметры с пометкой "ИЗМЕРИТЬ" взяты из спецификаций или угаданы.

Детали:
  deck          палуба: несущая плита, на ней электроника, снизу лучи и отсек батареи
  canopy        крышка электроники, защита от брызг (лабиринт по бортику, вентиляция под козырьками)
  battery_bay   отсек батареи под палубой
  hatch         люк отсека батареи, на двух барашковых винтах M3
  arm x4        полый луч, внутри ESC, на конце площадка мотора
  arm_cover x4  нижняя крышка луча с прорезями для охлаждения ESC и слива воды
  leg x4        опора шасси под мотором
  flow_bracket  кронштейн MTF-02P под любым лучом
"""

import math
from pathlib import Path

import cadquery as cq

# ---------------------------------------------------------------------------
# Печать: Elegoo Neptune 4, PETG
# ---------------------------------------------------------------------------
BED_X, BED_Y, BED_Z = 225, 225, 265
FIT = 0.3            # зазор на сторону для посадки
WALL = 2.4           # стенки (6 периметров при сопле 0.4)
INSERT_D = 4.2       # отверстие под термовставку M3 (ИЗМЕРИТЬ вставку)
INSERT_H = 5.0       # глубина под термовставку
M3_CLEAR = 3.4       # сквозное отверстие под винт M3
M3_HEAD_D = 6.5      # зенковка под головку M3
BOSS_D = 9.0         # бобышки под термовставки

# ---------------------------------------------------------------------------
# Компоненты (Д x Ш x В)
# ---------------------------------------------------------------------------
PIXHAWK = (81.5, 50.0, 15.5)       # Pixhawk 2.4.8 (ИЗМЕРИТЬ, клоны отличаются)
DAMPER_PLATE = (100.0, 70.0, 1.5)  # нижняя пластина виброразвязки из комплекта (ИЗМЕРИТЬ)
DAMPER_HOLES = (90.0, 60.0)        # шаг отверстий нижней пластины (ИЗМЕРИТЬ)
DAMPER_BALL_H = 12.0               # высота резиновых шаров (ИЗМЕРИТЬ)
DAMPER_TOP_T = 1.5
DAMPER_BOSS_H = 3.0                # бобышки под нижнюю пластину
PIXHAWK_TAPE = 1.0
PIXHAWK_STACK_H = (DAMPER_BOSS_H + DAMPER_PLATE[2] + DAMPER_BALL_H
                   + DAMPER_TOP_T + PIXHAWK_TAPE + PIXHAWK[2])

OPI = (65.0, 32.0, 1.6)            # Orange Pi Zero 3W (ИЗМЕРИТЬ)
OPI_HOLES = (58.0, 25.0)           # шаг отверстий платы (ИЗМЕРИТЬ)
OPI_TOP_H = 14.0                   # высота над платой с радиатором (ИЗМЕРИТЬ)
OPI_STANDOFF_H = 5.0
RX = (47.0, 26.2, 15.0)            # FlySky FS-iA6B, антенны остаются внутри крышки
PDB = (36.0, 36.0)                 # плата распределения питания (ИЗМЕРИТЬ свою)
PDB_HOLES = 30.5                   # квадратный шаг отверстий PDB
PDB_STANDOFF_H = 5.0
PM = (25.0, 21.0, 9.0)             # power module 3DR (ИЗМЕРИТЬ)
ESC = (73.0, 28.0, 12.0)           # Emax BLHeli 40A (ИЗМЕРИТЬ без проводов)
FLOW = (21.6, 16.0, 6.5)           # MicoAir MTF-02P
GIMBAL = (55.0, 55.0, 70.0)        # SIYI A8 Mini
GIMBAL_HOLES = (40.0, 40.0)        # шаг крепёжных отверстий гимбала (ИЗМЕРИТЬ)
GIMBAL_CABLE_D = 14.0
BATTERY = (150.0, 50.0, 42.0)      # Gens Ace 4S (ИЗМЕРИТЬ свою!)
BATT_FOAM = 4.0                    # пена между батареей и люком
BATT_SLIDE = 4.0                   # запас по длине

# Моторы 2212 и винты 1045
MOTOR_DIAG = 450.0                 # диагональ между осями моторов
MOTOR_HOLES = (16.0, 19.0)         # крестовой шаг отверстий мотора
MOTOR_SHAFT_D = 8.0                # отверстие под вал и шайбу снизу
MOTOR_D, MOTOR_H = 28.0, 30.0      # для макета в сборке
PROP_D = 254.0                     # 10 дюймов
PROP_CLEAR = 6.0                   # зазор от концов винтов до крышки

# ---------------------------------------------------------------------------
# Корпус
# ---------------------------------------------------------------------------
BODY_FRONT = 115.0       # передний край палубы и крышки
BODY_HW = 82.0           # половина ширины крышки
DECK_T = 5.0             # толщина палубы
RIM_T, RIM_H = 1.6, 5.0  # бортик-лабиринт по периметру крышки
CANOPY_INNER_H = PIXHAWK_STACK_H + 6.5   # запас на провода над Pixhawk
CANOPY_TOP_T = 2.4

# Раскладка на палубе (центры)
CABLE_GAP = 12.0                      # промежуток за Pixhawk под провод батареи
PIXHAWK_POS = (0.0, 0.0)              # в центре (ЦТ)
OPI_POS = (-(DAMPER_PLATE[0] / 2 + CABLE_GAP + OPI[1] / 2), 0.0)  # сзади, поперёк
PDB_POS = (15.0, 56.0)
RX_POS = (-10.0, -54.0)               # вдоль X
PM_POS = (30.0, -56.0)
BATT_SLOT = (8.0, 30.0)               # прорезь под провод батареи (X x Y)
BATT_SLOT_POS = (-(DAMPER_PLATE[0] / 2 + CABLE_GAP / 2), 0.0)
CANOPY_SCREWS = [(x, y) for x in (-40.0, 40.0) for y in (-72.7, 72.7)]

# Гимбал под носом палубы
GIMBAL_SPACER = 5.0
GIMBAL_X = BODY_FRONT - GIMBAL[0] / 2 - 2.0

# Крышка: безопасность и GPS
GPS_HOLDER_HOLES = (27.0, 27.0)   # шаг отверстий основания стойки GPS (ИЗМЕРИТЬ)
GPS_HOLDER_POS = (-20.0, 0.0)
GPS_CABLE_D = 8.0
GPS_PAD_T = 3.0
SAFETY_D = 12.0                   # корпус кнопки safety (ИЗМЕРИТЬ)
SAFETY_POS = (60.0, -35.0)
VENT_W, VENT_N = 3.0, 8           # прорези в боковых стенках под козырьками
HOOD_GAP, HOOD_T = 5.0, 1.6

# Отсек батареи
BAY_IN = (BATTERY[0] + BATT_SLIDE + 2 * FIT, BATTERY[1] + 2 * FIT,
          BATTERY[2] + BATT_FOAM + FIT)
BAY_OUT = (BAY_IN[0] + 2 * WALL, BAY_IN[1] + 2 * WALL)
BAY_X1 = GIMBAL_X - GIMBAL[0] / 2 - 3.0   # перед отсека упирается в зону гимбала
BAY_X0 = BAY_X1 - BAY_OUT[0]
BAY_CX = (BAY_X0 + BAY_X1) / 2
BAY_FLANGE_W, BAY_FLANGE_T = 6.0, 6.0     # фланцы вдоль бортов, крепление к палубе снизу
BAY_SCREW_X = [BAY_X0 + 4.5, -5.0, BAY_X1 - 3.0]
HATCH_T = 2.4
HATCH_LIP_H = BATT_FOAM - 0.5   # бортик люка внутри слоя пены
LUG_W, LUG_L, LUG_H = 10.0, 14.0, 8.0     # проушины под винты люка

# Лучи
ARM_ANGLES = (45.0, 135.0, 225.0, 315.0)
MOTOR_R = MOTOR_DIAG / 2
ARM_ROOT_R = 66.0                    # начало луча от центра (под палубой)
ARM_IN_W = ESC[1] + 2.0              # внутренний канал под ESC
ARM_IN_H = ESC[2] + 3.0
ARM_COVER_T = 2.0
ARM_W = ARM_IN_W + 2 * WALL
ARM_H = WALL + ARM_IN_H + ARM_COVER_T
MOTOR_X = MOTOR_R - ARM_ROOT_R        # ось мотора в координатах луча
MOTOR_PAD_R = 20.0
MOTOR_PLATE_T = 4.0                  # толщина под мотором (остальное зенковка)
CHANNEL_X0, CHANNEL_X1 = 3.0, MOTOR_X - 14.0
ARM_WIRE_X, ARM_WIRE_HOLE = 10.0, (14.0, 16.0)   # выход проводов в палубу
PILLAR_D = 8.0
PILLAR_Y = ARM_IN_W / 2 - 2.0
DECK_BOLT_X = (38.0, 58.0)           # стойки: сверху болты палубы, снизу крышка луча
COVER_TIP_X = CHANNEL_X1 - 4.0       # стойки у конца канала (только крышка)
LEG_X = MOTOR_X - 9.0                # опора под блоком мотора
LEG_HOLE_Y = 11.0
TAB_R = ARM_ROOT_R + DECK_BOLT_X[1] + 8.0   # ушки палубы над лучами

# Шасси
GROUND_Z = -(GIMBAL_SPACER + GIMBAL[2] + 15.0)   # гимбал на 15 мм выше земли
LEG_H = -GROUND_Z - ARM_H
LEG_T = 10.0
LEG_FOOT_W = 60.0

# Кронштейн MTF-02P
FLOW_BRACKET_T = 2.0
FLOW_WINDOW = (14.0, 10.0)   # окно под оптику и лидар (ИЗМЕРИТЬ)

OUT_DIR = Path(__file__).parent / "out"

MOTOR_XY = [(MOTOR_R * math.cos(math.radians(a)), MOTOR_R * math.sin(math.radians(a)))
            for a in ARM_ANGLES]
PROP_CUT_R = PROP_D / 2 + PROP_CLEAR
BODY_REAR = min(BAY_X0 - 1.0, -(TAB_R / math.sqrt(2) + ARM_W / (2 * math.sqrt(2))))


# ---------------------------------------------------------------------------
# Вспомогательные функции
# ---------------------------------------------------------------------------
def rot(pt, angle):
    a = math.radians(angle)
    x, y = pt
    return (x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a))


def arm_point(angle, x, y=0.0):
    """Точка из координат луча (x от корня) в координаты корпуса."""
    return rot((ARM_ROOT_R + x, y), angle)


def place_on_arm(shape, angle):
    return shape.translate((ARM_ROOT_R, 0, 0)).rotate((0, 0, 0), (0, 0, 1), angle)


def footprint(inset=0.0):
    """Контур крышки: прямоугольник минус круги винтов (с зазором)."""
    length = BODY_FRONT - BODY_REAR
    cx = (BODY_FRONT + BODY_REAR) / 2
    return (
        cq.Sketch()
        .push([(cx, 0)]).rect(length - 2 * inset, 2 * (BODY_HW - inset)).reset()
        .push(MOTOR_XY).circle(PROP_CUT_R + inset, mode="s").reset()
    )


def prism(sketch, z0, h):
    return cq.Workplane("XY").workplane(offset=z0).placeSketch(sketch).extrude(h)


def cyls(points, d, z0, h):
    return cq.Workplane("XY").workplane(offset=z0).pushPoints(points).circle(d / 2).extrude(h)


def rect_ring(center, size, wall, z0, h, angle=0.0):
    il, iw = size
    ring = (
        cq.Workplane("XY").workplane(offset=z0)
        .rect(il + 2 * wall, iw + 2 * wall).rect(il, iw).extrude(h)
    )
    return ring.rotate((0, 0, 0), (0, 0, 1), angle).translate((*center, 0))


def square_pts(center, sx, sy):
    cx, cy = center
    return [(cx + dx, cy + dy) for dx in (-sx / 2, sx / 2) for dy in (-sy / 2, sy / 2)]


# ---------------------------------------------------------------------------
# Палуба
# ---------------------------------------------------------------------------
def make_deck():
    deck = prism(footprint(), 0, DECK_T)
    for a in ARM_ANGLES:
        tab = (
            cq.Workplane("XY")
            .box(TAB_R - ARM_ROOT_R, ARM_W, DECK_T, centered=(False, True, False))
            .translate((ARM_ROOT_R, 0, 0)).rotate((0, 0, 0), (0, 0, 1), a)
        )
        deck = deck.union(tab)

    top = DECK_T
    # Бортик-лабиринт: стенка крышки заходит снаружи
    rim = prism(footprint(WALL + FIT), top, RIM_H).cut(
        prism(footprint(WALL + FIT + RIM_T), top, RIM_H))
    deck = deck.union(rim)

    # Бобышки под винты крышки
    deck = deck.union(cyls(CANOPY_SCREWS, BOSS_D, top, INSERT_H + 1))
    deck = deck.cut(cyls(CANOPY_SCREWS, INSERT_D, top + 1, INSERT_H))

    # Виброразвязка Pixhawk
    damper = square_pts(PIXHAWK_POS, *DAMPER_HOLES)
    deck = deck.union(cyls(damper, BOSS_D - 2, top, DAMPER_BOSS_H))
    deck = deck.cut(cyls(damper, INSERT_D, top + DAMPER_BOSS_H - INSERT_H, INSERT_H))

    # Стойки Orange Pi (плата поперёк), саморезы M2.5
    opi = square_pts(OPI_POS, OPI_HOLES[1], OPI_HOLES[0])
    deck = deck.union(cyls(opi, 6.0, top, OPI_STANDOFF_H))
    deck = deck.cut(cyls(opi, 2.2, 0, top + OPI_STANDOFF_H))

    # Стойки PDB с термовставками
    pdb = square_pts(PDB_POS, PDB_HOLES, PDB_HOLES)
    deck = deck.union(cyls(pdb, 7.0, top, PDB_STANDOFF_H))
    deck = deck.cut(cyls(pdb, INSERT_D, top + PDB_STANDOFF_H - INSERT_H, INSERT_H))

    # Карманы приёмника и power module (на липучке)
    deck = deck.union(rect_ring(RX_POS, (RX[0] + 2 * FIT, RX[1] + 2 * FIT), 1.6, top, 4.0))
    deck = deck.union(rect_ring(PM_POS, (PM[0] + 2 * FIT, PM[1] + 2 * FIT), 1.6, top, 4.0))

    # Гимбал: термовставки снизу в бобышках сверху палубы, кабель вверх
    gimbal = square_pts((GIMBAL_X, 0), *GIMBAL_HOLES)
    deck = deck.union(cyls(gimbal, BOSS_D - 2, top, 3.0))
    deck = deck.cut(cyls(gimbal, INSERT_D, 0, INSERT_H))
    deck = deck.cut(cyls([(GIMBAL_X, 0)], GIMBAL_CABLE_D, 0, top))

    # Провод батареи из отсека
    deck = deck.cut(
        cq.Workplane("XY").center(*BATT_SLOT_POS).slot2D(BATT_SLOT[1], BATT_SLOT[0], angle=90)
        .extrude(top))

    # Отсек батареи крепится снизу винтами в термовставки (вставки вровень с верхом)
    bay_pts = [(x, s * (BAY_OUT[1] / 2 + BAY_FLANGE_W / 2)) for x in BAY_SCREW_X for s in (-1, 1)]
    deck = deck.cut(cyls(bay_pts, INSERT_D, 0, top))

    # Лучи: болты сверху и окна под провода
    for a in ARM_ANGLES:
        bolts = [arm_point(a, x, y) for x in DECK_BOLT_X for y in (-PILLAR_Y, PILLAR_Y)]
        deck = deck.cut(cyls(bolts, M3_CLEAR, 0, top))
        wx, wy = arm_point(a, ARM_WIRE_X)
        hole = (
            cq.Workplane("XY").rect(*ARM_WIRE_HOLE).extrude(top)
            .rotate((0, 0, 0), (0, 0, 1), a).translate((wx, wy, 0))
        )
        deck = deck.cut(hole)
    return deck


# ---------------------------------------------------------------------------
# Крышка электроники
# ---------------------------------------------------------------------------
def make_canopy():
    """Крышка. Координаты: z = 0 — верх палубы. Печатать верхом вниз."""
    h = CANOPY_INNER_H + CANOPY_TOP_T
    canopy = prism(footprint(), 0, h).cut(prism(footprint(WALL), 0, CANOPY_INNER_H))

    # Колонны под винты до бобышек палубы
    col_z0 = INSERT_H + 1
    canopy = canopy.union(cyls(CANOPY_SCREWS, BOSS_D, col_z0, CANOPY_INNER_H - col_z0))
    canopy = canopy.cut(cyls(CANOPY_SCREWS, M3_CLEAR, 0, h))
    canopy = canopy.cut(cyls(CANOPY_SCREWS, M3_HEAD_D, h - 3.0, 3.0))

    # Стойка GPS: утолщение, 4 отверстия M3, кабель (уплотнить втулкой)
    gx, gy = GPS_HOLDER_POS
    pad = GPS_HOLDER_HOLES[0] + 12
    pad_z = CANOPY_INNER_H - GPS_PAD_T
    canopy = canopy.union(
        cq.Workplane("XY").workplane(offset=pad_z).center(gx, gy).rect(pad, pad).extrude(GPS_PAD_T))
    canopy = canopy.cut(cyls(square_pts(GPS_HOLDER_POS, *GPS_HOLDER_HOLES), M3_CLEAR, pad_z, h))
    canopy = canopy.cut(cyls([(gx - pad / 2 - GPS_CABLE_D / 2 - 1, gy)], GPS_CABLE_D, pad_z, h))

    # Кнопка safety
    canopy = canopy.cut(cyls([SAFETY_POS], SAFETY_D, CANOPY_INNER_H, CANOPY_TOP_T))

    # Вентиляция в боковых стенках, снаружи козырьки, открытые вниз
    vent_z0, vent_z1 = RIM_H + 4.0, CANOPY_INNER_H - 6.0
    pitch = VENT_W * 2.5
    span = pitch * (VENT_N - 1) + VENT_W
    for s in (-1, 1):
        y_wall = s * BODY_HW
        for i in range(VENT_N):
            x = (i - (VENT_N - 1) / 2) * pitch
            canopy = canopy.cut(
                cq.Workplane("XY").box(VENT_W, WALL * 3, vent_z1 - vent_z0, centered=(True, True, False))
                .translate((x, y_wall - s * WALL / 2, vent_z0)))
        hood_d = HOOD_GAP + HOOD_T
        hood = (
            cq.Workplane("XY")
            .box(span + 2 * HOOD_T + 6, hood_d, h - (vent_z0 - 4), centered=(True, True, False))
            .translate((0, y_wall + s * hood_d / 2, vent_z0 - 4)))
        air = (
            cq.Workplane("XY")
            .box(span + 6, HOOD_GAP, h - CANOPY_TOP_T - (vent_z0 - 5), centered=(True, True, False))
            .translate((0, y_wall + s * HOOD_GAP / 2, vent_z0 - 5)))
        canopy = canopy.union(hood).cut(air)
    return canopy


# ---------------------------------------------------------------------------
# Отсек батареи и люк
# ---------------------------------------------------------------------------
def make_battery_bay():
    """Отсек под палубой. Координаты: z = 0 — низ палубы. Печатать фланцами вниз."""
    bay = (
        cq.Workplane("XY").box(BAY_OUT[0], BAY_OUT[1], BAY_IN[2], centered=(True, True, False))
        .translate((BAY_CX, 0, -BAY_IN[2]))
    )
    bay = bay.cut(
        cq.Workplane("XY").box(BAY_IN[0], BAY_IN[1], BAY_IN[2], centered=(True, True, False))
        .translate((BAY_CX, 0, -BAY_IN[2])))

    # Фланцы вдоль бортов, винты снизу в термовставки палубы
    for s in (-1, 1):
        flange = (
            cq.Workplane("XY")
            .box(BAY_OUT[0], BAY_FLANGE_W, BAY_FLANGE_T, centered=(True, True, False))
            .translate((BAY_CX, s * (BAY_OUT[1] / 2 + BAY_FLANGE_W / 2), -BAY_FLANGE_T)))
        bay = bay.union(flange)
    pts = [(x, s * (BAY_OUT[1] / 2 + BAY_FLANGE_W / 2)) for x in BAY_SCREW_X for s in (-1, 1)]
    bay = bay.cut(cyls(pts, M3_CLEAR, -BAY_FLANGE_T, BAY_FLANGE_T))

    # Проушины люка по бортам, скос 45° сверху (печать без поддержек)
    for s in (-1, 1):
        y0 = s * BAY_OUT[1] / 2
        prof = [(0, 0), (s * LUG_W, 0), (s * LUG_W, LUG_H), (0, LUG_H + LUG_W)]
        lug = (
            cq.Workplane("YZ").polyline(prof).close().extrude(LUG_L)
            .translate((BAY_CX - LUG_L / 2, y0, -BAY_IN[2])))
        bay = bay.union(lug)
        bay = bay.cut(cyls([(BAY_CX, y0 + s * LUG_W / 2)], INSERT_D, -BAY_IN[2], INSERT_H + 1))
    return bay


def make_hatch():
    """Люк. Бортик заходит внутрь отсека (защита от брызг). Печатать плоскостью вниз."""
    hatch = (
        cq.Workplane("XY").box(BAY_OUT[0], BAY_OUT[1], HATCH_T, centered=(True, True, False))
        .edges("|Z").fillet(2.0)
    )
    il, iw = BAY_IN[0] - 2 * FIT, BAY_IN[1] - 2 * FIT
    hatch = hatch.union(
        cq.Workplane("XY").workplane(offset=HATCH_T)
        .rect(il, iw).rect(il - 2 * 1.6, iw - 2 * 1.6).extrude(HATCH_LIP_H))
    for s in (-1, 1):
        ear = (
            cq.Workplane("XY").center(0, s * (BAY_OUT[1] / 2 + LUG_W / 2))
            .rect(LUG_L, LUG_W).extrude(HATCH_T)
            .faces(">Z").workplane().circle(M3_CLEAR / 2).cutThruAll())
        hatch = hatch.union(ear)
    # Слив воды
    hatch = hatch.cut(cyls([(-BAY_IN[0] / 3, 0), (BAY_IN[0] / 3, 0)], 3.0, 0, HATCH_T))
    return hatch


# ---------------------------------------------------------------------------
# Луч, крышка луча, опора
# ---------------------------------------------------------------------------
def make_arm():
    """Луч в своих координатах: x от корня, верх z = 0. Печатать верхом вниз."""
    length = MOTOR_X
    arm = (
        cq.Workplane("XY").box(length, ARM_W, ARM_H, centered=(False, True, False))
        .translate((0, 0, -ARM_H))
    )
    arm = arm.union(cyls([(MOTOR_X, 0)], 2 * MOTOR_PAD_R, -ARM_H, ARM_H))

    # Канал под ESC, снизу открыт; фальц под крышку
    ch_len = CHANNEL_X1 - CHANNEL_X0
    arm = arm.cut(
        cq.Workplane("XY").box(ch_len, ARM_IN_W, ARM_H - WALL, centered=(False, True, False))
        .translate((CHANNEL_X0, 0, -ARM_H)))
    arm = arm.cut(
        cq.Workplane("XY").box(ch_len, ARM_IN_W + 2.4, ARM_COVER_T, centered=(False, True, False))
        .translate((CHANNEL_X0, 0, -ARM_H)))

    # Стойки: сверху вставки под болты палубы, снизу под крышку луча
    pillar_z0 = -ARM_H + ARM_COVER_T
    deck_pts = [(x, y) for x in DECK_BOLT_X for y in (-PILLAR_Y, PILLAR_Y)]
    tip_pts = [(COVER_TIP_X, y) for y in (-PILLAR_Y, PILLAR_Y)]
    arm = arm.union(cyls(deck_pts + tip_pts, PILLAR_D, pillar_z0, -pillar_z0))
    arm = arm.cut(cyls(deck_pts, INSERT_D, -INSERT_H, INSERT_H))
    arm = arm.cut(cyls(deck_pts + tip_pts, INSERT_D, pillar_z0, INSERT_H))

    # Окно проводов в палубу
    arm = arm.cut(
        cq.Workplane("XY").center(ARM_WIRE_X, 0).rect(*ARM_WIRE_HOLE).extrude(-WALL - 0.1))

    # Мотор: отверстия крестом, зенковки снизу, отверстие под вал, провода в канал
    mx = MOTOR_X
    holes = [(mx - MOTOR_HOLES[0] / 2, 0), (mx + MOTOR_HOLES[0] / 2, 0),
             (mx, -MOTOR_HOLES[1] / 2), (mx, MOTOR_HOLES[1] / 2)]
    arm = arm.cut(cyls(holes, M3_CLEAR, -ARM_H, ARM_H))
    arm = arm.cut(cyls(holes, M3_HEAD_D, -ARM_H, ARM_H - MOTOR_PLATE_T))
    arm = arm.cut(cyls([(mx, 0)], MOTOR_SHAFT_D, -ARM_H, ARM_H))
    arm = arm.cut(cyls([(CHANNEL_X1 - 6, 0)], 9.0, -WALL - 0.1, WALL + 0.2))

    # Вставки под опору шасси снизу
    arm = arm.cut(cyls([(LEG_X, -LEG_HOLE_Y), (LEG_X, LEG_HOLE_Y)], INSERT_D, -ARM_H, INSERT_H))
    return arm


def make_arm_cover():
    """Крышка канала луча. Координаты луча, низ z = 0."""
    ch_len = CHANNEL_X1 - CHANNEL_X0 - 2 * FIT
    w = ARM_IN_W + 2.4 - 2 * FIT
    cover = (
        cq.Workplane("XY").box(ch_len, w, ARM_COVER_T, centered=(False, True, False))
        .translate((CHANNEL_X0 + FIT, 0, 0))
    )
    pts = [(x, y) for x in (*DECK_BOLT_X, COVER_TIP_X) for y in (-PILLAR_Y, PILLAR_Y)]
    cover = cover.cut(cyls(pts, M3_CLEAR, 0, ARM_COVER_T))
    # Прорези охлаждения ESC и слива
    esc_x0 = DECK_BOLT_X[1] + PILLAR_D / 2 + 4
    esc_x1 = COVER_TIP_X - PILLAR_D / 2 - 4
    n = int((esc_x1 - esc_x0) // 8)
    slots = [(esc_x0 + 4 + i * 8, 0) for i in range(n)]
    cover = cover.cut(
        cq.Workplane("XY").pushPoints(slots).slot2D(ARM_IN_W - 10, 3.0, angle=90).extrude(ARM_COVER_T))
    # Отверстие под кабель датчика потока (если он на этом луче)
    cover = cover.cut(cyls([(DECK_BOLT_X[0] + 10, 0)], 6.0, 0, ARM_COVER_T))
    return cover


def make_leg():
    """Опора: лопасть поперёк луча. Координаты: верх z = 0 (низ луча), толщина вдоль X.
    Печатать лёжа на боку."""
    top_w = ARM_W
    prof = [(-top_w / 2, 0), (top_w / 2, 0), (LEG_FOOT_W / 2, -LEG_H),
            (-LEG_FOOT_W / 2, -LEG_H)]
    leg = cq.Workplane("YZ").polyline(prof).close().extrude(LEG_T).translate((-LEG_T / 2, 0, 0))
    # Облегчающее окно, через него закручиваются винты
    win = [(-top_w / 2 + 6, -10), (top_w / 2 - 6, -10),
           (LEG_FOOT_W / 2 - 10, -LEG_H + 8), (-LEG_FOOT_W / 2 + 10, -LEG_H + 8)]
    leg = leg.cut(cq.Workplane("YZ").polyline(win).close().extrude(LEG_T).translate((-LEG_T / 2, 0, 0)))
    leg = leg.cut(cyls([(0, -LEG_HOLE_Y), (0, LEG_HOLE_Y)], M3_CLEAR, -10, 10))
    return leg


def make_flow_bracket():
    """Кронштейн MTF-02P: крепится к стойкам луча через крышку луча.
    Координаты: верх z = 0 (низ крышки луча)."""
    il, iw = FLOW[0] + 2 * FIT, FLOW[1] + 2 * FIT
    h = FLOW_BRACKET_T + FLOW[2]
    span = 2 * PILLAR_Y + 8
    bracket = cq.Workplane("XY").box(il + 4, span, FLOW_BRACKET_T, centered=(True, True, False))
    bracket = bracket.union(
        cq.Workplane("XY").box(il + 4, iw + 4, h, centered=(True, True, False)))
    bracket = bracket.cut(
        cq.Workplane("XY").box(il, iw, FLOW[2], centered=(True, True, False))
        .translate((0, 0, FLOW_BRACKET_T)))
    bracket = bracket.cut(cq.Workplane("XY").rect(*FLOW_WINDOW).extrude(FLOW_BRACKET_T))
    bracket = bracket.cut(cyls([(0, -PILLAR_Y), (0, PILLAR_Y)], M3_CLEAR, 0, FLOW_BRACKET_T))
    # Повернуть карманом вниз: верх кронштейна прилегает к крышке луча
    return bracket.mirror("XY")


# ---------------------------------------------------------------------------
# Проверки, экспорт, сборка
# ---------------------------------------------------------------------------
def check_fits_bed(name, shape):
    bb = shape.val().BoundingBox()
    dims = sorted((bb.xlen, bb.ylen, bb.zlen))
    bed = sorted((BED_X, BED_Y, BED_Z))
    ok = all(d <= b for d, b in zip(dims, bed))
    status = "OK" if ok else "НЕ ВЛЕЗАЕТ НА СТОЛ"
    print(f"{name:14s} {bb.xlen:6.1f} x {bb.ylen:6.1f} x {bb.zlen:5.1f} мм  {status}")
    return ok


def ghost_box(size, center_xy, z0):
    return (cq.Workplane("XY").box(*size, centered=(True, True, False))
            .translate((*center_xy, z0)))


def build_assembly(parts):
    asm = cq.Assembly(name="drone_frame")
    body = cq.Color(0.75, 0.75, 0.78)
    asm.add(parts["deck"], name="deck", color=body)
    asm.add(parts["canopy"], name="canopy", color=cq.Color(0.2, 0.4, 0.8, 0.4),
            loc=cq.Location((0, 0, DECK_T)))
    asm.add(parts["battery_bay"], name="battery_bay", color=body)
    asm.add(parts["hatch"], name="hatch", color=body,
            loc=cq.Location((BAY_CX, 0, -BAY_IN[2] - HATCH_T)))
    for i, a in enumerate(ARM_ANGLES):
        asm.add(place_on_arm(parts["arm"], a), name=f"arm_{i}", color=body)
        asm.add(place_on_arm(parts["arm_cover"].translate((0, 0, -ARM_H)), a),
                name=f"arm_cover_{i}", color=body)
        asm.add(place_on_arm(parts["leg"].translate((LEG_X, 0, -ARM_H)), a),
                name=f"leg_{i}", color=body)
    asm.add(place_on_arm(parts["flow_bracket"].translate((DECK_BOLT_X[0], 0, -ARM_H)), ARM_ANGLES[2]),
            name="flow_bracket", color=body)

    # Макеты компонентов для проверки (не печатать)
    ghost = cq.Color(0.9, 0.5, 0.1, 0.6)
    top = DECK_T
    stack = top + PIXHAWK_STACK_H - PIXHAWK[2]
    ghosts = {
        "pixhawk": ghost_box(PIXHAWK, PIXHAWK_POS, stack),
        "damper": ghost_box((*DAMPER_PLATE[:2], stack - top - DAMPER_BOSS_H), PIXHAWK_POS,
                            top + DAMPER_BOSS_H),
        "orange_pi": ghost_box((OPI[1], OPI[0], OPI[2] + OPI_TOP_H), OPI_POS, top + OPI_STANDOFF_H),
        "receiver": ghost_box(RX, RX_POS, top),
        "pdb": ghost_box((*PDB, 5.0), PDB_POS, top + PDB_STANDOFF_H),
        "power_module": ghost_box(PM, PM_POS, top),
        "battery": ghost_box(BATTERY, (BAY_CX, 0), -BATTERY[2]),  # прижата пеной к палубе
        "gimbal": ghost_box(GIMBAL, (GIMBAL_X, 0), -GIMBAL_SPACER - GIMBAL[2]),
    }
    for i, a in enumerate(ARM_ANGLES):
        mx, my = MOTOR_XY[i]
        ghosts[f"motor_{i}"] = cyls([(mx, my)], MOTOR_D, 0, MOTOR_H)
        ghosts[f"prop_{i}"] = cyls([(mx, my)], PROP_D, MOTOR_H + 5, 2)
        ex, ey = arm_point(a, (DECK_BOLT_X[1] + COVER_TIP_X) / 2)
        ghosts[f"esc_{i}"] = (
            cq.Workplane("XY").box(*ESC, centered=(True, True, False))
            .rotate((0, 0, 0), (0, 0, 1), a).translate((ex, ey, -ARM_H + ARM_COVER_T)))
    for name, shape in ghosts.items():
        asm.add(shape, name=f"ref_{name}", color=ghost)
    return asm


def main():
    OUT_DIR.mkdir(exist_ok=True)
    parts = {
        "deck": make_deck(),
        "canopy": make_canopy(),
        "battery_bay": make_battery_bay(),
        "hatch": make_hatch(),
        "arm": make_arm(),
        "arm_cover": make_arm_cover(),
        "leg": make_leg(),
        "flow_bracket": make_flow_bracket(),
    }
    for name, shape in parts.items():
        check_fits_bed(name, shape)
        cq.exporters.export(shape, str(OUT_DIR / f"{name}.step"))
        cq.exporters.export(shape, str(OUT_DIR / f"{name}.stl"),
                            tolerance=0.05, angularTolerance=0.1)
    build_assembly(parts).export(str(OUT_DIR / "assembly.step"))
    print(f"Диагональ моторов {MOTOR_DIAG:.0f} мм, высота от земли до палубы {-GROUND_Z:.0f} мм")
    print(f"Файлы сохранены в {OUT_DIR}")


if __name__ == "__main__":
    main()
