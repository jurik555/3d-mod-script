"""Параметрическая рама-корпус спасательного дрона (CadQuery).

Своя рама вместо F450: обтекаемый корпус, внутри вся электроника, провода и батарея,
ESC в лучах, моторы на стойках над корпусом.
Запуск:  python drone_frame.py
Результат: папка out/ с STEP (для Onshape) и STL (для печати) по каждой детали
и общий STEP сборки с макетами компонентов.

Оси: X вперёд (по стрелке Pixhawk), Y влево, Z вверх. Все размеры в мм.
Z = 0 — нижняя плоскость палубы (верх лучей).
Параметры с пометкой "ИЗМЕРИТЬ" взяты из спецификаций или угаданы.

Корпус — лофт по сечениям (таблицы UPPER_STATIONS и LOWER_STATIONS):
верхняя половина — крышка, нижняя — отсек батареи. Форму меняют цифрами в таблицах.

Детали:
  deck          палуба: несущая плита, сверху электроника, снизу лучи, нижний корпус, гимбал
  canopy        верхний обтекатель над электроникой (печать с поддержками внутри)
  lower_shell   нижний корпус: отсек батареи, точки крепления сбрасываемого контейнера
  tail_cap      хвостовой люк: батарея вставляется сзади, два барашковых винта M3 по бокам
  arm x4        полый луч, внутри ESC
  arm_cover x4  нижняя крышка луча с прорезями для охлаждения ESC и слива воды
  motor_pod x4  стойка мотора: поднимает винты выше корпуса
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
DAMPER_BOSS_H = 3.0
PIXHAWK_TAPE = 1.0
PIXHAWK_STACK_H = (DAMPER_BOSS_H + DAMPER_PLATE[2] + DAMPER_BALL_H
                   + DAMPER_TOP_T + PIXHAWK_TAPE + PIXHAWK[2])

OPI = (65.0, 32.0, 1.6)            # Orange Pi Zero 3W (ИЗМЕРИТЬ)
OPI_HOLES = (58.0, 25.0)           # шаг отверстий платы (ИЗМЕРИТЬ)
OPI_TOP_H = 14.0                   # высота над платой с радиатором (ИЗМЕРИТЬ)
OPI_STANDOFF_H = 5.0
RX = (47.0, 26.2, 15.0)            # FlySky FS-iA6B, антенны внутри обтекателя
PDB = (36.0, 36.0)                 # плата распределения питания (ИЗМЕРИТЬ свою)
PDB_HOLES = 30.5
PDB_STANDOFF_H = 5.0
PM = (25.0, 21.0, 9.0)             # power module 3DR (ИЗМЕРИТЬ)
ESC = (73.0, 28.0, 12.0)           # Emax BLHeli 40A без проводов (ИЗМЕРИТЬ)
FLOW = (21.6, 16.0, 6.5)           # MicoAir MTF-02P
GIMBAL = (55.0, 55.0, 70.0)        # SIYI A8 Mini
GIMBAL_HOLES = (40.0, 40.0)        # шаг крепёжных отверстий гимбала (ИЗМЕРИТЬ)
GIMBAL_CABLE_D = 14.0
BATTERY = (140.0, 45.0, 40.0)      # Gens Ace 4S (ИЗМЕРИТЬ свою!)
BATT_FOAM = 4.0                    # пена сверху батареи
BATT_SLIDE = 4.0                   # запас по длине

# Моторы 2212 и винты 1045
MOTOR_DIAG = 450.0
MOTOR_HOLES = (16.0, 19.0)         # крестовой шаг отверстий мотора
MOTOR_SHAFT_D = 8.0
MOTOR_D, MOTOR_H = 28.0, 30.0      # для макета и расчёта высоты винтов
PROP_D = 254.0                     # 10 дюймов
PROP_HUB_H = 6.0                   # от верха мотора до плоскости винта
PROP_CLEAR = 6.0                   # зазор в плане от концов винтов
PROP_VCLEAR = 12.0                 # зазор по высоте над обтекателем

# ---------------------------------------------------------------------------
# Компоновка по длине (от носа назад)
# ---------------------------------------------------------------------------
BODY_FRONT = 115.0
GIMBAL_SPACER = 5.0
GIMBAL_X = BODY_FRONT - GIMBAL[0] / 2 - 2.0
LOWER_FRONT = GIMBAL_X - GIMBAL[0] / 2 - 2.0     # нижний корпус кончается перед гимбалом
BATT_FRONT = LOWER_FRONT - 8.0
BATT_REAR = BATT_FRONT - BATTERY[0] - BATT_SLIDE
X_TAIL = BATT_REAR - 1.0                         # стык нижнего корпуса и хвостового люка
TAIL_LEN = 10.0
BODY_REAR = X_TAIL - TAIL_LEN
LOWER_Z = -(BATTERY[2] + BATT_FOAM + WALL + FIT)  # низ нижнего корпуса

# Сечения корпуса: (x, полуширина, верх обтекателя z) и (x, полуширина, низ z)
R_TOP, R_BOTTOM = 30.0, 22.0       # радиусы скругления сечений
UPPER_STATIONS = [
    (BODY_REAR, 50.0, 24.0),
    (BODY_REAR + 12, 78.0, 34.0),
    (-70.0, 90.0, 44.0),
    (-20.0, 90.0, 54.0),
    (40.0, 90.0, 52.0),
    (75.0, 88.0, 42.0),
    (BODY_FRONT, 40.0, 18.0),
]
LOWER_STATIONS = [
    (BODY_REAR, 60.0, LOWER_Z + 12),
    (X_TAIL, 86.0, LOWER_Z),
    (-70.0, 90.0, LOWER_Z),
    (-20.0, 90.0, LOWER_Z),
    (LOWER_FRONT, 90.0, LOWER_Z),
]
LOWER_FRONT_R = 8.0                # скругление переднего торца нижнего корпуса

DECK_T = 5.0
RIM_T, RIM_H = 1.6, 5.0            # бортик-лабиринт под обтекателем

# Раскладка на палубе (центры)
CABLE_GAP = 12.0
PIXHAWK_POS = (0.0, 0.0)
OPI_POS = (-(DAMPER_PLATE[0] / 2 + CABLE_GAP + OPI[1] / 2), 0.0)
PDB_POS = (15.0, 56.0)
RX_POS = (-10.0, -54.0)
PM_POS = (30.0, -56.0)
BATT_SLOT = (8.0, 30.0)
BATT_SLOT_POS = (-(DAMPER_PLATE[0] / 2 + CABLE_GAP / 2), 0.0)
CANOPY_SCREWS = [(x, y) for x in (-40.0, 40.0) for y in (-79.0, 79.0)]
LOWER_SCREWS = [(-30.0, -81.0), (-30.0, 81.0), (20.0, -81.0), (20.0, 81.0),
                (LOWER_FRONT - 3.0, 0.0)]

# Обтекатель
GPS_HOLDER_HOLES = (27.0, 27.0)    # шаг отверстий основания стойки GPS (ИЗМЕРИТЬ)
GPS_HOLDER_POS = (-20.0, 0.0)
GPS_PAD_D = 44.0
GPS_CABLE_D = 8.0
SAFETY_D = 12.0                    # корпус кнопки safety (ИЗМЕРИТЬ)
SAFETY_POS = (45.0, -30.0)
VENT_X = (-40.0, 0.0)              # вентиляция в бортах под козырьками
VENT_Z = (DECK_T + RIM_H + 3, DECK_T + RIM_H + 15)
VENT_W, VENT_N = 3.0, 6
HOOD_GAP, HOOD_T = 4.0, 1.6

# Нижний корпус и хвост
RAIL_H = 3.0                       # направляющие батареи по полу
SLEEVE_L = 10.0                    # юбка хвостового люка внутрь корпуса
TAIL_SCREW_Z = LOWER_Z / 2
CONTAINER_HOLES = (80.0, 80.0)     # точки крепления будущего сбрасываемого контейнера
CONTAINER_X = (X_TAIL + LOWER_FRONT) / 2
CONTAINER_H = 0.0                  # высота контейнера: прибавится к высоте шасси

# Лучи
ARM_ANGLES = (45.0, 135.0, 225.0, 315.0)
MOTOR_R = MOTOR_DIAG / 2
ARM_ROOT_R = 66.0
ARM_IN_W = ESC[1] + 2.0
ARM_IN_H = ESC[2] + 3.0
ARM_COVER_T = 2.0
ARM_W = ARM_IN_W + 2 * WALL
ARM_H = WALL + ARM_IN_H + ARM_COVER_T
ARM_TOP_FILLET = 6.0
MOTOR_X = MOTOR_R - ARM_ROOT_R
ARM_TIP_R = 22.0
CHANNEL_TIP_R = 19.0
CHANNEL_X0 = 3.0
COVER_X1 = MOTOR_X - 12.0
PILLAR_D = 8.0
PILLAR_Y = ARM_IN_W / 2 - 2.0
DECK_BOLT_X = (8.0, 34.0)
COVER_TIP_X = MOTOR_X - ARM_TIP_R - 16.0   # кронштейн датчика не задевает опору
ARM_WIRE_X, ARM_WIRE_HOLE = 21.0, (12.0, 16.0)

# Стойка мотора и опора
POD_WALL = 3.0
POD_TOP_T = 4.0
POD_RING_IN_R = 11.0
POD_RING_T = 5.0
POD_SCREW_R = 14.5                 # 4 вставки: 2 крепят стойку, 2 — опору (длинные винты)
ARM_TOP_HOLE_R = 10.5              # доступ к винтам мотора снизу
POD_MIN_H = 10.0
LEG_D, LEG_WALL = 20.0, 3.0
LEG_FLANGE_T = 4.0
LEG_FOOT_D = 36.0

# Кронштейн MTF-02P
FLOW_BRACKET_T = 2.0
FLOW_WINDOW = (14.0, 10.0)   # окно под оптику и лидар (ИЗМЕРИТЬ)

# Высота шасси: гимбал и будущий контейнер на 15 мм выше земли
GROUND_Z = min(-(GIMBAL_SPACER + GIMBAL[2]), LOWER_Z - CONTAINER_H) - 15.0
LEG_H = -GROUND_Z - ARM_H

OUT_DIR = Path(__file__).parent / "out"
MOTOR_XY = [(MOTOR_R * math.cos(math.radians(a)), MOTOR_R * math.sin(math.radians(a)))
            for a in ARM_ANGLES]


# ---------------------------------------------------------------------------
# Вспомогательные функции
# ---------------------------------------------------------------------------
def rot(pt, angle):
    a = math.radians(angle)
    x, y = pt
    return (x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a))


def arm_point(angle, x, y=0.0):
    return rot((ARM_ROOT_R + x, y), angle)


def place_on_arm(shape, angle):
    return shape.translate((ARM_ROOT_R, 0, 0)).rotate((0, 0, 0), (0, 0, 1), angle)


def cyls(points, d, z0, h):
    return cq.Workplane("XY").workplane(offset=z0).pushPoints(points).circle(d / 2).extrude(h)


def zbox(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False)
            .translate((x0, y0, z0)))


def slab(z0, z1):
    return zbox(-500, 500, -500, 500, z0, z1)


def rect_ring(center, size, wall, z0, h, angle=0.0):
    il, iw = size
    ring = (cq.Workplane("XY").workplane(offset=z0)
            .rect(il + 2 * wall, iw + 2 * wall).rect(il, iw).extrude(h))
    return ring.rotate((0, 0, 0), (0, 0, 1), angle).translate((*center, 0))


def square_pts(center, sx, sy):
    cx, cy = center
    return [(cx + dx, cy + dy) for dx in (-sx / 2, sx / 2) for dy in (-sy / 2, sy / 2)]


def polar_pts(center, r, angles):
    cx, cy = center
    return [(cx + r * math.cos(math.radians(a)), cy + r * math.sin(math.radians(a)))
            for a in angles]


def _loft(sections):
    sk = [s.moved(cq.Location(cq.Vector(0, 0, x))) for x, s in sections]
    return cq.Workplane("YZ").placeSketch(*sk).loft()


def upper_loft(inset=0.0):
    """Верхняя половина корпуса (обтекатель + палуба), смещённая внутрь на inset."""
    secs = []
    n = len(UPPER_STATIONS)
    for i, (x, hw, zt) in enumerate(UPPER_STATIONS):
        if i == 0:
            x += inset
        elif i == n - 1:
            x -= inset
        zb = -1.0 - inset          # без совпадающих граней при вычитании
        top = zt - inset
        w = hw - inset
        r = max(1.0, min(R_TOP - inset, (top - zb) * 0.8, w - 1.0))
        s = (cq.Sketch().push([(0, (top + zb) / 2)]).rect(2 * w, top - zb).reset()
             .vertices(">Y").fillet(r))
        secs.append((x, s))
    return _loft(secs)


def lower_loft(inset=0.0):
    """Нижняя половина корпуса, смещённая внутрь на inset."""
    secs = []
    n = len(LOWER_STATIONS)
    for i, (x, hw, zb) in enumerate(LOWER_STATIONS):
        if i == 0:
            x += inset
        elif i == n - 1:
            x -= inset
        # Строится вверх, затем поворачивается вниз
        bot = -1.0 - inset
        top = -zb - inset
        w = hw - inset
        r = max(1.0, min(R_BOTTOM - inset, (top - bot) * 0.8, w - 1.0))
        s = (cq.Sketch().push([(0, (top + bot) / 2)]).rect(2 * w, top - bot).reset()
             .vertices(">Y").fillet(r))
        secs.append((x, s))
    body = _loft(secs).faces(">X").edges().fillet(max(1.0, LOWER_FRONT_R - inset))
    return body.rotate((0, 0, 0), (1, 0, 0), 180)  # симметрично по Y


def probe_z_top(solid, x, y):
    """Верхняя точка тела над точкой (x, y)."""
    return solid.intersect(cyls([(x, y)], 1.0, -100, 300)).val().BoundingBox().zmax


def probe_half_width(solid, x, z):
    return solid.intersect(zbox(x - 0.5, x + 0.5, -300, 300, z - 0.5, z + 0.5)).val().BoundingBox().ymax


# ---------------------------------------------------------------------------
# Палуба
# ---------------------------------------------------------------------------
def make_deck():
    deck = upper_loft().intersect(slab(0, DECK_T))
    top = DECK_T

    rim = (upper_loft(WALL + FIT).cut(upper_loft(WALL + FIT + RIM_T))
           .intersect(slab(top, top + RIM_H)))
    deck = deck.union(rim)

    deck = deck.union(cyls(CANOPY_SCREWS, BOSS_D, top, INSERT_H + 1))
    deck = deck.cut(cyls(CANOPY_SCREWS, INSERT_D, top + 1, INSERT_H))

    damper = square_pts(PIXHAWK_POS, *DAMPER_HOLES)
    deck = deck.union(cyls(damper, BOSS_D - 2, top, DAMPER_BOSS_H))
    deck = deck.cut(cyls(damper, INSERT_D, top + DAMPER_BOSS_H - INSERT_H, INSERT_H))

    opi = square_pts(OPI_POS, OPI_HOLES[1], OPI_HOLES[0])
    deck = deck.union(cyls(opi, 6.0, top, OPI_STANDOFF_H))
    deck = deck.cut(cyls(opi, 2.2, 0, top + OPI_STANDOFF_H))

    pdb = square_pts(PDB_POS, PDB_HOLES, PDB_HOLES)
    deck = deck.union(cyls(pdb, 7.0, top, PDB_STANDOFF_H))
    deck = deck.cut(cyls(pdb, INSERT_D, top + PDB_STANDOFF_H - INSERT_H, INSERT_H))

    deck = deck.union(rect_ring(RX_POS, (RX[0] + 2 * FIT, RX[1] + 2 * FIT), 1.6, top, 4.0))
    deck = deck.union(rect_ring(PM_POS, (PM[0] + 2 * FIT, PM[1] + 2 * FIT), 1.6, top, 4.0))

    gimbal = square_pts((GIMBAL_X, 0), *GIMBAL_HOLES)
    deck = deck.union(cyls(gimbal, BOSS_D - 2, top, 3.0))
    deck = deck.cut(cyls(gimbal, INSERT_D, 0, INSERT_H))
    deck = deck.cut(cyls([(GIMBAL_X, 0)], GIMBAL_CABLE_D, 0, top))

    deck = deck.cut(
        cq.Workplane("XY").center(*BATT_SLOT_POS).slot2D(BATT_SLOT[1], BATT_SLOT[0], angle=90)
        .extrude(top))

    # Нижний корпус: винты снизу во вставки палубы
    deck = deck.cut(cyls(LOWER_SCREWS, INSERT_D, 0, top))

    for a in ARM_ANGLES:
        bolts = [arm_point(a, x, y) for x in DECK_BOLT_X for y in (-PILLAR_Y, PILLAR_Y)]
        deck = deck.cut(cyls(bolts, M3_CLEAR, 0, top))
        wx, wy = arm_point(a, ARM_WIRE_X)
        deck = deck.cut(
            cq.Workplane("XY").rect(*ARM_WIRE_HOLE).extrude(top)
            .rotate((0, 0, 0), (0, 0, 1), a).translate((wx, wy, 0)))
    return deck


# ---------------------------------------------------------------------------
# Обтекатель
# ---------------------------------------------------------------------------
def make_canopy():
    """Обтекатель в координатах корпуса (стоит на палубе, z от DECK_T)."""
    outer = upper_loft()
    canopy = outer.cut(upper_loft(WALL)).intersect(slab(DECK_T, 300))

    # Колонны под винты
    col_z0 = DECK_T + INSERT_H + 1
    canopy = canopy.union(cyls(CANOPY_SCREWS, BOSS_D, col_z0, 300).intersect(outer))
    canopy = canopy.cut(cyls(CANOPY_SCREWS, M3_CLEAR, 0, 300))
    for p in CANOPY_SCREWS:
        canopy = canopy.cut(cyls([p], M3_HEAD_D, probe_z_top(outer, *p) - 3.0, 50))

    # Площадка стойки GPS: плоская, выступает над поверхностью
    gz = probe_z_top(outer, *GPS_HOLDER_POS)
    canopy = canopy.union(cyls([GPS_HOLDER_POS], GPS_PAD_D, gz - 8, 10))
    canopy = canopy.cut(cyls(square_pts(GPS_HOLDER_POS, *GPS_HOLDER_HOLES), M3_CLEAR, gz - 8, 20))
    gx, gy = GPS_HOLDER_POS
    canopy = canopy.cut(cyls([(gx - GPS_PAD_D / 2 + GPS_CABLE_D / 2 + 3, gy)], GPS_CABLE_D,
                             gz - 8, 20))

    # Кнопка safety
    canopy = canopy.cut(cyls([SAFETY_POS], SAFETY_D, DECK_T + 20, 300))

    # Вентиляция в бортах, снаружи козырьки, открытые вниз
    x0, x1 = VENT_X
    z0, z1 = VENT_Z
    pitch = (x1 - x0) / VENT_N
    hood_outer = upper_loft(-HOOD_GAP - HOOD_T)
    hood_air = upper_loft(-HOOD_GAP)
    for s in (-1, 1):
        ys = (0, 300) if s > 0 else (-300, 0)
        for i in range(VENT_N):
            xc = x0 + (i + 0.5) * pitch
            canopy = canopy.cut(zbox(xc - VENT_W / 2, xc + VENT_W / 2, *ys, z0, z1)
                                .intersect(outer).cut(upper_loft(WALL)))
        region = zbox(x0 - 3 - HOOD_T, x1 + 3 + HOOD_T, *ys, z0 - 4, z1 + 4)
        hood = hood_outer.intersect(region).cut(outer)
        air = hood_air.intersect(zbox(x0 - 3, x1 + 3, *ys, z0 - 5, z1 + 2)).cut(outer)
        canopy = canopy.union(hood.cut(air))
    return canopy


# ---------------------------------------------------------------------------
# Нижний корпус и хвостовой люк
# ---------------------------------------------------------------------------
def arm_clearance():
    cut = None
    for a in ARM_ANGLES:
        box = (cq.Workplane("XY")
               .box(MOTOR_R - ARM_ROOT_R + 2, ARM_W + 2 * FIT, ARM_H + FIT + 2,
                    centered=(False, True, False))
               .translate((ARM_ROOT_R - 1, 0, -ARM_H - FIT))
               .rotate((0, 0, 0), (0, 0, 1), a))
        cut = box if cut is None else cut.union(box)
    return cut


def lower_shell_raw():
    return lower_loft().cut(lower_loft(WALL)).intersect(slab(-300, 0))


def make_lower_shell():
    outer = lower_loft()
    shell = lower_shell_raw().intersect(zbox(X_TAIL, 500, -300, 300, -300, 0))

    # Проушины под винты в палубу
    shell = shell.union(cyls(LOWER_SCREWS, 10.0, -8.0, 8.0).intersect(outer))
    shell = shell.cut(cyls(LOWER_SCREWS, M3_CLEAR, -8.0, 8.0))

    # Направляющие батареи
    floor_z = LOWER_Z + WALL
    for s in (-1, 1):
        y = s * (BATTERY[1] / 2 + FIT + 1.0)
        rail = zbox(X_TAIL + SLEEVE_L + 1, BATT_FRONT, y - 1.0, y + 1.0, LOWER_Z, floor_z + RAIL_H)
        shell = shell.union(rail.intersect(outer))

    # Точки крепления сбрасываемого контейнера: вставки снизу
    cont = square_pts((CONTAINER_X, 0), *CONTAINER_HOLES)
    shell = shell.union(cyls(cont, BOSS_D, LOWER_Z, WALL + 5.0).intersect(outer))
    shell = shell.cut(cyls(cont, INSERT_D, LOWER_Z, INSERT_H))

    # Отверстия под винты хвостового люка
    xs = X_TAIL + SLEEVE_L / 2
    shell = shell.cut(
        cq.Workplane("XZ").center(xs, TAIL_SCREW_Z).circle(M3_CLEAR / 2).extrude(300, both=True))

    return shell.cut(arm_clearance())


def make_tail_cap():
    outer = lower_loft()
    cap = lower_shell_raw().intersect(zbox(-500, X_TAIL, -300, 300, -300, 0))
    # Юбка внутрь нижнего корпуса по бортам (снизу нет, чтобы батарея проходила)
    sleeve = (lower_loft(WALL + FIT).cut(lower_loft(WALL + FIT + 1.6))
              .intersect(zbox(X_TAIL - 1, X_TAIL + SLEEVE_L, -300, 300,
                              LOWER_Z + WALL + RAIL_H + 2, -1.0)))
    cap = cap.union(sleeve)
    # Бобышки под винты сбоку
    xs = X_TAIL + SLEEVE_L / 2
    yw = probe_half_width(lower_loft(WALL + FIT), xs, TAIL_SCREW_Z)
    for s in (-1, 1):
        start = cq.Vector(xs, s * yw, TAIL_SCREW_Z)
        inward = cq.Vector(0, -s, 0)
        cap = cap.union(cq.Workplane().add(cq.Solid.makeCylinder(4.5, 7.0, start, inward)))
        cap = cap.cut(cq.Workplane().add(cq.Solid.makeCylinder(INSERT_D / 2, INSERT_H, start, inward)))
    cap = cap.intersect(outer)
    # Зазор FIT до стенок нижнего корпуса
    wall_zone = lower_loft().cut(lower_loft(WALL + FIT)).intersect(zbox(X_TAIL, 500, -300, 300, -300, 0))
    return cap.cut(wall_zone).cut(arm_clearance())


# ---------------------------------------------------------------------------
# Луч, крышка луча, стойка мотора, опора
# ---------------------------------------------------------------------------
def make_arm():
    """Луч в своих координатах: x от корня, верх z = 0. Печатать верхом вниз."""
    arm = (cq.Workplane("XY").box(MOTOR_X, ARM_W, ARM_H, centered=(False, True, False))
           .translate((0, 0, -ARM_H)).edges("|X and >Z").fillet(ARM_TOP_FILLET))
    arm = arm.union(cyls([(MOTOR_X, 0)], 2 * ARM_TIP_R, -ARM_H, ARM_H))

    # Канал под ESC, снизу открыт
    ch = (cq.Workplane("XY").box(MOTOR_X - CHANNEL_X0, ARM_IN_W, ARM_H - WALL,
                                 centered=(False, True, False))
          .translate((CHANNEL_X0, 0, -ARM_H)))
    ch = ch.union(cyls([(MOTOR_X, 0)], 2 * CHANNEL_TIP_R, -ARM_H, ARM_H - WALL))
    arm = arm.cut(ch)
    # Фальц под крышку
    arm = arm.cut(zbox(CHANNEL_X0, COVER_X1, -(ARM_IN_W + 2.4) / 2, (ARM_IN_W + 2.4) / 2,
                       -ARM_H, -ARM_H + ARM_COVER_T))

    # Стойки: сверху вставки под болты палубы, снизу под крышку луча
    pillar_z0 = -ARM_H + ARM_COVER_T
    deck_pts = [(x, y) for x in DECK_BOLT_X for y in (-PILLAR_Y, PILLAR_Y)]
    tip_pts = [(COVER_TIP_X, y) for y in (-PILLAR_Y, PILLAR_Y)]
    arm = arm.union(cyls(deck_pts + tip_pts, PILLAR_D, pillar_z0, -pillar_z0))
    arm = arm.cut(cyls(deck_pts, INSERT_D, -INSERT_H, INSERT_H))
    arm = arm.cut(cyls(deck_pts + tip_pts, INSERT_D, pillar_z0, INSERT_H))

    arm = arm.cut(cq.Workplane("XY").center(ARM_WIRE_X, 0).rect(*ARM_WIRE_HOLE)
                  .extrude(-WALL - 0.1))

    # Под стойкой мотора: окно доступа к винтам мотора и 4 отверстия под винты стойки
    arm = arm.cut(cyls([(MOTOR_X, 0)], 2 * ARM_TOP_HOLE_R, -WALL - 0.1, WALL + 0.2))
    pod = polar_pts((MOTOR_X, 0), POD_SCREW_R, (45, 135, 225, 315))
    arm = arm.cut(cyls(pod, M3_CLEAR, -WALL - 0.1, WALL + 0.2))
    return arm


def make_arm_cover():
    """Крышка канала луча. Координаты луча, низ z = 0."""
    w = ARM_IN_W + 2.4 - 2 * FIT
    cover = zbox(CHANNEL_X0 + FIT, COVER_X1 - FIT, -w / 2, w / 2, 0, ARM_COVER_T)
    cover = cover.cut(cyls([(MOTOR_X, 0)], 2 * ARM_TIP_R + 2 * FIT, 0, ARM_COVER_T))
    pts = [(x, y) for x in (*DECK_BOLT_X, COVER_TIP_X) for y in (-PILLAR_Y, PILLAR_Y)]
    cover = cover.cut(cyls(pts, M3_CLEAR, 0, ARM_COVER_T))
    esc_x0 = DECK_BOLT_X[1] + PILLAR_D / 2 + 4
    esc_x1 = COVER_TIP_X - PILLAR_D / 2 - 4
    n = int((esc_x1 - esc_x0) // 8)
    slots = [(esc_x0 + 4 + i * 8, 0) for i in range(n)]
    cover = cover.cut(cq.Workplane("XY").pushPoints(slots)
                      .slot2D(ARM_IN_W - 10, 3.0, angle=90).extrude(ARM_COVER_T))
    # Кабель датчика потока (если он на этом луче)
    cover = cover.cut(cyls([(COVER_TIP_X - 12, 0)], 6.0, 0, ARM_COVER_T))
    return cover


def make_motor_pod(pod_h):
    """Стойка мотора. Координаты: низ z = 0 (верх луча), ось мотора в (0, 0).
    Печатать верхом вниз."""
    pod = cyls([(0, 0)], 2 * ARM_TIP_R, 0, pod_h)
    pod = pod.cut(cyls([(0, 0)], 2 * (ARM_TIP_R - POD_WALL), 0, pod_h - POD_TOP_T))
    ring = (cyls([(0, 0)], 2 * (ARM_TIP_R - POD_WALL), 0, POD_RING_T)
            .cut(cyls([(0, 0)], 2 * POD_RING_IN_R, 0, POD_RING_T)))
    pod = pod.union(ring)
    pod = pod.cut(cyls(polar_pts((0, 0), POD_SCREW_R, (45, 135, 225, 315)), INSERT_D, 0, INSERT_H))
    holes = [(-MOTOR_HOLES[0] / 2, 0), (MOTOR_HOLES[0] / 2, 0),
             (0, -MOTOR_HOLES[1] / 2), (0, MOTOR_HOLES[1] / 2)]
    pod = pod.cut(cyls(holes, M3_CLEAR, pod_h - POD_TOP_T, POD_TOP_T))
    pod = pod.cut(cyls([(0, 0)], MOTOR_SHAFT_D, pod_h - POD_TOP_T, POD_TOP_T))
    # Прорезь под провода мотора (в сторону корпуса)
    pod = pod.cut(zbox(-ARM_TIP_R - 1, -12.0, -4.0, 4.0, pod_h - POD_TOP_T - 6, pod_h))
    return pod


def make_leg():
    """Опора под мотором. Координаты: верх z = 0 (низ луча). Печатать фланцем вниз."""
    leg = cyls([(0, 0)], 2 * ARM_TIP_R, -LEG_FLANGE_T, LEG_FLANGE_T)
    leg = leg.union(cyls([(0, 0)], LEG_D, -LEG_H, LEG_H))
    leg = leg.union(cyls([(0, 0)], LEG_FOOT_D, -LEG_H, 3.0))
    leg = leg.cut(cyls([(0, 0)], LEG_D - 2 * LEG_WALL, -LEG_H, LEG_H))
    leg = leg.cut(cyls(polar_pts((0, 0), POD_SCREW_R, (45, 225)), M3_CLEAR, -LEG_FLANGE_T,
                       LEG_FLANGE_T))
    return leg


def make_flow_bracket():
    """Кронштейн MTF-02P под стойками у конца крышки луча. Верх z = 0."""
    il, iw = FLOW[0] + 2 * FIT, FLOW[1] + 2 * FIT
    h = FLOW_BRACKET_T + FLOW[2]
    span = 2 * PILLAR_Y + 8
    bracket = cq.Workplane("XY").box(il + 4, span, FLOW_BRACKET_T, centered=(True, True, False))
    bracket = bracket.union(cq.Workplane("XY").box(il + 4, iw + 4, h, centered=(True, True, False)))
    bracket = bracket.cut(cq.Workplane("XY").box(il, iw, FLOW[2], centered=(True, True, False))
                          .translate((0, 0, FLOW_BRACKET_T)))
    bracket = bracket.cut(cq.Workplane("XY").rect(*FLOW_WINDOW).extrude(FLOW_BRACKET_T))
    bracket = bracket.cut(cyls([(0, -PILLAR_Y), (0, PILLAR_Y)], M3_CLEAR, 0, FLOW_BRACKET_T))
    return bracket.mirror("XY")


# ---------------------------------------------------------------------------
# Проверки, экспорт, сборка
# ---------------------------------------------------------------------------
def pod_height(canopy):
    """Высота стоек, чтобы винты прошли над обтекателем с зазором."""
    discs = cyls(MOTOR_XY, PROP_D + 2 * PROP_CLEAR, 0, 300)
    common = canopy.intersect(discs)
    zmax = common.val().BoundingBox().zmax if common.val().Volume() > 0 else 0.0
    return max(POD_MIN_H, zmax + PROP_VCLEAR - MOTOR_H - PROP_HUB_H)


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


def build_assembly(parts, pod_h):
    asm = cq.Assembly(name="drone_frame")
    body = cq.Color(0.95, 0.45, 0.1)
    dark = cq.Color(0.2, 0.2, 0.2)
    light = cq.Color(0.9, 0.9, 0.9)
    asm.add(parts["deck"], name="deck", color=light)
    asm.add(parts["canopy"], name="canopy", color=body)
    asm.add(parts["lower_shell"], name="lower_shell", color=body)
    asm.add(parts["tail_cap"], name="tail_cap", color=body)
    for i, a in enumerate(ARM_ANGLES):
        asm.add(place_on_arm(parts["arm"], a), name=f"arm_{i}", color=body)
        asm.add(place_on_arm(parts["arm_cover"].translate((0, 0, -ARM_H)), a),
                name=f"arm_cover_{i}", color=body)
        asm.add(place_on_arm(parts["motor_pod"].translate((MOTOR_X, 0, 0)), a),
                name=f"motor_pod_{i}", color=light)
        asm.add(place_on_arm(parts["leg"].translate((MOTOR_X, 0, -ARM_H)), a),
                name=f"leg_{i}", color=dark)
    asm.add(place_on_arm(parts["flow_bracket"].translate((COVER_TIP_X, 0, -ARM_H)), ARM_ANGLES[2]),
            name="flow_bracket", color=dark)

    ghost = cq.Color(0.2, 0.6, 0.9, 0.6)
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
        "battery": ghost_box(BATTERY, ((BATT_FRONT + BATT_REAR) / 2, 0), LOWER_Z + WALL + FIT),
        "gimbal": ghost_box(GIMBAL, (GIMBAL_X, 0), -GIMBAL_SPACER - GIMBAL[2]),
    }
    prop_z = pod_h + MOTOR_H + PROP_HUB_H
    for i, a in enumerate(ARM_ANGLES):
        mx, my = MOTOR_XY[i]
        ghosts[f"motor_{i}"] = cyls([(mx, my)], MOTOR_D, pod_h, MOTOR_H)
        ghosts[f"prop_{i}"] = cyls([(mx, my)], PROP_D, prop_z, 2)
        ex, ey = arm_point(a, (DECK_BOLT_X[1] + COVER_TIP_X) / 2)
        ghosts[f"esc_{i}"] = (cq.Workplane("XY").box(*ESC, centered=(True, True, False))
                              .rotate((0, 0, 0), (0, 0, 1), a)
                              .translate((ex, ey, -ARM_H + ARM_COVER_T)))
    for name, shape in ghosts.items():
        asm.add(shape, name=f"ref_{name}", color=ghost)
    return asm


def make_parts():
    canopy = make_canopy()
    pod_h = pod_height(canopy)
    parts = {
        "deck": make_deck(),
        "canopy": canopy,
        "lower_shell": make_lower_shell(),
        "tail_cap": make_tail_cap(),
        "arm": make_arm(),
        "arm_cover": make_arm_cover(),
        "motor_pod": make_motor_pod(pod_h),
        "leg": make_leg(),
        "flow_bracket": make_flow_bracket(),
    }
    return parts, pod_h


def main():
    OUT_DIR.mkdir(exist_ok=True)
    parts, pod_h = make_parts()
    for name, shape in parts.items():
        check_fits_bed(name, shape)
        cq.exporters.export(shape, str(OUT_DIR / f"{name}.step"))
        cq.exporters.export(shape, str(OUT_DIR / f"{name}.stl"),
                            tolerance=0.05, angularTolerance=0.1)
    build_assembly(parts, pod_h).export(str(OUT_DIR / "assembly.step"))
    print(f"Длина корпуса {BODY_FRONT - BODY_REAR:.0f} мм, диагональ моторов {MOTOR_DIAG:.0f} мм")
    print(f"Стойки моторов {pod_h:.1f} мм, от земли до палубы {-GROUND_Z:.0f} мм")
    print(f"Файлы сохранены в {OUT_DIR}")


if __name__ == "__main__":
    main()
