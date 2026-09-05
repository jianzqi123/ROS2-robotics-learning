#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2D 地图切片测量脚本 —— 重算 README "The 2D map is a slice" 那张表。

用法:
    python3 ~/ros2_ws/src/my_robot_description/scripts/map_slice_check.py
    # 或指定其它地图
    python3 map_slice_check.py --map path/to/my_map.yaml

不需要 ROS 运行时, 不需要仿真, 纯离线读 pgm/yaml。

为什么要有这个脚本:
README 里那张表原来的数字是临时量的, 没有留下可复现的口径, 于是出现了一个
解释不了的矛盾 —— table1 的 4.1% 和"薄壳预测"几乎完全吻合, 但 box1 的 50.6%
是薄壳预测(19.0%)的 2.7 倍。同一个指标不可能对一个物体成立、对另一个失效,
所以要么口径不是"足迹内占据格比例", 要么占据带被加厚了。
这个脚本把两种可能同时量出来, 让它可证伪:

  - 占据率      = 足迹内被标为占据的格数 / 足迹总格数
  - 薄壳预测    = 足迹边界一圈的格数 / 足迹总格数   (2D 雷达只能看到实心物体的外轮廓)
  - 带厚度      = 占据格数 / 边界圈格数            (≈1 说明是一格薄壳, >1 说明被加厚)
  - 四边覆盖率  = 每条边上有多少比例被扫到          (区分"加厚"和"只看到几个面")

几何一律从 worlds/my_world.sdf 和 urdf/my_robot.urdf 解析, 不在本文件里
硬编码任何尺寸 —— 改了模型这里会自动跟着变, 不会像原来那样悄悄失同步。
"""

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
WORLD = os.path.join(PKG, 'worlds', 'my_world.sdf')
URDF = os.path.join(PKG, 'urdf', 'my_robot.urdf')
DEFAULT_MAP = os.path.join(PKG, 'maps', 'my_map.yaml')

# 只测这几个物体: 它们是当初按雷达垂直覆盖范围专门铺开的对照组。
# 墙不测 —— 墙是 1.5m 高的实心面, 任何高度的切片都完整命中, 没有信息量。
TARGETS = ['box1', 'box2', 'cylinder1', 'table1', 'shelf', 'platform',
           'overhead_beam', 'crates']

# 与 nav2_params.yaml 保持一致: 底盘 0.4x0.3 的角点 sqrt(0.20^2+0.15^2)
ROBOT_RADIUS = 0.25


# ---------------------------------------------------------------- SDF 解析

def _pose(elem):
    """取 <pose>x y z r p y</pose>, 缺省全 0。"""
    p = elem.find('pose')
    if p is None or not (p.text or '').strip():
        return [0.0] * 6
    v = [float(t) for t in p.text.split()]
    return (v + [0.0] * 6)[:6]


def _footprint_of_geometry(geom, cx, cy, cz):
    """返回 (xmin, xmax, ymin, ymax, zmin, zmax); 不支持的形状返回 None。"""
    box = geom.find('box')
    if box is not None:
        sx, sy, sz = [float(t) for t in box.find('size').text.split()]
        return (cx - sx / 2, cx + sx / 2,
                cy - sy / 2, cy + sy / 2,
                cz - sz / 2, cz + sz / 2)
    cyl = geom.find('cylinder')
    if cyl is not None:
        r = float(cyl.find('radius').text)
        h = float(cyl.find('length').text)
        return (cx - r, cx + r, cy - r, cy + r, cz - h / 2, cz + h / 2)
    return None


def parse_world(path):
    """
    解析每个模型的所有 collision, 返回:
        name -> {'parts': [(xmin,xmax,ymin,ymax,zmin,zmax), ...]}
    位姿按 模型 -> link -> collision 三级平移叠加。
    这个世界里所有 rpy 都是 0, 出现非零旋转就直接报错而不是默默算错。
    """
    root = ET.parse(path).getroot()
    world = root.find('world')
    out = {}
    for model in world.findall('model'):
        name = model.get('name')
        mp = _pose(model)
        if any(abs(a) > 1e-9 for a in mp[3:]):
            sys.exit(f'模型 {name} 带非零旋转, 本脚本只处理平移')
        parts = []
        for link in model.findall('link'):
            lp = _pose(link)
            for col in link.findall('collision'):
                cp = _pose(col)
                geom = col.find('geometry')
                if geom is None:
                    continue
                cx = mp[0] + lp[0] + cp[0]
                cy = mp[1] + lp[1] + cp[1]
                cz = mp[2] + lp[2] + cp[2]
                fp = _footprint_of_geometry(geom, cx, cy, cz)
                if fp:
                    parts.append(fp)
        if parts:
            out[name] = {'parts': parts}
    return out


# --------------------------------------------------------------- URDF 解析

def parse_urdf(path):
    """
    从 URDF 反推 /scan 的实际几何。这些数字全都影响切片高度,
    所以必须跟着 URDF 走, 不能写死。
    """
    root = ET.parse(path).getroot()

    def joint_origin(jname):
        for j in root.findall('joint'):
            if j.get('name') == jname:
                o = j.find('origin')
                return [float(t) for t in o.get('xyz').split()]
        sys.exit(f'URDF 里找不到 joint {jname}')

    def link_shape(lname, shape):
        for link in root.findall('link'):
            if link.get('name') == lname:
                g = link.find('visual/geometry/' + shape)
                if g is not None:
                    return g
        sys.exit(f'URDF 里找不到 link {lname} 的 {shape}')

    wheel_xyz = joint_origin('left_wheel_joint')
    wheel_r = float(link_shape('left_wheel', 'cylinder').get('radius'))
    caster_xyz = joint_origin('caster_front_joint')
    caster_r = float(link_shape('caster_front', 'sphere').get('radius'))
    lidar_xyz = joint_origin('lidar_joint')

    # 接地面: 轮子最低点 (base_link 坐标系下)
    wheel_contact_z = wheel_xyz[2] - wheel_r
    caster_low_z = caster_xyz[2] - caster_r
    clearance = caster_low_z - wheel_contact_z
    lever = caster_xyz[0]                      # 万向轮到轮轴的纵向距离
    pitch = math.atan2(clearance, lever)       # 静止俯仰(前倾), 弧度

    # 雷达垂直分布: N 条线均布在 [vmin, vmax], 步长 (vmax-vmin)/(N-1)。
    # ros_gz_bridge 压成 LaserScan 时取 index = N//2 那一行 —— 偶数线时
    # 没有任何一条落在 0°, 这就是 +1° 的来源。
    sensor = root.find(".//sensor[@type='gpu_lidar']")
    vert = sensor.find('lidar/scan/vertical')
    n = int(vert.find('samples').text)
    vmin = float(vert.find('min_angle').text)
    vmax = float(vert.find('max_angle').text)
    rng_max = float(sensor.find('lidar/range/max').text)
    step = (vmax - vmin) / (n - 1)
    beam_elev = vmin + step * (n // 2)

    # base_link 原点离地 = -接地面 z; 雷达在 base_link 下的 (x, z)
    base_h = -wheel_contact_z
    lx, lz = lidar_xyz[0], lidar_xyz[2]
    # 前倾 pitch 后雷达的世界高度
    lidar_h = base_h + (-lx * math.sin(pitch) + lz * math.cos(pitch))
    net_elev = beam_elev - pitch               # 正=朝上, 负=朝下

    return {
        'clearance': clearance, 'pitch': pitch, 'beam_elev': beam_elev,
        'net_elev': net_elev, 'lidar_h': lidar_h, 'range_max': rng_max,
        'n_beams': n, 'beam_index': n // 2,
    }


# ---------------------------------------------------------------- 地图读取

def read_pgm(path):
    """读 P5/P2 PGM, 返回 uint8 数组 (row 0 = 最上面一行 = 最大 y)。"""
    with open(path, 'rb') as f:
        data = f.read()
    # 逐个取 token, 跳过 # 注释
    toks, i = [], 0
    while len(toks) < 4:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while i < len(data) and data[i:i + 1] not in b'\r\n':
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        toks.append(data[i:j])
        i = j
    magic, w, h = toks[0], int(toks[1]), int(toks[2])
    i += 1                                     # 跳过 maxv (toks[3]) 后的单个空白
    if magic == b'P5':
        return np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)
    if magic == b'P2':
        vals = np.array(data[i:].split(), dtype=int)[:w * h]
        return vals.reshape(h, w).astype(np.uint8)
    sys.exit(f'不认识的 PGM 格式 {magic}')


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), meta['image'])
    pix = read_pgm(img)
    # ROS map_server trinary: p = (255-pixel)/255 (negate=0)
    p = (255.0 - pix.astype(float)) / 255.0
    if int(meta.get('negate', 0)):
        p = 1.0 - p
    occ = p > float(meta['occupied_thresh'])
    free = p < float(meta['free_thresh'])
    unknown = ~(occ | free)
    return meta, occ, free, unknown


def world_to_cell(wx, wy, meta, shape):
    """世界坐标 -> (row, col)。PGM 第 0 行是最大 y, 所以行号要翻转。"""
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    col = int(math.floor((wx - ox) / res))
    row_from_bottom = int(math.floor((wy - oy) / res))
    return shape[0] - 1 - row_from_bottom, col


# ------------------------------------------------------------------ 测量

def measure(name, parts, meta, occ, free, unknown):
    """对一个模型的足迹外接矩形做统计。"""
    xmin = min(p[0] for p in parts)
    xmax = max(p[1] for p in parts)
    ymin = min(p[2] for p in parts)
    ymax = max(p[3] for p in parts)
    zmin = min(p[4] for p in parts)
    zmax = max(p[5] for p in parts)

    shape = occ.shape
    r_hi, c_lo = world_to_cell(xmin, ymin, meta, shape)   # ymin -> 行号大
    r_lo, c_hi = world_to_cell(xmax, ymax, meta, shape)
    r_lo, r_hi = max(0, min(r_lo, r_hi)), min(shape[0] - 1, max(r_lo, r_hi))
    c_lo, c_hi = max(0, min(c_lo, c_hi)), min(shape[1] - 1, max(c_lo, c_hi))

    sub_occ = occ[r_lo:r_hi + 1, c_lo:c_hi + 1]
    sub_unk = unknown[r_lo:r_hi + 1, c_lo:c_hi + 1]
    h, w = sub_occ.shape
    total = h * w
    if total == 0:
        return None

    n_occ = int(sub_occ.sum())
    n_unk = int(sub_unk.sum())

    # 薄壳预测: 外圈一周的格数 (2D 雷达只能标出实心物体的外轮廓)
    ring = total if (h <= 2 or w <= 2) else total - (h - 2) * (w - 2)

    # 四边覆盖: 每条边最外一格里被标占据的比例
    sides = {
        'N': float(sub_occ[0, :].mean()),
        'S': float(sub_occ[-1, :].mean()),
        'W': float(sub_occ[:, 0].mean()),
        'E': float(sub_occ[:, -1].mean()),
    }

    return {
        'name': name, 'w': w, 'h': h, 'total': total,
        'n_occ': n_occ, 'pct': 100.0 * n_occ / total,
        'ring': ring, 'ring_pct': 100.0 * ring / total,
        'thickness': (n_occ / ring) if ring else float('nan'),
        'unk_pct': 100.0 * n_unk / total,
        'sides': sides, 'zmin': zmin, 'zmax': zmax,
        'dist': math.hypot((xmin + xmax) / 2, (ymin + ymax) / 2),
    }


def measure_walls(models, meta, occ):
    """
    量四面外墙: 完整度 + 尺度各向异性。

    完整度: 沿墙的长轴逐格看, 该处横跨墙厚的那一列里有没有占据格。
            缺口意味着那一段墙从没被扫到, 不是"扫到但标错"。
    各向异性: 用东西内表面间距 / 10.0 和南北内表面间距 / 8.0 两个比例尺相比。
            闭环没收敛的图会沿行进方向被拉长, 两个比例尺就不相等。
    """
    res = float(meta['resolution'])
    shape = occ.shape
    out, inner = {}, {}
    for name in ('wall_north', 'wall_south', 'wall_east', 'wall_west'):
        if name not in models:
            continue
        parts = models[name]['parts']
        xmin = min(p[0] for p in parts)
        xmax = max(p[1] for p in parts)
        ymin = min(p[2] for p in parts)
        ymax = max(p[3] for p in parts)
        r_hi, c_lo = world_to_cell(xmin, ymin, meta, shape)
        r_lo, c_hi = world_to_cell(xmax, ymax, meta, shape)
        r_lo, r_hi = max(0, min(r_lo, r_hi)), min(shape[0] - 1, max(r_lo, r_hi))
        c_lo, c_hi = max(0, min(c_lo, c_hi)), min(shape[1] - 1, max(c_lo, c_hi))
        sub = occ[r_lo:r_hi + 1, c_lo:c_hi + 1]
        if sub.size == 0:
            continue
        horizontal = sub.shape[1] >= sub.shape[0]
        # 长轴上每一格是否至少有一个占据格
        hit = sub.any(axis=0) if horizontal else sub.any(axis=1)
        out[name] = 100.0 * float(hit.mean())
        # 内表面: 朝房间中心那一侧最靠内的占据格
        rows, cols = np.nonzero(sub)
        if rows.size:
            if name == 'wall_north':
                inner[name] = (r_lo + rows.max())
            elif name == 'wall_south':
                inner[name] = (r_lo + rows.min())
            elif name == 'wall_east':
                inner[name] = (c_lo + cols.min())
            elif name == 'wall_west':
                inner[name] = (c_lo + cols.max())

    aniso = None
    if all(k in inner for k in ('wall_east', 'wall_west', 'wall_north', 'wall_south')):
        span_x = (inner['wall_east'] - inner['wall_west']) * res
        span_y = (inner['wall_south'] - inner['wall_north']) * res
        sx, sy = span_x / 10.0, span_y / 8.0
        aniso = {
            'span_x': span_x, 'span_y': span_y, 'sx': sx, 'sy': sy,
            'pct': 100.0 * abs(sx - sy) / ((sx + sy) / 2),
        }
    return out, aniso


def clearance_report(models, res=0.01):
    """
    用距离变换找出场地真正的通行瓶颈, 用来定 inflation_radius。

    为什么不能靠眼睛挑最窄的缝: 一开始就是这么定的 0.40, 结果挑错了 ——
    盯上了 wall_inner_a 西端到西墙的 1.0m, 漏了西墙到 platform 的 0.90m。
    代价是 DWB 在那条缝里找不出一条合法轨迹。把整个自由空间扫一遍
    就不会再漏。

    注意用世界几何而不是 my_map.pgm: 地图里实心物体只有一圈外轮廓,
    内部是"自由"的, 距离变换会算出穿过物体内部的假通道。
    """
    try:
        from scipy.ndimage import distance_transform_edt
    except ImportError:
        print('  (需要 scipy, 跳过通行余量分析)')
        return

    x0, x1, y0, y1 = -5.15, 5.15, -4.15, 4.15
    w, h = int((x1 - x0) / res), int((y1 - y0) / res)
    occ = np.zeros((h, w), bool)
    for name, d in models.items():
        if name == 'ground_plane':
            continue
        for (xa, xb, ya, yb, za, zb) in d['parts']:
            if zb < 0.19:          # 低于雷达线的东西, 2D 导航看不见
                continue
            i0, i1 = max(0, int((ya - y0) / res)), min(h, int((yb - y0) / res) + 1)
            j0, j1 = max(0, int((xa - x0) / res)), min(w, int((xb - x0) / res) + 1)
            occ[i0:i1, j0:j1] = True

    clear = distance_transform_edt(~occ) * res
    print()
    print('通行余量 (距离变换, 分辨率 %.0f cm):' % (res * 100))
    print(f'  自由空间最大余量 {clear.max():.3f} m')
    print(f'  {"半径":>7}{"可达面积 m^2":>14}')
    for r in (ROBOT_RADIUS, 0.30, 0.35, 0.40, 0.45, 0.50):
        print(f'  {r:>6.2f}{(clear >= r).sum() * res * res:>13.2f}')
    print('  零代价带宽 = 2*(中线余量 - inflation_radius);')
    print('  带宽 <=0 表示那条缝整段都是高代价, 规划器会绕路或贴着墙走。')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=DEFAULT_MAP)
    args = ap.parse_args()

    geom = parse_urdf(URDF)
    models = parse_world(WORLD)
    meta, occ, free, unknown = load_map(args.map)

    deg = math.degrees
    net = geom['net_elev']
    # 光束触地距离: 净仰角朝下才有解
    if net < 0:
        floor_r = geom['lidar_h'] / math.tan(-net)
    else:
        floor_r = float('inf')

    print('=' * 72)
    print('/scan 实际几何 (全部从 URDF 推出)')
    print('=' * 72)
    print(f"  万向轮离地间隙      {geom['clearance'] * 1000:.2f} mm")
    print(f"  静止前倾            {deg(geom['pitch']):.4f}°")
    print(f"  取用光束            第 {geom['beam_index']} 条 / 共 {geom['n_beams']} 条"
          f"  仰角 {deg(geom['beam_elev']):+.4f}°")
    print(f"  净仰角              {deg(net):+.4f}°  ({'朝下' if net < 0 else '朝上'})")
    print(f"  静止雷达高度        {geom['lidar_h']:.4f} m")
    print(f"  光束触地距离        {floor_r:.2f} m   (量程 {geom['range_max']:.0f} m)")
    verdict = '不可能出现地面回波' if floor_r > geom['range_max'] else '⚠ 量程内会打到地面'
    print(f"  判定                {verdict}")

    print()
    print('  注意: 上面是当前 URDF 的几何, 下面的地图却可能是改 URDF 之前建的。')
    print('        脚本无法从 pgm 反推它是在什么几何下建的 —— 引用这些数字前')
    print('        先确认地图与 URDF 同代, 否则两段对不上。')

    print()
    print('=' * 72)
    print(f"地图: {args.map}")
    print(f"  {occ.shape[1]} x {occ.shape[0]} 格 @ {meta['resolution']} m"
          f"  origin {meta['origin'][:2]}")
    print('=' * 72)

    walls, aniso = measure_walls(models, meta, occ)
    if walls:
        print('外墙完整度 (长轴上有占据格的比例):')
        for k, v in walls.items():
            print(f"  {k:<14}{v:6.1f} %")
    if aniso:
        print(f"  东西内表面间距  {aniso['span_x']:.3f} m  (真值 10.000)"
              f"  比例尺 {aniso['sx']:.4f}")
        print(f"  南北内表面间距  {aniso['span_y']:.3f} m  (真值  8.000)"
              f"  比例尺 {aniso['sy']:.4f}")
        print(f"  尺度各向异性    {aniso['pct']:.2f} %"
              f"   ({'闭环已收敛' if aniso['pct'] < 1.0 else '⚠ 疑似闭环未收敛'})")
        print()

    rows = []
    for name in TARGETS:
        if name not in models:
            print(f'  (世界文件里没有 {name}, 跳过)')
            continue
        m = measure(name, models[name]['parts'], meta, occ, free, unknown)
        if m:
            rows.append(m)

    hdr = (f"{'物体':<14}{'足迹格':>9}{'高度 m':>13}{'占据率':>9}"
           f"{'薄壳预测':>10}{'带厚度':>8}{'未知':>8}  四边覆盖 N/S/W/E")
    print(hdr)
    print('-' * len(hdr.encode('gbk', 'replace')))
    for m in rows:
        s = m['sides']
        print(f"{m['name']:<14}{m['w']}x{m['h']:<6}"
              f"{m['zmin']:>6.2f}-{m['zmax']:<6.2f}"
              f"{m['pct']:>8.1f}%{m['ring_pct']:>9.1f}%"
              f"{m['thickness']:>8.2f}{m['unk_pct']:>7.1f}%"
              f"   {s['N']:.2f}/{s['S']:.2f}/{s['W']:.2f}/{s['E']:.2f}")

    clearance_report(models)

    print()
    print('判读:')
    print('  带厚度 ≈1.0  -> 就是一格薄壳, 占据率应当 ≈ 薄壳预测')
    print('  带厚度 >1.5  -> 占据带被位姿不确定性加厚, 占据率会高于薄壳预测')
    print('  某边覆盖 ≈0  -> 那一面从没被扫到, 占据率会低于薄壳预测')
    print('  两者同时出现时, 占据率高低取决于哪一个占主导 —— 这正是原来')
    print('  box1(50.6%) 和 platform(11.8%) 无法用同一个解释说通的地方。')

    print()
    print('可直接粘进 README 的表格:')
    print()
    print('| Object | Height | Footprint | Occupied | Thin-shell | Band | Sides seen |')
    print('|---|---|---|---|---|---|---|')
    for m in rows:
        s = m['sides']
        seen = sum(1 for v in s.values() if v > 0.15)
        print(f"| `{m['name']}` | {m['zmin']:.2f}–{m['zmax']:.2f} m | "
              f"{m['w']}×{m['h']} | {m['pct']:.1f} % | {m['ring_pct']:.1f} % | "
              f"{m['thickness']:.2f}× | {seen}/4 |")


if __name__ == '__main__':
    main()
