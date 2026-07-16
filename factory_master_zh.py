"""
factory_master.py — 一鍵式建置 — 在 Isaac Sim 的 Script Editor 中執行。

依序整合了目前為止建立的所有內容：

  STEP 1  工廠場景配置        (build_factory_layout.py)
  STEP 2  物理設定            (add_factory_physics.py)
  STEP 3  半頭生產            (half_production_spawner.py，僅函式部分)
  STEP 4  狀態機面板          (state_machine_ui.py v2，輸送帶為真實功能)

因為現在所有東西都在同一個檔案裡，面板上的生成按鈕已經接上真正的生成器：
  產線列的「Spawn one」          -> produce_half("L"/"R")
  主控制「Produce Part」         -> produce_pair()（左右兩線各生一個半頭）
  除錯區「Spawn pair」           -> produce_pair()
  產線列的「Belt: ON/OFF」       -> 真正的進料輸送帶（surface velocity）
  出料區「Start/Stop conveyor」  -> 真正的出料輸送帶
其餘功能目前仍是待接線的空按鈕（機械手臂、吸附組裝、刪除）。

可安全重複執行：場景群組會被重建、物理設定重複套用無害，
面板／輸送帶在重新執行前會先清理乾淨。

執行後：按下 PLAY，再按「Produce Part」——兩條輸送帶上各會生成一個
半頭；輸送帶開啟後，半頭會被送進收集池中。
"""

import time
import math
import re
import asyncio
import omni.usd
import omni.kit.app
import omni.kit.commands
import omni.ui as ui
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf, Sdf

stage = omni.usd.get_context().get_stage()

# =================================================================
# CONFIG — 所有路徑與場景常數集中於此
# =================================================================
KUKA_USD_PATH = "C:/Users/User/Desktop/blender_to_usd/KukaArm1.usd"
ARM_SCALE     = 0.01   # 參考層級縮放：0.01 代表「以公分為單位製作的模型 -> 公尺場景」。
                       # 調整後重新執行：KR 10 站立高度應約 1.1-1.2 公尺，
                       # 與 0.8 公尺高的工作台相比大約到腰部高度。
HALF_PATHS = {
    "L": "C:/Users/User/Desktop/blender_to_usd/monkey_half_L.usd",
    "R": "C:/Users/User/Desktop/blender_to_usd/monkey_half_R.usd",
}

TABLE_POS      = (0.0, 0.0)
TABLE_SIZE     = (0.75, 0.75, 0.8)  # 由 (0.6, 0.6, 0.8) 放大
TABLE_RAIL_HEIGHT = 0.10  # 護欄高度，高於桌面（公尺）——
                          # 低到讓機械手臂垂直下降時不會卡到，
                          # 又高到能擋住被撞到／滑動的半頭掉出桌外
TABLE_RAIL_THICK  = 0.03
LINE_Y         = 1.1     # 生產線／收集池位於 y = +/-1.1（未變動）

# 機械手臂基座 (X, Y) — 兩臂「並非」鏡射對稱。之所以分別獨立調校，
# 是因為 KUKA 模型本身的原點並未對齊其實際站立的腳底位置，
# 因此單一鏡射公式無法讓兩隻手臂在各自的池子／工作台前站到相同的相對位置。
# 修改這兩個數字後重新執行即可移動手臂——絕對不要在畫面中用滑鼠拖曳。
ARM_POSITIONS = {
    "L": (1.07, 1.01),
    "R": (1.07, -1.16),
}

# 機械手臂動作參數 (STEP 5)
SEQ_FRAMES     = 60     # 單一關節動作所需影格數（一次只動一個關節）
A1_FRAMES      = 100    # 基座旋轉屬於較大幅度的擺動，用更長時間
HOVER_DZ       = 0.18   # 抓取點上方的懸停高度（公尺）
PLACE_DZ       = 0.10   # 放置標記點上方的釋放高度（公尺）
SAFE_TRANSIT_Z = 1.35   # 執行 A1 大幅擺動時所用的安全過渡世界 Z 高度——
                        # 淨空工作台桌面（0.8 公尺）與收集池護欄
                        # （頂端約 0.85 公尺），留有充足餘裕
GRAB_TOL       = 0.15   # 吸附容許誤差：吸嘴到零件的距離（公尺）
POOL_RADIUS    = 0.45   # 在 PickZone 周圍搜尋零件的半徑（公尺）
LOCATE_TIMEOUT_F = 300  # 約 5 秒：LOCATING 階段等待／重試零件靜止
                        # 的時間上限（原本只檢查一次——兩臂同時開始時，
                        # 常有一邊的零件比另一邊晚一點才靜止）

# 靜止判定門檻：零件在連續幾個影格之間的移動量必須小於此值，
# 且要連續維持這麼多影格，才允許任何機械手臂動作
# （尋找／接近／夾取）鎖定該零件。這裡量測的是「純粹的位置位移」，
# 而不是 RigidBodyAPI 的 velocity 屬性——那個屬性被證實不可靠
# （對著肉眼看起來完全靜止的零件，仍持續回報「還在移動」長達 20 秒以上，
# 很可能是碰撞分解網格產生的接觸抖動，在 PhysX 自身的速度回讀中
# 始終無法真正歸零）。位置位移不會說謊，能忠實反映物體是否真的在動。
SETTLE_MOVE_TOL = 0.0015   # 每影格允許的位移量（公尺）（持續換算約 9 cm/s）
SETTLE_FRAMES   = 15

# 收集池（每條生產線各一個）——零件掉落後、等待被夾取的地方
CPOOL_CENTER_X = -0.75   # 收集池中心 X 座標（兩條線共用同一個 X）
CPOOL_HALF_W   = 0.30    # 內部半寬（內部總寬 = 此值 x 2）
CPOOL_HALF_L   = 0.30    # 內部半長
CPOOL_WALL_H   = 0.25    # 池壁高度（相對池底）
CPOOL_WALL_T   = 0.05    # 池壁厚度
PICK_Z         = 0.15    # 機械手臂搜尋零件時瞄準的參考世界 Z 高度

# 進料輸送帶：把剛生成的半頭從生產機台送到對應的收集池
BELT_X_MIN     = -2.6    # 輸送帶起點 X（靠近生產機台）
BELT_X_MAX     = -1.12   # 輸送帶終點 X（掉入收集池處）
BELT_WIDTH     = 0.35
BELT_TOP       = 0.60    # 輸送帶表面高度（世界 Z）
BELT_THICK     = 0.08

# 生產機台——純視覺用的佔位方塊，沒有真正的「生產」邏輯
MACHINE_POS_X  = -2.9
MACHINE_SIZE   = (0.4, 0.5, 1.2)


PLACE_Y_OFF    = 0.20   # 每個半頭的放置點，相對 TABLE_POS[1]
                        # （工作台中心）的偏移量——由 0.16 加大，
                        # 讓每個目標點都穩穩落在自己那一側（工作台
                        # 半寬為 0.3 公尺），而不是太靠近中線
PLACE_Z        = 0.88

OUTFEED_X_MIN  = TABLE_POS[0] + TABLE_SIZE[0] / 2  # 精確貼齊工作台
                                                    # +X 邊緣——中間
                                                    # 沒有縫隙，直接由
                                                    # 工作台幾何算出
OUTFEED_X_MAX  = 2.40
OUTFEED_WIDTH  = TABLE_SIZE[1]   # 寬度與工作台一致（原本是 BELT_WIDTH，
                                 # 0.35 公尺），讓組裝完成的頭部能直接
                                 # 平順滑出，不會因變窄而卡住
OUTFEED_TOP    = 0.78
OUTFEED_THICK  = 0.08

# 刪除池——組裝完成頭部的最終目的地，抵達後即從場景中刪除
DPOOL_CENTER_X = 2.85
DPOOL_HALF_W   = 0.35
DPOOL_HALF_L   = 0.35
DPOOL_WALL_H   = 0.30
DPOOL_WALL_T   = 0.05

# 所有池子（收集池與刪除池）共用的池底／池壁顯示顏色
POOL_FLOOR_COLOR = (0.4, 0.4, 0.6)
POOL_WALL_COLOR  = (0.6, 0.6, 0.8)

# 3 種物理材質＋零件材質在場景中的存放路徑
# （分別由 add_physics() / ensure_part_material() 建立一次）
MAT_ROOT   = "/World/PhysicsMaterials"
BELT_MAT   = MAT_ROOT + "/BeltMaterial"
POOL_MAT   = MAT_ROOT + "/PoolMaterial"
TABLE_MAT  = MAT_ROOT + "/TableMaterial"
PART_MAT   = MAT_ROOT + "/PartMaterial"

# 每個生成的半頭存放位置，以及生成時的相關參數
PARTS_ROOT   = "/World/Parts"
SPAWN_MARGIN = 0.15   # 生成點與輸送帶機台端的內縮距離
SPAWN_DROP   = 0.25   # 生成點高於輸送帶表面的高度
SLIDE_SPEED  = 0.3    # 生成當下給予的小幅 +X 初速（輸送帶開啟後
                      # 才是真正負責搬運的機制）
PART_MASS    = 0.5    # 每個半頭的質量（公斤）

# =================================================================
# STEP 1 — 工廠場景配置
# =================================================================
_created = []

def make_xform(path, translate=(0, 0, 0)):
    """建立一個單純的群組／父層 prim，位於指定世界座標。每一個功能
    單元（LineL、ArmL_Group、AssemblyStation……）都是這種 Xform——
    在畫面中拖曳它，底下所有東西都會跟著移動。"""
    prim = stage.DefinePrim(Sdf.Path(path), "Xform")
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*translate))
    return prim

def make_box(path, center, size, color, opacity=1.0):
    """建立一個 Cube prim，並以縮放／位移做出軸對齊方塊。
    center/size 使用的是世界座標單位，而非 Cube 本身的單位立方縮放。"""
    prim = stage.DefinePrim(Sdf.Path(path), "Cube")
    cube = UsdGeom.Cube(prim)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    g = UsdGeom.Gprim(prim)
    g.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if opacity < 1.0:
        g.CreateDisplayOpacityAttr([opacity])
    _created.append((path, center))
    return prim

def make_marker(path, pos, color=(1.0, 0.2, 0.2), radius=0.03):
    """小型半透明球體，作為「非物理」的位置標記（PickZone、
    PlaceTarget）。其他腳本會在執行時讀取它的世界座標，而不是把
    座標寫死在程式碼裡，所以拖曳父層群組時目標點也會跟著移動。"""
    prim = stage.DefinePrim(Sdf.Path(path), "Sphere")
    UsdGeom.Sphere(prim).CreateRadiusAttr(radius)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    g = UsdGeom.Gprim(prim)
    g.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    g.CreateDisplayOpacityAttr([0.6])
    _created.append((path, pos))
    return prim

def make_pool(root, center_x, center_y, half_w, half_l, wall_h, wall_t):
    """一個池底＋四面池壁組成、頂部開放的圍欄式水池。收集池
    （每條生產線一個）與刪除池共用這個函式——形狀相同，
    只是大小／位置不同。"""
    inner_w = half_w * 2
    inner_l = half_l * 2
    outer_w = inner_w + wall_t * 2
    wall_z  = wall_h / 2
    cx, cy  = center_x, center_y
    make_xform(root)
    make_box(f"{root}/Floor", (cx, cy, 0.005), (inner_w, inner_l, 0.01), POOL_FLOOR_COLOR)
    make_box(f"{root}/Wall_Front", (cx, cy + half_l + wall_t / 2, wall_z),
             (outer_w, wall_t, wall_h), POOL_WALL_COLOR)
    make_box(f"{root}/Wall_Back", (cx, cy - (half_l + wall_t / 2), wall_z),
             (outer_w, wall_t, wall_h), POOL_WALL_COLOR)
    make_box(f"{root}/Wall_Left", (cx - (half_w + wall_t / 2), cy, wall_z),
             (wall_t, inner_l, wall_h), POOL_WALL_COLOR)
    make_box(f"{root}/Wall_Right", (cx + (half_w + wall_t / 2), cy, wall_z),
             (wall_t, inner_l, wall_h), POOL_WALL_COLOR)

def clean(path):
    """若 prim 已存在則刪除。在重建任何群組前都會先呼叫，確保
    重新執行腳本不會留下重複／過期的幾何物件。"""
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)

def disable_inherited_collision(root_prim):
    """明確關閉 root_prim 底下每一個 Mesh 的碰撞。機械手臂本身的
    碰撞設定原本刻意留到之後的階段再處理——但 KUKA_USD_PATH 這個
    模型檔，很可能本身就已經帶有原始檔案裡預先設定好的碰撞網格，
    與我們自己加上去的東西無關。這會讓擺動中的前臂真的去物理性地
    撞飛零件（實際觀察到的現象正是如此：某個半頭在被以毫米級精度
    放置後，卻被回報「靜止」在距離原位 0.89 公尺遠的地方——顯然是
    放下之後被什麼東西撞到，而不是放置本身不準）。在真正處理手臂
    碰撞之前，這個函式會直接讓來源檔案裡不論寫了什麼碰撞設定都失效。"""
    for p in Usd.PrimRange(root_prim):
        if p.IsA(UsdGeom.Mesh):
            p.CreateAttribute("physics:collisionEnabled",
                              Sdf.ValueTypeNames.Bool).Set(False)

def build_layout():
    """建立／重建整個靜態場景：地板、兩條生產線（機台＋輸送帶＋
    收集池＋標記點）、兩支 KUKA 機械手臂的參照、組裝工作台＋護欄、
    出料輸送帶＋護欄、刪除池。這裡只處理幾何外觀——真正讓它們
    具備物理行為的是底下的 add_physics()。可安全重複執行：
    每個群組在重建前都會先被 clean() 清除。"""
    if not stage.GetPrimAtPath("/World/GroundPlane").IsValid():
        make_box("/World/GroundPlane", (0, 0, -0.05), (8.0, 8.0, 0.1),
                 (0.35, 0.35, 0.38))

    belt_len = BELT_X_MAX - BELT_X_MIN
    belt_cx  = (BELT_X_MAX + BELT_X_MIN) / 2.0

    for pid, sign, tint in (("L", +1.0, (0.55, 0.45, 0.85)),
                            ("R", -1.0, (0.45, 0.55, 0.85))):
        grp = f"/World/Line{pid}"
        clean(grp)
        make_xform(grp)
        y = sign * LINE_Y
        make_box(f"{grp}/Machine", (MACHINE_POS_X, y, MACHINE_SIZE[2] / 2),
                 MACHINE_SIZE, tint)
        make_box(f"{grp}/Belt", (belt_cx, y, BELT_TOP - BELT_THICK / 2),
                 (belt_len, BELT_WIDTH, BELT_THICK), (0.25, 0.25, 0.25))
        # 進料輸送帶兩側的護欄——與工作台護欄同樣的樣式／高度——
        # 避免零件在從機台送往收集池的途中從側邊滑落
        belt_rail_z = BELT_TOP + TABLE_RAIL_HEIGHT / 2
        make_box(f"{grp}/Belt_Rail_Front",
                 (belt_cx, y + BELT_WIDTH / 2 + TABLE_RAIL_THICK / 2, belt_rail_z),
                 (belt_len, TABLE_RAIL_THICK, TABLE_RAIL_HEIGHT), (0.55, 0.45, 0.25))
        make_box(f"{grp}/Belt_Rail_Back",
                 (belt_cx, y - BELT_WIDTH / 2 - TABLE_RAIL_THICK / 2, belt_rail_z),
                 (belt_len, TABLE_RAIL_THICK, TABLE_RAIL_HEIGHT), (0.55, 0.45, 0.25))
        make_pool(f"{grp}/CollectionPool", CPOOL_CENTER_X, y,
                  CPOOL_HALF_W, CPOOL_HALF_L, CPOOL_WALL_H, CPOOL_WALL_T)
        make_marker(f"{grp}/PickZone", (CPOOL_CENTER_X, y, PICK_Z), (1.0, 0.4, 0.1))

    # KUKA 機械手臂：以參照方式引入模型，並在參照層級套用縮放。
    # 縮放放在 KukaArm 這個「子」prim 上（等比例縮放 ARM_SCALE）；
    # 群組本身的 Xform 不帶縮放，這樣基座的平移量、以及之後可能加上的
    # 子 prim（吸嘴墊片、標記點等）都不會被意外影響。
    # 位置來自 ARM_POSITIONS（每隻手臂各自獨立設定——參見前面
    # config 區塊的說明），而不是用鏡射公式算出來的。
    for pid in ("L", "R"):
        grp = f"/World/Arm{pid}_Group"
        clean(grp)
        ax, ay = ARM_POSITIONS[pid]
        make_xform(grp, translate=(ax, ay, 0.0))
        arm = stage.DefinePrim(Sdf.Path(f"{grp}/KukaArm"), "Xform")
        arm.GetReferences().AddReference(KUKA_USD_PATH)
        disable_inherited_collision(arm)
        disable_inherited_collision(arm)
        axf = UsdGeom.Xformable(arm)
        axf.ClearXformOpOrder()
        axf.AddScaleOp().Set(Gf.Vec3f(ARM_SCALE, ARM_SCALE, ARM_SCALE))
        _created.append((f"{grp}/KukaArm (scale {ARM_SCALE})", (ax, ay, 0.0)))

    clean("/World/AssemblyStation")
    make_xform("/World/AssemblyStation")
    make_box("/World/AssemblyStation/Table",
             (TABLE_POS[0], TABLE_POS[1], TABLE_SIZE[2] / 2),
             (TABLE_SIZE[0], TABLE_SIZE[1], TABLE_SIZE[2]), (0.75, 0.6, 0.3))

    # 護欄：在 3 個方向（Y+、Y-、X- 也就是機台那一側）設置低矮的
    # 圍欄，防止被撞到或滑動的半頭掉出桌面。機械手臂仍能順利伸進來，
    # 因為它們是「從上方」下降的，跟收集池的情況一樣。
    # +X 那一側（朝向 OutFeed 出料方向）刻意保持開放，
    # 讓組裝完成的頭部之後能順利被輸送帶送出——那一側不設護欄。
    rail_h = TABLE_RAIL_HEIGHT
    rail_t = TABLE_RAIL_THICK
    tx, ty = TABLE_POS
    hw, hl = TABLE_SIZE[0] / 2, TABLE_SIZE[1] / 2
    top = TABLE_SIZE[2]
    rail_color = (0.55, 0.45, 0.25)
    make_box("/World/AssemblyStation/Rail_Front",       # +Y 側
             (tx, ty + hl + rail_t / 2, top + rail_h / 2),
             (TABLE_SIZE[0] + rail_t * 2, rail_t, rail_h), rail_color)
    make_box("/World/AssemblyStation/Rail_Back",        # -Y 側
             (tx, ty - hl - rail_t / 2, top + rail_h / 2),
             (TABLE_SIZE[0] + rail_t * 2, rail_t, rail_h), rail_color)
    make_box("/World/AssemblyStation/Rail_MachineSide",  # -X 側
             (tx - hw - rail_t / 2, ty, top + rail_h / 2),
             (rail_t, TABLE_SIZE[1], rail_h), rail_color)
    # （+X 側刻意不設護欄——那裡是 OutFeed 的出口）

    make_marker("/World/AssemblyStation/PlaceTarget_L",
                (TABLE_POS[0], TABLE_POS[1] + PLACE_Y_OFF, PLACE_Z), (0.2, 0.9, 0.3))
    make_marker("/World/AssemblyStation/PlaceTarget_R",
                (TABLE_POS[0], TABLE_POS[1] - PLACE_Y_OFF, PLACE_Z), (0.2, 0.9, 0.3))

    clean("/World/OutFeed")
    make_xform("/World/OutFeed")
    of_len = OUTFEED_X_MAX - OUTFEED_X_MIN
    of_cx  = (OUTFEED_X_MAX + OUTFEED_X_MIN) / 2.0
    make_box("/World/OutFeed/Belt", (of_cx, 0.0, OUTFEED_TOP - OUTFEED_THICK / 2),
             (of_len, OUTFEED_WIDTH, OUTFEED_THICK), (0.25, 0.25, 0.25))
    # 沿著輸送帶整個長度設置兩側護欄，樣式／高度與工作台護欄相同，
    # 讓組裝完成的頭部不會從側邊滑落
    of_rail_z = OUTFEED_TOP + TABLE_RAIL_HEIGHT / 2
    make_box("/World/OutFeed/Rail_Front",
             (of_cx, OUTFEED_WIDTH / 2 + TABLE_RAIL_THICK / 2, of_rail_z),
             (of_len, TABLE_RAIL_THICK, TABLE_RAIL_HEIGHT), (0.55, 0.45, 0.25))
    make_box("/World/OutFeed/Rail_Back",
             (of_cx, -(OUTFEED_WIDTH / 2 + TABLE_RAIL_THICK / 2), of_rail_z),
             (of_len, TABLE_RAIL_THICK, TABLE_RAIL_HEIGHT), (0.55, 0.45, 0.25))
    make_pool("/World/OutFeed/DeletionPool", DPOOL_CENTER_X, 0.0,
              DPOOL_HALF_W, DPOOL_HALF_L, DPOOL_WALL_H, DPOOL_WALL_T)
    make_box("/World/OutFeed/DeletionPool/DeletionTrigger",
             (DPOOL_CENTER_X, 0.0, 0.20),
             (DPOOL_HALF_W * 1.8, DPOOL_HALF_L * 1.8, 0.35),
             (0.9, 0.15, 0.15), opacity=0.30)
    print(f"[1/4] layout built ({len(_created)} prims)")

# =================================================================
# STEP 2 — 物理設定
# =================================================================
def make_physics_material(path, restitution, dyn_fric, stat_fric,
                          rest_combine, fric_combine="average"):
    """定義一個 PhysX 材質（彈性＋摩擦力）。rest_combine 決定兩個
    互相接觸的材質，其彈性係數要如何合併——設為 "min" 代表兩者中
    「比較不彈」的那一個永遠勝出，這樣即使碰到的另一個表面很有
    彈性，零件也不會跟著彈跳。"""
    mat = UsdShade.Material.Define(stage, path)
    prim = mat.GetPrim()
    pm = UsdPhysics.MaterialAPI.Apply(prim)
    pm.CreateRestitutionAttr(restitution)
    pm.CreateDynamicFrictionAttr(dyn_fric)
    pm.CreateStaticFrictionAttr(stat_fric)
    try:
        from pxr import PhysxSchema
        px = PhysxSchema.PhysxMaterialAPI.Apply(prim)
        px.CreateRestitutionCombineModeAttr(rest_combine)
        px.CreateFrictionCombineModeAttr(fric_combine)
    except Exception:
        prim.CreateAttribute("physxMaterial:restitutionCombineMode",
                             Sdf.ValueTypeNames.Token).Set(rest_combine)
        prim.CreateAttribute("physxMaterial:frictionCombineMode",
                             Sdf.ValueTypeNames.Token).Set(fric_combine)
    return mat

def bind_physics_material(prim, mat_path):
    """把一個已經定義好的材質（以路徑指定）綁定到某個 prim 上。"""
    mat = UsdShade.Material(stage.GetPrimAtPath(mat_path))
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        mat, UsdShade.Tokens.strongerThanDescendants, "physics")

def add_static_collider(path, mat_path):
    """只加碰撞、永遠不會移動。用於牆壁、工作台、護欄、地板——
    任何「固定不動、又不是輸送帶也不是零件」的實體。"""
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return False
    UsdPhysics.CollisionAPI.Apply(prim)
    bind_physics_material(prim, mat_path)
    return True

def add_kinematic_body(path, mat_path):
    """碰撞 ＋ 運動學（kinematic）剛體。這是讓一個 prim 之後能夠
    被設定 PhysX 表面速度（也就是變成輸送帶）的前提條件——
    4 條輸送帶（進料 x2、出料 x1、工作台）都是用這個函式設定的。"""
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return False
    UsdPhysics.CollisionAPI.Apply(prim)
    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    rb.CreateKinematicEnabledAttr(True)
    bind_physics_material(prim, mat_path)
    return True

def add_physics():
    """讓 build_layout() 建立的所有東西具備物理行為：建立
    PhysicsScene、3 種材質（belt／pool／table——各自針對不同用途
    調整：輸送帶幾乎沒有彈性讓零件用「滑」的而不是彈跳；池子摩擦力
    較高讓翻滾的零件能快速靜止；工作台摩擦力最高，讓半頭在等待
    吸附組裝時能完全靜止不動），接著依路徑把碰撞套用到每個相關的
    prim 上。"""
    if not any(p.IsA(UsdPhysics.Scene) for p in stage.Traverse()):
        UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")

    stage.DefinePrim(Sdf.Path(MAT_ROOT), "Scope")
    make_physics_material(BELT_MAT,  0.05, 0.5, 0.6, "min")
    make_physics_material(POOL_MAT,  0.10, 0.8, 0.9, "multiply")
    make_physics_material(TABLE_MAT, 0.02, 0.9, 1.0, "min")

    add_static_collider("/World/GroundPlane", POOL_MAT)
    for pid in ("L", "R"):
        grp = f"/World/Line{pid}"
        add_static_collider(f"{grp}/Machine", BELT_MAT)
        add_kinematic_body(f"{grp}/Belt", BELT_MAT)
        for rail in ("Belt_Rail_Front", "Belt_Rail_Back"):
            add_static_collider(f"{grp}/{rail}", BELT_MAT)
        for wall in ("Floor", "Wall_Front", "Wall_Back", "Wall_Left", "Wall_Right"):
            add_static_collider(f"{grp}/CollectionPool/{wall}", POOL_MAT)
    add_kinematic_body("/World/AssemblyStation/Table", TABLE_MAT)  # 現在
                        # 已具備輸送帶能力（表面速度），不再只是靜態物件
    for rail in ("Rail_Front", "Rail_Back", "Rail_MachineSide"):
        add_static_collider(f"/World/AssemblyStation/{rail}", TABLE_MAT)
    add_kinematic_body("/World/OutFeed/Belt", BELT_MAT)
    for rail in ("Rail_Front", "Rail_Back"):
        add_static_collider(f"/World/OutFeed/{rail}", BELT_MAT)
    for wall in ("Floor", "Wall_Front", "Wall_Back", "Wall_Left", "Wall_Right"):
        add_static_collider(f"/World/OutFeed/DeletionPool/{wall}", POOL_MAT)
    print("[2/4] physics applied (belts = kinematic, conveyor-ready)")

# =================================================================
# STEP 3 — 半頭生產（純函式；UI 由面板負責）
# =================================================================
BELT_PRIMS = {"L": "/World/LineL/Belt", "R": "/World/LineR/Belt"}
counters = {"L": 0, "R": 0}

def ensure_part_material():
    """只建立一次 PART_MAT（重複呼叫無害）——所有生成的半頭共用
    這個材質，所以在這裡改一次彈性／摩擦力，會影響全部的半頭，
    包括已經吸附組裝完成的頭部。"""
    if stage.GetPrimAtPath(PART_MAT).IsValid():
        return
    # 彈性係數由 0.05 降到 0.01——撞擊時的彈跳更少。
    # combine="min" 已確保「兩個接觸材質中較不彈的那個」勝出，
    # 因此這個改動會套用到零件接觸的每一個表面
    # （輸送帶、池子、工作台），不需要再改其他地方。
    make_physics_material(PART_MAT, 0.01, 0.6, 0.7, "min")

def get_spawn_point(pid):
    """讀取該生產線輸送帶 prim「實際」的世界座標與縮放，回傳一個
    位於機台端、剛好在輸送帶表面上方的生成點。跟著輸送帶走
    （而不是寫死座標）代表拖曳 /World/LineL 或 /World/LineR 之後，
    生成位置依然正確。"""
    prim = stage.GetPrimAtPath(BELT_PRIMS[pid])
    if not prim.IsValid():
        return None
    xf = UsdGeom.Xformable(prim)
    m = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos = m.ExtractTranslation()
    sx = sz = None
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            s = op.Get()
            sx, sz = s[0], s[2]
    if sx is None:
        sx, sz = 1.48, 0.08   # 若輸送帶還沒有 scale op 時的備用值
    return (pos[0] - sx / 2.0 + SPAWN_MARGIN, pos[1],
            pos[2] + sz / 2.0 + SPAWN_DROP)

def produce_half(pid, slide_speed=None):
    """生成一個猴頭半頭（L 或 R），以參照方式引用對應的 .usd 檔，
    標記 piece_id／head_id（之後機械手臂與吸附組裝邏輯會讀取這兩個
    屬性），給予小幅 +X 初速，並套用 convexDecomposition 碰撞——
    這一點是必要的：若用單純的凸包（convex hull），會把切割面與
    卡榫／插槽整個封起來，兩個半頭就永遠無法真正密合。"""
    if pid not in HALF_PATHS:
        return None
    spawn = get_spawn_point(pid)
    if spawn is None:
        _panel.log(f"Line {pid} belt not found")
        return None
    ensure_part_material()
    stage.DefinePrim(Sdf.Path(PARTS_ROOT), "Scope")

    counters[pid] += 1
    n = counters[pid]
    path = f"{PARTS_ROOT}/Half_{pid}_{n}"

    prim = stage.DefinePrim(Sdf.Path(path), "Xform")
    prim.GetReferences().AddReference(HALF_PATHS[pid])
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*spawn))

    # piece_id：屬於哪一條線／哪一側。head_id：該條線目前累計生成的
    # 編號——同時也是機械手臂 locate_part() 用來排隊（FIFO）的順序號。
    prim.CreateAttribute("piece_id", Sdf.ValueTypeNames.String).Set(pid)
    prim.CreateAttribute("head_id", Sdf.ValueTypeNames.Int).Set(n)

    v = SLIDE_SPEED if slide_speed is None else slide_speed
    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    rb.CreateVelocityAttr(Gf.Vec3f(v, 0.0, -0.2))
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(PART_MASS)

    for p in Usd.PrimRange(prim):
        if p.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(p)
            UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr(
                "convexDecomposition")

    bind_physics_material(prim, PART_MAT)
    _panel.set_line_status(pid, produced=counters[pid])
    _panel.log(f"produced Half_{pid}_{n}")
    return path

def produce_pair():
    """左右兩線各生成一個半頭——這就是「Produce Part」按鈕真正做的事。"""
    produce_half("L")
    produce_half("R")

def clear_parts():
    """刪除 PARTS_ROOT 底下所有 prim，並重置所有計數器／靜止追蹤
    狀態。由「Clear all parts」按鈕呼叫，重新執行腳本時面板內部
    也會用到。"""
    root = stage.GetPrimAtPath(PARTS_ROOT)
    if root.IsValid():
        for child in list(root.GetChildren()):
            stage.RemovePrim(child.GetPath())
    counters["L"] = 0
    counters["R"] = 0
    _settle_state.clear()
    _settle_last_pos.clear()
    for pid in ("L", "R"):
        _panel.set_line_status(pid, produced=0, in_pool=0)
    _panel.log("all parts cleared")

# =================================================================
# STEP 3b — 靜止判定機制
#
# 全域只有「一個」訂閱（不是每個零件各一個——舊版的球池生成器就是
# 因為每個零件各自訂閱，造成永久存活的回呼函式而洩漏）。每一影格
# 都會量測每個零件的「世界座標」，並與前一影格比較，統計連續低於
# SETTLE_MOVE_TOL 的影格數。is_settled() 是唯一的判斷依據，任何
# 機械手臂動作在碰觸零件前都必須先檢查它——這正是「零件靜止之前，
# 一切動作都被禁止」的具體實作方式。
#
# 刻意「不」使用 RigidBodyAPI 的 velocity 屬性：測試證實那個屬性
# 並不可靠——即使零件在工作台上看起來完全靜止，它仍會持續回報
# 「還在移動」長達 20 秒以上，最可能的原因是 PhysX 的速度回讀
# 在持續的低程度接觸／碰撞分解抖動下，始終無法真正歸零。位置位移
# 則是直接、不依賴特定物理 API 的訊號——它不可能與畫面中實際看到
# 的情況不一致。
# =================================================================
_settle_state = {}      # 零件路徑 -> 連續低於門檻的影格數
_settle_last_pos = {}   # 零件路徑 -> 上一影格的世界座標
_settle_sub = None

def _settle_tick(e):
    """每影格執行一次的回呼：更新每個零件的靜止影格計數。"""
    root = stage.GetPrimAtPath(PARTS_ROOT)
    if not root.IsValid():
        return
    alive = set()
    for child in root.GetChildren():
        path = child.GetPath().pathString
        alive.add(path)
        w = UsdGeom.Xformable(child).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        pos = w.ExtractTranslation()
        prev = _settle_last_pos.get(path)
        _settle_last_pos[path] = pos
        if prev is None:
            _settle_state[path] = 0          # 第一次看到這個零件：還沒有資料
            continue
        moved = (pos - prev).GetLength()
        if moved < SETTLE_MOVE_TOL:
            _settle_state[path] = _settle_state.get(path, 0) + 1
        else:
            _settle_state[path] = 0
    for stale in [p for p in _settle_state if p not in alive]:
        del _settle_state[stale]     # 零件已被刪除／移走：清除其計數
        _settle_last_pos.pop(stale, None)

def start_settle_tracking():
    """訂閱一次每影格回呼——只會執行一次。"""
    global _settle_sub
    if _settle_sub is None:
        _settle_sub = omni.kit.app.get_app().get_update_event_stream() \
            .create_subscription_to_pop(_settle_tick, name="settle_tracker")

def is_settled(part_path):
    """其他所有邏輯都呼叫這個函式來判斷零件是否已經靜止。"""
    return _settle_state.get(part_path, 0) >= SETTLE_FRAMES

# =================================================================
# STEP 4 — 輸送帶控制 ＋ 狀態機面板
# =================================================================
class ConveyorController:
    """透過 PhysX 表面速度驅動全部 4 條輸送帶：輸送帶本身的幾何
    形狀不會移動，但它的接觸表面會帶動任何停在上面的東西——這是
    模擬中常見的輸送帶做法。前提是每個輸送帶 prim 必須已經是
    運動學剛體（見 STEP 2 的 add_kinematic_body）。"""
    BELTS = {
        "outfeed": "/World/OutFeed/Belt",
        "inL":     "/World/LineL/Belt",
        "inR":     "/World/LineR/Belt",
        "table":   "/World/AssemblyStation/Table",  # 第 4 條輸送帶——只要
                  # 加進這個字典，就會自動被 start_all()/stop_all()
                  # 以及面板上的「Start/Stop ALL belts」按鈕包含進去
    }
    DIRECTION = Gf.Vec3f(1.0, 0.0, 0.0)   # 每條輸送帶都朝 +X 方向搬運

    def __init__(self, log_fn=print):
        self._log = log_fn
        self._running = {name: False for name in self.BELTS}

    def _prim(self, name):
        """依短名稱（例如 "inL"）查找對應的輸送帶 prim。"""
        prim = stage.GetPrimAtPath(self.BELTS[name])
        if not prim.IsValid():
            self._log(f"conveyor '{name}': belt prim missing")
            return None
        return prim

    def _set_surface_velocity(self, prim, vel_vec, enabled):
        """套用 PhysxSurfaceVelocityAPI；若此版本沒有提供該 schema
        類別，則改用原始屬性設定作為備援。"""
        try:
            from pxr import PhysxSchema
            api = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
            api.CreateSurfaceVelocityAttr(vel_vec)
            api.CreateSurfaceVelocityEnabledAttr(enabled)
        except Exception:
            prim.CreateAttribute("physxSurfaceVelocity:surfaceVelocity",
                                 Sdf.ValueTypeNames.Float3).Set(vel_vec)
            prim.CreateAttribute("physxSurfaceVelocity:surfaceVelocityEnabled",
                                 Sdf.ValueTypeNames.Bool).Set(enabled)

    def start(self, name, speed=0.4):
        """以指定速度（m/s，方向 +X）啟動一條輸送帶。"""
        prim = self._prim(name)
        if prim is None:
            return False
        self._set_surface_velocity(prim, self.DIRECTION * float(speed), True)
        self._running[name] = True
        self._log(f"conveyor '{name}' START at {speed:.2f} m/s (+X)")
        return True

    def stop(self, name):
        """關閉一條輸送帶。"""
        prim = self._prim(name)
        if prim is None:
            return False
        self._set_surface_velocity(prim, Gf.Vec3f(0.0), False)
        self._running[name] = False
        self._log(f"conveyor '{name}' STOP")
        return True

    def start_all(self, speed=0.4):
        """啟動 BELTS 裡的每一條輸送帶——這就是「Start ALL belts」。"""
        for name in self.BELTS:
            self.start(name, speed)

    def stop_all(self):
        """關閉 BELTS 裡的每一條輸送帶——這就是「Stop ALL belts」。"""
        for name in self.BELTS:
            self.stop(name)

    def any_running(self):
        """只要有任一輸送帶在運轉就回傳 True（決定主控制按鈕上
        Start/Stop 的顯示文字）。"""
        return any(self._running.values())

    def is_running(self, name):
        return self._running.get(name, False)


# =================================================================
# STEP 5 — 機械手臂動作（從池子取件 -> 放到工作台）
#
# 建立在你提供的 20260328_9.py 這份參考腳本的作法之上：
#   - 每個關節的 Transform op 都以 smoothstep 緩動方式改變矩陣
#   - 每次只動「一個」關節（兩隻手臂可以同時各自進行）
#   - 吸附 = 將零件重新掛到 A6 底下，並補償世界座標姿態
#     （夾取期間該零件的剛體會被停用，放開時才重新啟用）
#
# 相較參考腳本的升級之處：關節角度是「即時計算」出來的，而不是
# 寫死的固定值。每隻手臂啟動時都會從場景本身校準出一套正向運動學
# 模型（讀取關節的初始矩陣＋父層世界座標），之後針對量測到的零件
# 位置，透過搜尋這套模型來求解「逆向運算」（IK）。完全沒有寫死
# 連桿長度、正負號或朝向——不論手臂上方疊了什麼修正用的變換，
# 都會自動被納入考量，因為這套模型本身就是直接從場景實際狀態
# 建立出來的。
# =================================================================
def _smooth(t):
    """smoothstep 緩動函式：0 到 1 之間，起末速度平滑趨近於零。"""
    return t * t * (3.0 - 2.0 * t)

def _rot(axis, deg):
    """繞指定軸旋轉 deg 度的矩陣。"""
    return Gf.Matrix4d().SetRotate(Gf.Rotation(axis, deg))

def _reset_to_transform_op(prim):
    """清除一個 prim 的 xform op，並給它一個全新的單一 Transform op。
    同時會移除 ClearXformOpOrder() 遺留下來、沒被清乾淨的
    xformOp:translate/rotate/scale 屬性（ClearXformOpOrder() 只會
    清空「順序清單」，底層的屬性本身並不會被刪除）——這正是先前
    造成「cannot find xform op xformOp:translate」這個 Hydra 警告
    的原因。"""
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    for stale in ("xformOp:translate", "xformOp:rotateXYZ",
                 "xformOp:rotateX", "xformOp:orient", "xformOp:scale"):
        if prim.HasAttribute(stale):
            prim.RemoveProperty(stale)
    return xf.AddTransformOp()

class ArmController:
    """每支手臂各一個實例（ARMS["L"]、ARMS["R"]）。並非手動推導
    連桿長度／正負號，而是每個實例啟動時都會直接從場景「校準」出
    屬於自己的正向運動學模型（讀取實際的關節 Transform op），
    之後再用暴力網格搜尋這套模型來求解逆向運動學。不論階層中在
    手臂上方疊了什麼縮放／位置修正，都會自動被納入，因為這套模型
    正是從場景實際狀態建立出來的。"""
    AXES = {"a1": Gf.Vec3d(0, 0, 1),   # 取自你提供的參考腳本
            "a2": Gf.Vec3d(0, 1, 0),
            "a3": Gf.Vec3d(0, 1, 0)}

    def __init__(self, pid):
        """pid："L" 或 "R"。只記錄路徑／預設值——使用手臂前務必
        先呼叫 calibrate()。"""
        self.pid = pid
        base = (f"/World/Arm{pid}_Group/KukaArm/Geometry/ROOT_0/"
                f"KR_10_R1440_2_1")
        self.base_path = base
        self.paths = {"a1": f"{base}/A1_3",
                      "a2": f"{base}/A1_3/A2_5",
                      "a3": f"{base}/A1_3/A2_5/A3_7"}
        self.a6_path = f"{base}/A1_3/A2_5/A3_7/A4_91/A5_93/A6_95"
        self.ops, self.init = {}, {}
        self.tail = Gf.Matrix4d(1.0)   # A4*A5*A6 的局部變換（固定不驅動）
        self.parent_w = Gf.Matrix4d(1.0)
        self.angles = {"a1": 0.0, "a2": 0.0, "a3": 0.0}
        self.busy = False
        self.held = None               # (part_path, rel_matrix, xform_op)
        self.retries = 0
        self.ok = False

    # ---- 校準：從場景建立正向運動學模型 --------------
    def _op_and_init(self, path):
        """尋找（或建立）某個關節上的 Transform xform op，並回傳
        (op, 目前的矩陣)——這個矩陣會成為 fk()/solve_ik() 所使用的
        「零角度」參考基準。"""
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            return None, None
        xf = UsdGeom.Xformable(prim)
        for op in xf.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTransform:
                m = op.Get()
                return op, (m if m is not None else Gf.Matrix4d(1.0))
        # 沒有 Transform op：把目前的局部變換直接烘焙成一個
        m = xf.GetLocalTransformation()
        xf.ClearXformOpOrder()
        op = xf.AddTransformOp()
        op.Set(m)
        return op, m

    def calibrate(self):
        """讀取 A1/A2/A3 目前的變換，作為正向運動學模型的零位姿，
        以及固定不動的手腕「尾端」（A4-A6，永遠不會被驅動）與手臂的
        世界座標位置。這個手臂必須先校準成功（self.ok=True）才能
        使用——若任何關節找不到，會記錄訊息並中止。"""
        for k, p in self.paths.items():
            op, init = self._op_and_init(p)
            if op is None:
                _panel.log(f"Arm {self.pid}: joint missing: {p}")
                self.ok = False
                return False
            self.ops[k], self.init[k] = op, init
        # 固定不動的手腕尾端（A4、A5、A6 的局部變換）
        tail = Gf.Matrix4d(1.0)
        node = stage.GetPrimAtPath(self.a6_path)
        while node.IsValid() and node.GetPath() != Sdf.Path(self.paths["a3"]):
            tail = tail * UsdGeom.Xformable(node).GetLocalTransformation()
            node = node.GetParent()
        self.tail = tail
        self.parent_w = UsdGeom.Xformable(
            stage.GetPrimAtPath(self.base_path)
        ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        self.ok = True
        tip = self.fk(0, 0, 0)
        _panel.log(f"Arm {self.pid} calibrated, home tip at "
                   f"({tip[0]:+.2f}, {tip[1]:+.2f}, {tip[2]:+.2f})")
        return True

    # ---- 正向運動學（純數學運算，完全不觸碰場景）-----
    def fk(self, a1, a2, a3):
        """給定 3 個關節角度（度），回傳吸嘴（工具端）的世界座標。
        純粹是對照已校準模型做數學運算——不讀取也不寫入場景，
        所以在 IK 搜尋過程中呼叫數千次也很輕量。"""
        m1 = _rot(self.AXES["a1"], a1) * self.init["a1"]
        m2 = _rot(self.AXES["a2"], a2) * self.init["a2"]
        m3 = _rot(self.AXES["a3"], a3) * self.init["a3"]
        m = self.tail * m3 * m2 * m1 * self.parent_w
        return m.Transform(Gf.Vec3d(0, 0, 0))     # A6 原點 = 吸嘴端點

    # ---- 逆向運動學：在正向運動學模型中做搜尋 ---------------------
    def solve_ik(self, target):
        """給定一個世界座標點，求出能讓吸嘴到達該處的 (a1, a2, a3)：
        先對 a1 做粗略網格搜尋（朝目標點偏航），再對 a2/a3 做粗略的
        二維網格搜尋，並以兩輪逐漸縮小的方式精修。回傳
        (a1, a2, a3, 殘餘誤差公尺)。"""
        tx, ty, tz = target[0], target[1], target[2]

        def herr(a1):   # 在目前 a2/a3 條件下的水平誤差
            p = self.fk(a1, self.angles["a2"], self.angles["a3"])
            return math.hypot(p[0] - tx, p[1] - ty)

        best_a1 = min((a * 3.0 for a in range(-60, 61)), key=herr)
        best_a1 = min((best_a1 + d * 0.5 for d in range(-6, 7)), key=herr)

        def perr(a2, a3):
            p = self.fk(best_a1, a2, a3)
            return math.hypot(math.hypot(p[0] - tx, p[1] - ty), p[2] - tz)

        best, be = (0.0, 0.0), perr(0.0, 0.0)
        for a2 in range(-100, 101, 5):
            for a3 in range(-130, 131, 5):
                e = perr(a2, a3)
                if e < be:
                    best, be = (float(a2), float(a3)), e
        for step in (1.0, 0.25):
            b2, b3 = best
            for a2 in (b2 + i * step for i in range(-5, 6)):
                for a3 in (b3 + j * step for j in range(-5, 6)):
                    e = perr(a2, a3)
                    if e < be:
                        best, be = (a2, a3), e
        return best_a1, best[0], best[1], be

    def safe_transit_pose(self):
        """求出能讓吸嘴同時淨空工作台（0.8 公尺）與收集池護欄
        （0.25 公尺）的 A2/A3 角度——透過對這支手臂自身基座正上方
        一個明確的高點做 IK 求解得出，而不是假設角度=0 剛好就夠高／
        夠收回。在每次 A1 大幅擺動之前都先切到這個姿勢（而不是
        盲目地用 0,0），這才是真正在擺動過程中確保淨空的關鍵——
        先前「動作不自然／會卡到東西」的問題，就是因為擺動時的高度
        是角度=0 剛好產生的高度，從未被驗證過是否真的能淨空任何東西。"""
        bx, by = ARM_POSITIONS[self.pid]
        # 基座前方一小段距離、明顯高於場景中所有障礙物的一個點；
        # 這裡只使用 a2/a3（決定高度／收回程度），a1 的結果會被捨棄
        target = Gf.Vec3d(bx * 0.5, by * 0.5, SAFE_TRANSIT_Z)
        _, a2, a3, _ = self.solve_ik(target)
        return a2, a3

    # ---- 單一關節緩動動作（沿用你的參考腳本作法）------------
    async def move_joint(self, key, target, frames=SEQ_FRAMES, side_guard=None):
        """side_guard：這支手臂相對工作台中心、屬於自己那一側的
        正負號（L 為 +1，R 為 -1）。若有設定，每一影格都會透過 FK
        計算吸嘴的 Y 座標，並在它「越界」（含 2 公分容許誤差）
        進入另一隻手臂的範圍那一瞬間記錄一次警告——這是實際證據，
        用來確認一次擺動是否真的離開了自己的範圍，而不是憑外部猜測。"""
        start = self.angles[key]
        if abs(target - start) < 1e-3:
            return
        app = omni.kit.app.get_app()
        warned = False
        for i in range(frames + 1):
            t = _smooth(i / frames)
            a = start + (target - start) * t
            self.ops[key].Set(_rot(self.AXES[key], a) * self.init[key])
            if side_guard is not None and not warned:
                cur = dict(self.angles)
                cur[key] = a
                p = self.fk(cur["a1"], cur["a2"], cur["a3"])
                if p[1] * side_guard < -0.02:
                    _panel.log(f"Arm {self.pid}: WARNING tool crossed "
                              f"table centerline during {key} sweep "
                              f"(y={p[1]:.2f} m)")
                    warned = True
            if self.held:
                self._update_held_pose()   # 跟隨吸嘴移動，不重新掛載階層
            await app.next_update_async()
        self.angles[key] = target

    # ---- 零件搜尋／吸附 ---------------------------------------
    def locate_part(self):
        """依零件名稱尾端的編號排隊（FIFO）——這條線收集池裡編號
        最小的候選零件永遠優先被提供，即使編號較大的零件更早靜止，
        也絕不會被跳過插隊。若池子是空的，或排在最前面的零件還沒
        靜止，會回傳 None（並記錄原因）；在這個函式對某個零件回傳
        非 None 之前，任何其他動作都不能鎖定該零件。"""
        zone = stage.GetPrimAtPath(f"/World/Line{self.pid}/PickZone")
        if not zone.IsValid():
            return None
        zp = UsdGeom.Xformable(zone).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation()
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                  [UsdGeom.Tokens.default_])
        root = stage.GetPrimAtPath(PARTS_ROOT)
        if not root.IsValid():
            return None

        candidates = []   # (編號, 路徑, 夾取點x, 夾取點y, 夾取點z)
        for child in root.GetChildren():
            attr = child.GetAttribute("piece_id")
            if not attr or attr.Get() != self.pid:
                continue
            rb_en = UsdPhysics.RigidBodyAPI(child).GetRigidBodyEnabledAttr()
            if rb_en and rb_en.Get() is False:
                continue        # 目前正被某隻手臂吸附中——不可用
            rng = cache.ComputeWorldBound(child).ComputeAlignedRange()
            if rng.IsEmpty():
                continue
            mn, mx = rng.GetMin(), rng.GetMax()
            cx, cy = (mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2
            if math.hypot(cx - zp[0], cy - zp[1]) > POOL_RADIUS:
                continue        # 還沒進到這條線的池子裡（例如還在輸送帶上）
            m = re.search(r'_(\d+)(?:_r\d+[A-Z])?$', child.GetName())
            n = int(m.group(1)) if m else 0
            candidates.append((n, child.GetPath().pathString, cx, cy, mx[2]))

        if not candidates:
            return None

        candidates.sort(key=lambda c: c[0])        # FIFO，不插隊
        n, path, cx, cy, top_z = candidates[0]

        if not is_settled(path):
            return None                    # 存在但還在動——
                                           # 呼叫端 (run_retrieve) 會自行
                                           # 輪詢，並以節流方式記錄心跳訊息

        return (path, Gf.Vec3d(cx, cy, top_z))     # 頂面中心 = 夾取點

    def _tip_world(self):
        """吸嘴（A6）目前的世界座標變換。"""
        return UsdGeom.Xformable(
            stage.GetPrimAtPath(self.a6_path)
        ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def attach(self, part_path):
        """吸附零件，但「不」重新掛載階層。在 PhysX 正在模擬運算時
        用 MovePrim 重新掛載一個活動中的剛體，是已知會導致當機的
        操作——它會在模擬進行到一半時強制改變場景結構。改用的做法
        是：先凍結該零件的物理模擬，把它的 xform 換成單一
        Transform op，接著在 move_joint() 裡每一影格都更新這個 op，
        讓它跟著吸嘴走。零件本身完全不會離開 /World/Parts；
        每一影格只是在編輯它自己的屬性而已。"""
        prim = stage.GetPrimAtPath(part_path)
        if not prim.IsValid():
            return False
        rb = UsdPhysics.RigidBodyAPI(prim)
        rb.CreateRigidBodyEnabledAttr(False)          # 凍結物理
        rb.CreateVelocityAttr(Gf.Vec3f(0.0))

        w = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        rel = w * self._tip_world().GetInverse()      # 零件相對吸嘴的姿態

        op = _reset_to_transform_op(prim)
        op.Set(w)
        self.held = (part_path, rel, op)
        self._update_held_pose()                      # 立即同步一次
        return True

    def _update_held_pose(self):
        """夾著零件時，每個動畫影格都會呼叫這個函式：透過純屬性
        Set（不改變 prim 階層，模擬進行中呼叫也安全）讓零件緊跟著
        吸嘴移動。"""
        if not self.held:
            return
        _, rel, op = self.held
        op.Set(rel * self._tip_world())

    def release(self):
        """放開目前夾著的零件：重新啟用其物理模擬，讓它從目前的
        （正確）姿態繼續受重力等物理影響、自然落下並靜止。"""
        if not self.held:
            return None
        part_path, rel, op = self.held
        prim = stage.GetPrimAtPath(part_path)
        if not prim.IsValid():
            self.held = None
            return None
        rb = UsdPhysics.RigidBodyAPI(prim)
        rb.CreateRigidBodyEnabledAttr(True)           # 重新啟用 -> 會自然靜止
        rb.CreateVelocityAttr(Gf.Vec3f(0.0))
        self.held = None
        return part_path

    # ---- 完整的取件流程（一次只動一個關節）----------------
    async def run_retrieve(self):
        """單隻手臂的完整取件循環：LOCATING（尋找）→ APPROACHING
        （接近）→ GRIPPING（夾取）→ LIFTING（抬升）→ TRANSITING
        （移動到工作台）→ PLACING（放置）→ RELEASING（釋放）→
        回到待機位置。全程每次只動一個關節。"""
        if self.busy or not self.ok:
            _panel.log(f"Arm {self.pid}: not ready (busy or uncalibrated)")
            return False
        self.busy = True
        st = lambda s: _panel.set_arm_status(self.pid, state=s)
        try:
            st("LOCATING")
            app = omni.kit.app.get_app()
            found = None
            for frame_i in range(LOCATE_TIMEOUT_F):
                found = self.locate_part()
                if found is not None:
                    break
                if frame_i % 60 == 0 and frame_i > 0:
                    _panel.log(f"Arm {self.pid}: still waiting for a "
                              f"part to settle in the pool")
                await app.next_update_async()
            if found is None:
                _panel.log(f"Arm {self.pid}: no part settled within "
                          f"{LOCATE_TIMEOUT_F/60:.0f}s, giving up")
                st("WAITING")
                return False
            part_path, grip = found
            hover = Gf.Vec3d(grip[0], grip[1], grip[2] + HOVER_DZ)

            a1g, a2g, a3g, eg = self.solve_ik(grip)
            a1h, a2h, a3h, eh = self.solve_ik(hover)
            _panel.log(f"Arm {self.pid}: IK grip err {eg*1000:.0f} mm, "
                       f"hover err {eh*1000:.0f} mm")

            own_half = 1.0 if self.pid == "L" else -1.0
            st("APPROACHING")                    # 每次只動一個關節：
            await self.move_joint("a1", a1g, A1_FRAMES, side_guard=own_half)
            await self.move_joint("a2", a2h)
            await self.move_joint("a3", a3h)
            await self.move_joint("a2", a2g)
            await self.move_joint("a3", a3g)

            # 依零件「目前」的實際位置檢查是否真的吸附成功
            tip = self._tip_world().ExtractTranslation()
            prim = stage.GetPrimAtPath(part_path)
            if not prim.IsValid():
                raise RuntimeError("part vanished")
            pw = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()).ExtractTranslation()
            dist = math.hypot(math.hypot(tip[0] - pw[0], tip[1] - pw[1]),
                              tip[2] - pw[2])
            if dist > GRAB_TOL + 0.15:
                self.retries += 1
                _panel.set_arm_status(self.pid, retries=self.retries)
                _panel.log(f"Arm {self.pid}: suction miss "
                           f"({dist*100:.0f} cm), retreating")
                await self.go_home(step_status=st)
                return False

            st("GRIPPING")
            self.attach(part_path)
            _panel.set_arm_status(self.pid, grip="HOLDING")

            st("LIFTING")
            a2s, a3s = self.safe_transit_pose()
            await self.move_joint("a3", a3s)
            await self.move_joint("a2", a2s)

            marker = stage.GetPrimAtPath(
                f"/World/AssemblyStation/PlaceTarget_{self.pid}")
            mp = UsdGeom.Xformable(marker).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()).ExtractTranslation()
            place = Gf.Vec3d(mp[0], mp[1], mp[2] + PLACE_DZ)
            a1p, a2p, a3p, ep = self.solve_ik(place)
            _panel.log(f"Arm {self.pid}: IK place err {ep*1000:.0f} mm")

            st("TRANSITING")
            await self.move_joint("a1", a1p, A1_FRAMES, side_guard=own_half)

            st("PLACING")
            await self.move_joint("a2", a2p)
            await self.move_joint("a3", a3p)

            st("RELEASING")
            out = self.release()
            _panel.set_arm_status(self.pid, grip="OPEN")
            _panel.log(f"Arm {self.pid}: placed {out}")

            await self.go_home(step_status=st)
            return True
        except Exception as e:
            _panel.log(f"Arm {self.pid} FAULT: {e}")
            st("WAITING")
            return False
        finally:
            self.busy = False

    async def go_home(self, step_status=None):
        """先切到安全過渡姿勢再收回，接著把 A1 轉回 0 度回到待機
        位置——每個階段依然是一次只動一個關節。"""
        if step_status:
            step_status("RETREATING")
        a2s, a3s = self.safe_transit_pose()
        await self.move_joint("a3", a3s)
        await self.move_joint("a2", a2s)
        if step_status:
            step_status("HOMING")
        own_half = 1.0 if self.pid == "L" else -1.0
        await self.move_joint("a1", 0.0, A1_FRAMES, side_guard=own_half)
        if step_status:
            step_status("WAITING")


# =================================================================
# STEP 6 — 自動吸附組裝與熔接
#
# 一旦兩隻手臂都完成放置，就會自動觸發（沒有對應按鈕）。
# 只有在「兩個」半頭都靠近各自的 PlaceTarget 標記點「且」都已靜止
# （沿用取件時的同一套 is_settled() 判定機制）時才會啟動——重複
# 利用既有機制，而不是另外發明一套新的。
#
# 動作方式：以 L 作為固定不動的錨點；R 則以運動學方式（位置線性
# 內插＋旋轉球面內插，搭配 smoothstep 時間曲線）逐漸移動到 L 目前
# 的世界座標姿態。因為兩個半頭在 Blender 匯出時就共用同一個原點
# （組裝完成頭部的中心），所以「L 的變換 == R 的變換」本身就是正確
# 的密合方式——完全不需要另外計算偏移量。
#
# 熔接：使用真正的 UsdPhysics.FixedJoint。Joint 是一種「關係」，
# 不是重新掛載階層，所以——不同於先前吸附時用 MovePrim 造成當機
# 的情況——它完全不會觸碰場景階層結構，模擬進行中建立也是安全的。
# =================================================================
PLACE_RADIUS   = 0.40   # 每個 PlaceTarget 標記點周圍的搜尋半徑（公尺）
                        # 由 0.25 加大——實測放置位置落在
                        # 0.30-0.32 公尺，剛好超出舊半徑範圍，
                        # 即使兩個半頭其實都已完全靜止
SNAP_FRAMES    = 90     # 內插動畫長度（影格數）
SNAP_TIMEOUT_F = 1200   # 約 20 秒（60fps 換算）：若某半頭始終不靜止就放棄等待

_snap_task = None
_weld_count = 0
_snap_debug = {"L": None, "R": None}   # 每一側最新的診斷資訊，供心跳訊息使用

def find_settled_on_table(pid):
    """跟 locate_part 類似，但搜尋範圍改成這個半頭對應的
    PlaceTarget 標記點，而不是收集池。回傳零件路徑，若還沒有
    符合條件／靜止的零件則回傳 None。每次都會把「原因」記錄到
    _snap_debug[pid]，讓心跳訊息能顯示具體數字，而不是只有
    none/present 這種模糊狀態。"""
    marker = stage.GetPrimAtPath(f"/World/AssemblyStation/PlaceTarget_{pid}")
    if not marker.IsValid():
        _snap_debug[pid] = {"reason": "PlaceTarget marker missing"}
        return None
    mp = UsdGeom.Xformable(marker).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()).ExtractTranslation()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    root = stage.GetPrimAtPath(PARTS_ROOT)
    if not root.IsValid():
        _snap_debug[pid] = {"reason": "no PARTS_ROOT"}
        return None
    found_any = False
    for child in root.GetChildren():
        attr = child.GetAttribute("piece_id")
        if not attr or attr.Get() != pid:
            continue
        welded_attr = child.GetAttribute("welded")
        if welded_attr and welded_attr.Get() is True:
            continue                       # 已經組裝完成——絕不再被提供
        found_any = True
        path = child.GetPath().pathString
        rb_en = UsdPhysics.RigidBodyAPI(child).GetRigidBodyEnabledAttr()
        held = bool(rb_en and rb_en.Get() is False)
        rng = cache.ComputeWorldBound(child).ComputeAlignedRange()
        if rng.IsEmpty():
            _snap_debug[pid] = {"path": path, "reason": "empty bbox", "held": held}
            continue
        mn, mx = rng.GetMin(), rng.GetMax()
        cx, cy = (mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2
        dist = math.hypot(cx - mp[0], cy - mp[1])
        settled = is_settled(path)
        _snap_debug[pid] = {"path": path, "dist_m": round(dist, 3),
                            "held": held, "settled": settled,
                            "settle_frames": _settle_state.get(path, 0)}
        if held or dist > PLACE_RADIUS:
            continue                       # 已被夾住，或距離太遠——繼續掃描
        if not settled:
            return None                    # 存在、距離在範圍內，但還在動

        return path
    if not found_any:
        _snap_debug[pid] = {"reason": f"no piece_id=={pid} child under {PARTS_ROOT}"}
    return None

def request_snap_check():
    """啟動（或維持既有的）監看程序，一旦兩個半頭都到位且靜止，
    就會觸發吸附組裝。"""
    global _snap_task
    if _snap_task is None or _snap_task.done():
        _snap_task = asyncio.ensure_future(_snap_watch())

async def _snap_watch():
    """持續輪詢，直到兩個半頭都靠近標記點且靜止為止；每秒記錄一次
    心跳訊息，方便診斷卡在哪裡。整個函式包在 try/except 裡——
    這樣輪詢迴圈中任何地方發生的例外（不只是 do_snap 內部）都會
    被記錄下來，不會悄悄消失。"""
    app = omni.kit.app.get_app()
    try:
        _panel.set_assembly_status(servo="WAITING", l_present=False, r_present=False)
        for frame_i in range(SNAP_TIMEOUT_F):
            lp = find_settled_on_table("L")
            rp = find_settled_on_table("R")
            _panel.set_assembly_status(l_present=lp is not None, r_present=rp is not None)
            if lp and rp:
                await do_snap(lp, rp)
                return
            if frame_i % 60 == 0 and frame_i > 0:   # 每約 1 秒記錄一次心跳
                _panel.log(f"Snap watching: L={_snap_debug['L']} "
                          f"R={_snap_debug['R']}")
            await app.next_update_async()
        _panel.log("Snap: timed out waiting for both halves to settle on the table")
        _panel.set_assembly_status(servo="INACTIVE")
        PIPE_COUNTERS["failed_snaps"] += 1
        _panel.set_counters(failed_snaps=PIPE_COUNTERS["failed_snaps"])
        _panel.set_pipeline_state("WAIT_FOR_PARTS")
    except Exception as e:
        # 這裡會捕捉監看程序裡「所有」的例外，不只是 do_snap() 呼叫——
        # 先前的版本只包住 do_snap() 那一行，所以輪詢迴圈本身如果出錯
        # （例如 find_settled_on_table 內部），依然會悄悄當掉、
        # 既不會有逾時訊息也不會出現在面板日誌裡。現在任何地方都不會
        # 再無聲無息地消失。
        import traceback
        _panel.log(f"Snap watcher FAILED: {e}")
        print("Snap watcher traceback:")
        traceback.print_exc()
        _panel.set_assembly_status(servo="INACTIVE")
        PIPE_COUNTERS["failed_snaps"] += 1
        _panel.set_counters(failed_snaps=PIPE_COUNTERS["failed_snaps"])
        _panel.set_pipeline_state("WAIT_FOR_PARTS")

async def do_snap(l_path, r_path):
    """實際執行吸附組裝：以 L 為錨點凍結雙方物理、將 R 內插到 L 的
    精確世界座標姿態，收斂後建立真正的 FixedJoint 完成熔接。"""
    global _weld_count
    _panel.log(f"Snap: engaging (anchor {l_path}, moving {r_path})")
    lprim = stage.GetPrimAtPath(l_path)
    rprim = stage.GetPrimAtPath(r_path)
    if not lprim.IsValid() or not rprim.IsValid():
        _panel.log("Snap: a half vanished, aborting")
        _panel.set_assembly_status(servo="INACTIVE")
        return False

    # 兩者都先凍結：錨點不能漂移，被移動的一方也不能跟物理互相打架
    for p in (lprim, rprim):
        rb = UsdPhysics.RigidBodyAPI(p)
        rb.CreateRigidBodyEnabledAttr(False)
        rb.CreateVelocityAttr(Gf.Vec3f(0.0))

    target = Gf.Transform(UsdGeom.Xformable(lprim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()))
    start = Gf.Transform(UsdGeom.Xformable(rprim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()))
    p0, p1 = start.GetTranslation(), target.GetTranslation()
    rot0, rot1 = start.GetRotation(), target.GetRotation()
    ang0 = (rot1 * rot0.GetInverse()).GetAngle()
    # 這個 pxr 版本的 Gf.Slerp 沒有 Gf.Rotation 的多載版本（從
    # ArgumentError 列出的多載清單可以確認這件事）——但它有支援
    # Gf.Quaternion，所以改用四元數做球面內插，再把結果包回
    # Gf.Rotation 供 SetTransform 使用（SetTransform 本身確實支援
    # Gf.Rotation，那部分從未出過錯）。
    q0, q1 = rot0.GetQuaternion(), rot1.GetQuaternion()

    rop = _reset_to_transform_op(rprim)

    _panel.set_assembly_status(servo="ALIGNING")
    app = omni.kit.app.get_app()
    for i in range(SNAP_FRAMES + 1):
        if not lprim.IsValid() or not rprim.IsValid():
            raise RuntimeError("a half was removed/invalidated mid-snap "
                              "(stage Reset while snapping?)")
        t = _smooth(i / SNAP_FRAMES)
        pos = p0 + (p1 - p0) * t
        rot = Gf.Rotation(Gf.Slerp(t, q0, q1))
        m = Gf.Matrix4d().SetTransform(rot, pos)
        rop.Set(m)
        _panel.set_assembly_status(pos_err_mm=(p1 - pos).GetLength() * 1000.0,
                                   ang_err_deg=ang0 * (1.0 - t))
        await app.next_update_async()

    _panel.set_assembly_status(servo="CONVERGED", pos_err_mm=0.0, ang_err_deg=0.0)

    # 熔接：這是一種「關係」（joint），不是重新掛載階層——模擬進行中建立也安全
    _weld_count += 1
    joint_path = f"{PARTS_ROOT}/Weld_{_weld_count}"
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(l_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(r_path)])
    joint.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))

    # 重新啟用物理，讓兩者成為一個由 joint 固定在一起的整體
    # （靠 joint 維持結合，不是靠程式碼硬綁）
    for p in (lprim, rprim):
        rb = UsdPhysics.RigidBodyAPI(p)
        rb.CreateRigidBodyEnabledAttr(True)
        rb.CreateVelocityAttr(Gf.Vec3f(0.0))
        # 標記起來，讓 find_settled_on_table 永遠不會再提供這一對——
        # 如果沒有這個標記，一顆完成品若還留在工作台上（因為 Deliver
        # 目前仍是空按鈕），就會在「下一次」Retrieve 循環中被重新
        # 發現，並試圖對這對已經熔接過的零件再建立一次 joint，
        # 這正是上次測試中出現「PxJoint::setActors: at least one
        # actor must be non-static」錯誤的真正原因。
        p.CreateAttribute("welded", Sdf.ValueTypeNames.Bool).Set(True)

    _panel.set_assembly_status(welded=True)
    _panel.log(f"Snap: welded {l_path} + {r_path} ({joint_path})")
    _panel.set_pipeline_state("WELDED")
    PIPE_COUNTERS["completed"] += 1
    _panel.set_counters(completed=PIPE_COUNTERS["completed"])
    return True


ARMS = {}
PIPE_COUNTERS = {"completed": 0, "grip_breaks": 0, "failed_snaps": 0}

def init_arms():
    """建立兩支手臂的 ArmController 並各自校準一次。"""
    for pid in ("L", "R"):
        ARMS[pid] = ArmController(pid)
        ARMS[pid].calibrate()

async def _retrieve_all():
    """讓兩隻手臂平行執行取件動作；若兩邊都成功放置並且已觸發
    吸附組裝流程，回傳 True（此時不代表已經「熔接完成」——呼叫端
    若想確認熔接結果，需要另外 await _snap_task）。"""
    ready = [a for a in ARMS.values() if a.ok and not a.busy]
    if not ready:
        _panel.log("Retrieve: no arm ready")
        return False
    _panel.set_pipeline_state("PICKING")
    results = await asyncio.gather(*[a.run_retrieve() for a in ready])
    if all(results):
        _panel.set_pipeline_state("SNAPPING")
        _panel.log("both halves placed — watching for settle, snap is automatic")
        request_snap_check()
        return True
    _panel.set_pipeline_state("WAIT_FOR_PARTS")
    return False

def start_retrieve():
    """按鈕點擊處理函式——單純包裝 _retrieve_all()，觸發後不等待結果。"""
    asyncio.ensure_future(_retrieve_all())

def home_arm(pid):
    """讓指定手臂回到待機位置（若目前沒有忙碌中）。"""
    arm = ARMS.get(pid)
    if arm and arm.ok and not arm.busy:
        asyncio.ensure_future(arm.go_home(
            step_status=lambda s: _panel.set_arm_status(pid, state=s)))
    else:
        _panel.log(f"Home ({pid}): arm busy or not ready")

# =================================================================
# STEP 7 — DELIVER（把組裝完成的頭部送到刪除池並刪除）
#          ＋ 自動循環（8 個步驟的循環，接到主控制的 Start/Stop
#          按鈕上；既有的「auto-loop」勾選框決定要跑單次循環
#          還是持續循環，跟按鈕上的文字說明完全一致）
# =================================================================
DELIVER_TIMEOUT_F = 1800   # 約 30 秒：輸送過程的安全逾時上限
POOL_WAIT_TIMEOUT_F = 900  # 約 15 秒：等待剛生成的零件靜止

async def _wait_pool_settled(timeout_f=POOL_WAIT_TIMEOUT_F):
    """輪詢兩隻手臂各自的 locate_part()（唯讀，不會有副作用），
    直到兩邊都在自己的池子裡找到已靜止的零件為止。重複使用與真正
    取件時完全相同的 FIFO＋靜止判定邏輯，所以這裡的「靜止」跟
    實際負責取件的手臂認定的「靜止」是同一件事。"""
    app = omni.kit.app.get_app()
    for _ in range(timeout_f):
        l_ready = ARMS["L"].locate_part() is not None
        r_ready = ARMS["R"].locate_part() is not None
        if l_ready and r_ready:
            return True
        await app.next_update_async()
    return False

def _find_current_welded_pair():
    """找出 PARTS_ROOT 底下目前標記為 welded=True、仍存在的那一對
    零件。只要 Deliver 都在下一次 Produce 之前執行完（自動循環會
    確保這一點，因為整個流程是依序執行的），任何時刻應該最多只會
    存在一組這樣的零件。"""
    root = stage.GetPrimAtPath(PARTS_ROOT)
    if not root.IsValid():
        return []
    found = []
    for child in root.GetChildren():
        w = child.GetAttribute("welded")
        if w and w.Get() is True:
            found.append(child.GetPath().pathString)
    return found

async def deliver_head():
    """循環中的第 6-8 步：啟動工作台＋出料輸送帶，把組裝完成的
    頭部送到刪除池，將其（連同熔接用的 joint）從場景中刪除，
    再關閉輸送帶。成功時回傳 True。"""
    pair = _find_current_welded_pair()
    if not pair:
        _panel.log("Deliver: no welded head found on the table")
        return False

    _panel.set_pipeline_state("CONVEYING")
    _panel.set_outfeed_status(head_on_belt=True, in_trigger=False)
    _panel.conveyor.start("table", 0.4)
    _panel.conveyor.start("outfeed", 0.4)
    _panel.log(f"Deliver: conveying {pair} toward the deletion pool")

    app = omni.kit.app.get_app()
    ref_path = pair[0]
    arrived = False
    for _ in range(DELIVER_TIMEOUT_F):
        prim = stage.GetPrimAtPath(ref_path)
        if not prim.IsValid():
            break
        x = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation()[0]
        _panel.set_outfeed_status(in_trigger=(x >= DPOOL_CENTER_X - DPOOL_HALF_W))
        if x >= DPOOL_CENTER_X:
            arrived = True
            break
        await app.next_update_async()

    _panel.conveyor.stop("table")
    _panel.conveyor.stop("outfeed")

    if not arrived:
        _panel.log("Deliver: timed out before reaching the deletion pool")
        _panel.set_outfeed_status(head_on_belt=False, in_trigger=False)
        _panel.set_pipeline_state("WAIT_FOR_PARTS")
        return False

    _panel.set_pipeline_state("DELETING")
    root = stage.GetPrimAtPath(PARTS_ROOT)
    for child in list(root.GetChildren()) if root.IsValid() else []:
        name = child.GetName()
        if name.startswith("Weld_"):
            rel = child.GetRelationship("physics:body0")
            targets = [str(t) for t in rel.GetTargets()] if rel else []
            if any(t in pair for t in targets):
                stage.RemovePrim(child.GetPath())
    for p in pair:
        if stage.GetPrimAtPath(p).IsValid():
            stage.RemovePrim(p)

    _panel.set_outfeed_status(head_on_belt=False, in_trigger=False)
    _panel.log(f"Deliver: deleted {pair}")
    _panel.set_pipeline_state("IDLE")
    return True

_auto_loop_task = None
_auto_loop_running = False

async def _auto_loop_body():
    """完整的 8 步驟自動循環（對應使用者要求的腳本內容）：
    生產 -> 開輸送帶 -> 等待靜止 -> 關輸送帶 -> 取件 ->
    等待組裝完成 -> 送往刪除池並刪除 -> （依 auto-loop 勾選框
    決定是否重複）。任一步驟失敗都會記錄原因並停止循環，
    不會靜悄悄地卡住。"""
    global _auto_loop_running
    _auto_loop_running = True
    cycle = 0
    try:
        while _auto_loop_running:
            cycle += 1
            _panel.log(f"===== Auto-loop cycle {cycle} =====")

            # 1. 生產一次零件
            _panel.set_pipeline_state("PRODUCING")
            produce_pair()

            # 2. 啟動輸送帶
            _panel.conveyor.start_all(0.4)
            _panel.set_pipeline_state("WAIT_FOR_PARTS")

            # 3. 等待零件在池中靜止，再關閉輸送帶
            settled = await _wait_pool_settled()
            _panel.conveyor.stop_all()
            if not settled:
                _panel.log("Auto-loop: parts never settled in the pool "
                          "— stopping loop")
                break

            # 4. 開始取件
            picked = await _retrieve_all()
            if not picked:
                _panel.log("Auto-loop: retrieve failed — stopping loop")
                break

            # 5. 等待組裝（吸附熔接）完全完成
            if _snap_task is not None:
                await _snap_task
            welded_ok = (_panel._widgets["pipeline_state"].text == "WELDED")
            if not welded_ok:
                _panel.log("Auto-loop: snap did not complete — stopping loop")
                break

            # 6+7+8. 啟動輸送帶、送到刪除池、關閉輸送帶
            delivered = await deliver_head()
            if not delivered:
                _panel.log("Auto-loop: deliver failed — stopping loop")
                break

            if not _panel._widgets["auto_loop"].model.get_value_as_bool():
                _panel.log("Auto-loop: single-cycle mode, one head done")
                break
    finally:
        _auto_loop_running = False
        _panel.log("Auto-loop stopped")

def start_auto_loop():
    """接到面板的「Start」按鈕。若已在執行中則不重複啟動。"""
    global _auto_loop_task
    if _auto_loop_task is not None and not _auto_loop_task.done():
        _panel.log("Auto-loop already running")
        return
    _auto_loop_task = asyncio.ensure_future(_auto_loop_body())

def stop_auto_loop():
    """接到面板的「Stop」按鈕。屬於「柔性」停止——會先讓目前這一步
    完整跑完，才不會開始下一輪循環，不會在動作進行到一半時硬生生
    打斷。"""
    global _auto_loop_running
    if not _auto_loop_running:
        _panel.log("Auto-loop is not running")
        return
    _auto_loop_running = False
    _panel.log("Auto-loop: stop requested — finishing the current step, "
              "will not start a new cycle")


# 面板文字顏色（ABGR）：灰＝待機、橘＝進行中、綠＝正常、紅＝異常
COL_IDLE    = 0xFF888888
COL_ACTIVE  = 0xFF00A5FF
COL_OK      = 0xFF4CC44C
COL_FAULT   = 0xFF4444DD
COL_TEXT    = 0xFFCCCCCC

PIPELINE_STATES = ["IDLE", "PRODUCING", "WAIT_FOR_PARTS", "PICKING",
                   "PLACING", "SNAPPING", "WELDED", "CONVEYING",
                   "DELETING", "FAULT"]


class StateMachinePanel:
    """整個 UI 視窗。設計原則：面板本身「絕對不」包含邏輯——
    每一個狀態顯示元件，底下都對應一個 set_*() 方法，由協調邏輯
    呼叫它來更新畫面；每一個按鈕的 clicked_fn，也都是呼叫檔案前面
    已定義好的真正函式（若該功能還沒接上，就呼叫 self._stub()）。
    _build() 純粹是版面配置——想知道某個按鈕／標籤是什麼，
    去那裡找。"""
    def __init__(self):
        self._log_lines = []
        self._widgets = {}
        self.conveyor = ConveyorController(log_fn=self.log)
        self._build()

    def _stub(self, name):
        """尚未接上真正邏輯的按鈕會走到這裡——只會記錄一筆按下的
        訊息，讓你知道有註冊到點擊事件。"""
        self.log(f"[STUB] button pressed: {name}")

    # ---- 事件日誌 ------------------------------------------------
    def log(self, msg):
        stamp = time.strftime("%H:%M:%S")
        line = f"[{stamp}] {msg}"
        self._log_lines.append(line)
        self._log_lines = self._log_lines[-200:]
        if "log_stack" in self._widgets:
            self._rebuild_log()
        print(line)

    def _rebuild_log(self):
        stack = self._widgets["log_stack"]
        stack.clear()
        with stack:
            with ui.VStack(spacing=1):
                for line in reversed(self._log_lines[-40:]):
                    ui.Label(line, height=14,
                             style={"font_size": 11, "color": COL_TEXT})

    def _clear_log(self):
        self._log_lines = []
        self._rebuild_log()

    # ---- 真正有作用的處理函式 --------------------------------------
    def _outfeed_speed(self):
        return self._widgets["of_speed"].model.get_value_as_float()

    def _on_outfeed_start(self):
        if self.conveyor.start("outfeed", self._outfeed_speed()):
            self.set_outfeed_status(running=True)
        self._sync_all_belts_button()

    def _on_outfeed_stop(self):
        if self.conveyor.stop("outfeed"):
            self.set_outfeed_status(running=False)
        self._sync_all_belts_button()

    def _on_line_belt_toggle(self, pid):
        name = f"in{pid}"
        if self.conveyor.is_running(name):
            self.conveyor.stop(name)
            self._widgets[f"line{pid}_belt_btn"].text = "Belt: OFF"
        else:
            self.conveyor.start(name, self._outfeed_speed())
            self._widgets[f"line{pid}_belt_btn"].text = "Belt: ON"
        self._sync_all_belts_button()

    def _on_all_belts_toggle(self):
        """單一按鈕：同時啟動或關閉全部輸送帶。"""
        if self.conveyor.any_running():
            self.conveyor.stop_all()
        else:
            self.conveyor.start_all(self._outfeed_speed())
        self._sync_belt_widgets()

    def _sync_belt_widgets(self):
        """讓所有跟輸送帶有關的元件，顯示與實際輸送帶狀態一致。"""
        for pid in ("L", "R"):
            on = self.conveyor.is_running(f"in{pid}")
            self._widgets[f"line{pid}_belt_btn"].text = \
                f"Belt: {'ON' if on else 'OFF'}"
        self.set_outfeed_status(running=self.conveyor.is_running("outfeed"))
        self._sync_all_belts_button()

    def _sync_all_belts_button(self):
        btn = self._widgets.get("all_belts_btn")
        if btn:
            btn.text = ("Stop ALL belts" if self.conveyor.any_running()
                        else "Start ALL belts")

    def _on_produce_part(self):
        self.set_pipeline_state("PRODUCING")
        produce_pair()
        self.set_pipeline_state("WAIT_FOR_PARTS")

    # ---- 狀態更新函式（供外部呼叫，更新畫面顯示）----------------------
    def set_pipeline_state(self, state, fault=False):
        w = self._widgets["pipeline_state"]
        w.text = state
        w.style = {"font_size": 22,
                   "color": COL_FAULT if fault else
                            (COL_IDLE if state == "IDLE" else COL_ACTIVE)}

    def set_line_status(self, pid, in_pool=None, produced=None):
        if in_pool is not None:
            self._widgets[f"line{pid}_pool"].text = f"parts in pool: {in_pool}"
        if produced is not None:
            self._widgets[f"line{pid}_total"].text = f"produced: {produced}"

    def set_arm_status(self, pid, state=None, grip=None, retries=None):
        if state is not None:
            w = self._widgets[f"arm{pid}_state"]
            w.text = state
            w.style = {"font_size": 14,
                       "color": COL_IDLE if state in ("WAITING", "HOMING")
                                else COL_ACTIVE}
        if grip is not None:
            w = self._widgets[f"arm{pid}_grip"]
            w.text = f"grip: {grip}"
            w.style = {"font_size": 13,
                       "color": COL_FAULT if grip == "GRIP-BREAK" else
                                (COL_OK if grip == "HOLDING" else COL_TEXT)}
        if retries is not None:
            self._widgets[f"arm{pid}_retry"].text = f"retries: {retries}"

    def set_assembly_status(self, l_present=None, r_present=None,
                            servo=None, pos_err_mm=None, ang_err_deg=None,
                            welded=None):
        if l_present is not None:
            self._widgets["asm_l"].text = f"half L: {'PRESENT' if l_present else 'none'}"
        if r_present is not None:
            self._widgets["asm_r"].text = f"half R: {'PRESENT' if r_present else 'none'}"
        if servo is not None:
            self._widgets["asm_servo"].text = f"servo: {servo}"
        if pos_err_mm is not None and ang_err_deg is not None:
            self._widgets["asm_err"].text = \
                f"pose error: {pos_err_mm:.1f} mm / {ang_err_deg:.1f} deg"
        if welded is not None:
            w = self._widgets["asm_weld"]
            w.text = f"weld: {'CREATED' if welded else 'none'}"
            w.style = {"font_size": 14, "color": COL_OK if welded else COL_TEXT}

    def set_outfeed_status(self, running=None, head_on_belt=None, in_trigger=None):
        if running is not None:
            w = self._widgets["of_run"]
            w.text = f"conveyor: {'RUNNING' if running else 'STOPPED'}"
            w.style = {"font_size": 13, "color": COL_OK if running else COL_TEXT}
        if head_on_belt is not None:
            self._widgets["of_head"].text = f"head on belt: {'YES' if head_on_belt else 'none'}"
        if in_trigger is not None:
            self._widgets["of_trig"].text = f"in trigger zone: {'YES' if in_trigger else 'none'}"

    def set_counters(self, completed=None, last_cycle_s=None,
                     grip_breaks=None, failed_snaps=None):
        if completed is not None:
            self._widgets["cnt_done"].text = f"heads completed: {completed}"
        if last_cycle_s is not None:
            self._widgets["cnt_cycle"].text = f"last cycle: {last_cycle_s:.1f} s"
        if grip_breaks is not None:
            self._widgets["cnt_break"].text = f"grip-breaks: {grip_breaks}"
        if failed_snaps is not None:
            self._widgets["cnt_snapfail"].text = f"failed snaps: {failed_snaps}"

    # ---- build -------------------------------------------------------
    def _build(self):
        """建構整個視窗的版面配置。下面各個 CollapsableFrame 的標題
        （Master control、Production lines、Arms、Assembly station、
        Out-feed、Counters、Event log、Debug）就是面板上實際看到的
        各個區塊——這裡的程式碼由上到下的順序，跟畫面上看到的順序
        完全一致。"""
        self.window = ui.Window("Assembly State Machine", width=430, height=880)
        with self.window.frame:
            with ui.ScrollingFrame(
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF):
                with ui.VStack(spacing=6):

                    with ui.CollapsableFrame("Master control", collapsed=False):
                        with ui.VStack(spacing=6):
                            lbl = ui.Label("IDLE", height=30,
                                           alignment=ui.Alignment.CENTER,
                                           style={"font_size": 22, "color": COL_IDLE})
                            self._widgets["pipeline_state"] = lbl

                            ui.Label("Cycle stages", style={"font_size": 12})
                            with ui.HStack(spacing=4, height=40):
                                ui.Button("Produce Part",
                                          clicked_fn=self._on_produce_part)
                                ui.Button("Retrieve",
                                          clicked_fn=lambda: start_retrieve())
                                ui.Button("Deliver",
                                          clicked_fn=lambda: self._stub("Deliver"))

                            ui.Label("Conveyors", style={"font_size": 12})
                            btn = ui.Button("Start ALL belts", height=32,
                                            clicked_fn=self._on_all_belts_toggle)
                            self._widgets["all_belts_btn"] = btn

                            ui.Label("Run control", style={"font_size": 12})
                            with ui.HStack(spacing=4, height=28):
                                ui.Button("Start", clicked_fn=lambda: start_auto_loop())
                                ui.Button("Pause", clicked_fn=lambda: self._stub("Pause"))
                                ui.Button("Stop", clicked_fn=lambda: stop_auto_loop())
                                ui.Button("Reset", clicked_fn=lambda: self._stub("Reset"))

                            with ui.HStack(height=20):
                                cb = ui.CheckBox(width=24)
                                self._widgets["auto_loop"] = cb
                                ui.Label("auto-loop (off = single cycle)",
                                         style={"font_size": 12})

                    with ui.CollapsableFrame("Production lines", collapsed=False):
                        with ui.VStack(spacing=4):
                            for pid in ("L", "R"):
                                with ui.HStack(spacing=6, height=24):
                                    ui.Label(f"Line {pid}", width=50,
                                             style={"font_size": 14})
                                    self._widgets[f"line{pid}_pool"] = ui.Label(
                                        "parts in pool: 0", width=110,
                                        style={"font_size": 12})
                                    self._widgets[f"line{pid}_total"] = ui.Label(
                                        "produced: 0", width=90,
                                        style={"font_size": 12})
                                    ui.Button("Spawn one", width=80,
                                              clicked_fn=lambda p=pid: produce_half(p))
                                    btn = ui.Button(
                                        "Belt: OFF", width=80,
                                        clicked_fn=lambda p=pid:
                                        self._on_line_belt_toggle(p))
                                    self._widgets[f"line{pid}_belt_btn"] = btn
                            ui.Button("Clear all parts", height=22,
                                      clicked_fn=lambda: clear_parts())

                    with ui.CollapsableFrame("Arms", collapsed=False):
                        with ui.VStack(spacing=4):
                            for pid in ("L", "R"):
                                with ui.HStack(spacing=6, height=24):
                                    ui.Label(f"Arm {pid}", width=50,
                                             style={"font_size": 14})
                                    self._widgets[f"arm{pid}_state"] = ui.Label(
                                        "WAITING", width=100,
                                        style={"font_size": 14, "color": COL_IDLE})
                                    self._widgets[f"arm{pid}_grip"] = ui.Label(
                                        "grip: OPEN", width=100,
                                        style={"font_size": 13, "color": COL_TEXT})
                                    self._widgets[f"arm{pid}_retry"] = ui.Label(
                                        "retries: 0", width=70,
                                        style={"font_size": 12})
                                with ui.HStack(spacing=4, height=22):
                                    ui.Spacer(width=50)
                                    ui.Button("Home", width=70,
                                              clicked_fn=lambda p=pid:
                                              home_arm(p))
                                    ui.Button("Force release", width=100,
                                              clicked_fn=lambda p=pid:
                                              self._stub(f"Force release ({p})"))
                                    ui.Button("Retry", width=70,
                                              clicked_fn=lambda p=pid:
                                              self._stub(f"Retry ({p})"))

                    with ui.CollapsableFrame("Assembly station", collapsed=False):
                        with ui.VStack(spacing=4):
                            with ui.HStack(spacing=10, height=20):
                                self._widgets["asm_l"] = ui.Label(
                                    "half L: none", width=110,
                                    style={"font_size": 13})
                                self._widgets["asm_r"] = ui.Label(
                                    "half R: none", width=110,
                                    style={"font_size": 13})
                            self._widgets["asm_servo"] = ui.Label(
                                "servo: INACTIVE", height=18,
                                style={"font_size": 13})
                            self._widgets["asm_err"] = ui.Label(
                                "pose error: -- mm / -- deg", height=18,
                                style={"font_size": 14, "color": COL_ACTIVE})
                            self._widgets["asm_weld"] = ui.Label(
                                "weld: none", height=18,
                                style={"font_size": 14, "color": COL_TEXT})
                            with ui.HStack(spacing=4, height=24):
                                ui.Button("Force weld",
                                          clicked_fn=lambda: self._stub("Force weld"))
                                ui.Button("Break weld",
                                          clicked_fn=lambda: self._stub("Break weld"))

                    with ui.CollapsableFrame("Out-feed", collapsed=False):
                        with ui.VStack(spacing=4):
                            with ui.HStack(spacing=10, height=20):
                                self._widgets["of_run"] = ui.Label(
                                    "conveyor: STOPPED", width=150,
                                    style={"font_size": 13})
                                self._widgets["of_head"] = ui.Label(
                                    "head on belt: none", width=130,
                                    style={"font_size": 13})
                            self._widgets["of_trig"] = ui.Label(
                                "in trigger zone: none", height=18,
                                style={"font_size": 13})
                            with ui.HStack(spacing=6, height=22):
                                ui.Label("belt speed (m/s):", width=110,
                                         style={"font_size": 12})
                                spd = ui.FloatField(width=60)
                                spd.model.set_value(0.4)
                                self._widgets["of_speed"] = spd
                            with ui.HStack(spacing=4, height=26):
                                ui.Button("Start conveyor",
                                          clicked_fn=self._on_outfeed_start)
                                ui.Button("Stop conveyor",
                                          clicked_fn=self._on_outfeed_stop)
                                ui.Button("Force delete",
                                          clicked_fn=lambda: self._stub("Force delete"))

                    with ui.CollapsableFrame("Counters", collapsed=False):
                        with ui.VStack(spacing=2):
                            with ui.HStack(spacing=10, height=18):
                                self._widgets["cnt_done"] = ui.Label(
                                    "heads completed: 0", width=150,
                                    style={"font_size": 12})
                                self._widgets["cnt_cycle"] = ui.Label(
                                    "last cycle: -- s", width=120,
                                    style={"font_size": 12})
                            with ui.HStack(spacing=10, height=18):
                                self._widgets["cnt_break"] = ui.Label(
                                    "grip-breaks: 0", width=150,
                                    style={"font_size": 12})
                                self._widgets["cnt_snapfail"] = ui.Label(
                                    "failed snaps: 0", width=120,
                                    style={"font_size": 12})

                    with ui.CollapsableFrame("Event log", collapsed=False):
                        with ui.VStack(spacing=2):
                            log_frame = ui.ScrollingFrame(
                                height=130,
                                horizontal_scrollbar_policy=
                                ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF)
                            self._widgets["log_stack"] = log_frame
                            ui.Button("Clear log", height=20,
                                      clicked_fn=self._clear_log)

                    with ui.CollapsableFrame("Debug / injection", collapsed=True):
                        with ui.VStack(spacing=4):
                            with ui.HStack(spacing=4, height=26):
                                ui.Button("Spawn pair",
                                          clicked_fn=lambda: produce_pair())
                                ui.Button("Teleport-place both halves",
                                          clicked_fn=lambda:
                                          self._stub("Teleport-place both halves"))
                            with ui.HStack(spacing=4, height=24):
                                ui.Label("skip to:", width=55,
                                         style={"font_size": 12})
                                combo = ui.ComboBox(0, *PIPELINE_STATES)
                                self._widgets["skip_combo"] = combo
                                ui.Button("Go", width=40,
                                          clicked_fn=self._skip_stub)

        self._rebuild_log()

    def _skip_stub(self):
        idx = self._widgets["skip_combo"].model.get_item_value_model().get_value_as_int()
        self._stub(f"Skip to state: {PIPELINE_STATES[idx]}")


# =================================================================
# 執行所有步驟
# =================================================================
try:
    _panel.conveyor.stop_all()
    _panel.window.visible = False
except NameError:
    pass

build_layout()                       # 步驟 1
add_physics()                        # 步驟 2

# 每支手臂的可達距離＋淨空間隙報告（KR 10 R1440 約 1.44 公尺可達距離）
print("-" * 64)
_pool_x_near = {"L": CPOOL_CENTER_X + (CPOOL_HALF_W + CPOOL_WALL_T),
                "R": CPOOL_CENTER_X + (CPOOL_HALF_W + CPOOL_WALL_T)}
for pid, sign in (("L", +1.0), ("R", -1.0)):
    ax, ay = ARM_POSITIONS[pid]
    print(f"arm {pid} base: ({ax:+.3f}, {ay:+.3f})")
    clearance = ax - _pool_x_near[pid]
    cflag = "  <-- may clip pool wall!" if clearance < 0.3 else ""
    print(f"  X-clearance to own pool's outer wall: {clearance:.2f} m{cflag}")
    for name, tx, ty in (
        ("own collection pool",   CPOOL_CENTER_X, sign * LINE_Y),
        ("own place target",      0.0,            sign * PLACE_Y_OFF),
        ("partner place target",  0.0,           -sign * PLACE_Y_OFF),
    ):
        d = math.hypot(tx - ax, ty - ay)
        flag = "  <-- OVER 1.44 m REACH!" if d > 1.44 else ""
        print(f"  -> {name:22s} {d:.2f} m{flag}")
print("-" * 64)
_panel = StateMachinePanel()         # 步驟 4（面板；用到前面步驟 3 的函式）
start_settle_tracking()              # 步驟 5a（零件靜止判定機制）
init_arms()                          # 步驟 5b（從場景校準正向運動學）
_panel.log("factory ready: layout + physics + production + conveyors")
_panel.log("press PLAY, then Produce Part / Belt: ON to test the lines")

print("[3/4] production functions ready (wired to panel buttons)")
print("[4/4] state machine panel ready")
print("=" * 64)
print("ONE-SHOT SETUP COMPLETE")
print("  wired for real: Produce Part, Spawn one, Spawn pair, Clear all")
print("                  parts, Belt toggles, Start/Stop conveyor")
print("  wired for real: ...also Deliver, delete, Start/Stop (auto-loop)")
print("  still stubs:    Pause, Reset, Force weld/release/Retry")
print("=" * 64)
