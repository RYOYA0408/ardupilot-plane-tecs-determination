-- 必要な固定変数を取得/設定
local k_throttle = 70                           -- スロットルチャンネル番号
local gravity = 9.80665                         -- 重力加速度

-- 研究室基準のパラメータ初期値
local arspd_min_lab = 20                        -- 研究室での最小速度デフォルト値
local arspd_max_lab = 30                        -- 研究室での最大速度デフォルト値
local arspd_cruise_lab = 25                     -- 研究室での巡航速度
local trim_thr_lab = 65                         -- 研究室でのトリムスロットル率デフォルト値
local thr_slr_lab = 33                          -- 研究室でのスロットルスルーレート

-- TECS Tune パラメータ初期値
local arspd_cruise = 25                         -- 目標巡航速度
local set2up_arspd = 28                         -- 加速フェーズにおける加速後速度
local thr_min = 10                              -- 最小スロットル率 Phase 1, 2, 5, 6 用
local thr_max = 100                             -- 最大スロットル率
local temp_thr_min = 10                         -- Phase 4 用の一時的な最小スロットル率（上昇前の速度調整を柔軟にするため）
local temp_thr_slr = 20                         -- Phase 5 用の一時的なスロットルスルーレート
local ttc = 5.0                                 -- Phase5用の TECS_TIME_CONST

-- MissionPlanner 関連の定義 (Plan name : 2025.06.26_TecsTuning_and_LAND_develop_4.6.0.waypoints)
--local last_nav_index = -1         -- -1 : 前回のミッションインデックス記録用 (初期化用変数, 数字には意味無い)
local default_circle_cmd = 3      -- 3  : デフォルトの周回飛行コマンド番号（高度 80 m）
local default_circle_cmd2 = 2     -- 2  : 初期周回 WP
local accel_sdby_cmd_num = 5      -- 5  : 上昇飛行直前の加速フェーズ"スタンバイ"のコマンド番号
local dojump_aft_cmd_num = 8      -- 8  : 上昇後に到達するコマンド番号
--local dojump_aft_cmd_num2 = 10    -- 10 : 上昇飛行後のdo_jumpを検知（滑空飛行を開始位置を明示）
local tkoff_before_cmd = 6        -- 6  : 上昇飛行中断後に到達するコマンド番号
local tkoff_cmd_num = 10          -- 11 : 上昇飛行コマンドの代替となる TAKEOFF コマンドの番号
local phase5_spdw_cmd = 12        -- 12 : 速度配分を変更する基準ミッション番号
local phase5_mes_cmd = 13         -- 13 : Phase 5 の測定開始合図コマンド

-- 時間定義
local FREQUENCY = 20              -- コードのサンプリング周波数(ms) 20251028変更
local dt = FREQUENCY / 1000       -- 定常性評価時の刻み幅
local dt_log = 0.02               -- UAV Log Viewer のログ周期 (Ver4.3.8 時のログ周期)
local obs_time = 4.0              -- AIRSPEED_MIN/MAX 決定時の観測時間
local obs_sink_time = 3.0         -- 定常滑空フェーズの観測時間
local obs_climb_time = 3.5        -- 上昇フェーズ観測時間

-- Phase 毎に定常性評価バッファーを初期化する際の変数宣言
local arspd_ctx, accel_ctx, dh_ctx, alt_ctx

-- ー目標パラメータ・許容誤差ー
local scale = math.sqrt(dt / dt_log)  -- UAV Log Viewer から取得した諸々の閾値を実験用にスケーリングする係数

-- 20241218 の実験データから算出した閾値 (dt_log 秒周期で計算したデータ値)
local arspd_sigma_log = 0.52--0.519403471             -- 速度不偏標準偏差
local slope_sigma_log = 0.55--0.548936579             -- 加速度不偏標準偏差
local dh_sigma_log = 0.76--0.760163022                -- 上昇率不偏標準偏差
local height_sigma_log = 0.71--0.707958572            -- 高度不偏標準偏差
-- ノイズ有り状態変数生成用不偏標準偏差
local arspd_std = arspd_sigma_log * scale
local slope_std = slope_sigma_log * scale
local dh_std = dh_sigma_log * scale
local height_std = height_sigma_log * scale

-- 定常性評価用マージン : dt_log [s] 間隔のデータの MAE
local margin_scale = 1.0
local arspd_mae = 0.53--0.53273493
local slope_mae = 0.55--0.5515607
local dh_mae = 0.61--0.60717419
local height_mae = 0.61--0.60958849
-- マージン値を 本スクリプトで使用できるようにスケーリング
local arspd_margin = arspd_mae * scale * margin_scale       -- 許容速度誤差
local accel_margin = slope_mae * scale * margin_scale       -- 許容加速度誤差
local dh_margin = dh_mae * scale * margin_scale             -- 許容上昇率誤差
local height_margin = height_mae * scale * margin_scale     -- 許容目標巡航高度誤差
local roll_margin = 10.0                                    -- Phase 5 における許容ロール角
-- 絶対目標値
local height_fbw_target = 80.0                              -- AIRSPEED_MAX/MIN 決定時の目標巡航高度
local accel_target = 0                                      -- 機首方向加速度目標値
local height_target = 50.0                                  -- 目標巡航高度 (ミッションに依存)
local dh_target = 0.0                                       -- 目標巡航上昇率

-- AIRSPEED_MIN 決定フェーズの
-- タイムアウト設定 & 減速速度テーブルリスト
local decel_timeout = 20
local decel_start_ts = nil
local decel_time = nil
local decel_cnt = 0
local decel_arspd_max = 24
local decel_arspd_min = 16  -- 実験から値を制限/実験では16
local decel_arspd_table = {}
-- テーブル生成
for d = decel_arspd_max, decel_arspd_min, -1 do
    table.insert(decel_arspd_table, d)
end

-- AIRSPEED_MAX 決定フェーズの
-- タイムアウト設定 & 加速速度テーブルリスト
local accel_timeout = decel_timeout
local accel_start_ts = nil
local accel_time = nil
local accel_cnt = 0
local accel_arspd_min = 26
local accel_arspd_max = 35 -- 実験から値を制限/実験では34
local accel_arspd_table = {}
-- テーブル生成
for a = accel_arspd_min, accel_arspd_max, 1 do
    table.insert(accel_arspd_table, a)
end

-- 上昇フェーズの高度目標値
local target_alt = 140                           -- 目標上昇高度
local target_alt_limit = target_alt - 5.0        -- 上昇運動強制終了高度

-- 上昇フェーズにおける上昇回数初期化 & 回数制限 & 初期上昇ピッチ角
local climb_cnt = 0         -- 上昇回数の初期化
local climb_cnt_max = 5
local next_pitch_cmd = 23

-- 滑空飛行（Phase 5）の最低高度リミット
--local phase5_spdw_start_h = 120
local phase5_height_limit = 30
local phase5_cnt = 0
local phase5_cnt_max = 3

-- TECS チューニング用スイッチ ch
local scripting_rc = rc:find_channel_for_option(300)

-- フラグ定義
local tecstune_running = false
local deceleration_sw = false
local decel_transition = false
local acceleration_sw = false
local accel_transition = false
local trimspd_sw = false
local climb_sw = false
local climb_rotate = false
local get_climb_rate_sw = false
local phase4_init_sw = false
local phase4_finish_sw = false
local pitch_min_avo_sw = false
local spdw_sw = false
local phase5_start_sw = false
local sink_trans_sw = false
local cruise_trans_sw = false

-- チューニングフェーズ
-- 0  : 未実行
-- 1  : ARISPEED_MIN 決定フェーズ
-- 2  : ARISPEED_MAX 決定フェーズ
-- 2.5: 2 ~ 3 遷移待機フェーズ
-- 3  : 加速フェーズ
-- 4  : 最大上昇率&(最大・最小ピッチ角, 理論最大降下率)の決定フェーズ
-- 5  : 最小降下率の決定フェーズ
-- 6  : トリムスロットル率決定フェーズ
local tecstune_phase = 0

-- パラメータ一括リセット関数
function reset_all_param(tecstune_phase)
    -- TecsTuning 中に一時変更するパラメータ
    param:set_and_save("AIRSPEED_CRUISE", 25)
    param:set_and_save("TKOFF_THR_SLEW", 20)
    param:set_and_save("TKOFF_THR_DELAY", 2)
    param:set_and_save("TECS_SPDWEIGHT", 1)
    param:set_and_save("PTCH_LIM_MAX_DEG", 30)
    param:set_and_save("PTCH_LIM_MIN_DEG", -20)
    param:set_and_save("TKOFF_ROTATE_SPD", 18)
    param:set_and_save("THR_MIN", thr_min)
    --param:set_and_save("TECS_TIME_CONST", 5)
    param:set_and_save("SCR_USER6", 0)  -- フルパラメータリストにおけるTECS Tuning の ON/OFF フラグ

    if tecstune_phase == 1 then
        param:set_and_save("TRIM_THROTTLE", trim_thr_lab)
        param:set_and_save("AIRSPEED_MIN", arspd_min_lab)
    elseif tecstune_phase == 2 then
        param:set_and_save("TRIM_THROTTLE", trim_thr_lab)
        param:set_and_save("AIRSPEED_MAX", arspd_max_lab)
    elseif tecstune_phase == 3 or tecstune_phase == 4 or tecstune_phase == 5 then
        param:set("THR_MIN", thr_min)
        param:set("TRIM_THROTTLE", trim_thr_lab)
        param:set("THR_SLEWRATE", thr_slr_lab)
    elseif tecstune_phase == 6 then
        param:set("TRIM_THROTTLE", trim_thr_lab)
    end
end

function param_cleanup()
    param:set_and_save("THR_MAX", 100)
    param:set_and_save("THR_MIN", thr_min)
    param:set("THR_SLEWRATE", thr_slr_lab)
    param:set_and_save("TRIM_THROTTLE", 65)
    param:set_and_save("AIRSPEED_MAX", 30)
    param:set_and_save("AIRSPEED_MIN", 20)
    param:set_and_save("TECS_PITCH_MAX", 30)
    param:set_and_save("TECS_PITCH_MIN", -30)
    param:set_and_save("PTCH_LIM_MIN_DEG", -30)
    param:set_and_save("TECS_CLMB_MAX", 5)
    param:set_and_save("TECS_SINK_MIN", 2)
    param:set_and_save("TECS_SINK_MAX", 20) -- 10 から大きくしてみた
    param:set_and_save("SCR_USER6", 1)
end

-- フラグ一括リセット関数
function reset_all_flags(tecstune_phase)
    trimspd_sw = false
    if tecstune_phase == 1 then
        deceleration_sw = false
        decel_transition = false
    elseif tecstune_phase == 2 then
        acceleration_sw = false
        accel_transition = false
    elseif tecstune_phase == 3 or tecstune_phase == 4 then
        if not phase4_finish_sw then
            climb_sw = false
            climb_rotate = false
            get_climb_rate_sw = false
            phase4_finish_sw = false
        else
            climb_sw = false
            climb_rotate = false
            get_climb_rate_sw = true
            phase4_finish_sw = true
        end
    elseif tecstune_phase == 5 then
        spdw_sw = false
        phase5_start_sw = false
        sink_trans_sw = false
    else
        cruise_trans_sw = false
        pitch_min_avo_sw = false
    end
end

-- 途中でフェーズを中断した際に復帰する際の各フェーズパラメータセット関数群
function phase1_set_param()
    param:set("AIRSPEED_MIN", decel_arspd_min)
    param:set("TRIM_THROTTLE", thr_min)
    gcs:send_text(6, string.format("AIRSPEED_MIN set temporarily to: %.2f m/s", param:get("AIRSPEED_MIN")))
    gcs:send_text(6, string.format("TRIM_THROTTLE set temporarily to: %.2f%%", param:get("TRIM_THROTTLE")))
    gcs:send_text(6, "Start get to AIRSPEED_MIN (Phase 1)")
end

function phase2_set_param()
    param:set("AIRSPEED_MAX", accel_arspd_max)
    param:set("TRIM_THROTTLE", thr_max)
    gcs:send_text(6, string.format("AIRSPEED_MAX set temporarily to: %.2f m/s", param:get("AIRSPEED_MAX")))
    gcs:send_text(6, string.format("TRIM_THROTTLE set temporarily to: %.2f%%", param:get("TRIM_THROTTLE")))
    gcs:send_text(0, "Start get AIRSPEED_MAX (Phase 2)")
end

function phase2_5_set_param()
    param:set("TRIM_THROTTLE", trim_thr_lab)
    param:set("TECS_SPDWEIGHT", 1.0)
    --param:set("THR_SLEWRATE", temp_thr_slr)
    gcs:send_text(6, string.format("TRIM_THROTTLE set to: %.2f%%", param:get("TRIM_THROTTLE")))
    gcs:send_text(6, string.format("TECS_SPDWEIGHT set to: %d", param:get("TECS_SPDWEIGHT")))
    --gcs:send_text(6, string.format("THR_SLEWRATE set to: %.2f%%", param:get("THR_SLEWRATE")))
    if not phase4_init_sw then
        param:set("AIRSPEED_CRUISE", arspd_cruise)
        param:set("TECS_PITCH_MAX", 0)
        param:set("TECS_PITCH_MIN", 0)
        param:set("TKOFF_THR_DELAY", 0)
        param:set("TKOFF_ROTATE_SPD", 25)
        param:set("TKOFF_THR_SLEW", 100)
        param:set("THR_MIN", temp_thr_min)
        gcs:send_text(6, string.format("AIRSPEED_CRUISE set to: %.2f m/s", param:get("AIRSPEED_CRUISE")))
        gcs:send_text(6, string.format("TECS_PITCH_MAX set to: %.2f deg.", param:get("TECS_PITCH_MAX")))
        gcs:send_text(6, string.format("TECS_PITCH_MIN set to: %.2f deg.", param:get("TECS_PITCH_MIN")))
        gcs:send_text(6, string.format("TKOFF_THR_DELAY set to: %.2f ds", param:get("TKOFF_THR_DELAY")))
        gcs:send_text(6, string.format("TKOFF_ROTATE_SPD set to: %.2f m/s", param:get("TKOFF_ROTATE_SPD")))
        gcs:send_text(6, string.format("TKOFF_THR_SLEW set to: %.2f %%/s", param:get("TKOFF_THR_SLEW")))
        gcs:send_text(6, string.format("THR_MIN set to: %.2f%%", param:get("THR_MIN")))
    end
    if climb_cnt == 0 or (phase5_cnt == 0 and phase4_finish_sw) then
        mission:set_current_cmd(accel_sdby_cmd_num)
    end
    gcs:send_text(0, "Phase 3 Standby")
end

function phase5_set_param()
    param:set("THR_SLEWRATE", temp_thr_slr)
    param:set("THR_MIN", thr_min)
    param:set("TECS_SPDWEIGHT", 1)
    param:set("TECS_PITCH_MIN", -30)
    param:set("TKOFF_THR_SLEW", 20)
    param:set("TKOFF_THR_DELAY", 2)
    --param:set("TECS_TIME_CONST", ttc)
    gcs:send_text(6, string.format("THR_SLEWRATE set to: %.2f%%", param:get("THR_SLEWRATE")))
    gcs:send_text(6, string.format("THR_MIN set to: %.2f%%", param:get("THR_MIN")))
    gcs:send_text(6, string.format("TECS_SPDWEIGHT set to: %d", param:get("TECS_SPDWEIGHT")))
    gcs:send_text(6, string.format("TECS_PITCH_MIN set to: %.2f deg.", param:get("TECS_PITCH_MIN")))
    gcs:send_text(6, string.format("TKOFF_THR_SLEW return to: %.2f %%/s", param:get("TKOFF_THR_SLEW")))
    gcs:send_text(6, string.format("TKOFF_THR_DELAY return to: %.2f ds", param:get("TKOFF_THR_DELAY")))
    --gcs:send_text(6, string.format("TECS_TIME_CONST set to %.1f", param:get("TECS_TIME_CONST")))
    gcs:send_text(0, "Phase 5 Standby")
end

function phase6_set_param()
    param:set("TECS_SPDWEIGHT", 1)
    param:set("TRIM_THROTTLE", trim_thr_lab)
    param:set("THR_MIN", thr_min)
    --param:set("TECS_TIME_CONST", 5)
    gcs:send_text(6, string.format("TECS_SPDWEIGHT set to: %d", param:get("TECS_SPDWEIGHT")))
    gcs:send_text(6, string.format("TRIM_THROTTLE return to: %.2f%%", param:get("TRIM_THROTTLE")))
    gcs:send_text(6, string.format("THR_MIN return to: %.2f%%", param:get("THR_MIN")))
    --gcs:send_text(6, string.format("TECS_TIME_CONST return to: %.2f", param:get("TECS_TIME_CONST")))
    gcs:send_text(0, "Start get TRIM_THROTTLE (Phase 6)")
    mission:set_current_cmd(default_circle_cmd2)
end

--[[
-- Phase 3 突入時の 次 WP 番号が更新されたことを検出 & 次 WP まで上昇飛行を安全に行える距離があることを確認する関数
-- また, 極端な速度値のズレがある場合は Phase 3 に移行せず +1 周回する
function check_jump_trigger_with_margin(after_index)
    local current_index = mission:get_current_nav_index()   -- 現在のインデックスを取得
    local arspd_target = param:get("AIRSPEED_CRUISE")  -- 現在の目標速度
    local arspd_max = set2up_arspd                     -- 上昇時に想定される最高速度
    
    -- ミッションインデックスの前後を監視することで, WP4 → WP2 に移行したことを検知した場合の処理
    if current_index == after_index and last_nav_index ~= after_index and last_nav_index ~= 0 then
        -- 現在の WP インデックスを取得して前回値に格納
        last_nav_index = after_index

        -- ーPhase 4 (上昇フェーズ) に必要な 次 WP までのマージン距離を算出ー
        -- 上昇回数を参照して任意回数目の上昇ピッチ角を取得
        local pitch_cmd = next_pitch_cmd
        -- 上昇時に想定される最大上昇率 & 上昇高度の変化量から上昇時間を概算
        local dh_max = 10.0
        local climb_alt = target_alt_limit - height_target
        local climb_time = climb_alt / dh_max
        -- 余弦方向の速度ベクトル & Phase 3 で想定される平均加速度
        local vx = arspd_max * math.cos(math.rad(pitch_cmd))
        local accel = 1.0
        -- Phase 3 における想定加速距離
        local accel_dis = (arspd_max*arspd_max - arspd_target*arspd_target) / (2 * accel)
        -- 最終的に上昇に必要とされるマージン距離の算出
        local next_wp_dis_margin = vx * climb_time + accel_dis
        
        -- ー上昇フェーズの次に到達する WP との水平距離を計算ー
        -- 上昇終了後に到達する mission item & ロケーション情報を取得
        local next_wp_item = mission:get_item(after_index)
        local now = ahrs:get_location()
        -- nil check
        if next_wp_item and now then
            -- 次 WP までの 3 軸距離を取得 (上昇前は, z方向の変位はゼロに近いと仮定) 
            local next_wp_loc = Location()
            next_wp_loc:lat(next_wp_item:x())
            next_wp_loc:lng(next_wp_item:y())
            next_wp_loc:alt(math.floor(next_wp_item:z()*100))
            -- 次 WP 中心位置 と　現在位置の直線距離
            local raw_dist = now:get_distance(next_wp_loc)
            -- 次 WP (TurnPoint) の旋回半径を取得
            local next_wp_radius = next_wp_item:param3()
            -- 三平方の定理から 余弦方向の距離を計算
            local dis2wp = math.sqrt(raw_dist*raw_dist - next_wp_radius*next_wp_radius)

            -- GCS に 次 WP までの距離と任意の上昇時におけるマージン値の結果を送信
            gcs:send_text(6, string.format("Distance to Next WP: %.1f m, Required margin: %.1f m", dis2wp, next_wp_dis_margin))
            
            -- 上記条件をすべて満たせば true を返す
            return dis2wp >= next_wp_dis_margin
        else
            return false
        end
    end
    --  WP4 → WP2 移行以外はミッションインデックスの監視のみ
    last_nav_index = current_index
    return false
end
]]--

-- WP 毎に前回の NAV index を保有するテーブル
local last_nav_index_map = {}

-- 単純に指定の WP（after_index）に目標 WP が切り替わった瞬間に検知する関数
function check_jump_trigger(after_index)
    -- 現在のインデックスを取得
    local current_index = mission:get_current_nav_index()
    local last = last_nav_index_map[after_index]
    if last == nil then
        last_nav_index_map[after_index] = current_index
    end
    -- 初回呼び出し時の誤検知防止
    --last_nav_index = last_nav_index or current_index
    -- 直前は after_index ではなく, 今 after_index になったらトリガー
    local trigger = (current_index == after_index and last ~= after_index)
    -- 次回比較用に更新
    --last_nav_index = current_index
    last_nav_index_map[after_index] = current_index

    return trigger
end

-- 現在の基本状態変数取得関数 (飛行実験で本スクリプトを!!!動作させる時は noise を除外!!!)
--[[
-- 飛行実験時はこれを使う
-- @return pitch_deg, arspd_now, accel_x_now, dh_now, height_now, throttle_now
function update_sensors()
    local pitch_deg = math.deg(ahrs:get_pitch())                                     -- ピッチ角
    local arspd_now = ahrs:airspeed_estimate()                                       -- 速度
    local accel_x_now = (ahrs:get_accel():x() - gravity*math.sin(ahrs:get_pitch()))  -- 機種方向加速度
    local dh_now =  - ahrs:get_velocity_NED():z()                                    -- 右手系 z 軸下向きの速度ベクトル (上昇率)
    local height_now =  - ahrs:get_relative_position_NED_home():z()                  -- 地面からの相対高度 (下向き正)
    local throttle_now = SRV_Channels:get_output_scaled(k_throttle)                  -- スロットル率
    return pitch_deg, arspd_now, accel_x_now, dh_now, height_now, throttle_now
end
]]--

-- Box-Muller 法によるガウシアンノイズ生成関数
function noise(sigma)
    local a, b
    -- a > 0 を保証して z の計算オーバーフローを防止
    repeat
        a = math.random()
    until a > 0
    b = math.random()
    local z = math.sqrt(-2*math.log(a)) * math.cos(2*math.pi*b)
    return z * sigma
end

-- シミュレーション時は以下(ノイズ生成関数を含む)を使う
-- @return pitch_deg, arspd_now, accel_x_now, dh_now, height_now, throttle_now
function update_sensors()
    local pitch_deg = math.deg(ahrs:get_pitch())                                                        -- ピッチ角
    local arspd_now = ahrs:airspeed_estimate() + noise(arspd_std)                                       -- 速度
    local accel_x_now = (ahrs:get_accel():x() - gravity*math.sin(ahrs:get_pitch())) + noise(slope_std)  -- 機種方向加速度
    local dh_now =  - ahrs:get_velocity_NED():z() + noise(dh_std)                                       -- 右手系 z 軸下向きの速度ベクトル (上昇率)
    local height_now =  - ahrs:get_relative_position_NED_home():z() + noise(height_std)                 -- 地面からの相対高度 (下向き正)
    local throttle_now = SRV_Channels:get_output_scaled(k_throttle)                                     -- スロットル率
    return pitch_deg, arspd_now, accel_x_now, dh_now, height_now, throttle_now
end

-- 定常性評価パラメータリセット関数
function make_stab_ctx(obs_time, dt)
    local buff_size = math.floor(obs_time / dt) -- バッファーサイズ (整数)
    return{
        buff_size = buff_size,
        val_buff = {},
        mae_sum = 0,
    }
end

-- 定常性評価関数
-- 監視対象値の平均絶対誤差 <= 実験データ各値の MAE (平均絶対誤差)
-- であれば true を返す
-- @global param  dt         観測刻み幅
-- @global param  buff_size  観測時間 / dt
-- @param  ctx.              評価対象情報 (make_stab_ctx(obs_time, dt))
-- @param  now_val           現在の値 (例：arspd_now)
-- @param  target_val        目標値 (例：AIRSPEED_CRUISE * 0.01)
-- @param  err_margin        値の許容誤差 (例：arspd_margin)
-- @return boolean           stable = mae <= err_margin (true or false)
function val_stab_check(ctx, now_val, target_val, err_margin)
    -- 値を随時バッファに記録 & サイズを超えたら古いデータから削除
    local vb = ctx.val_buff
    local abs_err_new = math.abs(now_val - target_val)
    table.insert(vb, now_val)
    ctx.mae_sum = ctx.mae_sum + abs_err_new
    -- バッファサイズ超過時は、古い値の絶対誤差を差し引く
    if #vb > ctx.buff_size then
        local old = table.remove(vb, 1)
        local abs_err_old = math.abs(old - target_val)
        ctx.mae_sum = ctx.mae_sum - abs_err_old
    end

    -- バッファ内データ数の充填を確認
    if #vb < ctx.buff_size then
        return false
    end

    -- MAEを計算
    local mae = ctx.mae_sum / ctx.buff_size

    return mae <= err_margin
end


function tecstuning(sw)

    -- プロポスイッチ ON でスクリプト有効化 (初回限定)
    if sw and not tecstune_running then
        gcs:send_text(0, string.format("TECS Tune Switch ON (Script Standby)"))
        -- 途中中断した場合, 前回のフェーズを保存している "SCR_USER1" の値から任意フェーズに復帰
        local saved_phase = param:get("SCR_USER1")
        local saved_decel_cnt = param:get("SCR_USER2")
        local saved_accel_cnt = param:get("SCR_USER3")
        if saved_phase and saved_phase > 1 then
            tecstune_phase = saved_phase
            if tecstune_phase == 2 then
                phase2_set_param()
                if saved_accel_cnt and saved_accel_cnt > 1 then
                    accel_cnt = saved_accel_cnt - 1
                end
            
            -- Phase 3, 4 は連携する必要があるため一括で Phase 2.5 から開始するよう調整
            elseif tecstune_phase == 2.5 or tecstune_phase == 3 or tecstune_phase == 4 then
                if phase5_cnt > 0 or (phase5_cnt == 0 and phase4_finish_sw) then
                    climb_sw = false
                    get_climb_rate_sw = true
                    phase4_finish_sw = true
                    spdw_sw = false
                    phase5_start_sw = false
                    sink_trans_sw = false
                    gcs:send_text(0, string.format("Phase 5 Standby (skip Phase 4)"))
                end
                phase2_5_set_param()
                tecstune_phase = 2.5
            
            -- Phase 5 では, Phase 3, 4 を再利用(Phase 4 時の測定は Skip)して滑空飛行を再度行う
            elseif tecstune_phase == 5 then
                gcs:send_text(0, string.format("Phase 5 Standby (skip Phase 4)"))
                climb_sw = false
                get_climb_rate_sw = true
                phase4_finish_sw = true
                phase2_5_set_param()
                tecstune_phase = 2.5
            
            elseif tecstune_phase == 6 then
                phase6_set_param()
            end
            
            gcs:send_text(0, string.format("Resuming TECS tuning from saved Phase %s", tecstune_phase))
        else
            param_cleanup()
            gcs:send_text(6, string.format("Parameters Cleanup"))
            
            phase1_set_param()
            if saved_decel_cnt and saved_decel_cnt > 1 then
                decel_cnt = saved_decel_cnt - 1
                gcs:send_text(0, string.format("Resuming TECS tuning from saved Phase %s", tecstune_phase))
            end
            tecstune_phase = 1
            
            -- 下記を適切にコメントアウトして最初から強制的に任意フェーズから始められるようにできるオプション
            --[[
            -- Phase 2 からスタート
            phase2_set_param()
            if saved_accel_cnt and saved_accel_cnt > 1 then
                accel_cnt = saved_accel_cnt - 1
            end
            tecstune_phase = 2
            ]]--
            --[[
            -- phase 3 → 4 からスタート
            phase2_5_set_param()
            tecstune_phase = 2.5
            ]]--
            --[[
            -- Phase 5からスタート
            phase2_5_set_param()
            gcs:send_text(0, string.format("Phase 5 Standby（skip Phase 4）"))
            climb_sw = false
            get_climb_rate_sw = true
            phase4_finish_sw = true
            tecstune_phase = 2.5
            ]]--
            --[[
            -- Phase 6 からスタート
            phase6_set_param()
            tecstune_phase = 6
            ]]--
        end
        tecstune_running = true
    end

    -- スイッチ ON 中かつ 任意の待機フェーズ中に DO_JUMP を検知したら指定フェーズに移行
    if tecstune_running then
        -- 上昇フェーズ待機時の処理
        if tecstune_phase == 2.5 then
            if check_jump_trigger(dojump_aft_cmd_num) then
                local _, arspd_now, _, _, _ = update_sensors()
                local arspd_err = math.abs(arspd_now - param:get("AIRSPEED_CRUISE"))
                -- AIRSPEED_MAX 取得直後の上昇フェーズ移行を防止
                if (arspd_err <= 3) or (arspd_now <= arspd_cruise) then
                    tecstune_phase = 3
                    gcs:send_text(6, "Start Acceleration (Phase 3)")
                else
                    gcs:send_text(6, "Over speed")
                    gcs:send_text(6, "Phase 3 Abort, Wait for next lap")
                end
            end
        end
    end

    -- プロポスイッチを OFF にした時の処理
    if not sw then
        if tecstune_running then
            -- 現在のフェーズ番号を記録
            param:set("SCR_USER1", tecstune_phase)
            gcs:send_text(6, string.format("Saved Phase %s to SCR_USER1", tecstune_phase))
            -- 減速&加速フェーズ中断時の減速&加速回数を記録
            if tecstune_phase == 1 and decel_cnt ~= 0 then
                param:set("SCR_USER2", decel_cnt)
                gcs:send_text(6, string.format("Saved decel count %s to SCR_USER2", decel_cnt))
            elseif tecstune_phase == 2 and accel_cnt ~= 0 then
                param:set("SCR_USER3", accel_cnt)
                gcs:send_text(6, string.format("Saved accel count %s to SCR_USER3", accel_cnt))
            end

            -- 中断時, phase=3, 4, 5 の場合は MP2
            -- 中断時, phase=2.5 の場合は MP3
            -- 中断時, 上記以外は、何もしないで既存の MP に任せる
            if tecstune_phase == 3 or tecstune_phase == 4 or tecstune_phase == 5 then
                mission:set_current_cmd(default_circle_cmd2)
                gcs:send_text(0, string.format("Jump to Mission: %d WP", default_circle_cmd2))
            elseif tecstune_phase == 2.5 then
                mission:set_current_cmd(default_circle_cmd2)
                gcs:send_text(0, string.format("Jump to Mission: %d WP", default_circle_cmd2))
            end

            -- Phase 4 → 5 に遷移後に TECS_PITCH_MIN を書き換えたため Phase 4 で決定された TECS_PITCH_MIN に戻す
            if pitch_min_avo_sw then
                local phase4_pitch_min = param:get("SCR_USER4")
                param:set_and_save("TECS_PITCH_MIN", phase4_pitch_min)
                gcs:send_text(6, string.format("Return to TECS_PITCH_MIN from SCR_USER4:%.2f deg.", phase4_pitch_min))
            end

            reset_all_param(tecstune_phase)
            gcs:send_text(0, string.format("TECS Tune Switch OFF"))
        end
        
        -- sw 管理 (TecsTune Switch OFF で, すべて false)
        reset_all_flags(tecstune_phase)
        tecstune_running = false
        tecstune_phase = 0
        --decel_cnt = 0
        --accel_cnt = 0
        --phase4_init_sw = false
        --climb_cnt = 0
        --phase5_cnt = 0
    end

    -- ARISPEED_MIN(AIRSPEED_MIN) 決定フェーズ
    if tecstune_phase == 1 then
        -- 必要状態量取得
        local pitch_deg, arspd_now, accel_x_now, dh_now, _, throttle_now = update_sensors()
        -- 減速回数に応じた目標速度値をテーブルから取り出す
        local arspd_cmd = decel_arspd_table[math.min(decel_cnt + 1, #decel_arspd_table)]
        local total_decel_cnt = #decel_arspd_table
        
        -- 減速後の目標速度を設定
        if not deceleration_sw then
            -- トリム速度値を変更
            param:set("AIRSPEED_CRUISE", arspd_cmd)
            gcs:send_text(6, string.format("Set AIRSPEED_CRUISE to: %d m/s", param:get("AIRSPEED_CRUISE")))
            -- 減速回数を加算
            decel_cnt = decel_cnt + 1
            gcs:send_text(6, string.format("Deceleration Count: %d", decel_cnt))
            -- 速度指令値が decel_arspd_min になった瞬間強制的にPhase2へ
            if arspd_cmd == decel_arspd_min then
                -- "現在のインデックス"のトリム速度設定値に対して 20 % 大きい値を arspd_min と決定 を廃止
                -- "現在のインデックスに対して1つ前"のトリム速度設定値 を arspd_min と決定
                local arspd_min_idx = math.max(1, decel_cnt)
                --local arspd_min = decel_arspd_table[arspd_min_idx] * 1.20
                local arspd_min = decel_arspd_table[arspd_min_idx]
                -- GCS へ結果を送信
                gcs:send_text(6, "Phase 1 Abort")
                gcs:send_text(6, "Parameters determined forcibly")
                gcs:send_text(6, string.format("AIRSPEED_MIN get to %.2f m/s", arspd_min))
                param:set_and_save("AIRSPEED_MIN", arspd_min)
                -- 次フェーズのためのパラメータ設定
                phase2_set_param()
                tecstune_phase = 2
            end

            -- 定常評価 ctx (監視する状態量のリストや観測時間) の初期化
            arspd_ctx = make_stab_ctx(obs_time, dt)
            accel_ctx = make_stab_ctx(obs_time, dt)
            dh_ctx = make_stab_ctx(obs_time, dt)
            -- 開始時刻を記録 & 速度変更フラグを true
            decel_start_ts = millis():tofloat()
            deceleration_sw = true
        end

        -- 減速遷移 transition 確認
        if deceleration_sw and not decel_transition then
            local decel_err = math.abs(arspd_cmd - arspd_now)
            if decel_err < arspd_margin and arspd_now <= arspd_cmd then
                -- 開始時刻を記録 & 速度変更フラグを true
                decel_start_ts = millis():tofloat()
                gcs:send_text(6, "Deceleration Transition Complete")
                decel_transition = true
            elseif (millis():tofloat() - decel_start_ts) > 30000 then
                gcs:send_text(0, "Transition timeout, Forcing next step.")
                decel_transition = true
                --decel_start_ts = millis():tofloat()
            end
        end

        -- 定常性評価
        if deceleration_sw and decel_transition then
            -- 記録時間の更新 & 目標速度を取得
            decel_time = (millis():tofloat() - decel_start_ts) * 0.001
            local arspd_target = param:get("AIRSPEED_CRUISE")
            --local accel_target = gravity * math.sin(math.rad(pitch_deg))

            -- 速度
            local arspd_stable = val_stab_check(arspd_ctx, arspd_now,
                                                arspd_target, arspd_margin)

            -- 加速度
            local accel_x_stable = val_stab_check(accel_ctx, accel_x_now,
                                                 accel_target, accel_margin)


            -- 上昇率
            local dh_stable = val_stab_check(dh_ctx, dh_now,
                                             dh_target, dh_margin)
            
            -- 要求速度で水平定常飛行を達成できた場合, 現在の要求速度より - 1 m/s してループ
            if arspd_stable and accel_x_stable and dh_stable then 
                gcs:send_text(6, string.format("Stall not confirmed"))
                gcs:send_text(0, string.format("Retry Phase 1"))
                decel_start_ts = 0  -- 記録開始時間を初期化
                decel_time = 0      -- 記録時間を初期化
                deceleration_sw = false
                decel_transition = false
            end

            -- 任意時間以上, 定常性を確保できなかった場合, AIRSPEED_MIN を決定
            --if (decel_time >= decel_timeout or decel_cnt >= total_decel_cnt) and throttle_now <= 20 then
            --if decel_time >= decel_timeout and throttle_now <= 20 and throttle_now >= 10 then
            if decel_time >= decel_timeout then
                -- "現在のインデックス"のトリム速度設定値に対して 20 % 大きい値を arspd_min と決定 を廃止
                -- "現在のインデックスに対して1つ前"のトリム速度設定値 を arspd_min と決定
                local arspd_min_idx = math.max(1, decel_cnt - 1)
                local arspd_min = decel_arspd_table[arspd_min_idx]
                if arspd_min >= arspd_cruise then
                    arspd_min = arspd_cruise
                    gcs:send_text(0, string.format("Phase 1 Abnormal Termination"))
                end
                if throttle_now <= 20 and throttle_now >= 10 then
                    -- GCS へ結果を送信
                    gcs:send_text(6, string.format("AIRSPEED_MIN get to %.2f m/s", arspd_min))
                    param:set_and_save("AIRSPEED_MIN", arspd_min)
                else
                    -- GCS へ結果を送信
                    gcs:send_text(6, "Forcing Phase 1 Time out")
                    gcs:send_text(6, string.format("AIRSPEED_MIN get to %.2f m/s", arspd_min))
                    param:set_and_save("AIRSPEED_MIN", arspd_min)
                end
                -- 次フェーズのためのパラメータ設定
                phase2_set_param()
                tecstune_phase = 2
            end
        end
    end

    -- ARISPEED_MAX(AIRSPEED_MAX) 決定フェーズ
    if tecstune_phase == 2 then
        -- 必要状態量取得
        local pitch_deg, arspd_now, accel_x_now, dh_now, height_now, throttle_now = update_sensors()
        -- 減速回数に応じた目標速度値をテーブルから取り出す
        local arspd_cmd = accel_arspd_table[math.min(accel_cnt + 1, #accel_arspd_table)]
        local total_accel_cnt = #accel_arspd_table
        
        -- 加速後の目標速度を設定
        if not acceleration_sw then
            -- トリム速度値を変更
            param:set("AIRSPEED_CRUISE", arspd_cmd)
            gcs:send_text(6, string.format("Set AIRSPEED_CRUISE to: %d m/s", param:get("AIRSPEED_CRUISE")))
            accel_cnt = accel_cnt + 1
            gcs:send_text(6, string.format("Acceleration Count: %d", accel_cnt))
            -- 定常性評価 ctx の初期化
            arspd_ctx = make_stab_ctx(obs_time, dt)
            accel_ctx = make_stab_ctx(obs_time, dt)
            dh_ctx = make_stab_ctx(obs_time, dt)
            -- 開始時刻を記録 & 速度変更フラグを true
            accel_start_ts = millis():tofloat()
            acceleration_sw = true
            -- 速度指令値が AIRSPEED_MAX になった瞬間強制的にPhase2.5へ
            if arspd_cmd == accel_arspd_max then
                -- "現在のインデックス"のトリム速度設定値に対して -1 m/sを AIRSPEED_MAX と決定
                local arspd_max_idx = math.max(1, accel_cnt - 1)
                local arspd_max = accel_arspd_table[arspd_max_idx]
                -- GCS へ結果を送信
                gcs:send_text(6, "Phase 2 Abort")
                gcs:send_text(6, "Parameters determined forcibly")
                gcs:send_text(6, string.format("AIRSPEED_MAX get to %.2f m/s", arspd_max))
                param:set_and_save("AIRSPEED_MAX", arspd_max)
                -- 次フェーズのためのパラメータ設定
                phase2_5_set_param()
                tecstune_phase = 2.5
            end
        end

        -- 加速遷移 transition 確認
        if acceleration_sw and not accel_transition then
            local accel_err = math.abs(arspd_cmd - arspd_now)
            if accel_err < arspd_margin and arspd_now >= arspd_cmd then
                -- 開始時刻を記録 & 速度変更フラグを true
                accel_start_ts = millis():tofloat()
                gcs:send_text(6, "Acceleration Transition Complete")
                accel_transition = true
            elseif (millis():tofloat() - accel_start_ts) > 30000 then
                gcs:send_text(0, "Transition timeout, Forcing next step.")
                accel_transition = true
                --accel_start_ts = millis():tofloat()
            end
        end

        -- 定常性評価
        if acceleration_sw and accel_transition then
            accel_time = (millis():tofloat() - accel_start_ts) * 0.001
            local arspd_target = param:get("AIRSPEED_CRUISE")
            --local accel_target = gravity * math.sin(math.rad(pitch_deg))

            -- 速度
            local arspd_stable = val_stab_check(arspd_ctx, arspd_now,
                                                arspd_target, arspd_margin)

            -- 加速度
            local accel_x_stable = val_stab_check(accel_ctx, accel_x_now,
                                                accel_target, accel_margin)

            -- 上昇率
            local dh_stable = val_stab_check(dh_ctx, dh_now,
                                            dh_target, dh_margin)
            
            -- 要求速度で水平定常飛行を達成できた場合, 現在の要求速度より + 1 m/s してループ
            if arspd_stable and accel_x_stable and dh_stable then
                gcs:send_text(0, string.format("Retry Phase 2"))
                accel_start_ts = 0
                accel_time = 0
                acceleration_sw = false
                accel_transition = false
            end

            -- 任意時間以上, 定常性を確保できなかった場合, AIRSPEED_MAX を決定
            if (accel_time >= accel_timeout or accel_cnt >= total_accel_cnt) then
                -- "1つ前のインデックス"のトリム速度設定値を arspd_max と決定
                local arspd_max_idx = math.max(1, accel_cnt - 1)
                local arspd_max = accel_arspd_table[arspd_max_idx]
                if throttle_now >= 90 then
                    -- GCS へ結果を送信
                    gcs:send_text(6, string.format("AIRSPEED_MAX get to %.2f m/s", arspd_max))
                    param:set_and_save("AIRSPEED_MAX", arspd_max)
                else
                    -- GCS へ結果を送信
                    gcs:send_text(6, "Forcing Phase 2 Time out")
                    gcs:send_text(6, string.format("AIRSPEED_MAX get to %.2f m/s", arspd_max))
                    param:set_and_save("AIRSPEED_MAX", arspd_max)
                end
                
                -- 次フェーズのためのパラメータ設定
                phase2_5_set_param()
                tecstune_phase = 2.5
            end
        end
    end

    -- 加速フェーズ：上昇飛行直前に AIRSPEED_CRUISE 一時的に 25 → 30 m/s とすることで間接的にスロットル率を上昇させる
    if tecstune_phase == 3 then
        -- 必要状態量取得
        local _, arspd_now, _, _, _, _ = update_sensors()
        local arspd_limit = set2up_arspd
        if not trimspd_sw then
            -- スロットル率の低下を防ぐため, トリムスロットル率を一時的に変更
            param:set("TRIM_THROTTLE", thr_max)
            gcs:send_text(6, string.format("TRIM_THROTTLE set temporarily to: %.2f%%", param:get("TRIM_THROTTLE")))
            param:set("AIRSPEED_CRUISE", set2up_arspd)   --トリム速度値を変更
            gcs:send_text(6, string.format("Set AIRSPEED_CRUISE to: %d m/s", param:get("AIRSPEED_CRUISE")))
            trimspd_sw = true
        end

        --if trimspd_sw and arspd_now < arspd_cruise and arspd_now >= arspd_limit then
        if trimspd_sw and arspd_now >= arspd_limit then
            tecstune_phase = 4
        end
    end

    -- 最大上昇率 & 最大・最小ピッチ角 & 理論最大降下率 決定フェーズ
    if tecstune_phase == 4 then
        -- 必要状態量取得
        local pitch_deg, arspd_now, accel_x_now, dh_now, height_now, throttle_now = update_sensors()
        -- TAKEOFF コマンドを格納しているミッションアイテムを取得
        local item = mission:get_item(tkoff_cmd_num)
        -- 上昇回数に応じてピッチ角テーブルから次の上昇ピッチ角を決定
        local pitch_cmd = math.ceil(next_pitch_cmd * 10) / 10

        -- 上昇させるために 現在のミッション番号を TKOFF ミッション番号へ以降
        if item and not climb_sw then
            -- 機体の最大ピッチ角を再設定して上昇時のピッチ角オーバーシュートを防止
            param:set("PTCH_LIM_MAX_DEG", pitch_cmd)
            gcs:send_text(6, string.format("Pitch up angle set to: %.2f deg.", pitch_cmd))
            
            -- TAKEOFF ミッション内容を上書き変更
            item:command(22)                        -- TAKEOFFコマンド番号
            item:param1(pitch_cmd)                  -- 任意の上昇ピッチ角目標値
            item:z(target_alt)                      -- 高度目標値
            mission:set_item(tkoff_cmd_num, item)   -- TKOFFコマンドの指示内容を上書き
            mission:set_current_cmd(tkoff_cmd_num)  -- TKOFFコマンドへジャンプ
            gcs:send_text(0, "Start climb motion")
            climb_sw = true

            if not phase4_init_sw then
                phase4_init_sw = true
            end
        end
        
        -- @param THR_MAX, TECS_CLIMB_MAX, PTCH_LIM_MAX_DEG 決定フェーズ時の
        -- 上昇中処理 & 速度/加速度の監視
        if climb_sw then
            local climb_max_rate
            
            -- rotation 完了待機
            --if not climb_rotate and pitch_deg >= pitch_rotate_margin and not phase4_finish_sw then
            if not climb_rotate and not phase4_finish_sw then
                -- rotation する前に, トリム速度値を 25 m/s に復帰
                param:set("AIRSPEED_CRUISE", arspd_cruise)
                gcs:send_text(6, string.format("Return Set AIRSPEED_CRUISE to: %d m/s", param:get("AIRSPEED_CRUISE")))
                -- 速度のみ追従するように変更
                param:set("TECS_SPDWEIGHT", 2.0)
                gcs:send_text(6, string.format("TECS_SPDWEIGHT set to: %d", param:get("TECS_SPDWEIGHT")))
                gcs:send_text(6, "Start measurement of Phase 4")
                arspd_ctx = make_stab_ctx(obs_climb_time, dt)
                accel_ctx = make_stab_ctx(obs_climb_time, dt)
                climb_rotate = true
            end

            -- rotation が完了し, 上昇飛行状態での速度/加速度監視
            local arspd_target = param:get("AIRSPEED_CRUISE")
            if climb_sw and climb_rotate and not get_climb_rate_sw then
                -- 速度定常性評価
                local arspd_stable = val_stab_check(arspd_ctx, arspd_now,
                                                    arspd_target, arspd_margin)

                -- 加速度の定常性評価
                local accel_x_stable = val_stab_check(accel_ctx, accel_x_now,
                                                    accel_target, accel_margin)
                
                --gcs:send_text(0, string.format("arspd_stable=%s, accel_x_stable=%s", tostring(arspd_stable), tostring(accel_x_stable)))

                -- 定常性チェック後, @param THR_MAX, TECS_CLIMB_MAX, PTCH_LIM_MAX_DEG を決定
                if accel_x_stable and arspd_stable then
                    climb_max_rate = - ahrs:get_velocity_NED():z()             -- 現在の上昇率を最大上昇率と決定
                    get_climb_rate_sw = true
                    gcs:send_text(6, "Steady-state evaluation Type 1")

                --[[
                -- 速度定常性は確認できないが, 速度が 25 ~ 26.25 (+5%) m/s 内であり, 加速度が定常である場合の処理
                -- @param THR_MAX, TECS_CLIMB_MAX, PTCH_LIM_MAX_DEG
                elseif not arspd_stable and accel_x_stable and
                arspd_now >= arspd_target and arspd_now <= arspd_target*1.05 then
                    climb_max_rate = - ahrs:get_velocity_NED():z()--math.min(dh_now, theoretical_climb_max_rate) -- 最大上昇率を理論最大上昇率と決定
                    get_climb_rate_sw = true
                    gcs:send_text(6, "Steady-state evaluation Type 2")
                ]]--
                end
            end
                

                -- 定常性チェック後, @param THR_MAX, TECS_CLIMB_MAX, PTCH_LIM_MAX_DEG を決定
                if get_climb_rate_sw  and not phase4_finish_sw then
                    -- GCS 送信用の生データ
                    local throttle_max = throttle_now     -- 最大スロットル率決定
                    local pitch_max = pitch_deg           -- この時の最大ピッチ角
                    local pitch_min = - (pitch_max - 5.0) -- 最大ピッチ角に合わせた最小ピッチ角算出
                    local sink_max_rate = - param:get("AIRSPEED_MAX") * math.sin(math.rad(pitch_min))
                    -- GCS へ結果を送信
                    gcs:send_text(6, string.format("THR_MAX get to %.2f%%", throttle_max))
                    gcs:send_text(6, string.format("TECS_CLIMB_MAX get to %.2f m/s", climb_max_rate))
                    gcs:send_text(6, string.format("TECS_PITCH_MAX get to %.2f deg.", pitch_max))
                    gcs:send_text(6, string.format("TECS_PITCH_MIN get to %.2f deg.", pitch_min))
                    gcs:send_text(6, string.format("TECS_SINK_MAX get to %.2f m/s", -sink_max_rate))
                    gcs:send_text(6, string.format("Last AIRSPEED %.2f m/s", arspd_now))
                    -- パラメータを保存
                    param:set_and_save("THR_MAX", math.min(100, math.floor(throttle_max + 0.5)))
                    param:set_and_save("TECS_CLMB_MAX", climb_max_rate)
                    param:set_and_save("TECS_PITCH_MAX", pitch_max)
                    param:set_and_save("TECS_PITCH_MIN", pitch_min)
                    param:set_and_save("TECS_SINK_MAX", math.abs(sink_max_rate))
                    param:set("SCR_USER4", pitch_min) -- Phase 5で TECS_PITCH_MIN を書き換えるため待避
                    gcs:send_text(6, string.format("Saved TECS_PITCH_MIN %.2f to SCR_USER4", pitch_min))

                    -- 次フェーズに向けた準備
                    pitch_min_avo_sw = true
                    phase4_finish_sw = true
                    return
                end
            end

        -- 最大上昇高度に達した場合は強制終了し, 再度 Phase 2.5 からやり直し
        if climb_sw and height_now >= target_alt_limit then
            if not phase4_finish_sw then
                reset_all_flags(tecstune_phase)
                -- 上昇回数を加算
                climb_cnt = climb_cnt + 1
                gcs:send_text(6, string.format("Climb Count: %d", climb_cnt))
                -- 中断速度を表示
                gcs:send_text(6, string.format("Interruption AIRSPEED: %.2f m/s", arspd_now))

                -- climb_cntが上限までいったらPhase5へ強制移行
                if climb_cnt == climb_cnt_max then
                    gcs:send_text(0, "Reached max climb count. Transition to Phase5.")
                    phase5_set_param()
                    tecstune_phase = 5
                    return
                else
                    -- 上昇フェーズを Retry
                    next_pitch_cmd = pitch_deg
                    gcs:send_text(6, string.format("next pitch cmd: %.1f deg.", next_pitch_cmd))
                    mission:set_current_cmd(tkoff_before_cmd)
                    gcs:send_text(0, "Retry Phase 4")
                    phase2_5_set_param()
                    tecstune_phase = 2.5
                    return
                end
            elseif phase4_finish_sw then
                phase5_set_param()
                tecstune_phase = 5
                return
            end
        end
        -- climb_cntが上限までいったらPhase5へ強制移行
    end

    -- TECS_SINK_MIN 決定
    if tecstune_phase == 5 then
        -- 必要状態量取得
        local pitch_deg, arspd_now, accel_x_now, dh_now, height_now, throttle_now = update_sensors()
        local arspd_target = param:get("AIRSPEED_CRUISE")

        -- リトライ処理
        if height_now <= phase5_height_limit  or check_jump_trigger(tkoff_before_cmd) then
            phase5_cnt = phase5_cnt + 1
            if height_now <= phase5_height_limit then
                gcs:send_text(6, string.format("Minimum altitude reached: %d m", phase5_height_limit))
            else
                gcs:send_text(6, "Next WP reached")
            end

            if phase5_cnt >= phase5_cnt_max then
                -- 降下回数が上限に達したら強制で Phase 6 へ移行
                gcs:send_text(6, "Reached max Phase 5 count")
                phase6_set_param()
                tecstune_phase = 6
            else
                gcs:send_text(0, "Retry Phase 5")
                phase2_5_set_param()
                climb_sw = false
                get_climb_rate_sw = true
                phase4_finish_sw = true
                spdw_sw = false
                phase5_start_sw = false
                sink_trans_sw = false
                climb_cnt = 0
                tecstune_phase = 2.5
            end
            return
        end

        -- 速度配分変更
        if not spdw_sw and check_jump_trigger(phase5_spdw_cmd) then
            --param:set("TECS_SPDWEIGHT", 2.0)
            --gcs:send_text(6, string.format("TECS_SPDWEIGHT set to: %d", param:get("TECS_SPDWEIGHT")))
            spdw_sw = true
        end

        -- Phase 5 開始の地点
        if spdw_sw and not phase5_start_sw and check_jump_trigger(accel_sdby_cmd_num) then
            gcs:send_text(6, "Start get TECS_SINK_MIN (Phase 5)")
            phase5_start_sw = true
        end

        -- 滑空状態を疑似再現するために, 高度誤差を無視し, 速度のみ制御するように変更
        if phase5_start_sw and not sink_trans_sw then
            param:set("TECS_SPDWEIGHT", 2.0)
            param:set("TRIM_THROTTLE", thr_min)
            gcs:send_text(6, string.format("TECS_SPDWEIGHT set to: %d", param:get("TECS_SPDWEIGHT")))
            gcs:send_text(6, string.format("TRIM_THROTTLE return to: %.2f%%", param:get("TRIM_THROTTLE")))
            arspd_ctx = make_stab_ctx(obs_sink_time, dt)
            accel_ctx = make_stab_ctx(obs_sink_time, dt)
            sink_trans_sw = true
            gcs:send_text(6, "Start measurement of Phase 5")
        end

        -- 滑空状態 & スロットル率最小時の処理
        if sink_trans_sw and throttle_now == thr_min then
            -- 速度定常性評価
            local arspd_stable = val_stab_check(arspd_ctx, arspd_now,
                                                arspd_target, arspd_margin)

            -- 加速度定常性評価
            local accel_x_stable = true--val_stab_check(accel_ctx, accel_x_now,
                                                --accel_target, accel_margin)

            -- 速度定常性をチェック後, パラメータ決定
            if arspd_stable and accel_x_stable and dh_now < 0 then
                -- GCS 送信用の生データ
                local sink_min_rate = - ahrs:get_velocity_NED():z()
                -- GCS へ結果を送信
                gcs:send_text(0, string.format("TECS_SINK_MIN get to %.2f m/s", sink_min_rate))
                -- パラメータを保存
                param:set_and_save("TECS_SINK_MIN", math.abs(sink_min_rate))
                -- 次フェーズに向けたパラメータ設定
                phase6_set_param()
                tecstune_phase = 6
            end
        end
    end

    -- TRIM_THROTTLE 率の決定フェーズ：速度 (25 m/s), 高度, 上昇率の側面から定常性評価を行った後のスロットル率
    if tecstune_phase == 6 then
        -- 必要状態量取得
        local pitch_deg, arspd_now, accel_x_now, dh_now, height_now, throttle_now = update_sensors()
        local arspd_target = param:get("AIRSPEED_CRUISE")
        --local accel_target = gravity * math.sin(math.rad(pitch_deg))

        if not cruise_trans_sw and height_now >= (height_fbw_target - 5) and dh_now <= dh_margin then
            arspd_ctx = make_stab_ctx(obs_time, dt)
            accel_ctx = make_stab_ctx(obs_time, dt)
            dh_ctx = make_stab_ctx(obs_time, dt)
            alt_ctx = make_stab_ctx(obs_time, dt)
            gcs:send_text(6, "Start measurement of Phase 6")
            cruise_trans_sw = true
        end

        if cruise_trans_sw then
            -- 速度定常性評価
            local arspd_stable = val_stab_check(arspd_ctx, arspd_now,
                                                arspd_target, arspd_margin)

            -- 加速度定常性評価
            local accel_x_stable = val_stab_check(accel_ctx, accel_x_now,
                                                accel_target, accel_margin)
            
            -- 上昇率定常性評価
            local dh_stable = val_stab_check(dh_ctx, dh_now,
                                            dh_target, dh_margin)

            -- 高度定常性評価
            local alt_stable = val_stab_check(alt_ctx, height_now,
                                            height_fbw_target, height_margin)

            -- 定常性チェック後, TRIM_THROTTLE 決定
            if arspd_stable and accel_x_stable and dh_stable and alt_stable then
                -- GCS 送信用の生データ
                local trim_throttle = throttle_now
                -- GCS へ結果を送信
                gcs:send_text(0, string.format("TRIM_THROTTLE get to %.2f%%", trim_throttle))
                -- スロットル率は int で保存 (切り上げして安全側へ)
                param:set_and_save("TRIM_THROTTLE", math.floor(trim_throttle + 0.5))
                --mission:set_current_cmd(default_circle_cmd) -- 通常周回モードへ復帰
                gcs:send_text(0, string.format("Finished TecsTune"))
                tecstune_phase = 0
            end
        end
    end
end

function update()
    local mode = vehicle:get_mode()
    if mode == 10 and scripting_rc then
        local sw_pos = scripting_rc:get_aux_switch_pos()
        if sw_pos == 1 then
            tecstuning(true)
        else
            tecstuning(false)
        end
    end

    return update, FREQUENCY
end

return update()