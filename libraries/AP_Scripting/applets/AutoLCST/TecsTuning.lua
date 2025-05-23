-- Plane TECS Tune Script (Adjust Ver 4.3.8)

--[[
基本的には, ArduPilot の TecsTuning Guide に従う
パラメータの詳細決定方法は Full parameter list に則る

以下 URL

TECS (Total Energy Control System) for Speed and Height Tuning Guide
https://ardupilot.org/plane/docs/tecs-total-energy-control-system-for-speed-height-tuning-guide.html

ArduPlane Complete Parameter List V4.3.8
https://ardupilot.org/plane/docs/parameters-Plane-stable-V4.3.8.html
]]--

-- 必要な固定変数を取得/設定
local k_throttle = 70                           -- スロットルチャンネル番号
local thr_min = param:get("THR_MIN")            -- 最小スロットル率
local set2up_arspd = 2800                       -- 加速フェーズにおける達成速度
local gravity = 9.80665

-- MissionPlanner 関連の定義 (Plan name : AutoLSCT_TecsTuning_4.waypoints)
local last_nav_index = -1        -- 前回のミッションインデックス記録用 (初期化用変数, 数字には意味無い)
local dojump_aft_cmd_num = 2     -- DO_JUMP 後に到達する コマンド番号
local def_turnp_num = 3
local tkoff_cmd_num = 6          -- 巡航時用ピッチアップコマンドの代替となる TAKEOFF コマンドの番号
local rll2_cmd_num = 7           -- TECS_RLL2THR 決定用コマンド

-- 時間定義
local FREQUENCY = 10             -- コードのサンプリング周波数(ms)
local dt = FREQUENCY / 1000      -- 定常性評価時の刻み幅
local dt_log = 0.02              -- UAV Log Viewer のログ周期
local obs_fbw_time = 5.0         -- AIRSPEED_MIN/MAX, TECS_SINK_MIN 観測時間
local obs_climb_time = 4.8       -- 上昇フェーズ観測時間
local obs_rll2_time = 3.0        -- RLL2_THR 観測時間

-- Phase 毎に定常性評価バッファーを初期化する際の変数宣言
local arspd_ctx, accel_ctx, dh_ctx, alt_ctx

-- ー目標パラメータ・許容誤差ー
local scale = math.sqrt(dt / dt_log)            -- UAV Log Viewer から取得した諸々の閾値を実験用にスケーリングする係数
-- 20241218 の実験データから算出した閾値 (dt_log 秒周期で計算したデータ値)
local arspd_sigma_log = 0.519403471             -- 速度不偏標準偏差
local slope_sigma_log = 0.548936579             -- 加速度不偏標準偏差
local dh_sigma_log = 0.760163022                -- 上昇率不偏標準偏差
local height_sigma_log = 0.707958572            -- 高度不偏標準偏差
-- ノイズ有り状態変数生成用
local arspd_std = arspd_sigma_log * scale
local slope_std = slope_sigma_log * scale
local dh_std = dh_sigma_log * scale
local height_std = height_sigma_log * scale
-- 定常性評価用マージン : dt_log [s] 間隔のデータの MAE
local margin_scale = 1
local arspd_mae = 0.53273493
local slope_mae = 0.5515607
local dh_mae = 0.60717419
local height_mae = 0.60958849
-- マージン値を 本スクリプトで使用できるようにスケーリング
local arspd_margin = arspd_mae * scale * margin_scale       -- 許容速度誤差
local accel_margin = slope_mae * scale * margin_scale       -- 許容加速度誤差
local dh_margin = dh_mae * scale * margin_scale             -- 許容上昇率誤差
local height_margin = height_mae * scale * margin_scale     -- 許容目標巡航高度誤差
-- 絶対目標値
local height_target = 40.0                                  -- 目標巡航高度 (ミッションに依存)
local dh_target = 0.0                                       -- 目標巡航上昇率

-- AIRSPEED_MIN(ARSPD_FBW_MIN) 決定フェーズの
-- タイムアウト設定 & 減速速度テーブルリスト
local decel_timeout = 25
local decel_start_ts = 0.0
local decel_cnt = 0
local decel_arspd_max = 20
local decel_arspd_min = 5
local decel_arspd_table = {}
-- テーブル生成
for d = decel_arspd_max, decel_arspd_min, -1 do
    table.insert(decel_arspd_table, d)
end

-- AIRSPEED_MAX(ARSPD_FBW_MAX) 決定フェーズの
-- タイムアウト設定 & 加速速度テーブルリスト
local accel_timeout = decel_timeout
local accel_start_ts = 0
local accel_cnt = 0
local accel_arspd_min = 35
local accel_arspd_max = 45
local accel_arspd_table = {}
-- テーブル生成
for a = accel_arspd_min, accel_arspd_max, 1 do
    table.insert(accel_arspd_table, a)
end

-- AIRSPEED_MIN/MAX 時の閾値調整
local arspd_offset = 0.5

-- 上昇フェーズの高度目標値
local target_alt = 150                     -- 目標上昇高度
local target_alt_limit = target_alt - 10   -- 上昇運動強制終了高度

-- 上昇フェーズにおける上昇回数に応じた上昇ピッチ角のテーブルリスト
local climb_cnt = 0         -- 上昇回数の初期化
local pitch_table_max = 25  -- 初回上昇時のピッチ角
local pitch_table_min = 15  -- 最終上昇時のピッチ角
local pitch_table = {}      -- ピッチ角テーブルの生成
-- テーブル生成
for p = pitch_table_max, pitch_table_min, -1 do
    table.insert(pitch_table, p)
end

-- RLL2_THR 用変数
local turn_time = nil
local turn_start_time = nil

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
local set_spdw_sw = false
local rll2ms_edit_sw = false
local rll2_rotate = false

-- チューニングフェーズ
-- 0  : 未実行
-- 1  : ARISPEED_MIN(ARSPD_FBW_MIN) 決定フェーズ
-- 2  : ARISPEED_MAX(ARSPD_FBW_MAX) 決定フェーズ
-- 2.5: 2 ~ 3 遷移待機フェーズ
-- 3  : 加速フェーズ
-- 4  : 最大上昇率&(最大・最小ピッチ角, 理論最大降下率)の決定フェーズ
-- 5  : 最小降下率の決定フェーズ
-- 6  : トリムスロットル率決定フェーズ
local tecstune_phase = 0

-- パラメータ一括リセット関数
function reset_all_param()
    -- TecsTuning 中に一時変更するパラメータ
    param:set_and_save("TRIM_ARSPD_CM", 2500)
    param:set_and_save("TKOFF_THR_SLEW", 20)
    param:set_and_save("TKOFF_THR_DELAY", 2)
    param:set_and_save("TECS_SPDWEIGHT", 1)
    param:set_and_save("TECS_OPTIONS", 0)
    -- 以下, 自動決定する主要 TECS 関連パラメータ (デバックのためリセット)
    --[[
    param:set_and_save("THR_MAX", 100)
    param:set_and_save("TRIM_THROTTLE", 65)
    param:set_and_save("ARSPD_FBW_MAX", 30)
    param:set_and_save("ARSPD_FBW_MIN", 16)
    param:set_and_save("LIM_PITCH_MAX", 2500)
    param:set_and_save("LIM_PITCH_MIN", -2000)
    param:set_and_save("TECS_CLIMB_MAX", 5)
    param:set_and_save("TECS_SINK_MIN", 2)
    param:set_and_save("TECS_SINK_MAX", 5)
    param:set_and_save("TECS_RLL2THR", 0)
    ]]--
end

-- フラグ一括リセット関数
function reset_all_flags()
    trimspd_sw = false
    deceleration_sw = false
    decel_transition = false
    acceleration_sw = false
    accel_transition = false
    climb_sw = false
    climb_rotate = false
    set_spdw_sw = false
    rll2ms_edit_sw = false
    rll2_rotate = false
end

-- DO_JUMP 検出 & 次 WP まで上昇飛行を安全に行える距離があることを確認する関数
function check_jump_trigger_with_margin(after_index)
    local current_index = mission:get_current_nav_index()   -- 現在のインデックスを取得
    local arspd_target = param:get("TRIM_ARSPD_CM") * 0.01  -- 現在の目標速度
    local arspd_max = set2up_arspd * 0.01                   -- 上昇時に想定される最高速度
    
    -- ミッションインデックスの前後を監視することで, WP4 → WP2 に移行したことを検知した場合の処理
    if current_index == after_index and last_nav_index ~= after_index and
       last_nav_index ~= 0 then
        -- 現在の WP インデックスを取得して前回値に格納
        last_nav_index = after_index

        -- ーPhase 4 (上昇フェーズ) に必要な 次 WP までのマージン距離を算出ー
        -- 上昇回数を参照して任意回数目の上昇ピッチ角を取得
        local pitch_cmd = pitch_table[math.min(climb_cnt + 1, #pitch_table)]
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

-- 現在の基本状態変数取得関数 (飛行実験で本スクリプトを!!!動作させる時は noise を除外!!!)
-- @return pitch_deg, arspd_now, accel_x_now, dh_now, height_now, throttle_now
function update_sensors()
    local pitch_deg = math.deg(ahrs:get_pitch())                                        -- ピッチ角
    local arspd_now = ahrs:airspeed_estimate() + noise(arspd_std)                       -- 速度
    local accel_x_now = ahrs:get_accel():x() + noise(slope_std)                         -- 機種方向加速度
    local dh_now =  - ahrs:get_velocity_NED():z() + noise(dh_std)                       -- 右手系 z 軸下向きの速度ベクトル (上昇率)
    local height_now =  - ahrs:get_relative_position_NED_home():z() + noise(height_std) -- 地面からの相対高度 (下向き正)
    local throttle_now = SRV_Channels:get_output_scaled(k_throttle)                     -- スロットル率
    return pitch_deg, arspd_now, accel_x_now, dh_now, height_now, throttle_now
end

-- 定常性評価パラメータリセット関数
function make_stab_ctx(obs_time, dt)
    local buff_size = math.floor(obs_time / dt) -- バッファーサイズ (整数)
    return{
        buff_size = buff_size,
        val_buff = {},
        diff_buff = {},
        sum_val = 0,
    }
end

-- 定常性評価関数
-- 監視対象値の平均絶対誤差 <= 実験データ各値の MAE (平均絶対誤差)
-- であれば true を返す
-- @global param  dt         観測刻み幅
-- @global param  buff_size  観測時間 / dt
-- @param  ctx               評価対象のコンテキスト (make_stab_ctx(obs_time, dt))
-- @param  now_val           現在の値 (例：arspd_now)
-- @param  target_val        目標値 (例：TRIM_ARSPD_CM * 0.01)
-- @param  err_margin        値の許容誤差 (例：arspd_margin)
-- @return boolean           stable (true or false)
function val_stab_check(ctx, now_val, target_val, err_margin)
    -- 値を随時バッファに記録 & サイズを超えたら古いデータから削除
    local vb = ctx.val_buff
    table.insert(vb, now_val)
    ctx.sum_val = ctx.sum_val + now_val
    if #vb > ctx.buff_size then
        local old = table.remove(vb, 1)
        ctx.sum_val = ctx.sum_val - old
    end

    -- バッファ内データ数の充填を確認 
    if #vb < ctx.buff_size then
        return false
    end

    -- buff_size 分の平均絶対誤差が実験データの平均絶対誤差内に収まっているか判定
    local err_ok = true
    if err_margin then
        local ma = ctx.sum_val / ctx.buff_size
        err_ok = math.abs(ma - target_val) <= err_margin
    end

    -- 定常性フラグ
    local stable = err_ok

    return stable
end


function tecstuning(sw)

    -- プロポスイッチ ON でスクリプト有効化 (初回限定)
    if sw and not tecstune_running then
        gcs:send_text(0, string.format("TECS Tune Switch ON (Script Standby)"))
        gcs:send_text(6, "Start get to ARSPD_FBW_MIN (Phase 1)")
        -- 初期パラメータを安全側へ変更
        param:set("ARSPD_FBW_MIN", 5)
        param:set("ARSPD_FBW_MAX", 50)
        param:set("TRIM_THROTTLE", 10)
        tecstune_running = true
        tecstune_phase = 1
    end

    -- スイッチ ON 中かつ 任意の待機フェーズ中に DO_JUMP を検知したら指定フェーズに移行
    if tecstune_running then
        -- 上昇フェーズ待機時の処理
        if tecstune_phase == 2.5 then
            if check_jump_trigger_with_margin(dojump_aft_cmd_num) then
                local _, arspd_now, _, _, _ = update_sensors()
                local arspd_err = math.abs(arspd_now - param:get("TRIM_ARSPD_CM")*0.01)
                -- ARSPD_FBW_MAX 取得直後の上昇フェーズ移行を防止
                if arspd_err <= 2.0 then
                    tecstune_phase = 3
                    gcs:send_text(6, "Start Acceleration (Phase 3)")
                else
                    gcs:send_text(6, "Phase 3 Abort, Wait for next lap")
                end
            end
        end

        if tecstune_phase == 6.5 then
            if check_jump_trigger_with_margin(dojump_aft_cmd_num) then
                tecstune_phase = 7
            end
        end
    end

    -- プロポスイッチを OFF にした時の処理
    if not sw then
        if tecstune_running then
            reset_all_param()
            gcs:send_text(0, string.format("TECS Tune Switch OFF"))
        end
        -- sw 管理 (TecsTune Switch OFF で, すべて false)
        reset_all_flags()
        tecstune_running = false
        tecstune_phase = 0
        decel_cnt = 0
        accel_cnt = 0
        climb_cnt = 0
    end

    -- ARISPEED_MIN(ARSPD_FBW_MIN) 決定フェーズ
    if tecstune_phase == 1 then
        -- 必要状態量取得
        local pitch_deg, arspd_now, accel_x_now, dh_now, height_now, _ = update_sensors()
        -- 減速回数に応じた目標速度値をテーブルから取り出す
        local arspd_cmd = decel_arspd_table[math.min(decel_cnt + 1, #decel_arspd_table)]
        
        -- 減速後の目標速度を設定
        if not deceleration_sw then
            -- トリム速度値を変更
            param:set("TRIM_ARSPD_CM", arspd_cmd*100)
            local trim_arspd_now = param:get("TRIM_ARSPD_CM")
            gcs:send_text(6, string.format("Set TRIM_ARSPD to: %d m/s", trim_arspd_now*0.01))
            -- 減速回数を加算
            decel_cnt = decel_cnt + 1
            gcs:send_text(6, string.format("Deceleration Count: %d", decel_cnt))
            -- 定常評価 ctx (監視する状態量のリストや観測時間) の初期化
            arspd_ctx = make_stab_ctx(obs_fbw_time, dt)
            accel_ctx = make_stab_ctx(obs_climb_time, dt)
            dh_ctx = make_stab_ctx(obs_fbw_time, dt)
            alt_ctx = make_stab_ctx(obs_fbw_time, dt)
            -- 開始時刻を記録 & 速度変更フラグを true
            decel_start_ts = millis():tofloat()
            deceleration_sw = true
        end

        -- 初回減速時の transition 確認
        if deceleration_sw and not decel_transition then
            local decel_err = math.abs(arspd_cmd - arspd_now)
            if decel_err < arspd_margin then
                -- 開始時刻を記録 & 速度変更フラグを true
                decel_start_ts = millis():tofloat()
                gcs:send_text(6, "Deceleration Transition Complete")
                decel_transition = true
            end
        end

        -- 定常性評価
        if deceleration_sw and decel_transition then
            -- 記録時間の更新 & 目標速度を取得
            local decel_time = (millis():tofloat() - decel_start_ts) * 0.001
            local arspd_target = param:get("TRIM_ARSPD_CM") * 0.01
            local accel_target = gravity * math.sin(math.rad(pitch_deg))

            -- 速度
            local arspd_stable = val_stab_check(arspd_ctx, arspd_now,
                                                arspd_target, arspd_margin)

            -- 加速度
            local accel_x_stable = val_stab_check(accel_ctx, accel_x_now,
                                                  accel_target, accel_margin)


            -- 上昇率
            local dh_stable = val_stab_check(dh_ctx, dh_now,
                                             dh_target, dh_margin)

            -- 高度
            local alt_stable = val_stab_check(alt_ctx, height_now,
                                              height_target, height_margin)
            
            -- 要求速度で水平定常飛行を達成できた場合, 現在の要求速度より - 1 m/s してループ
            -- もしくは目標速度に一定程度近づいた場合, 上記と同様の処理
            if (arspd_now < (arspd_target + arspd_offset) or arspd_stable) and
               accel_x_stable and dh_stable and alt_stable then
                gcs:send_text(0, string.format("Retry Phase 1"))
                decel_time = 0  -- 記録時間を初期化
                deceleration_sw = false
            end

            -- 任意時間以上, 定常性を確保できなかった場合, ARSPD_FBW_MIN を決定
            if decel_time >= decel_timeout then
                -- "現在のインデックス"のトリム速度設定値に対して 20 % 大きい値を FBW_MIN と決定
                local fbw_min_idx = math.max(1, decel_cnt - 1)
                local fbw_min = decel_arspd_table[fbw_min_idx] * 1.20
                -- GCS へ結果を送信
                gcs:send_text(6, string.format("ARSPD_FBW_MIN get to %.2f m/s", fbw_min))
                param:set_and_save("ARSPD_FBW_MIN", fbw_min)
                -- 次フェーズのためのパラメータ設定
                param:set("TRIM_THROTTLE", 100)
                gcs:send_text(6, string.format("TRIM_THROTTLE set temporarily to: %.2f%%", param:get("TRIM_THROTTLE")))
                
                gcs:send_text(0, "Start get ARSPD_FBW_MAX (Phase 2)")
                tecstune_phase = 2
            end
        end
    end

    -- ARISPEED_MAX(ARSPD_FBW_MAX) 決定フェーズ
    if tecstune_phase == 2 then
        -- 必要状態量取得
        local pitch_deg, arspd_now, accel_x_now, dh_now, height_now, _ = update_sensors()
        -- 減速回数に応じた目標速度値をテーブルから取り出す
        local arspd_cmd = accel_arspd_table[math.min(accel_cnt + 1, #accel_arspd_table)]
        
        -- 加速後の目標速度を設定
        if not acceleration_sw then
            -- トリム速度値を変更
            param:set("TRIM_ARSPD_CM", arspd_cmd*100)
            local trim_arspd_now = param:get("TRIM_ARSPD_CM")
            gcs:send_text(6, string.format("Set TRIM_ARSPD to: %d m/s", trim_arspd_now*0.01))
            accel_cnt = accel_cnt + 1
            gcs:send_text(6, string.format("Acceleration Count: %d", accel_cnt))
            -- 定常性評価 ctx の初期化
            arspd_ctx = make_stab_ctx(obs_fbw_time, dt)
            accel_ctx = make_stab_ctx(obs_climb_time, dt)
            dh_ctx = make_stab_ctx(obs_fbw_time, dt)
            alt_ctx = make_stab_ctx(obs_fbw_time, dt)
            -- 開始時刻を記録 & 速度変更フラグを true
            accel_start_ts = millis():tofloat()
            acceleration_sw = true
        end

        -- 初回加速時の transition 確認
        if acceleration_sw and not accel_transition then
            local accel_err = math.abs(arspd_cmd - arspd_now)
            if accel_err < arspd_margin and height_now <= 45 then
                -- 開始時刻を記録 & 速度変更フラグを true
                accel_start_ts = millis():tofloat()
                gcs:send_text(6, "Acceleration Transition Complete")
                accel_transition = true
            end
        end

        -- 定常性評価
        if acceleration_sw and accel_transition then
            local accel_time = (millis():tofloat() - accel_start_ts) * 0.001
            local arspd_target = param:get("TRIM_ARSPD_CM") * 0.01
            local accel_target = gravity * math.sin(math.rad(pitch_deg))

            -- 速度
            local arspd_stable = val_stab_check(arspd_ctx, arspd_now,
                                                arspd_target, arspd_margin)

            -- 加速度
            local accel_x_stable = val_stab_check(accel_ctx, accel_x_now,
                                                  accel_target, accel_margin)

            -- 上昇率
            local dh_stable = val_stab_check(dh_ctx, dh_now,
                                             dh_target, dh_margin)

            -- 高度
            local alt_stable = val_stab_check(alt_ctx, height_now,
                                              height_target, height_margin)
            
            -- 要求速度で水平定常飛行を達成できた場合, 現在の要求速度より - 1 m/s してループ
            -- もしくは目標速度に一定程度近づいた場合, 上記と同様の処理
            if (arspd_now >= (arspd_target - 0.5) or arspd_stable) and
               accel_x_stable and dh_stable and alt_stable then
                gcs:send_text(0, string.format("Retry Phase 2"))
                accel_time = 0
                acceleration_sw = false
            end

            -- 任意時間以上, 定常性を確保できなかった場合, ARSPD_FBW_MAX を決定
            if accel_time >= accel_timeout then
                -- "1つ前のインデックス"のトリム速度設定値を FBW_MAX と決定
                local fbw_max_idx = math.max(1, accel_cnt - 1)
                local fbw_max = accel_arspd_table[fbw_max_idx]
                -- GCS へ結果を送信
                gcs:send_text(6, string.format("ARSPD_FBW_MAX get to %.2f m/s", fbw_max))
                param:set_and_save("ARSPD_FBW_MAX", fbw_max)
                -- 元のトリム速度に復帰
                param:set("TRIM_ARSPD_CM", 2500)
                -- 次フェーズのためのパラメータ設定
                param:set("TKOFF_THR_SLEW", 100)
                param:set("TKOFF_THR_DELAY", 0)
                param:set("TRIM_THROTTLE", 65)
                gcs:send_text(6, string.format("TKOFF_THR_SLEW set to: %.2f %%/s", param:get("TKOFF_THR_SLEW")))
                gcs:send_text(6, string.format("TKOFF_THR_DELAY set to: %.2f ds", param:get("TKOFF_THR_DELAY")))
                gcs:send_text(6, string.format("TRIM_THROTTLE return to: %.2f%%", param:get("TRIM_THROTTLE")))
                
                gcs:send_text(0, "Phase 3 Standby")
                tecstune_phase = 2.5
            end
        end
    end

    -- 加速フェーズ：上昇飛行直前に TRIM_ARSPD 一時的に 25 → 30 m/s とすることで間接的にスロットル率を上昇させる
    if tecstune_phase == 3 then
        -- 必要状態量取得
        local _, arspd_now, _, _, _, _ = update_sensors()
        local arspd_limit = set2up_arspd*0.01
        if not trimspd_sw then
            param:set("TRIM_ARSPD_CM", set2up_arspd)   --トリム速度値を変更
            gcs:send_text(6, string.format("Set TRIM_ARSPD to: %d m/s", param:get("TRIM_ARSPD_CM")*0.01))
            trimspd_sw = true
        end

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
        local pitch_cmd = pitch_table[math.min(climb_cnt + 1, #pitch_table)]

        -- 上昇させるために TKOFF ミッションを書き換え & 上昇回数の記録
        if item and not climb_sw then
            -- 機体の最大ピッチ角を再設定して上昇時のピッチ角オーバーシュートを防止
            param:set("LIM_PITCH_MAX", pitch_cmd*100)
            gcs:send_text(6, string.format("Pitch up angle set to: %.2f deg.", pitch_cmd))
            
            -- スロットル率の低下を防ぐため, トリムスロットル率を一時的に変更
            param:set("TRIM_THROTTLE", 100)
            gcs:send_text(6, string.format("TRIM_THROTTLE set temporarily to: %.2f%%", param:get("TRIM_THROTTLE")))
            
            -- TAKEOFF ミッション内容を上書き変更
            item:command(22)                        -- TAKEOFFコマンド番号
            item:param1(pitch_cmd)                  -- 任意の上昇ピッチ角目標値
            item:z(target_alt)                      -- 高度目標値
            mission:set_item(tkoff_cmd_num, item)   -- TKOFFコマンドの指示内容を上書き
            mission:set_current_cmd(tkoff_cmd_num)  -- TKOFFコマンドへジャンプ

            --  rotation する前に, トリム速度値を 25 m/s に復帰
            param:set("TRIM_ARSPD_CM", 2500)
            gcs:send_text(6, string.format("Return Set TRIM_ARSPD to: %d m/s", param:get("TRIM_ARSPD_CM")*0.01))

            gcs:send_text(0, "Start climb motion")
            climb_sw = true
        end
        
        -- @param THR_MAX, TECS_CLIMB_MAX, LIM_PITCH_MAX 決定フェーズ時の
        -- 上昇中処理 & 速度/加速度の監視
        if climb_sw then
            -- 上昇フェーズに rotation が完了したか判定するピッチ角基準値
            local pitch_rotate_margin = pitch_cmd * 0.95
            -- 定常性評価の速度目標値
            local arspd_target = param:get("TRIM_ARSPD_CM") * 0.01
            -- 参照する加速度が NED 座標系であるため, 定常時における機体固定座標系基準の加速度目標値を算出
            local climb_accel_target = gravity * math.sin(math.rad(pitch_cmd))
            -- 現在のピッチ角における理論最大上昇率
            local theoretical_climb_max_rate = arspd_target * math.sin(math.rad(pitch_cmd))
            
            -- rotation 完了待機
            if not climb_rotate and pitch_deg >= pitch_rotate_margin then
                gcs:send_text(6, "Start measurement of Phase 5")
                arspd_ctx = make_stab_ctx(obs_climb_time, dt)
                accel_ctx = make_stab_ctx(obs_climb_time, dt)
                climb_rotate = true
            end

            -- rotation が完了し, 上昇飛行状態での速度/加速度監視
            if climb_rotate and not get_climb_rate_sw then
                local climb_max_rate
                -- 速度定常性評価
                local arspd_stable = val_stab_check(arspd_ctx, arspd_now,
                                                    arspd_target, arspd_margin)

                -- 加速度の定常性評価
                local accel_x_stable = val_stab_check(accel_ctx, accel_x_now,
                                                      climb_accel_target, accel_margin)

                -- 定常性チェック後, @param THR_MAX, TECS_CLIMB_MAX, LIM_PITCH_MAX を決定
                if arspd_stable and accel_x_stable then
                    climb_max_rate = dh_now         -- 最大上昇率の決定
                    get_climb_rate_sw = true

                -- 速度定常性は確認できないが, 速度が 25 ~ 26.25 (+5%) m/s 内であり, 加速度が定常である場合の処理
                -- @param THR_MAX, TECS_CLIMB_MAX, LIM_PITCH_MAX
                elseif not arspd_stable and accel_x_stable and
                       arspd_now >= arspd_target and arspd_now <= arspd_target*1.05 then
                    climb_max_rate = math.min(dh_now, theoretical_climb_max_rate) -- 最大上昇率の決定
                    get_climb_rate_sw = true
                end

                -- 定常性チェック後, @param THR_MAX, TECS_CLIMB_MAX, LIM_PITCH_MAX を決定
                if get_climb_rate_sw then
                    -- GCS 送信用の生データ
                    local throttle_max = throttle_now     -- 最大スロットル率決定
                    local pitch_max = pitch_deg           -- この時の最大ピッチ角
                    local pitch_min = - (pitch_max - 5.0) -- 最大ピッチ角に合わせた最小ピッチ角算出
                    local sink_max_rate = - param:get("ARSPD_FBW_MAX") * math.sin(math.rad(pitch_min))
                    -- GCS へ結果を送信
                    gcs:send_text(6, string.format("THR_MAX get to %.2f%%", throttle_max))
                    gcs:send_text(6, string.format("TECS_CLIMB_MAX get to %.2f m/s", climb_max_rate))
                    gcs:send_text(6, string.format("LIM_PITCH_MAX_DEG get to %.2f deg.", pitch_max))
                    gcs:send_text(0, string.format("LIM_PITCH_MIN_DEG get to %.2f deg.", pitch_min))
                    gcs:send_text(0, string.format("TECS_SINK_MAX get to %.2f m/s", sink_max_rate))
                    -- パラメータを保存
                    param:set_and_save("THR_MAX", math.min(100, math.floor(throttle_max + 0.5)))
                    param:set_and_save("TECS_CLMB_MAX", climb_max_rate)
                    param:set_and_save("LIM_PITCH_MAX", pitch_max*100)
                    param:set_and_save("LIM_PITCH_MIN", pitch_min*100)
                    param:set_and_save("TECS_SINK_MAX", math.abs(sink_max_rate))
                    -- 次フェーズに向けた準備
                    mission:set_current_cmd(dojump_aft_cmd_num) -- TAKEOFF コマンドから通常周回モードへ
                    param:set("TRIM_THROTTLE", 65)
                    param:set("TKOFF_THR_SLEW", 20)
                    param:set("TKOFF_THR_DELAY", 2)
                    --param:set("TECS_OPTIONS", 1)
                    gcs:send_text(6, string.format("TRIM_THROTTLE return to: %.2f%%", param:get("TRIM_THROTTLE")))
                    gcs:send_text(6, string.format("TKOFF_THR_SLEW return to: %.2f %%/s", param:get("TKOFF_THR_SLEW")))
                    gcs:send_text(6, string.format("TKOFF_THR_DELAY return to: %.2f ds", param:get("TKOFF_THR_DELAY")))
                    --gcs:send_text(6, "TECS_OPTIONS set to:".. tostring(param:get("TECS_OPTIONS")))
                    gcs:send_text(0, "Start get TECS_SINK_MIN (Phase 5)")
                    tecstune_phase = 5
                end
            end
        end

        -- 最大上昇高度に達した場合は強制終了し, 再度やり直し
        if climb_sw and height_now >= target_alt_limit then
            reset_all_flags()
            -- 上昇回数を加算
            climb_cnt = climb_cnt + 1
            gcs:send_text(6, string.format("Climb Count: %d", climb_cnt))
            -- Retry 時に向けた準備
            param:set("TRIM_THROTTLE", 65)
            gcs:send_text(6, string.format("TRIM_THROTTLE return to: %.2f%%", param:get("TRIM_THROTTLE")))
            -- 上昇フェーズを Retry
            mission:set_current_cmd(dojump_aft_cmd_num)
            
            gcs:send_text(0, "Retry Phase 4")
            tecstune_phase = 2.5
        end
    end

    -- TECS_SINK_MIN 決定
    if tecstune_phase == 5 then
        -- 必要状態量取得
        local pitch_deg, arspd_now, accel_x_now, dh_now, _, throttle_now = update_sensors()
        local arspd_target = param:get("TRIM_ARSPD_CM") * 0.01

        -- 滑空状態を疑似再現するために, 高度誤差を無視し, 速度のみ制御するように変更
        if not set_spdw_sw then
            param:set("TECS_SPDWEIGHT", 2.0)
            gcs:send_text(6, string.format("TECS_SPDWEIGHT set to: %d", param:get("TECS_SPDWEIGHT")))
            arspd_ctx = make_stab_ctx(obs_fbw_time, dt)
            accel_ctx = make_stab_ctx(obs_fbw_time, dt)
            set_spdw_sw = true
        end

        -- 滑空状態 & スロットル率最小時の処理
        if set_spdw_sw and throttle_now == thr_min then
            local accel_target = gravity * math.sin(math.rad(pitch_deg))
            -- 速度定常性評価
            local arspd_stable = val_stab_check(arspd_ctx, arspd_now,
                                                arspd_target, arspd_margin)

            -- 加速度定常性評価
            local accel_x_stable = val_stab_check(accel_ctx, accel_x_now,
                                                  accel_target, accel_margin)

            -- 速度定常性をチェック後, パラメータ決定
            if arspd_stable and accel_x_stable and dh_now < 0 then
                -- GCS 送信用の生データ
                local sink_min_rate = dh_now
                -- GCS へ結果を送信
                gcs:send_text(0, string.format("TECS_SINK_MIN get to %.2f m/s", sink_min_rate))
                -- パラメータを保存
                param:set_and_save("TECS_SINK_MIN", math.abs(sink_min_rate))
                -- 次フェーズに向けた準備
                -- 定常性評価 ctx の初期化
                arspd_ctx = make_stab_ctx(obs_fbw_time, dt)
                dh_ctx = make_stab_ctx(obs_fbw_time, dt)
                alt_ctx = make_stab_ctx(obs_fbw_time, dt)
                param:set("TECS_SPDWEIGHT", 1)
                param:set("TRIM_THROTTLE", 65)
                param:set("TECS_OPTIONS", 0)
                gcs:send_text(6, string.format("TECS_SPDWEIGHT set to: %d", param:get("TECS_SPDWEIGHT")))
                gcs:send_text(6, string.format("TRIM_THROTTLE return to: %.2f%%", param:get("TRIM_THROTTLE")))
                gcs:send_text(6, "TECS_OPTIONS set to:".. tostring(param:get("TECS_OPTIONS")))
                
                gcs:send_text(0, "Start get TRIM_THROTTLE (Phase 6)")
                tecstune_phase = 6
            end
        end
    end

    -- TRIM_THROTTLE 率の決定フェーズ：速度 (25 m/s), 高度, 上昇率の側面から定常性評価を行った後のスロットル率
    if tecstune_phase == 6 then
        -- 必要状態量取得
        local pitch_deg, arspd_now, accel_x_now, dh_now, height_now, throttle_now = update_sensors()
        local arspd_target = param:get("TRIM_ARSPD_CM") * 0.01
        local accel_target = gravity * math.sin(math.rad(pitch_deg))

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
                                          height_target, height_margin)

        -- 定常性チェック後, TRIM_THROTTLE 決定
        if arspd_stable and accel_x_stable and dh_stable and alt_stable then
            -- GCS 送信用の生データ
            local trim_throttle = throttle_now
            -- GCS へ結果を送信
            gcs:send_text(0, string.format("TRIM_THROTTLE get to %.2f%%", trim_throttle))
            -- スロットル率は int で保存 (切り上げして安全側へ)
            param:set_and_save("TRIM_THROTTLE", math.floor(trim_throttle + 0.5))
            tecstune_phase = 6.5
        end
    end

    -- TECS_RLL2THR 決定
    if tecstune_phase == 7 then
        local _, _, _, dh_now, _, _ = update_sensors()
        local roll_deg = math.deg(ahrs:get_roll())
        local arspd_target = param:get("TRIM_ARSPD_CM") * 0.01

        -- 旋回半径の上書き (チューニングガイドに従い, 旋回半径を算出し, 指定 WP に上書き)
        if not rll2ms_edit_sw then
            local radius = 45
            local turn_r = math.floor((arspd_target*arspd_target)/(gravity * math.tan(math.rad(radius))) + 0.5)
            local item = mission:get_item(rll2_cmd_num)
            if item then
                item:param3(math.floor(turn_r + 0.5))
                mission:set_item(rll2_cmd_num, item)
                mission:set_current_cmd(rll2_cmd_num)
                gcs:send_text(6, string.format("Turn radius set: %.1f m", turn_r))
                dh_ctx = make_stab_ctx(obs_rll2_time, dt)
                alt_ctx = make_stab_ctx(obs_rll2_time, dt)
                turn_time = (2*math.pi*turn_r) / arspd_target

                rll2ms_edit_sw = true
            end
        end

        if rll2ms_edit_sw and not rll2_rotate and roll_deg <= -10 then
            turn_start_time = millis():tofloat()
            rll2_rotate = true
        end

        if rll2_rotate then
            local rll2_time = (millis():tofloat() - turn_start_time) * 0.001
            -- 上昇率定常性評価
            local dh_stable = val_stab_check(dh_ctx, dh_now,
                                            dh_target, dh_margin)
                
            if dh_stable and dh_now < 0 then
                local rll2_sink_rate = dh_now
                local rll2_thr = math.abs(rll2_sink_rate) * 10
                param:set_and_save("TECS_RLL2THR", rll2_thr)
                gcs:send_text(0, string.format("TECS_RLL2THR set to %.2f", rll2_thr))
                mission:set_current_cmd(def_turnp_num) -- 通常周回モードへ復帰
                
                gcs:send_text(0, string.format("Finished TecsTune"))
                tecstune_phase = 0
            end

            if rll2_time >= turn_time then
                reset_all_flags()
                mission:set_current_cmd(def_turnp_num)
                tecstune_phase = 6.5
                gcs:send_text(0, string.format("Retry Phase 7"))
            end

        end
    end
end

function update()
    local mode = vehicle:get_mode()
    if mode == 10 and scripting_rc then
        local sw_pos = scripting_rc:get_aux_switch_pos()
        if sw_pos == 1 then
            --vehicle:set_mode(10)
            tecstuning(true)
        else
            tecstuning(false)
        end
    end

    return update, FREQUENCY
end

return update()