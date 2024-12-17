-- 定数
local TARGET_PITCH_DEG = 28 -- 目標ピッチ角
local THROTTLE_CHANNEL = 3 -- スロットルのチャンネル
local NAV_SCRIPT_TIME_CMD = 1 -- スクリプト開始トリガーのコマンド
local THR_MAX_PARAM = "THR_MAX" -- THR_MAXパラメータ名
local k_throttle = 70
local INITIAL_PWM = 1600 -- 初期スロットルPWM値
local PWM_STEP = 10 -- スロットル調整ステップ
local MAX_TEST_DURATION_MS = 20000 -- テスト最大時間

-- 変数
local start_time = 0
local target_pwm = INITIAL_PWM

-- 初期化
local function init()
    local id, cmd = vehicle:nav_script_time()
    if id ~= nil and cmd == 1 then
        gcs:send_text(6, "Script Started: THR_MAX determination")
        attitude_control:set_offset_roll_pitch(0, TARGET_PITCH_DEG) -- ピッチ設定
        start_time = millis():tofloat() -- スタート時間の記録
        throttle_now = SRV_Channels:get_output_scaled(k_throttle)
        vehicle:set_target_throttle_rate_rpy(throttle_now, 0, 0, 0) -- 初期スロットル設定
    else
        gcs:send_text(4, "Invalid command received. Script terminating.")
        return terminate()
    end
end

-- メインループ
local function update()
    local current_time = millis():toint()
    local elapsed_time = current_time - start_time

    -- 時間制限チェック
    if elapsed_time > MAX_TEST_DURATION_MS then
        gcs:send_text(6, "Test duration exceeded, finalizing THR_MAX.")
        return finalize()
    end

    -- スロットル調整ロジック
    target_pwm = target_pwm + PWM_STEP
    if target_pwm > 1900 then
        target_pwm = 1900 -- 最大PWM制限
    end
    SRV_Channels:set_output_pwm_chan(THROTTLE_CHANNEL, target_pwm)
    gcs:send_named_float("Current PWM", target_pwm)

    -- 高度変化チェック（高度センサデータが必要な場合、適宜追加）
end

-- THR_MAXの反映
local function finalize()
    local calculated_thr_max = (target_pwm - 1100) / 8 -- THR_MAXの推定
    local success = param:set_and_save(THR_MAX_PARAM, calculated_thr_max)
    if success then
        gcs:send_text(6, string.format("THR_MAX set to %.2f", calculated_thr_max))
    else
        gcs:send_text(3, "Failed to set THR_MAX.")
    end
    return terminate() 
end

-- 終了処理
local function terminate()
    gcs:send_text(6, "Script Terminated")
    return
end

return {init = init, update = update, finalize = finalize, terminate = terminate}