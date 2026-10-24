/* 2026/09/24
serp_car V2.0.11 モーターバランスFlash保存版
Colorセンサーのキャリブレーション値はRAMだけで保持し、Flashへ保存しない
Flashの1キー(mBal)へ距離指定・連続前進・連続後進の最適補正値を保存
起動後の最初の走行前に保存値を読み込み、最初のPWM出力から補正する
Bluetooth用に2キー以上の空きを確保
serp_car V2.0.10 色境界の白誤判定対策版
色判定を3回平均化し、黒は暗さ、白は明るさとRGB均等度を満たす場合だけ判定
白・黒以外は、キャリブレーションした赤・緑・青のうち最も近い色を判定
serp_car V2.0.9 色キャリブレーション3キー保存版
K/W/R/G/Bの保存を5キーから3キーへ変更し、Bluetooth書き込み用に2キー分を確保
旧5キー形式(cK/cW/cR/cG/cB)は、初回読み込み時に3キー形式へ自動移行
serp_car V2.0.8 連続直進・時間指定直進の中心探索対応版
car_derection1 / car_derection2 の前進・後進に中心探索を追加
前進用と後進用の中心補正値を別々に学習・保持
右・左・右回転・左回転は従来の連続動作を維持
serp_car V2.0.7 ブロック表示修正版
角度指定回転の左右補正版
K/W/R/G/B 5色順番キャリブレーションブロック追加版
5色キャリブレーション実行中、Bボタンで色センサー値と判定内容をシリアル出力
色名1文字返却・色センサーキャリブレーション flashstorage 5キー保存修正版
V1.8.8で公開されていたブロックを再表示し、機能別の10グループに整理
超音波センサ
    I2C ADD:0x57
左・フォトリフレクター（B input)
    P1
右・フォトリフレクター(A input)
    P0
360サーボモーター
    左 P16
    右 P15

左ホイール・フォトセンサー(D input)
    P13
右ホイール・フォトセンサー(C input)
    P2
カラーセンサー
    I2C

この版は settings / filesystem / files.open を使いません。
github:bsiever/microbit-pxt-flashstorage を使用し、モーターバランスだけを1キー(mBal)へ保存します。
Colorセンサーの調整値はFlashへ保存せず、実行中のRAMだけで保持します。
MakeCodeの新しい環境で pxt-settings-blocks が入らない場合用です。
*/
let drive_mode = 0;
let step_mode = 0;
// 直進後の最初の回転だけ、停止状態で追加500ms待つ。
// 停止命令を挟んでも保持し、回転への切り替え時に消費する。
let straight_to_turn_wait_pending = false;
let L_U = 0;
let L_e_pre = 0;
let L_ie = 0;
let L_e = 0;
let L_r = 0;
let L_y = 0;
let R_U = 0;
let R_e_pre = 0;
let R_ie = 0;
let R_e = 0;
let R_r = 0;
let R_y = 0;
let L_counter = 0;
let R_counter = 0;
let T = 0;
let L_ki = 0;
let L_kp = 0;
let R_ki = 0;
let R_kp = 0;
let p = 0;
let photo_sikii = 30;
let u_dis = 999; // 超音波センサー未読時は「遠い」として扱う
let sonar_fail_count = 0; // 超音波センサー読み取り連続失敗回数
let sonar_prev = 999; // 超音波センサーの前回実測値
let sonar_now = 999; // 超音波センサーの今回実測値
let sonar_predicted = 999; // 予測補正後の距離
let sonar_sample_count = 0; // 予測に使える実測値の数
let sonar_predict_max = 4; // 予測補正の最大量(cm)。大きすぎると誤検出しやすい
let masu = 15;
let white_t = 250;
let black_t = 30;
let yaw_integral = 0;
let just_switched = false; // ← 追加：切替直後1フレームの固定出力フラグ
let left_wheel_adjust = 100; // 左タイヤの目標カウント補正(%)
let right_wheel_adjust = 100; // 右タイヤの目標カウント補正(%)
let motor_balance = 0; // 角度指定回転の補正。-20～-1=左回転を多く、1～20=右回転を多く、0=補正なし
let step_straight_speed = 55; // 長さ指定の前進・後進スピード(%). 100=従来最大, 小さいほど低速
let step_rotation_speed = 45; // 角度指定の回転スピード(%). 100=従来最大, 小さいほど低速
let straight_recover_threshold = 8; // 直進時、左右差がこのカウント以上になったら立て直し開始（自動バランス優先のため少し大きめ）
let straight_recover_overshoot = 0; // 立て直し時、反対側へこのカウント分だけ行き過ぎてから通常制御へ戻る
let straight_recover_slow_percent = 20; // 立て直し時、進みすぎた側を何%で動かすか。0なら一時停止
let straight_auto_balance_on = 1; // 1=直進開始直後に左右の癖を自動学習する
let straight_auto_sample_count = 35; // 何カウント付近までを学習区間にするか
let straight_auto_strength = 8; // 左右差1カウントあたり、どれだけ出力補正するか
let straight_auto_trim_max = 90; // 自動補正の最大出力差
let straight_auto_trim_u = 0; // 距離指定・前進用。+なら右を弱める、-なら左を弱める
let straight_backward_auto_trim_u = 0; // 距離指定・後進用
let straight_center_learning_on = 1; // 1=走行中も中心を探しながら基本補正を更新する
let straight_center_window = 40; // 何カウントごとに中心補正を見直すか
let straight_center_strength = 2; // 走行中の平均ズレ1カウントあたり、基本補正をどれだけ更新するか
let straight_center_deadband = 1; // この範囲の左右差は許容して、基本補正を更新しない
let straight_path_correction_on = 1; // 1=初期に曲がった分を、後半で逆向きに少し走って相殺する
let straight_path_correction_start = 35; // 何カウント以降から初期曲がり補正を始めるか
let straight_path_correction_strength = 4; // 積み上がったズレをどれくらい強く相殺するか
let straight_path_correction_max_diff = 5; // 補正中に狙う左右差の最大値。大きいほど強く戻す
let straight_short_correction_on = 1; // 1=15cmなど短距離でも早い段階から補正を効かせる
let straight_short_auto_sample_count = 12; // 短距離時、何カウント付近で初期バランスを決めるか
let straight_short_center_window = 12; // 短距離時、何カウントごとに中心補正を見直すか
let straight_short_path_start = 10; // 短距離時、何カウント以降から初期曲がり補正を始めるか
let straight_short_path_strength = 8; // 短距離時、初期曲がり補正の強さ
let straight_short_balance_gain = 8; // 短距離時、左右差1カウントあたりの通常補正量

// ---- 連続前進・後進用の中心探索 ----
// car_derection1（動かし続ける）と car_derection2（時間指定）は、
// どちらも cont_forward / cont_backward を通るため、この共通設定を使用する。
// trim が + のときは右モーターを弱め、- のときは左モーターを弱める。
let continuous_forward_trim_percent = 0; // 前進で見つけた中心補正（出力%）
let continuous_backward_trim_percent = 0; // 後進で見つけた中心補正（出力%）
let continuous_auto_sample_count = 24; // 開始からこの平均カウント付近まで中心を探索
let continuous_auto_strength = 2; // 左右差1カウントあたりの補正量（出力%）
let continuous_trim_max_percent = 40; // 車体差に対応する基本補正の上限（出力ポイント）
let continuous_center_window = 16; // 累積差の区間平均から基本補正を見直す間隔
let continuous_center_update_max = 2; // 1回の見直しで変更できる最大値（出力%）
let continuous_sync_gain = 15; // 累積差1カウントにつき先行側の出力を15%減らす
let continuous_sync_stop_diff = 6; // 大きな差では先行側を止め、後続側を待つ

// 連続直進1回分の状態。
// phase 0=中心探索、1=開始時に生じた左右差の回復、2=中心値で安定走行。
let continuous_straight_phase = 0;
let continuous_move_trim_percent = 0;
let continuous_base_trim_percent = 0;
let continuous_learn_start_avg = -1;
let continuous_learn_start_diff = 0;
let continuous_center_start_avg = 0;
let continuous_center_error_sum = 0;
let continuous_center_last_avg = 0;
// 発進直後だけ使う強い姿勢復帰。最初に先行した側を止め、反対側が1カウント
// 追い越した時点で解除する。その後は通常の滑らかな補正へ移る。
let continuous_start_recovery_state = 0; // +1=右先行、-1=左先行
let continuous_start_recovery_done = false;
// 強い過補正は、同じプログラム実行中に前進・後進それぞれ1回だけ使う。
// 車体交換やプログラム再実行で値はfalseに戻り、新しい車体で再測定される。
let forward_start_overcorrection_used = false;
let backward_start_overcorrection_used = false;
// 直進のソフトスタート。左右平均がこのカウントに達するまで、
// 指定出力の55%から100%へ滑らかに上げる。
let straight_start_power_percent = 55;
let straight_start_ramp_counts = 8;
let motor_balance_memory_loaded = false;
let motor_balance_memory_dirty = false;
let motor_balance_last_saved_data = "";
let rotation_debug_enabled = false;
let rotation_debug_sequence = 0;
let rotation_fault_latched = false;
let rotation_fault_rows: string[] = [];
let rotation_fault_printing = false;
let rotation_history: string[][] = [];
let rotation_log_printing = false;
let step_action_active = false;
let motion_stop_generation = 0;
let straight_debug_enabled = false;
let straight_debug_rows: string[] = [];
let straight_debug_start_ms = 0;
let straight_debug_last_ms = -100;

R_kp = 30;
R_ki = 20;
L_kp = 30;
L_ki = 20;

// 制御周期分だけ待つ処理（ｍｓ）
T = 0.01;
pins.setPull(DigitalPin.P13, PinPullMode.PullNone);
pins.setPull(DigitalPin.P2, PinPullMode.PullNone);

pins.setEvents(DigitalPin.P13, PinEventType.Pulse);
pins.setEvents(DigitalPin.P2, PinEventType.Pulse);

let noservo = 0;

let color_value = 0;
let volt = 0;
let color_cycle = 10;

// ---- color sensor calibration ----
// calibrated values use normalized RGB plus clear/light value.
// color_auto_calibration_on = 1 means use calibrated nearest-color judgement when values are available.
let color_auto_calibration_on: number = 1;
let color_cal_margin: number = 80;
// 通常の色判定は3回平均し、色境界での瞬間的な揺れを抑える。
let color_judgement_samples: number = 3;
// 白と判断するRGB均等度の基準。最小成分が最大成分の何%以上かで判定する。
// 白キャリブレーション済みの場合は、記録した白の均等度から自動調整する。
let color_white_balance_default: number = 65;
// B/Gを百分率で比較する。境界を下げると青、上げると緑になりやすい。
let color_green_blue_threshold_percent: number = 90;
let color_red_green_threshold_percent: number = 150;
let color_red_green_manual_override: boolean = false;
let color_white_balance_margin: number = 15;
let color_cal_red_ok: number = 0;
let color_cal_green_ok: number = 0;
let color_cal_blue_ok: number = 0;
let color_cal_black_ok: number = 0;
let color_cal_white_ok: number = 0;
let color_cal_red_r: number = 256;
let color_cal_red_g: number = 0;
let color_cal_red_b: number = 0;
let color_cal_red_l: number = 0;
let color_cal_green_r: number = 0;
let color_cal_green_g: number = 256;
let color_cal_green_b: number = 0;
let color_cal_green_l: number = 0;
let color_cal_blue_r: number = 0;
let color_cal_blue_g: number = 0;
let color_cal_blue_b: number = 256;
let color_cal_blue_l: number = 0;
let color_cal_black_r: number = 0;
let color_cal_black_g: number = 0;
let color_cal_black_b: number = 0;
let color_cal_black_l: number = 0;
let color_cal_white_r: number = 256;
let color_cal_white_g: number = 256;
let color_cal_white_b: number = 256;
let color_cal_white_l: number = 0;
let color_memory_loaded: number = 0;
let color_memory_valid: number = 0;
let color_guided_calibration_step: number = -1;
let color_guided_calibration_busy: number = 0;
let color_guided_calibration_handler_registered: number = 0;
let color_debug_handler_registered: number = 0;

let neo_serp = neopixel.create(DigitalPin.P9, 2, NeoPixelMode.RGB);
neo_serp.setBrightness(255);

pins.servoWritePin(AnalogPin.P15, 90);
pins.servoWritePin(AnalogPin.P16, 90);

let io_neo = neopixel.create(DigitalPin.P9, 3, NeoPixelMode.RGB);
io_neo.setBrightness(30);

for (let n = 0; n < 3; n++) {
  io_neo.setPixelColor(n, neopixel.colors(NeoPixelColors.Black));
}
io_neo.show();

//% color="#ff4500" weight=94
//% groups='["1 Basic car movement", "2 Simple control", "3 Ultrasonic distance sensor", "4 Photo reflectors", "5 Servo motor", "6 Color sensor", "7 Color LEDs", "8 Motion sensor", "9 micro:bit light sensor"]'
namespace serp {
  export enum direction {
    //% block="forward"
    forward,
    //% block="right",
    right,
    //% block="left",
    left,
    //% block="turn right",
    right_rotation,
    //% block="turn left",
    left_rotation,
    //% block="backward",
    backward,
    //% block="stop",
    Stop,
  }

  export enum direction2 {
    //% block="forward"
    forward,
    //% block="turn right",
    right_rotation,
    //% block="turn left",
    left_rotation,
    //% block="backward",
    backward,
  }

  export enum direction3 {
    //% block="forward"
    forward,
    //% block="backward",
    backward,
  }

  export enum direction4 {
    //% block="right"
    right_rotation,
    //% block="left"
    left_rotation,
  }

  export enum direction5 {
    //% block="right",
    right,
    //% block="left",
    left,
  }

  export enum direction7 {
    //% block="right"
    right,
    //% block="left"
    left,
  }

  export enum direction6 {
    //% block="forward"
    forward,
    //% block="backward",
    backward,
  }

  let volt = (pins.analogReadPin(AnalogPin.P1) / 500) * 6;

  export enum kyori {
    //% block="farther"
    long,
    //% block="nearer"
    short,
  }

  export enum onoff {
    //% block="ON"
    ON,
    //% block="OFF"
    OFF,
  }

  export enum whiteblack {
    //% block="black"
    black,
    //% block="white"
    white,
  }

  export enum BT {
    //% block="brighter"
    lighter,
    //% block="darker"
    darker,
  }

  export enum EventSide {
    //% block="left only"
    Left_onle,
    //% block="right only"
    Right_onle,
    //% block="both"
    Both,
  }

  export enum colorcycle {
    //% block="1"
    cycle1,
    //% block="10"
    cycle10,
    //% block="42"
    cycle42,
    //% block="64"
    cycle64,
  }

  export enum colorgain {
    //% block="×1"
    gain1,
    //% block="×4"
    gain4,
    //% block="×16"
    gain16,
    //% block="×60"
    gain60,
  }

  export enum color_scene {
    //% block="red"
    Red,
    //% block="green"
    Green,
    //% block="blue"
    Blue,
    //% block="black"
    Black,
    //% block="white"
    White,
  }

  export enum color_sensor {
    //% block="red"
    Red,
    //% block="green"
    Green,
    //% block="blue"
    Blue,
    //% block="light"
    Light,
  }

  export enum ServoPin {
    P3 = AnalogPin.P3,
    P4 = AnalogPin.P4,
    P8 = AnalogPin.P8,
    P14 = AnalogPin.P14,
  }

  export enum LEDSide {
    //% block="left"
    Left,
    //% block="right"
    Right,
    //% block="both"
    Both,
  }

  pins.onPulsed(DigitalPin.P2, PulseValue.High, function () {
    R_counter += 1;
  });
  pins.onPulsed(DigitalPin.P2, PulseValue.Low, function () {
    R_counter += 1;
  });
  pins.onPulsed(DigitalPin.P13, PulseValue.High, function () {
    L_counter += 1;
  });
  pins.onPulsed(DigitalPin.P13, PulseValue.Low, function () {
    L_counter += 1;
  });

  // ===== モーターバランスのFlash保存 =====
  // micro:bit V2のKeyValueStorageは最大5キーのため、4つの補正値を
  // 1つのキーへまとめる。残りはBluetooth通信で使用できる。
  function motor_balance_storage_text(): string {
    return (
      "2," +
      Math.round(straight_auto_trim_u) +
      "," +
      Math.round(straight_backward_auto_trim_u) +
      "," +
      Math.round(continuous_forward_trim_percent) +
      "," +
      Math.round(continuous_backward_trim_percent)
    );
  }

  function motor_balance_remove_color_storage(): void {
    // 現行・旧Colorキャリブレーションが占有していたキーを解放する。
    flashstorage.remove("c01");
    flashstorage.remove("c02");
    flashstorage.remove("c03");
    flashstorage.remove("cK");
    flashstorage.remove("cW");
    flashstorage.remove("cR");
    flashstorage.remove("cG");
    flashstorage.remove("cB");
    color_remove_old_storage_keys();
  }

  function motor_balance_load_internal(): void {
    if (motor_balance_memory_loaded) return;
    motor_balance_memory_loaded = true;

    let data = flashstorage.get("mBal");
    if (data.length == 0) {
      // 初回移行時だけColorセンサーが使っていたFlash領域を空ける。
      motor_balance_remove_color_storage();
      // 0は「まだ最適値なし」。移行済みの印にもなり、次回起動時に
      // 存在しない旧キーを何度も削除しない。
      data = "0,0,0,0,0";
      flashstorage.put("mBal", data);
    }
    motor_balance_last_saved_data = data;

    let parts = data.split(",");
    if (parts[0] != "1" && parts[0] != "2") return;

    let stepForwardTrim = parseInt(parts[1]);
    let stepBackwardTrim = stepForwardTrim;
    let forwardTrim = 0;
    let backwardTrim = 0;
    if (parts[0] == "2" && parts.length == 5) {
      stepBackwardTrim = parseInt(parts[2]);
      forwardTrim = parseInt(parts[3]);
      backwardTrim = parseInt(parts[4]);
    } else if (parts[0] == "1" && parts.length == 4) {
      // V1形式では距離指定の前後進が共通値だった。
      forwardTrim = parseInt(parts[2]);
      backwardTrim = parseInt(parts[3]);
    } else {
      return;
    }
    if (
      isNaN(stepForwardTrim) ||
      isNaN(stepBackwardTrim) ||
      isNaN(forwardTrim) ||
      isNaN(backwardTrim)
    ) return;

    if (stepForwardTrim > straight_auto_trim_max)
      stepForwardTrim = straight_auto_trim_max;
    if (stepForwardTrim < 0 - straight_auto_trim_max)
      stepForwardTrim = 0 - straight_auto_trim_max;
    if (stepBackwardTrim > straight_auto_trim_max)
      stepBackwardTrim = straight_auto_trim_max;
    if (stepBackwardTrim < 0 - straight_auto_trim_max)
      stepBackwardTrim = 0 - straight_auto_trim_max;
    continuous_forward_trim_percent = clamp_continuous_trim(forwardTrim);
    continuous_backward_trim_percent = clamp_continuous_trim(backwardTrim);
    straight_auto_trim_u = Math.round(stepForwardTrim);
    straight_backward_auto_trim_u = Math.round(stepBackwardTrim);

    // 保存値がある場合は、強い初回過補正を省略して最初から最適値を使う。
    forward_start_overcorrection_used = true;
    backward_start_overcorrection_used = true;
  }

  function motor_balance_save_internal(): void {
    if (!motor_balance_memory_loaded || !motor_balance_memory_dirty) return;
    let data = motor_balance_storage_text();
    motor_balance_memory_dirty = false;
    if (data == motor_balance_last_saved_data) return;
    flashstorage.put("mBal", data);
    motor_balance_last_saved_data = data;
  }

  //% color="#3943c6" weight=96 block="wait %second seconds" group="1 Basic car movement"
  //% second.min=0 second.max=10 second.defl=1
  export function driveForwards(second: number): void {
    basic.pause(second * 1000);
  }

  /**
   * 角度指定回転の左右差を1つの値で調整します。
   * 負の値は左回転、正の値は右回転の角度を多くします。
   * 前進・後進の距離には影響しません。
   */
  //% color="#3943c6" weight=95 blockId=serp_motor_balance
  //% block="more left turn %balance more right turn" group="1 Basic car movement"
  //% balance.min=-20 balance.max=20 balance.defl=0
  export function set_motor_balance(balance: number): void {
    if (balance < -20) balance = -20;
    if (balance > 20) balance = 20;
    motor_balance = Math.round(balance);
  }

  //% color="#3943c6" weight=94 blockId=serp_move_continuous
  //% block="move car %sinkou_houkou at speed %power" group="1 Basic car movement"
  //% power.min=0 power.max=100 power.defl=50
  //% blockHidden=true
  export function car_move(sinkou_houkou: direction3, power: number): void {
    if (power <= 0) {
      cont_stop(0);
    } else if (sinkou_houkou == direction3.forward) {
      cont_forward(power);
    } else {
      cont_backward(power);
    }
  }

  //% color="#3943c6" weight=93 blockId=serp_car_stop
  //% block="stop car" group="1 Basic car movement"
  export function car_stop(): void {
    cont_stop(0);
  }

  //% color="#3943c6" weight=90
  //% block="move car %sinkou_houkou at output %Power percent" group="1 Basic car movement"
  //% block.loc.ja="車を %sinkou_houkou へ 出力 %Power %で動かす"
  //% Power.min=0 Power.max=100 Power.defl=100
  export function car_derection1(
    sinkou_houkou: direction,
    Power: number,
  ): void {
    let next_mode = 0;

    switch (sinkou_houkou) {
      case direction.forward:
        next_mode = 1;
        break;
      case direction.backward:
        next_mode = 2;
        break;
      case direction.left_rotation:
        next_mode = 3;
        break;
      case direction.right_rotation:
        next_mode = 4;
        break;
      case direction.left:
        next_mode = 5;
        break;
      case direction.right:
        next_mode = 6;
        break;
      case direction.Stop:
        next_mode = 7;
        break;
    }

    // ★重要修正
    // Power=0 のときは、方向が forward / backward / rotation などでも
    // Stop と同じ完全停止処理に入る。
    // ここで先に return することで、cont_forward(0) などに入らないようにする。
    if (Power <= 0 || sinkou_houkou == direction.Stop) {
      cont_stop(0);
      return;
    }

    // 同じ命令・同じパワーが連続して来た場合は、
    // サーボ出力を上書きせず、補正用 forever に処理時間を渡す。
    // これにより basic.forever の中に「前進」だけを入れても補正が働きやすくなる。
    if (drive_mode == next_mode && p == Power) {
      basic.pause(20);
      return;
    }

    switch (sinkou_houkou) {
      case direction.forward:
        cont_forward(Power);
        break;
      case direction.backward:
        cont_backward(Power);
        break;
      case direction.left:
        cont_L(Power);
        break;
      case direction.right:
        cont_R(Power);
        break;
      case direction.right_rotation:
        cont_Rrotate(Power);
        break;
      case direction.left_rotation:
        cont_Lrotate(Power);
        break;
    }
  }

  //% color="#3943c6" weight=88
  //% block="move car %sinkou_houkou for %time_sec seconds at power %Power" group="1 Basic car movement"
  //% Power.min=0 Power.max=100 Power.defl=100
  //% time_sec.min=0 time_sec.max=10 time_sec.defl=0
  export function car_derection2(
    sinkou_houkou: direction,
    time_sec: number,
    Power: number,
  ): void {
    // 出力0または停止指定では、前進・後進の中心探索を開始せず完全停止する。
    if (Power <= 0 || sinkou_houkou == direction.Stop) {
      cont_stop(0);
      if (time_sec > 0) basic.pause(time_sec * 1000);
      cont_stop(0);
      return;
    }

    switch (sinkou_houkou) {
      case direction.forward:
        cont_forward(Power);
        basic.pause(time_sec * 1000);
        cont_stop(Power);
        break;
      case direction.left:
        cont_L(Power);
        basic.pause(time_sec * 1000);
        cont_stop(Power);
        break;
      case direction.right:
        cont_R(Power);
        basic.pause(time_sec * 1000);
        cont_stop(Power);
        break;
      case direction.right_rotation:
        cont_Rrotate(Power);
        basic.pause(time_sec * 1000);
        cont_stop(Power);
        break;
      case direction.left_rotation:
        cont_Lrotate(Power);
        basic.pause(time_sec * 1000);
        cont_stop(Power);
        break;
      case direction.backward:
        cont_backward(Power);
        basic.pause(time_sec * 1000);
        cont_stop(Power);
        break;
    }
  }

  //% color="#3943c6" weight=80
  //% block="move car %sinkou_houkou %step steps" group="1 Basic car movement"
  //% step.min=0 step.max=100 step.defl=0
  export function car_stepmove6(sinkou_houkou: direction2, step: number): void {
    switch (sinkou_houkou) {
      case direction2.forward:
        step_forward(step);
        break;
      case direction2.backward:
        step_backward(step);
        break;
      case direction2.left_rotation:
        step_Lrotate(step);
        break;
      case direction2.right_rotation:
        step_Rrotate(step);
        break;
    }
  }

  //% color="#3943c6" weight=92
  //% block="move car %sinkou_houkou %step cm" group="1 Basic car movement"
  //% step.min=0 step.max=100 step.defl=15
  export function car_stepmove7(sinkou_houkou: direction3, step: number): void {
    switch (sinkou_houkou) {
      case direction3.forward:
        step_forward(step * 4.1);
        break;
      case direction3.backward:
        step_backward(step * 4.1);
        break;
    }
  }

  //% color="#3943c6" weight=91
  //% block="turn car %sinkou_houkou %step degrees" group="1 Basic car movement"
  //% step.min=0 step.max=360 step.defl=90
  export function car_stepmove8(sinkou_houkou: direction4, step: number): void {
    let adjustedStep = step;

    // マイナス値は左回転だけ、プラス値は右回転だけを多くする。
    // 例：-5で左へ90度を指定すると、内部では94.5度分を回す。
    //     +5で右へ90度を指定すると、内部では94.5度分を回す。
    if (sinkou_houkou == direction4.left_rotation && motor_balance < 0) {
      adjustedStep = (step * (100 - motor_balance)) / 100;
    } else if (
      sinkou_houkou == direction4.right_rotation &&
      motor_balance > 0
    ) {
      adjustedStep = (step * (100 + motor_balance)) / 100;
    }

    switch (sinkou_houkou) {
      case direction4.right_rotation:
        step_Rrotate(adjustedStep * 0.33);
        break;
      case direction4.left_rotation:
        step_Lrotate(adjustedStep * 0.33);
        break;
    }
  }

  //% color="#3943c6" weight=89 blockId=serp_car_pivot_90_basic
  //% block="turn car %sinkou_houkou %angle degrees using one wheel" group="1 Basic car movement"
  //% block.loc.ja="片方のタイヤだけで %sinkou_houkou へ %angle °曲がる"
  //% angle.min=0 angle.max=360 angle.defl=90
  export function car_pivot_90_basic(
    sinkou_houkou: direction7,
    angle: number,
  ): void {
    car_pivot_90(sinkou_houkou, angle);
  }

  //% color="#3943c6" weight=83
  //% block="run motors left %L_p right %R_p" group="1 Basic car movement"
  //% L_p.min=0 L_p.max=100 L_p.defl=50
  //% R_p.min=0 R_p.max=100 R_p.defl=50
  export function car_stepmove5(L_p: number, R_p: number): void {
    cont_stop(0);
    if (rotation_fault_latched) return;
    pins.servoWritePin(AnalogPin.P16, 90 + (90 * L_p) / 100);
    pins.servoWritePin(AnalogPin.P15, 90 - (90 * R_p) / 100);
  }

  //% color="#4b0082" weight=90 blockId=blocklength
  //% block="set one square length to %square cm" group="2 Simple control"
  //% square.min=0 square.max=30 square.defl=15
  export function blocklength(square: number): void {
    masu = square;
  }

  //% color="#4b0082" weight=88 blockId=wheel_rotation_adjust
  //% block="wheel rotation adjustment left %leftPercent right %rightPercent" group="2 Simple control"
  //% leftPercent.min=50 leftPercent.max=150 leftPercent.defl=100
  //% rightPercent.min=50 rightPercent.max=150 rightPercent.defl=100
  //% blockHidden=true
  export function wheel_rotation_adjust(
    leftPercent: number,
    rightPercent: number,
  ): void {
    if (leftPercent < 50) leftPercent = 50;
    if (leftPercent > 150) leftPercent = 150;
    if (rightPercent < 50) rightPercent = 50;
    if (rightPercent > 150) rightPercent = 150;
    left_wheel_adjust = leftPercent;
    right_wheel_adjust = rightPercent;
  }

  //% color="#4b0082" weight=87 blockId=step_speed_adjust
  //% block="step movement speed forward/back %straightPercent rotation %rotationPercent" group="2 Simple control"
  //% straightPercent.min=20 straightPercent.max=100 straightPercent.defl=55
  //% rotationPercent.min=20 rotationPercent.max=100 rotationPercent.defl=45
  //% blockHidden=true
  export function step_speed_adjust(
    straightPercent: number,
    rotationPercent: number,
  ): void {
    if (straightPercent < 20) straightPercent = 20;
    if (straightPercent > 100) straightPercent = 100;
    if (rotationPercent < 20) rotationPercent = 20;
    if (rotationPercent > 100) rotationPercent = 100;
    step_straight_speed = straightPercent;
    step_rotation_speed = rotationPercent;
  }

  //% color="#4b0082" weight=86 blockId=straight_recovery_adjust
  //% block="straight recovery difference %threshold overshoot %overshoot slow side %slowPercent %" group="2 Simple control"
  //% threshold.min=2 threshold.max=30 threshold.defl=5
  //% overshoot.min=0 overshoot.max=10 overshoot.defl=1
  //% slowPercent.min=0 slowPercent.max=60 slowPercent.defl=0
  //% blockHidden=true
  export function straight_recovery_adjust(
    threshold: number,
    overshoot: number,
    slowPercent: number,
  ): void {
    if (threshold < 2) threshold = 2;
    if (threshold > 30) threshold = 30;
    if (overshoot < 0) overshoot = 0;
    if (overshoot > 10) overshoot = 10;
    if (slowPercent < 0) slowPercent = 0;
    if (slowPercent > 60) slowPercent = 60;
    straight_recover_threshold = threshold;
    straight_recover_overshoot = overshoot;
    straight_recover_slow_percent = slowPercent;
  }

  //% color="#4b0082" weight=85 blockId=straight_auto_balance_adjust
  //% block="straight auto balance %sw sample %sampleCount strength %strength max %maxTrim" group="2 Simple control"
  //% sampleCount.min=10 sampleCount.max=100 sampleCount.defl=35
  //% strength.min=2 strength.max=20 strength.defl=8
  //% maxTrim.min=10 maxTrim.max=200 maxTrim.defl=90
  //% blockHidden=true
  export function straight_auto_balance_adjust(
    sw: onoff,
    sampleCount: number,
    strength: number,
    maxTrim: number,
  ): void {
    if (sampleCount < 10) sampleCount = 10;
    if (sampleCount > 100) sampleCount = 100;
    if (strength < 2) strength = 2;
    if (strength > 20) strength = 20;
    if (maxTrim < 10) maxTrim = 10;
    if (maxTrim > 200) maxTrim = 200;
    if (sw == onoff.ON) straight_auto_balance_on = 1;
    else straight_auto_balance_on = 0;
    straight_auto_sample_count = sampleCount;
    straight_auto_strength = strength;
    straight_auto_trim_max = maxTrim;
  }

  //% color="#4b0082" weight=84 blockId=straight_auto_balance_reset
  //% block="reset straight auto balance" group="2 Simple control"
  //% blockHidden=true
  export function straight_auto_balance_reset(): void {
    straight_auto_trim_u = 0;
    straight_backward_auto_trim_u = 0;
    continuous_forward_trim_percent = 0;
    continuous_backward_trim_percent = 0;
    forward_start_overcorrection_used = false;
    backward_start_overcorrection_used = false;
    motor_balance_memory_loaded = true;
    motor_balance_memory_dirty = false;
    motor_balance_last_saved_data = "";
    flashstorage.remove("mBal");
  }

  //% color="#4b0082" weight=83 blockId=straight_center_learning_adjust
  //% block="straight center learning %sw window %windowCount strength %strength deadband %deadband" group="2 Simple control"
  //% windowCount.min=15 windowCount.max=120 windowCount.defl=40
  //% strength.min=1 strength.max=8 strength.defl=2
  //% deadband.min=0 deadband.max=5 deadband.defl=1
  //% blockHidden=true
  export function straight_center_learning_adjust(
    sw: onoff,
    windowCount: number,
    strength: number,
    deadband: number,
  ): void {
    if (windowCount < 15) windowCount = 15;
    if (windowCount > 120) windowCount = 120;
    if (strength < 1) strength = 1;
    if (strength > 8) strength = 8;
    if (deadband < 0) deadband = 0;
    if (deadband > 5) deadband = 5;
    if (sw == onoff.ON) straight_center_learning_on = 1;
    else straight_center_learning_on = 0;
    straight_center_window = windowCount;
    straight_center_strength = strength;
    straight_center_deadband = deadband;
  }

  //% color="#4b0082" weight=82 blockId=straight_path_correction_adjust
  //% block="straight path correction %sw start %startCount strength %strength max difference %maxDiff" group="2 Simple control"
  //% startCount.min=10 startCount.max=150 startCount.defl=35
  //% strength.min=0 strength.max=20 strength.defl=4
  //% maxDiff.min=0 maxDiff.max=12 maxDiff.defl=5
  //% blockHidden=true
  export function straight_path_correction_adjust(
    sw: onoff,
    startCount: number,
    strength: number,
    maxDiff: number,
  ): void {
    if (startCount < 10) startCount = 10;
    if (startCount > 150) startCount = 150;
    if (strength < 0) strength = 0;
    if (strength > 20) strength = 20;
    if (maxDiff < 0) maxDiff = 0;
    if (maxDiff > 12) maxDiff = 12;
    if (sw == onoff.ON) straight_path_correction_on = 1;
    else straight_path_correction_on = 0;
    straight_path_correction_start = startCount;
    straight_path_correction_strength = strength;
    straight_path_correction_max_diff = maxDiff;
  }

  //% color="#4b0082" weight=81 blockId=straight_short_correction_adjust
  //% block="short straight correction %sw sample %sampleCount window %windowCount path strength %pathStrength" group="2 Simple control"
  //% sampleCount.min=6 sampleCount.max=30 sampleCount.defl=12
  //% windowCount.min=6 windowCount.max=40 windowCount.defl=12
  //% pathStrength.min=0 pathStrength.max=20 pathStrength.defl=8
  //% blockHidden=true
  export function straight_short_correction_adjust(
    sw: onoff,
    sampleCount: number,
    windowCount: number,
    pathStrength: number,
  ): void {
    if (sampleCount < 6) sampleCount = 6;
    if (sampleCount > 30) sampleCount = 30;
    if (windowCount < 6) windowCount = 6;
    if (windowCount > 40) windowCount = 40;
    if (pathStrength < 0) pathStrength = 0;
    if (pathStrength > 20) pathStrength = 20;
    if (sw == onoff.ON) straight_short_correction_on = 1;
    else straight_short_correction_on = 0;
    straight_short_auto_sample_count = sampleCount;
    straight_short_center_window = windowCount;
    straight_short_path_start = sampleCount;
    straight_short_path_strength = pathStrength;
  }

  //% color="#4b0082" weight=80 blockId=car_zengo2
  //% block="move car %sinkou_houkou %F_block squares" group="2 Simple control"
  //% F_block.min=1 F_block.max=10 F_block.defl=1
  export function car_square1(
    sinkou_houkou: direction6,
    F_block: number,
  ): void {
    switch (sinkou_houkou) {
      case direction6.forward:
        step_forward(F_block * masu * 4.1);
        break;
      case direction6.backward:
        step_backward(F_block * masu * 4.1);
        break;
    }
  }

  //% color="#4b0082" weight=70
  //% block="turn car %sinkou_houkou 90 degrees" group="2 Simple control"
  export function car_square2(sinkou_houkou: direction5): void {
    switch (sinkou_houkou) {
      case direction5.right:
        step_Rrotate(90 * 0.33);
        break;
      case direction5.left:
        step_Lrotate(90 * 0.33);
        break;
    }
  }

  //% color="#4b0082" weight=69 blockId=serp_car_pivot_90
  //% block="turn car %sinkou_houkou %angle degrees using one wheel" group="2 Simple control"
  //% block.loc.ja="片方のタイヤだけで %sinkou_houkou へ %angle °曲がる"
  //% angle.min=0 angle.max=360 angle.defl=90
  export function car_pivot_90(
    sinkou_houkou: direction7,
    angle: number,
  ): void {
    if (angle < 0) angle = 0;
    if (angle > 360) angle = 360;
    // 片輪旋回では、同じ車体角度まで外側のタイヤが進む距離は
    // 左右輪を逆回転させる超信地旋回のおよそ2倍になる。
    if (sinkou_houkou == direction7.right) {
      step_Rpivot(angle * 0.66);
    } else {
      step_Lpivot(angle * 0.66);
    }
  }

  function step_forward(step_number: number) {
    cont_stop(0);
    basic.pause(50);
    straight_to_turn_wait_pending = true;
    step_mode = 1;
    L_counter = 0;
    R_counter = 0;
    drive_mode = 0;
    step_action(step_number);
  }
  function step_backward(step_number: number) {
    cont_stop(0);
    basic.pause(50);
    straight_to_turn_wait_pending = true;
    step_mode = 2;
    L_counter = 0;
    R_counter = 0;
    drive_mode = 0;
    step_action(step_number);
  }
  function wait_before_turn(): void {
    if (!straight_to_turn_wait_pending) return;
    // 連続直進から呼ばれた場合も、待機中は両輪を確実に停止する。
    cont_stop(0);
    straight_to_turn_wait_pending = false;
    basic.pause(500);
  }

  function step_Lrotate(step_number: number) {
    cont_stop(0);
    basic.pause(50);
    wait_before_turn();
    step_mode = 3;
    L_counter = 0;
    R_counter = 0;
    drive_mode = 0;
    step_action(step_number);
  }
  function step_Rrotate(step_number: number) {
    cont_stop(0);
    basic.pause(50);
    wait_before_turn();
    step_mode = 4;
    L_counter = 0;
    R_counter = 0;
    drive_mode = 0;
    step_action(step_number);
  }

  // 右折: 右タイヤを止め、外側の左タイヤだけを前進させる。
  function step_Rpivot(step_number: number) {
    cont_stop(0);
    basic.pause(50);
    wait_before_turn();
    step_mode = 5;
    L_counter = 0;
    R_counter = 0;
    drive_mode = 0;
    step_action(step_number);
  }

  // 左折: 左タイヤを止め、外側の右タイヤだけを前進させる。
  function step_Lpivot(step_number: number) {
    cont_stop(0);
    basic.pause(50);
    wait_before_turn();
    step_mode = 6;
    L_counter = 0;
    R_counter = 0;
    drive_mode = 0;
    step_action(step_number);
  }

  function save_step_straight_trim(value: number): void {
    if (step_mode == 2) straight_backward_auto_trim_u = value;
    else straight_auto_trim_u = value;
    motor_balance_memory_dirty = true;
  }

  // 診断用。回転中はRAMへ記録し、停止後にCSVを送信する。
  //% blockHidden=true
  export function rotation_debug(enable: boolean): void {
    rotation_debug_enabled = enable;
  }

  // 異常記録はRAMのみ。停止後に再送でき、再起動で消える。
  //% blockHidden=true
  export function rotation_fault_print(): void {
    if (!rotation_fault_latched || rotation_fault_printing) return;
    rotation_fault_printing = true;
    for (let i = 0; i < rotation_fault_rows.length; i++) {
      serial.writeLine(rotation_fault_rows[i]);
    }
    rotation_fault_printing = false;
  }

  //% blockHidden=true
  export function rotation_has_fault(): boolean {
    return rotation_fault_latched;
  }

  // 4往復分（左・右で計8回）を保持。各回のサンプルは直近20件まで。
  // 診断詳細モードでもRAM使用量を制限し、目標・停止・惰性の行は残す。
  function rotation_remember(rows: string[]): void {
    let sampleCount = 0;
    for (let i = 0; i < rows.length; i++) {
      if (rows[i].indexOf("ROT_SAMPLE,") == 0) sampleCount++;
    }
    let retained: string[] = [];
    for (let i = 0; i < rows.length; i++) {
      if (rows[i].indexOf("ROT_SAMPLE,") == 0 && sampleCount > 20) {
        sampleCount--;
      } else retained.push(rows[i]);
    }
    if (rotation_history.length >= 8) rotation_history.shift();
    rotation_history.push(retained);
  }

  // 正常・異常を問わず再送。走行中には呼ばず、停止後に利用する。
  //% blockHidden=true
  export function rotation_log_print(): void {
    if (step_action_active || (drive_mode != 0 && drive_mode != 7) ||
        rotation_log_printing || rotation_fault_printing) return;
    rotation_log_printing = true;
    let history = rotation_history.slice();
    serial.writeLine("ROT_LOG_BEGIN");
    if (history.length == 0) serial.writeLine("ROT_LOG_EMPTY");
    for (let record = 0; record < history.length; record++) {
      for (let row = 0; row < history[record].length; row++) {
        serial.writeLine(history[record][row]);
      }
    }
    serial.writeLine("ROT_LOG_END");
    rotation_log_printing = false;
  }

  function motion_outputs_off(): void {
    pins.digitalWritePin(DigitalPin.P15, 0);
    pins.digitalWritePin(DigitalPin.P16, 0);
    p = 0;
    drive_mode = 7;
    just_switched = false;
  }

  function rotation_watch_reason(elapsed: number, limit: number,
    leftIdle: number, rightIdle: number, leftDone: boolean, rightDone: boolean): string {
    if (leftDone && rightDone) return "";
    if (elapsed >= limit) return "TIMEOUT";
    let leftStall = !leftDone && leftIdle >= 1500;
    let rightStall = !rightDone && rightIdle >= 1500;
    if (leftStall && rightStall) return "BOTH_STALL";
    if (leftStall) return "LEFT_STALL";
    if (rightStall) return "RIGHT_STALL";
    return "";
  }

  function step_action(STEP: number) {
    if (rotation_fault_latched) { motion_outputs_off(); return; }
    step_action_active = true;
    let stopGeneration = motion_stop_generation;
    // 保存済みの最適値を、最初のモーター出力より前に読み込む。
    motor_balance_load_internal();
    R_counter = 0;
    L_counter = 0;
    R_e_pre = 0;
    L_e_pre = 0;
    R_ie = 0;
    L_ie = 0;

    let yaw_integral = 0;
    let yaw_k = 0.005; // 姿勢補正用ゲイン

    let isRotation =
      step_mode == 3 || step_mode == 4 || step_mode == 5 || step_mode == 6;
    let isRightPivot = step_mode == 5;
    let isLeftPivot = step_mode == 6;

    // 左右タイヤの個体差を、従来の目標カウント補正値で補正する。
    // 「左へ多く回る／右へ多く回る」ブロックの値はここでは使わないため、
    // 前進・後進の距離には影響しない。
    let L_adjust = left_wheel_adjust;
    let R_adjust = right_wheel_adjust;
    let L_target = (STEP * L_adjust) / 100;
    let R_target = (STEP * R_adjust) / 100;
    if (isRightPivot) R_target = 0;
    if (isLeftPivot) L_target = 0;
    if (L_target < 1) L_target = 1;
    if (R_target < 1) R_target = 1;
    if (isRightPivot) R_target = 0;
    if (isLeftPivot) L_target = 0;

    // 回転動作だけ、慣性によるオーバーランを見込んで早めに停止する。
    // 90度の場合：STEP=29.7、停止指令は約27.7カウント。
    let stopAdvance = 0;
    if (isRotation) stopAdvance = 2;

    let L_stopTarget = L_target - stopAdvance;
    let R_stopTarget = R_target - stopAdvance;
    if (L_stopTarget < 1) L_stopTarget = 1;
    if (R_stopTarget < 1) R_stopTarget = 1;
    if (isRightPivot) R_stopTarget = 0;
    if (isLeftPivot) L_stopTarget = 0;

    // 通常も直近20件をRAM記録。明示的な診断時だけ40ms/80件に増やす。
    let debugRotation = isRotation;
    let debugVerbose = rotation_debug_enabled && isRotation;
    let debugCapacity = debugVerbose ? 80 : 20;
    let debugInterval = debugVerbose ? 40 : 100;
    let debugRows: string[] = [];
    let debugNext = 0;
    let debugStart = input.runningTime();
    let debugLast = debugStart - 40;
    let debugId = 0;
    if (debugRotation) {
      rotation_debug_sequence += 1;
      debugId = rotation_debug_sequence;
    }

    // シリアル出力の重複を抑えるための前回値

    // 直進の立て直し状態
    //  0: 通常補正
    //  1: 右タイヤが進みすぎたので、右側を弱めて左側を追いつかせる
    // -1: 左タイヤが進みすぎたので、左側を弱めて右側を追いつかせる
    let recoverState = 0;

    // 直進開始直後の学習用。
    // moveAutoTrimU が + のときは右タイヤを基本的に弱める。
    // moveAutoTrimU が - のときは左タイヤを基本的に弱める。
    let moveAutoTrimU = step_mode == 2
      ? straight_backward_auto_trim_u
      : straight_auto_trim_u;
    let autoBaseTrimU = moveAutoTrimU;
    let autoLearnStarted = false;
    let autoLearnLocked = false;
    let autoStartDiff = 0;
    let autoStartAvg = 0;

    // 車体が交換されても、発進時に最初に現れた左右差をその場で戻す。
    // 一度だけ反対側へ1カウント行き過ぎさせ、その後は従来の補正へ渡す。
    let startRecoveryTriggered = false;
    let startRecoveryDone = step_mode == 1
      ? forward_start_overcorrection_used
      : backward_start_overcorrection_used;

    // 走りながら中心を求めるための学習用。
    // 一度決めた trim を固定せず、一定距離ごとの平均ズレや立て直し発生を見て少しずつ更新する。
    let centerWindowStartAvg = 0;
    let centerDiffSum = 0;
    let centerSampleN = 0;
    let centerLastL = -1;
    let centerLastR = -1;

    // 初期に斜めへ進んでしまった分を相殺するための積分補正。
    // pathDebt が + なら、これまで右タイヤ先行が多く、車体は左へ流れたとみなす。
    // その後は pathTargetDiff を - にして、少し右へ向ける。
    let pathDebt = 0;
    let pathTargetDiff = 0;
    let pathLastL = -1;
    let pathLastR = -1;

    // 15cm程度の短い直進では、従来の Sample35 / Window40 では遅すぎる。
    // そこで短距離時だけ、より早く初期学習・中心学習・経路補正を始める。
    let shortStraightMode =
      !isRotation && straight_short_correction_on == 1 && STEP <= 80;
    let effectiveAutoSample = straight_auto_sample_count;
    let effectiveCenterWindow = straight_center_window;
    let effectivePathStart = straight_path_correction_start;
    let effectivePathStrength = straight_path_correction_strength;
    let effectivePathDivisor = 1000;
    let effectiveBalanceGain = 5;
    if (shortStraightMode) {
      effectiveAutoSample = straight_short_auto_sample_count;
      if (effectiveAutoSample > Math.round(STEP / 2))
        effectiveAutoSample = Math.round(STEP / 2);
      if (effectiveAutoSample < 6) effectiveAutoSample = 6;
      effectiveCenterWindow = straight_short_center_window;
      if (effectiveCenterWindow > Math.round(STEP / 3))
        effectiveCenterWindow = Math.round(STEP / 3);
      if (effectiveCenterWindow < 6) effectiveCenterWindow = 6;
      effectivePathStart = straight_short_path_start;
      if (effectivePathStart > effectiveAutoSample)
        effectivePathStart = effectiveAutoSample;
      effectivePathStrength = straight_short_path_strength;
      effectivePathDivisor = 300;
      effectiveBalanceGain = straight_short_balance_gain;
    }

    let rotationLastTime = input.runningTime();
    let rotationLeftStopped = isLeftPivot;
    let rotationRightStopped = isRightPivot;
    let rotationLastL = 0;
    let rotationLastR = 0;
    let rotationProgressL = rotationLastTime;
    let rotationProgressR = rotationLastTime;
    // 90度は最低4秒、長い角度は目標カウントに応じて延長する。
    let rotationLimit = Math.max(4000, Math.ceil(Math.max(L_target, R_target) * 100));

    while (true) {
      basic.pause(1);
      // 別イベントの停止指令を、次のPWM出力で上書きしない。
      if (rotation_fault_latched || stopGeneration != motion_stop_generation) {
        motion_outputs_off();
        if (isRotation) {
          rotation_remember(["ROT_CANCELLED," + debugId + ",mode," + step_mode +
            ",ms," + (input.runningTime() - debugStart) +
            ",L," + L_counter + ",R," + R_counter]);
        }
        step_action_active = false;
        return;
      }
      let rotationNow = input.runningTime();
      if (isRotation) {
        if (L_counter > rotationLastL) rotationProgressL = rotationNow;
        if (R_counter > rotationLastR) rotationProgressR = rotationNow;
        rotationLastL = L_counter;
        rotationLastR = R_counter;
        let faultReason = rotation_watch_reason(rotationNow - debugStart, rotationLimit,
          rotationNow - rotationProgressL, rotationNow - rotationProgressR,
          rotationLeftStopped || L_counter >= L_stopTarget,
          rotationRightStopped || R_counter >= R_stopTarget);
        if (faultReason != "") {
          // 先にラッチと出力遮断。表示や通信で待っている間も再発進させない。
          rotation_fault_latched = true;
          motion_outputs_off();
          rotation_fault_rows = [
            "ROT_FAULT," + faultReason + ",id," + debugId + ",mode," + step_mode +
            ",ms," + (rotationNow - debugStart) + ",limit," + rotationLimit +
            ",L," + L_counter + ",R," + R_counter +
            ",stopL," + L_stopTarget + ",stopR," + R_stopTarget +
            ",lastL_U," + Math.round(L_U) + ",lastR_U," + Math.round(R_U),
            "type,id,ms,L_count,R_count,L_U,R_U,maxU,L_integral,R_integral,yaw_offset,stage"
          ];
          let firstFaultRow = debugRows.length == debugCapacity ? debugNext : 0;
          for (let row = 0; row < debugRows.length; row++) {
            rotation_fault_rows.push(debugRows[(firstFaultRow + row) % debugRows.length]);
          }
          rotation_fault_rows.push("ROT_FAULT_END");
          rotation_remember(rotation_fault_rows);
          step_action_active = false;
          basic.showIcon(IconNames.No);
          rotation_fault_print();
          return;
        }
      }
      let rotationDt = Math.min(0.05, Math.max(0, (rotationNow - rotationLastTime) / 1000));
      rotationLastTime = rotationNow;

      // 現時刻の情報取得
      R_y = R_counter;
      R_r = R_target;
      R_e = R_r - R_y;
      if (!isRotation) R_ie = R_ie + ((R_e + R_e_pre) * T) / 2;
      R_U = R_kp * R_e + R_ki * R_ie;
      if (R_U >= 600) R_U = 600;
      else if (R_U <= 0) R_U = 0;

      L_y = L_counter;
      L_r = L_target;
      L_e = L_r - L_y;
      if (!isRotation) L_ie = L_ie + ((L_e + L_e_pre) * T) / 2;
      L_U = L_kp * L_e + L_ki * L_ie;
      if (L_U >= 600) L_U = 600;
      else if (L_U <= 0) L_U = 0;

      p = (R_U + L_U) / 2;

      let diff = L_counter - R_counter;
      let total = L_counter + R_counter;
      if (total === 0) total = 1;

      let percentDiff = (Math.abs(diff) / Math.abs(total)) * 100;

      // 左右差のP補正
      let coefficient = percentDiff * 0.15;
      if (coefficient > 0.3) coefficient = 0.3;

      let isBackward = STEP < 0;

      // 回転の左右同期は出力上限適用後に行う。ここは直進用。
      if (!isRotation) {
        if (!isBackward) {
          if (L_counter > R_counter) L_U = L_U * (1 - coefficient);
          else if (R_counter > L_counter) R_U = R_U * (1 - coefficient);
        } else {
          if (L_counter > R_counter) R_U = R_U * (1 - coefficient);
          else if (R_counter > L_counter) L_U = L_U * (1 - coefficient);
        }

        // 姿勢補正
        yaw_integral += diff;
        let yaw_offset = yaw_integral * yaw_k;
        if (isBackward) yaw_offset = -yaw_offset;
        L_U += yaw_offset;
        R_U -= yaw_offset;
      }

      // 長さ指定の前進・後進、角度指定の回転ともに、
      // ここで最高出力を下げる。終盤ではさらに段階的に減速する。
      // U=600 は従来最大、U=300 は中心から約27度、U=150 は約13.5度の出力。
      let L_remaining = L_stopTarget - L_counter;
      let R_remaining = R_stopTarget - R_counter;
      let remaining = Math.min(L_remaining, R_remaining);
      if (isRightPivot) remaining = L_remaining;
      if (isLeftPivot) remaining = R_remaining;
      let speedStage = "HIGH";
      let maxU = 600;

      if (isRotation) {
        // 角度指定の回転スピード。初期値45なら、HIGH=270程度。
        let highU = Math.round(step_rotation_speed * 6);
        if (highU < 140) highU = 140;
        let middleU = Math.round(highU * 0.65);
        let lowU = Math.round(highU * 0.45);
        if (middleU < 110) middleU = 110;
        if (lowU < 90) lowU = 90;

        maxU = highU;
        // 90度回転（停止目標約28カウント）の場合、
        // 約16カウントから中速、約23カウントから低速になる。
        if (remaining <= 5) {
          maxU = lowU;
          speedStage = "LOW";
        } else if (remaining <= 12) {
          maxU = middleU;
          speedStage = "MIDDLE";
        }
      } else {
        // 長さ指定の前進・後進スピード。初期値55なら、HIGH=330程度。
        let highU = Math.round(step_straight_speed * 6);
        if (highU < 140) highU = 140;
        let middleU = Math.round(highU * 0.7);
        let lowU = Math.round(highU * 0.5);
        if (middleU < 120) middleU = 120;
        if (lowU < 100) lowU = 100;

        maxU = highU;
        speedStage = "MOVE-HIGH";
        // 1マス15cmなら約62カウントなので、終盤で見て分かる程度に減速する。
        if (remaining <= 8) {
          maxU = lowU;
          speedStage = "MOVE-LOW";
        } else if (remaining <= 20) {
          maxU = middleU;
          speedStage = "MOVE-MIDDLE";
        }

        // 発進直後は低い出力から始め、数カウントかけて通常出力へ上げる。
        // 急発進による最初の車体の振れを減らしながら、初動補正に測定時間を与える。
        let startAvgCount = Math.round((L_counter + R_counter) / 2);
        if (startAvgCount < straight_start_ramp_counts) {
          let startScalePercent =
            straight_start_power_percent +
            Math.round(
              ((100 - straight_start_power_percent) * startAvgCount) /
                straight_start_ramp_counts,
            );
          maxU = Math.round((maxU * startScalePercent) / 100);
          speedStage = "MOVE-START";
        }
      }

      if (isRotation) {
        // 回転だけ実経過時間で積分する。上限飽和中は積み増さず、
        // 積分による出力を現在の上限の30%以内に収める。
        if (L_ki > 0) {
          L_ie = Math.max(0, Math.min(L_ie, maxU * 0.3 / L_ki));
          if (L_e > 0 && L_kp * L_e + L_ki * L_ie < maxU)
            L_ie = Math.min(L_ie + L_e * rotationDt, maxU * 0.3 / L_ki);
        } else L_ie = 0;
        if (R_ki > 0) {
          R_ie = Math.max(0, Math.min(R_ie, maxU * 0.3 / R_ki));
          if (R_e > 0 && R_kp * R_e + R_ki * R_ie < maxU)
            R_ie = Math.min(R_ie + R_e * rotationDt, maxU * 0.3 / R_ki);
        } else R_ie = 0;
        L_U = Math.max(0, Math.min(maxU, L_kp * L_e + L_ki * L_ie));
        R_U = Math.max(0, Math.min(maxU, R_kp * R_e + R_ki * R_ie));

        // 先行輪だけ減速。目標比率で比較し、車輪回転量調整も尊重する。
        if (!isRightPivot && !isLeftPivot) {
          let rotationDiff = (L_y / L_target - R_y / R_target) *
            Math.min(L_target, R_target);
          let reduction = Math.min(0.3, Math.abs(rotationDiff) * 0.08);
          if (rotationDiff > 0) L_U *= 1 - reduction;
          else if (rotationDiff < 0) R_U *= 1 - reduction;
        }

        // 先に到達した輪は停止を保持し、もう片輪待ちで再駆動しない。
        if (L_y >= L_stopTarget) rotationLeftStopped = true;
        if (R_y >= R_stopTarget) rotationRightStopped = true;
        if (rotationLeftStopped) { L_U = 0; L_ie = 0; }
        if (rotationRightStopped) { R_U = 0; R_ie = 0; }
      }
      if (L_U > maxU) L_U = maxU;
      if (R_U > maxU) R_U = maxU;

      // 直進開始直後に、左右タイヤの「もともとの癖」を学習して基本補正として入れる。
      // 従来の差分補正は、曲がってから戻す後追い制御。
      // ここでは、学習した trim を先に入れて、できるだけ立て直し動作が起きないようにする。
      if (!isRotation && straight_auto_balance_on == 1) {
        let avgCount = Math.round((L_counter + R_counter) / 2);
        let diffForAuto = R_counter - L_counter; // +なら右が進みすぎ

        if (!autoLearnStarted && avgCount >= 5) {
          autoLearnStarted = true;
          autoStartDiff = diffForAuto;
          autoStartAvg = avgCount;
        }

        if (autoLearnStarted && !autoLearnLocked && avgCount > autoStartAvg) {
          let learnedDelta = Math.round(
            (diffForAuto - autoStartDiff) * straight_auto_strength,
          );
          let learnedNow = autoBaseTrimU + learnedDelta;
          if (learnedNow > straight_auto_trim_max)
            learnedNow = straight_auto_trim_max;
          if (learnedNow < 0 - straight_auto_trim_max)
            learnedNow = 0 - straight_auto_trim_max;

          // 走行中は少しずつ反映して、急に蛇行しないようにする。
          // 以前は learnedNow を0基準で作っていたため、短い移動のたびに学習済みtrimが薄まることがあった。
          // 今回は直前までの学習値を基準に、そこへ今回のズレ分だけ足す。
          moveAutoTrimU = Math.round((moveAutoTrimU * 2 + learnedNow) / 3);

          if (avgCount >= effectiveAutoSample) {
            autoLearnLocked = true;
            // 次回以降の直進では、最初からこの補正を使う。
            save_step_straight_trim(moveAutoTrimU);
            centerWindowStartAvg = avgCount;
            centerDiffSum = 0;
            centerSampleN = 0;
          }
        }

        // 学習した基本補正を先に入れる。
        // +なら右が速い癖なので右を弱める。-なら左を弱める。
        if (moveAutoTrimU > 0) {
          R_U = R_U - moveAutoTrimU;
        } else if (moveAutoTrimU < 0) {
          L_U = L_U + moveAutoTrimU; // moveAutoTrimU は負なので左を弱める
        }

        if (L_U > maxU) L_U = maxU;
        if (R_U > maxU) R_U = maxU;
        if (L_U < 0) L_U = 0;
        if (R_U < 0) R_U = 0;
      }

      // 直進時の左右バランス補正。
      // 通常補正だけだと、一度斜めを向いた後に「斜めのまま直進」しやすい。
      // そのため、左右差が一定以上になったら、進みすぎた側を一時的に止める/弱める。
      // そして反対側へ少し行き過ぎるまで戻してから、通常の直進補正に戻す。
      let straightBalance = 0;
      let recoveryLabel = "OFF";
      if (!isRotation) {
        let diffForBalance = R_counter - L_counter; // +なら右が進みすぎ
        let moveRemaining = Math.min(
          L_stopTarget - L_counter,
          R_stopTarget - R_counter,
        );

        let avgForBalance = Math.round((L_counter + R_counter) / 2);
        // 発進直後は距離の長短にかかわらず、1カウント差から強く戻す。
        // この強い補正は最初の1回だけで、反対側が1カウント追い越すと終了する。
        let startRecoveryActive =
          !startRecoveryDone &&
          avgForBalance <= effectiveAutoSample &&
          moveRemaining > 6;

        // 通常の強い立て直しは、従来どおり長距離かつ停止直前以外だけ使う。
        let canRecover =
          startRecoveryActive || (STEP >= 80 && moveRemaining > 25);
        let canSoftCorrect =
          canRecover || (shortStraightMode && STEP >= 20 && moveRemaining > 6);
        let activeRecoverThreshold = startRecoveryActive
          ? 1
          : straight_recover_threshold;
        let activeRecoverOvershoot = startRecoveryActive
          ? 1
          : straight_recover_overshoot;

        // 初期に曲がった分を「ズレの借金」として積み上げる。
        // +がたまるほど左に流れたとみなし、少しだけ右向きに走る目標差を作る。
        pathTargetDiff = 0;
        let pathControlDiff = diffForBalance;
        if (straight_path_correction_on == 1 && canSoftCorrect) {
          let avgForPath = Math.round((L_counter + R_counter) / 2);
          if (
            avgForPath >= 5 &&
            (L_counter != pathLastL || R_counter != pathLastR)
          ) {
            pathDebt = pathDebt + diffForBalance;
            if (pathDebt > 3000) pathDebt = 3000;
            if (pathDebt < -3000) pathDebt = -3000;
            pathLastL = L_counter;
            pathLastR = R_counter;
          }
          if (
            avgForPath >= effectivePathStart &&
            Math.abs(pathDebt) > 8 &&
            effectivePathStrength > 0
          ) {
            pathTargetDiff =
              0 -
              Math.round(
                (pathDebt * effectivePathStrength) / effectivePathDivisor,
              );
            if (pathTargetDiff > straight_path_correction_max_diff)
              pathTargetDiff = straight_path_correction_max_diff;
            if (pathTargetDiff < 0 - straight_path_correction_max_diff)
              pathTargetDiff = 0 - straight_path_correction_max_diff;
          }
          pathControlDiff = diffForBalance - pathTargetDiff;
        }

        if (canRecover) {
          if (recoverState == 0) {
            if (pathControlDiff >= activeRecoverThreshold) {
              recoverState = 1;
              if (startRecoveryActive) {
                startRecoveryTriggered = true;
                if (step_mode == 1) forward_start_overcorrection_used = true;
                else backward_start_overcorrection_used = true;
              }
              if (
                straight_center_learning_on == 1 &&
                straight_auto_balance_on == 1 &&
                pathTargetDiff == 0
              ) {
                let boost = straight_center_strength * 3;
                moveAutoTrimU = moveAutoTrimU + boost;
                if (moveAutoTrimU > straight_auto_trim_max)
                  moveAutoTrimU = straight_auto_trim_max;
                save_step_straight_trim(moveAutoTrimU);
                centerWindowStartAvg = Math.round((L_counter + R_counter) / 2);
                centerDiffSum = 0;
                centerSampleN = 0;
              }
            } else if (pathControlDiff <= 0 - activeRecoverThreshold) {
              recoverState = -1;
              if (startRecoveryActive) {
                startRecoveryTriggered = true;
                if (step_mode == 1) forward_start_overcorrection_used = true;
                else backward_start_overcorrection_used = true;
              }
              if (
                straight_center_learning_on == 1 &&
                straight_auto_balance_on == 1 &&
                pathTargetDiff == 0
              ) {
                let boost = straight_center_strength * 3;
                moveAutoTrimU = moveAutoTrimU - boost;
                if (moveAutoTrimU < 0 - straight_auto_trim_max)
                  moveAutoTrimU = 0 - straight_auto_trim_max;
                save_step_straight_trim(moveAutoTrimU);
                centerWindowStartAvg = Math.round((L_counter + R_counter) / 2);
                centerDiffSum = 0;
                centerSampleN = 0;
              }
            }
          } else if (recoverState == 1) {
            // 右が進みすぎていた場合、左が少し追い越すまで戻す。
            if (pathControlDiff <= 0 - activeRecoverOvershoot) {
              recoverState = 0;
              if (startRecoveryTriggered) startRecoveryDone = true;
            }
          } else if (recoverState == -1) {
            // 左が進みすぎていた場合、右が少し追い越すまで戻す。
            if (pathControlDiff >= activeRecoverOvershoot) {
              recoverState = 0;
              if (startRecoveryTriggered) startRecoveryDone = true;
            }
          }
        } else {
          recoverState = 0;
        }

        if (recoverState == 0) {
          // 普段の補正。短距離時は1カウント差あたりの補正を少し強くして早く効かせる。
          straightBalance = pathControlDiff * effectiveBalanceGain;
          L_U = L_U + straightBalance;
          R_U = R_U - straightBalance;
          recoveryLabel = "OFF";
        } else {
          // 立て直し補正。進みすぎた側を弱め、反対側で向きを戻す。
          let slowPercent = startRecoveryTriggered && !startRecoveryDone
            ? 0
            : straight_recover_slow_percent;
          let slowU = Math.round((maxU * slowPercent) / 100);
          if (recoverState == 1) {
            // 右が進みすぎ。右を弱めて、左を進める。
            L_U = maxU;
            R_U = slowU;
            recoveryLabel = "R_AHEAD";
          } else {
            // 左が進みすぎ。左を弱めて、右を進める。
            L_U = slowU;
            R_U = maxU;
            recoveryLabel = "L_AHEAD";
          }
        }

        // 走行中も、一定距離ごとに平均的なズレを見て基本補正を少しずつ更新する。
        // +DIFFが続くなら右が速いので trim を増やす。-DIFFが続くなら左が速いので trim を減らす。
        if (
          straight_center_learning_on == 1 &&
          straight_auto_balance_on == 1 &&
          recoverState == 0 &&
          canSoftCorrect
        ) {
          let avgNowForCenter = Math.round((L_counter + R_counter) / 2);
          if (centerWindowStartAvg == 0) centerWindowStartAvg = avgNowForCenter;
          if (L_counter != centerLastL || R_counter != centerLastR) {
            centerDiffSum = centerDiffSum + pathControlDiff;
            centerSampleN = centerSampleN + 1;
            centerLastL = L_counter;
            centerLastR = R_counter;
          }
          if (
            avgNowForCenter - centerWindowStartAvg >= effectiveCenterWindow &&
            centerSampleN > 0
          ) {
            let avgDiffForCenter = Math.round(centerDiffSum / centerSampleN);
            let updateTrim = 0;
            if (avgDiffForCenter > straight_center_deadband) {
              updateTrim =
                (avgDiffForCenter - straight_center_deadband) *
                straight_center_strength;
            } else if (avgDiffForCenter < 0 - straight_center_deadband) {
              updateTrim =
                (avgDiffForCenter + straight_center_deadband) *
                straight_center_strength;
            }
            if (updateTrim != 0) {
              moveAutoTrimU = moveAutoTrimU + updateTrim;
              if (moveAutoTrimU > straight_auto_trim_max)
                moveAutoTrimU = straight_auto_trim_max;
              if (moveAutoTrimU < 0 - straight_auto_trim_max)
                moveAutoTrimU = 0 - straight_auto_trim_max;
              save_step_straight_trim(moveAutoTrimU);
            } else {
            }
            centerWindowStartAvg = avgNowForCenter;
            centerDiffSum = 0;
            centerSampleN = 0;
          }
        }

        // 補正後も、その段階の最高出力を超えないようにする
        if (L_U > maxU) L_U = maxU;
        if (R_U > maxU) R_U = maxU;
        if (L_U < 0) L_U = 0;
        if (R_U < 0) R_U = 0;

        // 片側が目標カウントに到達したら、その側のモーターを止める。
        // これにより、速い側が目標を越えても積分値で回り続けることを防ぐ。
        if (L_counter >= L_target) {
          L_U = 0;
          L_ie = 0;
        }
        if (R_counter >= R_target) {
          R_U = 0;
          R_ie = 0;
        }
      }

      // 出力制限
      if (L_U > 600) L_U = 600;
      if (L_U < 0) L_U = 0;
      if (R_U > 600) R_U = 600;
      if (R_U < 0) R_U = 0;

      if (debugRotation) {
        let debugNow = input.runningTime();
        if (debugNow - debugLast >= debugInterval ||
            (L_y >= L_stopTarget && R_y >= R_stopTarget)) {
          debugRows[debugNext] = "ROT_SAMPLE," + debugId + "," +
            (debugNow - debugStart) + "," + L_y + "," + R_y + "," +
            Math.round(L_U) + "," + Math.round(R_U) + "," + maxU + "," +
            Math.round(L_ie * 100) / 100 + "," +
            Math.round(R_ie * 100) / 100 + "," +
            Math.round(yaw_integral * yaw_k * 100) / 100 + "," + speedStage;
          debugNext = (debugNext + 1) % debugCapacity;
          debugLast = debugNow;
        }
      }

      if (step_mode == 1) {
        // 前
        pins.servoWritePin(AnalogPin.P16, 90 + (90 * L_U) / 1000);
        pins.servoWritePin(AnalogPin.P15, 90 - (90 * R_U) / 1000);
      } else if (step_mode == 2) {
        // 後
        pins.servoWritePin(AnalogPin.P16, 90 - (90 * L_U) / 1000);
        pins.servoWritePin(AnalogPin.P15, 90 + (90 * R_U) / 1000);
      } else if (isRotation) {
        // PWMの90度指定では個体差で動く場合があるため、到達輪はLOWにする。
        if (rotationLeftStopped) pins.digitalWritePin(DigitalPin.P16, 0);
        else pins.servoWritePin(AnalogPin.P16,
          90 + (step_mode == 3 ? -1 : 1) * (90 * L_U) / 1000);
        if (rotationRightStopped) pins.digitalWritePin(DigitalPin.P15, 0);
        else pins.servoWritePin(AnalogPin.P15,
          90 + (step_mode == 4 ? 1 : -1) * (90 * R_U) / 1000);
      }

      R_e_pre = R_e;
      L_e_pre = L_e;

      // 回転時は早めの停止位置、直進時は従来どおりSTEPで停止する。
      let reached = false;
      if (isRightPivot) {
        reached = L_stopTarget <= L_y;
      } else if (isLeftPivot) {
        reached = R_stopTarget <= R_y;
      } else if (isRotation) {
        reached = L_stopTarget <= L_y && R_stopTarget <= R_y;
      } else {
        reached = L_target <= L_y && R_target <= R_y;
      }

      if (reached) {
        // PWMの90度指定ではなくLOWにして完全停止する。
        // カウンターはここではリセットせず、停止後の惰性分を測る。
        pins.digitalWritePin(DigitalPin.P15, 0);
        pins.digitalWritePin(DigitalPin.P16, 0);
        drive_mode = 0;
        let debugStopTime = input.runningTime() - debugStart;
        let debugStopL = L_counter;
        let debugStopR = R_counter;

        // 直進中に更新した最適値を、モーター停止後に1回だけ保存する。
        if (!isRotation) {
          save_step_straight_trim(moveAutoTrimU);
          motor_balance_memory_dirty = true;
          motor_balance_save_internal();
        }

        // 停止後の惰性カウントを取得
        basic.pause(200);
        if (isRotation) {
          // 送信中の時間を惰性計測に含めない。
          let debugFinalL = L_counter;
          let debugFinalR = R_counter;
          let completedRows: string[] = ["ROT_BEGIN," + debugId + ",mode," + step_mode +
            ",targetL," + L_target + ",targetR," + R_target +
            ",stopL," + L_stopTarget + ",stopR," + R_stopTarget,
            "type,id,ms,L_count,R_count,L_U,R_U,maxU,L_integral,R_integral,yaw_offset,stage"];
          let first = debugRows.length == debugCapacity ? debugNext : 0;
          for (let row = 0; row < debugRows.length; row++) {
            completedRows.push(debugRows[(first + row) % debugRows.length]);
          }
          completedRows.push("ROT_STOP," + debugId + "," + debugStopTime +
            "," + debugStopL + "," + debugStopR);
          completedRows.push("ROT_COAST," + debugId + ",200," +
            debugFinalL + "," + debugFinalR);
          rotation_remember(completedRows);
          if (debugVerbose) {
            for (let row = 0; row < completedRows.length; row++) serial.writeLine(completedRows[row]);
          }
        }
        step_action_active = false;
        break;
      }
    }
  }

  // ===== ここから連続動作用のモード切替関数 =====
  // 前進・後進だけは、開始後に左右バランスの中心出力を探す。
  // 右・左・右回転・左回転は、下の従来処理をそのまま使用する。

  // 連続直進の診断専用。走行中はRAMに最大60件記録し、通信しない。
  //% blockHidden=true
  export function straight_debug_start(): void {
    straight_debug_rows = [];
    straight_debug_start_ms = input.runningTime();
    straight_debug_last_ms = -100;
    straight_debug_enabled = true;
  }

  // 停止後に呼び出す。走行中のシリアル送信は行わない。
  //% blockHidden=true
  export function straight_debug_print(): void {
    if (drive_mode != 7) return;
    straight_debug_enabled = false;
    serial.writeLine("STRAIGHT_BEGIN");
    serial.writeLine("type,ms,L_count,R_count,R_minus_L,trim,L_power,R_power,phase");
    for (let i = 0; i < straight_debug_rows.length; i++) {
      serial.writeLine(straight_debug_rows[i]);
    }
    serial.writeLine("STRAIGHT_END,rows," + straight_debug_rows.length);
    straight_debug_rows = [];
  }

  function clamp_continuous_trim(value: number): number {
    if (value > continuous_trim_max_percent)
      value = continuous_trim_max_percent;
    if (value < 0 - continuous_trim_max_percent)
      value = 0 - continuous_trim_max_percent;
    return Math.round(value);
  }

  function save_continuous_trim(): void {
    continuous_move_trim_percent = clamp_continuous_trim(
      continuous_move_trim_percent,
    );
    if (drive_mode == 1) {
      continuous_forward_trim_percent = continuous_move_trim_percent;
    } else if (drive_mode == 2) {
      continuous_backward_trim_percent = continuous_move_trim_percent;
    }
    motor_balance_memory_dirty = true;
  }

  function start_continuous_straight(is_forward: boolean): void {
    // 保存済みの前進・後進別の最適値を、最初のPWM出力より前に読み込む。
    motor_balance_load_internal();
    if (is_forward) {
      continuous_move_trim_percent = continuous_forward_trim_percent;
    } else {
      continuous_move_trim_percent = continuous_backward_trim_percent;
    }
    continuous_move_trim_percent = clamp_continuous_trim(
      continuous_move_trim_percent,
    );
    continuous_base_trim_percent = continuous_move_trim_percent;
    continuous_straight_phase = 0;
    continuous_learn_start_avg = -1;
    continuous_learn_start_diff = 0;
    continuous_center_start_avg = 0;
    continuous_center_error_sum = 0;
    continuous_center_last_avg = 0;
    continuous_start_recovery_state = 0;
    continuous_start_recovery_done = is_forward
      ? forward_start_overcorrection_used
      : backward_start_overcorrection_used;
  }

  // 連続前進・後進の中心探索と出力。
  // 左右差へ毎回強い逆補正をかけず、中心値そのものを少しずつ探すことで
  // 左右交互のハンチングを防ぐ。
  function update_continuous_straight(): void {
    if (rotation_fault_latched) return;
    if (drive_mode != 1 && drive_mode != 2) return;

    let avg_count = Math.round((L_counter + R_counter) / 2);
    let wheel_diff = R_counter - L_counter; // +なら右が進みすぎ

    // phase 0: 開始直後の左右差の増え方から、釣り合う中心出力を探す。
    if (continuous_straight_phase == 0) {
      if (continuous_learn_start_avg < 0 && avg_count >= 4) {
        continuous_learn_start_avg = avg_count;
        continuous_learn_start_diff = wheel_diff;
      }

      if (
        continuous_learn_start_avg >= 0 &&
        avg_count > continuous_learn_start_avg
      ) {
        let learned_delta =
          (wheel_diff - continuous_learn_start_diff) * continuous_auto_strength;
        let learned_trim = clamp_continuous_trim(
          continuous_base_trim_percent + learned_delta,
        );

        // 一度に出力を変えず、3回分の平均でゆっくり中心へ寄せる。
        continuous_move_trim_percent = clamp_continuous_trim(
          Math.round((continuous_move_trim_percent * 2 + learned_trim) / 3),
        );

        if (avg_count >= continuous_auto_sample_count) {
          save_continuous_trim();

          // 差ゼロまで初期ずれを戻す。以後も累積差を常時補正する。
          if (wheel_diff != 0) {
            continuous_straight_phase = 1;
          } else {
            continuous_straight_phase = 2;
          }
          continuous_center_start_avg = avg_count;
          continuous_center_last_avg = avg_count;
          continuous_center_error_sum = 0;
        }
      }
    }

    // phase 1: 中心探索中に生じた左右差を一度だけ戻す。
    if (continuous_straight_phase == 1) {
      if (wheel_diff == 0) {
        continuous_straight_phase = 2;
      }
    }

    // 初期ずれ回復中も学習する。区間内の「累積差」を距離で重み付けし、
    // 小さな同方向のずれも捨てない。左右交互の差は平均して打ち消す。
    // カウントが変わらない間は加算しないため、処理周期や停止に依存しない。
    if (continuous_straight_phase != 0 && avg_count > continuous_center_last_avg) {
      continuous_center_error_sum +=
        wheel_diff * (avg_count - continuous_center_last_avg);
      continuous_center_last_avg = avg_count;
    }
    if (
      continuous_straight_phase != 0 &&
      avg_count - continuous_center_start_avg >= continuous_center_window
    ) {
      let mean_error = continuous_center_error_sum /
        (avg_count - continuous_center_start_avg);
      let trim_update = Math.round(Math.abs(mean_error));
      if (mean_error < 0) trim_update = 0 - trim_update;
      if (trim_update > continuous_center_update_max)
        trim_update = continuous_center_update_max;
      if (trim_update < 0 - continuous_center_update_max)
        trim_update = 0 - continuous_center_update_max;

      if (trim_update != 0) {
        continuous_move_trim_percent = clamp_continuous_trim(
          continuous_move_trim_percent + trim_update,
        );
        save_continuous_trim();
      }
      continuous_center_start_avg = avg_count;
      continuous_center_error_sum = 0;
    }

    let left_power = p;
    let right_power = p;

    // 学習した中心補正を常時先に入れる。
    if (continuous_move_trim_percent > 0) {
      right_power = right_power - continuous_move_trim_percent;
    } else if (continuous_move_trim_percent < 0) {
      left_power = left_power + continuous_move_trim_percent;
    }

    // 発進直後に最初の1カウント差が出たら、先行側を強く抑える。
    // 反対側が1カウント追い越したら解除し、以後は現在の滑らかな制御を使う。
    if (
      continuous_straight_phase == 0 &&
      !continuous_start_recovery_done &&
      avg_count <= continuous_auto_sample_count
    ) {
      if (continuous_start_recovery_state == 0) {
        if (wheel_diff >= 1) {
          continuous_start_recovery_state = 1;
          if (drive_mode == 1) forward_start_overcorrection_used = true;
          else backward_start_overcorrection_used = true;
        } else if (wheel_diff <= -1) {
          continuous_start_recovery_state = -1;
          if (drive_mode == 1) forward_start_overcorrection_used = true;
          else backward_start_overcorrection_used = true;
        }
      }

      if (continuous_start_recovery_state == 1) {
        if (wheel_diff <= -1) {
          continuous_start_recovery_done = true;
          continuous_start_recovery_state = 0;
        } else {
          right_power = right_power * 0.15;
        }
      } else if (continuous_start_recovery_state == -1) {
        if (wheel_diff >= 1) {
          continuous_start_recovery_done = true;
          continuous_start_recovery_state = 0;
        } else {
          left_power = left_power * 0.15;
        }
      }
    }

    // 1カウントからゼロへ戻す。低出力でも急停止しすぎないよう割合で減速。
    // 初回過補正を行わない発進にも適用するが、初回過補正とは重ねない。
    if (continuous_straight_phase != 0 || continuous_start_recovery_done) {
      let sync_reduction = Math.abs(wheel_diff) * continuous_sync_gain;
      if (Math.abs(wheel_diff) >= continuous_sync_stop_diff) sync_reduction = 100;
      if (sync_reduction > 100) sync_reduction = 100;
      if (wheel_diff > 0) right_power *= (100 - sync_reduction) / 100;
      else if (wheel_diff < 0) left_power *= (100 - sync_reduction) / 100;
    }

    if (left_power > 100) left_power = 100;
    if (right_power > 100) right_power = 100;
    if (left_power < 0) left_power = 0;
    if (right_power < 0) right_power = 0;

    // 連続・時間指定直進も、最初の数カウントだけソフトスタートする。
    if (avg_count < straight_start_ramp_counts) {
      let start_scale =
        (straight_start_power_percent +
          ((100 - straight_start_power_percent) * avg_count) /
            straight_start_ramp_counts) /
        100;
      left_power = left_power * start_scale;
      right_power = right_power * start_scale;
    }

    // 差が大きい先行輪はPWMを切る（逆回転はさせない）。惰性分は残り得る。
    if (left_power <= 0) pins.digitalWritePin(DigitalPin.P16, 0);
    else pins.servoWritePin(AnalogPin.P16,
      90 + (drive_mode == 1 ? 1 : -1) * 90 * left_power / 100);
    if (right_power <= 0) pins.digitalWritePin(DigitalPin.P15, 0);
    else pins.servoWritePin(AnalogPin.P15,
      90 + (drive_mode == 1 ? -1 : 1) * 90 * right_power / 100);

    if (straight_debug_enabled && straight_debug_rows.length < 60) {
      let elapsed = input.runningTime() - straight_debug_start_ms;
      if (elapsed - straight_debug_last_ms >= 100) {
        straight_debug_last_ms = elapsed;
        straight_debug_rows.push(
          "STRAIGHT_SAMPLE," + elapsed + "," + L_counter + "," + R_counter +
          "," + wheel_diff + "," + continuous_move_trim_percent +
          "," + Math.round(left_power * 100) / 100 +
          "," + Math.round(right_power * 100) / 100 +
          "," + continuous_straight_phase,
        );
      }
    }
  }

  // 同じ方向・同じ出力の命令が繰り返された場合は学習を継続する。
  // 方向または出力が変わった場合だけ、中心探索を新しく開始する。
  function cont_forward(power: number) {
    if (rotation_fault_latched) return;
    if (power > 0) straight_to_turn_wait_pending = true;
    let restart_straight = drive_mode != 1 || p != power;
    p = power;
    if (restart_straight) {
      L_counter = 0;
      R_counter = 0;
      drive_mode = 1;
      start_continuous_straight(true);
      // 学習済み補正があれば、最初のPWM出力から適用する。
      update_continuous_straight();
      just_switched = false;
    }
  }

  function cont_backward(power: number) {
    if (rotation_fault_latched) return;
    if (power > 0) straight_to_turn_wait_pending = true;
    let restart_straight = drive_mode != 2 || p != power;
    p = power;
    if (restart_straight) {
      L_counter = 0;
      R_counter = 0;
      drive_mode = 2;
      start_continuous_straight(false);
      // 学習済み補正があれば、最初のPWM出力から適用する。
      update_continuous_straight();
      just_switched = false;
    }
  }

  function cont_Lrotate(power: number) {
    wait_before_turn();
    if (rotation_fault_latched) return;
    p = power;
    if (drive_mode != 3) {
      L_counter = 0;
      R_counter = 0;
      drive_mode = 3;
      pins.servoWritePin(AnalogPin.P16, 90 - (90 * p) / 100);
      pins.servoWritePin(AnalogPin.P15, 90 - (90 * p) / 100);
      just_switched = true;
    }
  }

  function cont_Rrotate(power: number) {
    wait_before_turn();
    if (rotation_fault_latched) return;
    p = power;
    if (drive_mode != 4) {
      L_counter = 0;
      R_counter = 0;
      drive_mode = 4;
      pins.servoWritePin(AnalogPin.P16, 90 + (90 * p) / 100);
      pins.servoWritePin(AnalogPin.P15, 90 + (90 * p) / 100);
      just_switched = true;
    }
  }

  function cont_L(power: number) {
    wait_before_turn();
    if (rotation_fault_latched) return;
    p = power;
    if (drive_mode != 5) {
      L_counter = 0;
      R_counter = 0;
      drive_mode = 5;
      pins.servoWritePin(AnalogPin.P16, 90);
      pins.servoWritePin(AnalogPin.P15, 90 - (90 * p) / 100);
      just_switched = true;
    }
  }

  function cont_R(power: number) {
    wait_before_turn();
    if (rotation_fault_latched) return;
    p = power;
    if (drive_mode != 6) {
      L_counter = 0;
      R_counter = 0;
      drive_mode = 6;
      pins.servoWritePin(AnalogPin.P16, 90 + (90 * p) / 100);
      pins.servoWritePin(AnalogPin.P15, 90);
      just_switched = true;
    }
  }

  function cont_stop(power: number) {
    motion_stop_generation += 1;
    // Power=0 または Stop のときは、サーボPWMではなくデジタルLOWで完全停止する。
    // 片側だけゆっくり回る現象を防ぐため、毎回確実に状態もリセットする。
    p = 0;
    L_counter = 0;
    R_counter = 0;
    drive_mode = 7;
    just_switched = false;
    continuous_straight_phase = 0;
    continuous_learn_start_avg = -1;
    pins.digitalWritePin(DigitalPin.P15, 0);
    pins.digitalWritePin(DigitalPin.P16, 0);
    // 走行中に変化した値は、停止してから書き込んでFlash寿命を守る。
    motor_balance_save_internal();
  }

  //% color="#009A00" weight=90 blockId=sonar_ping_2 block="distance to obstacle (cm)" group="3 Ultrasonic distance sensor"
  export function sonar_ping_2(): number {
    let SR04_I2C_ADDR = 0x57;

    // 反応速度を優先するため、ここでは復帰待ちの長いpauseを入れない。
    // 測定開始命令を送る
    pins.i2cWriteNumber(SR04_I2C_ADDR, 1, NumberFormat.UInt8BE);
    basic.pause(120);

    // 3バイトの距離データを読む
    let readbuf = pins.i2cReadBuffer(
      SR04_I2C_ADDR,
      pins.sizeOf(NumberFormat.UInt8LE) * 3,
    );
    let distance = (readbuf[0] * 65536 + readbuf[1] * 256 + readbuf[2]) / 10000;

    // 正常値として扱う範囲
    if (distance > 0 && distance < 400) {
      sonar_fail_count = 0;
      return Math.round(distance);
    }

    // 読み取り失敗時は、短時間なら前回値を維持する。
    // ここで500msなどの復帰待ちは入れない。障がい物反応が遅くなるため。
    sonar_fail_count += 1;

    // 長く失敗した場合だけ「遠い」として扱い、古い近距離値で曲がり続けるのを防ぐ。
    if (sonar_fail_count >= 6) {
      return 999;
    }

    return u_dis;
  }

  // 超音波センサーの実測値から、少し先の距離を予測する
  // 例：前回20cm、今回15cmなら、最大4cmだけ先読みして11cmとして扱う
  function update_sonar_prediction(d: number): void {
    if (d > 0 && d < 400) {
      if (sonar_sample_count == 0) {
        sonar_prev = d;
        sonar_now = d;
        sonar_predicted = d;
        sonar_sample_count = 1;
        u_dis = d;
        return;
      }

      sonar_prev = sonar_now;
      sonar_now = d;
      if (sonar_sample_count < 2) {
        sonar_sample_count += 1;
      }

      let change = sonar_prev - sonar_now;

      // 近づいているときだけ予測する
      if (change > 0) {
        // 予測しすぎを防ぐ
        if (change > sonar_predict_max) {
          change = sonar_predict_max;
        }

        sonar_predicted = sonar_now - change;
        if (sonar_predicted < 1) {
          sonar_predicted = 1;
        }
      } else {
        // 遠ざかっている、または変化がないときは実測値を使う
        sonar_predicted = sonar_now;
      }

      u_dis = sonar_predicted;
    } else if (d == 999) {
      // 長時間読めない場合は「遠い」とし、予測履歴も初期化する
      u_dis = 999;
      sonar_prev = 999;
      sonar_now = 999;
      sonar_predicted = 999;
      sonar_sample_count = 0;
    }
  }

  //% color="#009A00" weight=88 block="distance to obstacle compared with %limit cm is %nagasa" group="3 Ultrasonic distance sensor"
  //% block.loc.ja="障害物までの距離が %limit cm より %nagasa"
  //% limit.min=5 limit.max=30 limit.defl=5
  export function sonar_ping_3(limit: number, nagasa: kyori): boolean {
    // 判定には実測値ではなく、予測補正後の u_dis を使う
    let d1 = u_dis;
    switch (nagasa) {
      case kyori.short:
        return d1 < limit;
      case kyori.long:
        return !(d1 < limit);
    }
  }

  //% color="#009A00" weight=89 blockId=serp_obstacle_near
  //% block="obstacle is nearer than %limit cm" group="3 Ultrasonic distance sensor"
  //% limit.min=5 limit.max=100 limit.defl=15
  //% blockHidden=true
  export function obstacle_near(limit: number): boolean {
    return u_dis <= limit;
  }

  function photosensor() {
    let L = Math.round((pins.analogReadPin(AnalogPin.P0) / 1023) * 100);
    let R = Math.round((pins.analogReadPin(AnalogPin.P1) / 1023) * 100);

    if (L < photo_sikii) serp.neopixel_set_block(1, 0xffd700);
    else serp.neopixel_set_block(1, 0xffffff);

    if (R < photo_sikii) serp.neopixel_set_block(0, 0xffd700);
    else serp.neopixel_set_block(0, 0xffffff);

    return [L, R];
  }

  //% color="#696969" weight=90 block="set photo sensor threshold to %threshold"
  //% threshold.min=10 threshold.max=90 threshold.defl=30
  //% group="4 Photo reflectors"
  export function setPhotoSikii(threshold: number) {
    photo_sikii = threshold;
  }

  //% color="#696969" weight=87 blockId=auto_photo_R block="right photo reflector value" group="4 Photo reflectors"
  export function phto_R() {
    let v = photosensor();
    return v[0];
  }

  //% color="#696969" weight=86 blockId=auto_photo_L block="left photo reflector value" group="4 Photo reflectors"
  export function phto_L() {
    let v = photosensor();
    return v[1];
  }

  //% color="#696969" weight=89 block="line sensor %pos is %wb" group="4 Photo reflectors"
  export function photoSensor(pos: EventSide, wb: whiteblack): boolean {
    switch (pos) {
      case EventSide.Right_onle:
        switch (wb) {
          case whiteblack.black:
            return phto_R() < photo_sikii && phto_L() > photo_sikii;
          case whiteblack.white:
            return phto_R() > photo_sikii && phto_L() < photo_sikii;
        }
        break;
      case EventSide.Left_onle:
        switch (wb) {
          case whiteblack.black:
            return phto_L() < photo_sikii && phto_R() > photo_sikii;
          case whiteblack.white:
            return phto_L() > photo_sikii && phto_R() < photo_sikii;
        }
        break;
      case EventSide.Both:
        switch (wb) {
          case whiteblack.black:
            return phto_R() < photo_sikii && phto_L() < photo_sikii;
          case whiteblack.white:
            return phto_R() > photo_sikii && phto_L() > photo_sikii;
        }
        break;
    }
    return false;
  }

  //% color="#696969" weight=88 blockId=serp_line_detected
  //% block="line is at %pos" group="4 Photo reflectors"
  //% blockHidden=true
  export function line_detected(pos: EventSide): boolean {
    return photoSensor(pos, whiteblack.black);
  }

  //% block="when %side is %wb"
  //% color="#696969" weight=85 group="4 Photo reflectors"
  export function onEvent(
    side: EventSide,
    wb: whiteblack,
    handler: () => void,
  ) {
    control.inBackground(function () {
      while (true) {
        let leftCondition =
          wb === whiteblack.black
            ? phto_L() < photo_sikii
            : phto_L() > photo_sikii;
        let rightCondition =
          wb === whiteblack.black
            ? phto_R() < photo_sikii
            : phto_R() > photo_sikii;

        switch (side) {
          case EventSide.Left_onle:
            if (leftCondition && !rightCondition) handler();
            break;
          case EventSide.Right_onle:
            if (rightCondition && !leftCondition) handler();
            break;
          case EventSide.Both:
            if (leftCondition && rightCondition) handler();
            break;
        }
        basic.pause(20);
      }
    });
  }

  //% blockId=servo_motor_set_angle group="5 Servo motor"
  //% block="set servo motor %pin to %angle degrees"
  //% angle.min=0 angle.max=180
  //% inlineInputMode=inline
  export function setServoAngle(pin: ServoPin, angle: number): void {
    if (rotation_fault_latched && (<number>pin == 15 || <number>pin == 16)) return;
    pins.servoWritePin(<AnalogPin>(<number>pin), angle);
  }

  /*
        smbus.writeByte(0x81, 0x00)  //0x81=10000001  RGB timing 700ms
        smbus.writeByte(0x81, 0x10)  //16×gain
        smbus.writeByte(0x80, 0x03)  //0x03を書くと動作開始
        smbus.writeByte(0x81, 0x2b)  //this.addr 0x29 0x81=10000001 0x2b=00101011
    */

  // ---- カラーセンサー設定 ----
  smbus.writeByte(0x81, 0xf6); // cycle10
  // PON -> wait -> AEN（安定化）
  smbus.writeByte(0x80, 0x01); // PON
  basic.pause(5);
  smbus.writeByte(0x80, 0x03); // PON + AEN
  basic.pause(30);

  // ---- 補正係数（ご指定）----
  const R_COEF = 0.8;
  const G_COEF = 1.0;
  const B_COEF = 1.2;

  /**
   * 各色のセンサーで取得した値を返す
   */
  //% color="#ffa500" weight=84 block="color sensor %color value" group="6 Color sensor"
  export function colorsensor_value(color: color_sensor): number {
    switch (color) {
      case color_sensor.Light:
        return color_L();
      case color_sensor.Red:
        return color_R();
      case color_sensor.Green:
        return color_G();
      case color_sensor.Blue:
        return color_B();
    }
  }

  // ★★★ ここが今回の変更点：RGBそれぞれに係数をかけてから正規化 ★★★
  function color_R() {
    let v = rgb();
    let r = v[0] * R_COEF;
    let g = v[1] * G_COEF;
    let b = v[2] * B_COEF;
    let m = Math.max(r, Math.max(g, b));
    if (m <= 0) return 0;
    return Math.round((r / m) * 256);
  }
  function color_G() {
    let v = rgb();
    let r = v[0] * R_COEF;
    let g = v[1] * G_COEF;
    let b = v[2] * B_COEF;
    let m = Math.max(r, Math.max(g, b));
    if (m <= 0) return 0;
    return Math.round((g / m) * 256);
  }
  function color_B() {
    let v = rgb();
    let r = v[0] * R_COEF;
    let g = v[1] * G_COEF;
    let b = v[2] * B_COEF;
    let m = Math.max(r, Math.max(g, b));
    if (m <= 0) return 0;
    return Math.round((b / m) * 256);
  }

  function color_L() {
    let result: Buffer = smbus.readBuffer(
      0xb4,
      pins.sizeOf(NumberFormat.UInt16LE) * 4,
    );
    return (smbus.unpack("HHHH", result)[0] / color_cycle) * 10;
  }

  export function rgb(): number[] {
    let result: number[] = raw();
    let clear: number = result.shift();
    if (clear <= 0) return [0, 0, 0];
    for (let x: number = 0; x < result.length; x++) {
      result[x] = (result[x] * 255) / clear;
    }
    return result; // [R,G,B] (0..255相当)
  }

  export function raw(): number[] {
    let result: Buffer = smbus.readBuffer(
      0xb4,
      pins.sizeOf(NumberFormat.UInt16LE) * 4,
    );
    return smbus.unpack("HHHH", result); // [C,R,G,B]
  }

  function normalized_color_once(): number[] {
    let v = raw();
    let clear = v[0];
    if (clear <= 0) return [0, 0, 0, 0];
    let r = ((v[1] * 255) / clear) * R_COEF;
    let g = ((v[2] * 255) / clear) * G_COEF;
    let b = ((v[3] * 255) / clear) * B_COEF;
    let m = Math.max(r, Math.max(g, b));
    if (m <= 0) return [0, 0, 0, 0];
    let light = (clear / color_cycle) * 10;
    return [
      Math.round((r / m) * 256),
      Math.round((g / m) * 256),
      Math.round((b / m) * 256),
      Math.round(light),
    ];
  }

  function normalized_color_average(samples: number): number[] {
    if (samples < 1) samples = 1;
    if (samples > 20) samples = 20;
    let sr = 0;
    let sg = 0;
    let sb = 0;
    let sl = 0;
    for (let i = 0; i < samples; i++) {
      let v = normalized_color_once();
      sr += v[0];
      sg += v[1];
      sb += v[2];
      sl += v[3];
      basic.pause(20);
    }
    return [
      Math.round(sr / samples),
      Math.round(sg / samples),
      Math.round(sb / samples),
      Math.round(sl / samples),
    ];
  }

  function color_distance(
    r: number,
    g: number,
    b: number,
    l: number,
    cr: number,
    cg: number,
    cb: number,
    cl: number,
    use_light: boolean,
  ): number {
    let dr = r - cr;
    let dg = g - cg;
    let db = b - cb;
    let d = dr * dr + dg * dg + db * db;
    if (use_light) {
      let dl = l - cl;
      // light is useful for white/black, but too strong if used directly.
      dl = dl / 10;
      d += dl * dl;
    }
    return d;
  }

  function has_color_calibration(): boolean {
    return (
      color_cal_red_ok +
        color_cal_green_ok +
        color_cal_blue_ok +
        color_cal_black_ok +
        color_cal_white_ok >
      0
    );
  }

  function update_calibrated_white_black_thresholds(): void {
    if (color_cal_white_ok == 1 && color_cal_black_ok == 1) {
      let gap = color_cal_white_l - color_cal_black_l;
      if (gap < 20) gap = 20;
      black_t = Math.round(color_cal_black_l + gap * 0.25);
      white_t = Math.round(color_cal_black_l + gap * 0.7);
    } else if (color_cal_white_ok == 1) {
      white_t = Math.round(color_cal_white_l * 0.7);
    } else if (color_cal_black_ok == 1) {
      black_t = Math.round(color_cal_black_l * 1.4);
    }
  }

  // RGBの均等度を0～100で返す。白に近いほど100になる。
  function color_rgb_balance_percent(R: number, G: number, B: number): number {
    let maxRGB = Math.max(R, Math.max(G, B));
    let minRGB = Math.min(R, Math.min(G, B));
    if (maxRGB <= 0) return 0;
    return Math.round((minRGB * 100) / maxRGB);
  }

  // キャリブレーションした白のRGBバランスに合わせて、白判定基準を自動調整する。
  function color_white_balance_limit(): number {
    let limit = color_white_balance_default;
    if (color_cal_white_ok == 1) {
      limit =
        color_rgb_balance_percent(
          color_cal_white_r,
          color_cal_white_g,
          color_cal_white_b,
        ) - color_white_balance_margin;
    }
    // センサー差があっても白を認識でき、赤＋緑などの混色は白にしにくい範囲。
    if (limit < 65) limit = 65;
    if (limit > 75) limit = 75;
    return limit;
  }

  // 白は「十分明るい」かつ「RGBが十分均等」の両方を満たした時だけ候補にする。
  function color_is_white(R: number, G: number, B: number, L: number): boolean {
    if (L < white_t) return false;
    return color_rgb_balance_percent(R, G, B) >= color_white_balance_limit();
  }

  function determineColorIDCalibrated(
    R: number,
    G: number,
    B: number,
    L: number,
  ): number {
    // 黒は明るさ、白は明るさ＋RGB均等度で先に判定する。
    // この順序により、赤と緑の境界などの混色を白と誤判定しにくくする。
    if (L <= black_t) return 0;
    if (color_is_white(R, G, B, L)) return 4;

    // 赤・緑の手動設定後は、記録したRGBの最短距離より設定境界を優先する。
    if (color_red_green_manual_override) return determineColorID(R, G, B, L);

    // 白・黒でない場合だけ、赤・緑・青の最短距離を調べる。
    let bestID = -1;
    let bestD = 999999999;
    if (color_cal_red_ok == 1) {
      let d = color_distance(
        R,
        G,
        B,
        L,
        color_cal_red_r,
        color_cal_red_g,
        color_cal_red_b,
        color_cal_red_l,
        false,
      );
      if (d < bestD) {
        bestD = d;
        bestID = 1;
      }
    }
    if (color_cal_green_ok == 1) {
      let d = color_distance(
        R,
        G,
        B,
        L,
        color_cal_green_r,
        color_cal_green_g,
        color_cal_green_b,
        color_cal_green_l,
        false,
      );
      if (d < bestD) {
        bestD = d;
        bestID = 2;
      }
    }
    if (color_cal_blue_ok == 1) {
      let d = color_distance(
        R,
        G,
        B,
        L,
        color_cal_blue_r,
        color_cal_blue_g,
        color_cal_blue_b,
        color_cal_blue_l,
        false,
      );
      if (d < bestD) {
        bestD = d;
        bestID = 3;
      }
    }
    // 緑・青の候補には、調整ブロックの境界をキャリブレーション後も適用する。
    if (bestID == 2 || bestID == 3) return determineGreenBlueID(G, B);
    if (bestID >= 0) return bestID;
    return determineColorID(R, G, B, L);
  }

  function color_scene_name(scene: color_scene): string {
    if (scene == color_scene.Red) return "R";
    if (scene == color_scene.Green) return "G";
    if (scene == color_scene.Blue) return "B";
    if (scene == color_scene.Black) return "K";
    if (scene == color_scene.White) return "W";
    return "?";
  }

  function color_name_from_id(id: number): string {
    if (id == 0) return "K";
    if (id == 1) return "R";
    if (id == 2) return "G";
    if (id == 3) return "B";
    if (id == 4) return "W";
    return "?";
  }

  /**
   * 現在値、保存済みの5色、各色までの参考距離、白判定条件、判定結果をシリアルへ表示します。
   * 「5色のキャリブレーションをする」実行後は、Bボタンでも呼び出せます。
   */
  //% color="#ffa500" weight=80 blockId=color_serial_values
  //% block="show color sensor debug values in serial" group="6 Color sensor"
  //% blockHidden=true
  export function color_serial_values(): void {
    color_ensure_settings_loaded();

    let rawValue = raw();
    let v = normalized_color_average(5);
    let R = v[0];
    let G = v[1];
    let B = v[2];
    let L = v[3];

    let blackLimit = black_t;
    if (color_cal_black_ok == 1 && color_cal_white_ok == 1) {
      let gap = color_cal_white_l - color_cal_black_l;
      if (gap < 20) gap = 20;
      blackLimit = Math.round(color_cal_black_l + gap * 0.25);
    }

    let dK = -1;
    let dR = -1;
    let dG = -1;
    let dB = -1;
    let dW = -1;
    if (color_cal_black_ok == 1) {
      dK = color_distance(
        R,
        G,
        B,
        L,
        color_cal_black_r,
        color_cal_black_g,
        color_cal_black_b,
        color_cal_black_l,
        true,
      );
    }
    if (color_cal_red_ok == 1) {
      dR = color_distance(
        R,
        G,
        B,
        L,
        color_cal_red_r,
        color_cal_red_g,
        color_cal_red_b,
        color_cal_red_l,
        false,
      );
    }
    if (color_cal_green_ok == 1) {
      dG = color_distance(
        R,
        G,
        B,
        L,
        color_cal_green_r,
        color_cal_green_g,
        color_cal_green_b,
        color_cal_green_l,
        false,
      );
    }
    if (color_cal_blue_ok == 1) {
      dB = color_distance(
        R,
        G,
        B,
        L,
        color_cal_blue_r,
        color_cal_blue_g,
        color_cal_blue_b,
        color_cal_blue_l,
        false,
      );
    }
    if (color_cal_white_ok == 1) {
      dW = color_distance(
        R,
        G,
        B,
        L,
        color_cal_white_r,
        color_cal_white_g,
        color_cal_white_b,
        color_cal_white_l,
        true,
      );
    }

    let resultID = determineColorID(R, G, B, L);
    if (color_auto_calibration_on == 1 && has_color_calibration()) {
      resultID = determineColorIDCalibrated(R, G, B, L);
    }

    serial.writeLine("===== COLOR DEBUG =====");
    serial.writeLine(
      "RAW C=" +
        rawValue[0] +
        " R=" +
        rawValue[1] +
        " G=" +
        rawValue[2] +
        " B=" +
        rawValue[3],
    );
    serial.writeLine("NOW R=" + R + " G=" + G + " B=" + B + " L=" + L);
    serial.writeLine(
      "WHITE balance=" +
        color_rgb_balance_percent(R, G, B) +
        " limit=" +
        color_white_balance_limit() +
        " bright=" +
        (L >= white_t) +
        " candidate=" +
        color_is_white(R, G, B, L),
    );
    serial.writeLine(
      "READY K=" +
        color_cal_black_ok +
        " W=" +
        color_cal_white_ok +
        " R=" +
        color_cal_red_ok +
        " G=" +
        color_cal_green_ok +
        " B=" +
        color_cal_blue_ok,
    );
    serial.writeLine(
      "INFO blackLimit=" +
        blackLimit +
        " white=" +
        white_t +
        " LbelowBlack=" +
        (L <= blackLimit),
    );
    serial.writeLine(
      "CAL K R=" +
        color_cal_black_r +
        " G=" +
        color_cal_black_g +
        " B=" +
        color_cal_black_b +
        " L=" +
        color_cal_black_l,
    );
    serial.writeLine(
      "CAL W R=" +
        color_cal_white_r +
        " G=" +
        color_cal_white_g +
        " B=" +
        color_cal_white_b +
        " L=" +
        color_cal_white_l,
    );
    serial.writeLine(
      "CAL R R=" +
        color_cal_red_r +
        " G=" +
        color_cal_red_g +
        " B=" +
        color_cal_red_b +
        " L=" +
        color_cal_red_l,
    );
    serial.writeLine(
      "CAL G R=" +
        color_cal_green_r +
        " G=" +
        color_cal_green_g +
        " B=" +
        color_cal_green_b +
        " L=" +
        color_cal_green_l,
    );
    serial.writeLine(
      "CAL B R=" +
        color_cal_blue_r +
        " G=" +
        color_cal_blue_g +
        " B=" +
        color_cal_blue_b +
        " L=" +
        color_cal_blue_l,
    );
    serial.writeLine(
      "DIST K=" + dK + " W=" + dW + " R=" + dR + " G=" + dG + " B=" + dB,
    );
    serial.writeLine("RESULT=" + color_name_from_id(resultID));
    serial.writeLine("=======================");
  }

  // micro:bit V2 の KeyValueStorage は最大5組、1つの値は32byteまで。
  // Bluetooth書き込み用に2組を空けるため、色データは次の3キーだけを使う。
  // c01 = K/W, c02 = R/G, c03 = B
  // 1つの数値を64進数3文字にして、0～65535を欠損なく保存する。
  // 1色は「有効フラグ1文字 + R/G/B/L 各3文字」= 13文字。
  let color_storage_alphabet =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz-_";

  function color_string_to_number(s: string): number {
    if (s.length == 0) return 0;
    return parseInt(s);
  }

  function color_unpack_values(s: string): number[] {
    let result: number[] = [0, 0, 0, 0];
    if (s.length == 0) return result;
    let parts = s.split(",");
    if (parts.length != 4) return result;
    result[0] = color_string_to_number(parts[0]);
    result[1] = color_string_to_number(parts[1]);
    result[2] = color_string_to_number(parts[2]);
    result[3] = color_string_to_number(parts[3]);
    return result;
  }

  function color_encode_fixed_number(v: number): string {
    v = Math.round(v);
    if (v < 0) v = 0;
    if (v > 65535) v = 65535;
    let d0 = Math.floor(v / 4096);
    let d1 = Math.floor((v % 4096) / 64);
    let d2 = v % 64;
    return (
      color_storage_alphabet.charAt(d0) +
      color_storage_alphabet.charAt(d1) +
      color_storage_alphabet.charAt(d2)
    );
  }

  function color_decode_fixed_number(s: string, start: number): number {
    if (start < 0 || start + 3 > s.length) return -1;
    let d0 = color_storage_alphabet.indexOf(s.charAt(start));
    let d1 = color_storage_alphabet.indexOf(s.charAt(start + 1));
    let d2 = color_storage_alphabet.indexOf(s.charAt(start + 2));
    if (d0 < 0 || d1 < 0 || d2 < 0) return -1;
    return d0 * 4096 + d1 * 64 + d2;
  }

  function color_pack_scene(
    ok: number,
    r: number,
    g: number,
    b: number,
    l: number,
  ): string {
    return (
      (ok == 1 ? "1" : "0") +
      color_encode_fixed_number(r) +
      color_encode_fixed_number(g) +
      color_encode_fixed_number(b) +
      color_encode_fixed_number(l)
    );
  }

  // 戻り値: [形式OK, 色有効, R, G, B, L]
  function color_unpack_scene(s: string, start: number): number[] {
    let result: number[] = [0, 0, 0, 0, 0, 0];
    if (start < 0 || start + 13 > s.length) return result;
    let okText = s.charAt(start);
    if (okText != "0" && okText != "1") return result;
    let r = color_decode_fixed_number(s, start + 1);
    let g = color_decode_fixed_number(s, start + 4);
    let b = color_decode_fixed_number(s, start + 7);
    let l = color_decode_fixed_number(s, start + 10);
    if (r < 0 || g < 0 || b < 0 || l < 0) return result;
    result[0] = 1;
    result[1] = okText == "1" ? 1 : 0;
    result[2] = r;
    result[3] = g;
    result[4] = b;
    result[5] = l;
    return result;
  }

  function color_remove_old_storage_keys(): void {
    // 旧版は26キーを使っていたため、V2では先頭5キーだけが残ることがある。
    flashstorage.remove("scal_v");
    flashstorage.remove("scal_ro");
    flashstorage.remove("scal_go");
    flashstorage.remove("scal_bo");
    flashstorage.remove("scal_ko");
    flashstorage.remove("scal_wo");
    flashstorage.remove("scal_rr");
    flashstorage.remove("scal_rg");
    flashstorage.remove("scal_rb");
    flashstorage.remove("scal_rl");
    flashstorage.remove("scal_gr");
    flashstorage.remove("scal_gg");
    flashstorage.remove("scal_gb");
    flashstorage.remove("scal_gl");
    flashstorage.remove("scal_br");
    flashstorage.remove("scal_bg");
    flashstorage.remove("scal_bb");
    flashstorage.remove("scal_bl");
    flashstorage.remove("scal_kr");
    flashstorage.remove("scal_kg");
    flashstorage.remove("scal_kb");
    flashstorage.remove("scal_kl");
    flashstorage.remove("scal_wr");
    flashstorage.remove("scal_wg");
    flashstorage.remove("scal_wb");
    flashstorage.remove("scal_wl");
  }

  function color_save_settings_internal(): void {
    // ColorキャリブレーションはRAMだけで使用する。
    // FlashはモーターバランスとBluetooth用に空けるため、保存しない。
    color_memory_valid = 0;
    color_memory_loaded = 1;
    return;

    // 旧5キーが全枠を使っていても新キーを作れるよう、先に解放する。
    flashstorage.remove("cK");
    flashstorage.remove("cW");
    flashstorage.remove("cR");
    flashstorage.remove("cG");
    flashstorage.remove("cB");

    let data01 =
      color_pack_scene(
        color_cal_black_ok,
        color_cal_black_r,
        color_cal_black_g,
        color_cal_black_b,
        color_cal_black_l,
      ) +
      color_pack_scene(
        color_cal_white_ok,
        color_cal_white_r,
        color_cal_white_g,
        color_cal_white_b,
        color_cal_white_l,
      );
    let data02 =
      color_pack_scene(
        color_cal_red_ok,
        color_cal_red_r,
        color_cal_red_g,
        color_cal_red_b,
        color_cal_red_l,
      ) +
      color_pack_scene(
        color_cal_green_ok,
        color_cal_green_r,
        color_cal_green_g,
        color_cal_green_b,
        color_cal_green_l,
      );
    let data03 = color_pack_scene(
      color_cal_blue_ok,
      color_cal_blue_r,
      color_cal_blue_g,
      color_cal_blue_b,
      color_cal_blue_l,
    );

    // 長さはそれぞれ 26, 26, 13byteで、32byte上限以内。
    flashstorage.put("c01", data01);
    flashstorage.put("c02", data02);
    flashstorage.put("c03", data03);
    color_memory_valid = 1;
    color_memory_loaded = 1;
  }

  function color_load_settings_internal(showMessage: boolean): boolean {
    // ColorキャリブレーションはFlashから読み込まず、現在の実行中に
    // 測定・設定されたRAM上の値だけを使用する。
    color_memory_loaded = 1;
    color_memory_valid = 0;
    return false;

    // 旧形式が残っている場合は、5キー上限を塞ぐので自動削除する。
    if (flashstorage.get("scal_v").length > 0) {
      color_remove_old_storage_keys();
    }

    let data01 = flashstorage.get("c01");
    let data02 = flashstorage.get("c02");
    let data03 = flashstorage.get("c03");

    // ---- 新3キー形式を読み込む ----
    if (data01.length == 26 && data02.length == 26 && data03.length == 13) {
      let k = color_unpack_scene(data01, 0);
      let w = color_unpack_scene(data01, 13);
      let r = color_unpack_scene(data02, 0);
      let g = color_unpack_scene(data02, 13);
      let b = color_unpack_scene(data03, 0);
      if (k[0] == 1 && w[0] == 1 && r[0] == 1 && g[0] == 1 && b[0] == 1) {
        color_cal_black_ok = k[1];
        color_cal_black_r = k[2];
        color_cal_black_g = k[3];
        color_cal_black_b = k[4];
        color_cal_black_l = k[5];
        color_cal_white_ok = w[1];
        color_cal_white_r = w[2];
        color_cal_white_g = w[3];
        color_cal_white_b = w[4];
        color_cal_white_l = w[5];
        color_cal_red_ok = r[1];
        color_cal_red_r = r[2];
        color_cal_red_g = r[3];
        color_cal_red_b = r[4];
        color_cal_red_l = r[5];
        color_cal_green_ok = g[1];
        color_cal_green_r = g[2];
        color_cal_green_g = g[3];
        color_cal_green_b = g[4];
        color_cal_green_l = g[5];
        color_cal_blue_ok = b[1];
        color_cal_blue_r = b[2];
        color_cal_blue_g = b[3];
        color_cal_blue_b = b[4];
        color_cal_blue_l = b[5];

        update_calibrated_white_black_thresholds();
        color_auto_calibration_on = 1;
        color_memory_valid = 1;
        return true;
      }
    }

    // ---- 旧5キー形式を読み込み、新3キー形式へ自動移行する ----
    // 5個すべてをRAMへ読んでから旧キーを削除する。
    let kText = flashstorage.get("cK");
    let wText = flashstorage.get("cW");
    let rText = flashstorage.get("cR");
    let gText = flashstorage.get("cG");
    let bText = flashstorage.get("cB");

    if (
      kText.length > 0 ||
      wText.length > 0 ||
      rText.length > 0 ||
      gText.length > 0 ||
      bText.length > 0
    ) {
      let v: number[];
      v = color_unpack_values(kText);
      color_cal_black_ok = kText.length > 0 ? 1 : 0;
      color_cal_black_r = v[0];
      color_cal_black_g = v[1];
      color_cal_black_b = v[2];
      color_cal_black_l = v[3];
      v = color_unpack_values(wText);
      color_cal_white_ok = wText.length > 0 ? 1 : 0;
      color_cal_white_r = v[0];
      color_cal_white_g = v[1];
      color_cal_white_b = v[2];
      color_cal_white_l = v[3];
      v = color_unpack_values(rText);
      color_cal_red_ok = rText.length > 0 ? 1 : 0;
      color_cal_red_r = v[0];
      color_cal_red_g = v[1];
      color_cal_red_b = v[2];
      color_cal_red_l = v[3];
      v = color_unpack_values(gText);
      color_cal_green_ok = gText.length > 0 ? 1 : 0;
      color_cal_green_r = v[0];
      color_cal_green_g = v[1];
      color_cal_green_b = v[2];
      color_cal_green_l = v[3];
      v = color_unpack_values(bText);
      color_cal_blue_ok = bText.length > 0 ? 1 : 0;
      color_cal_blue_r = v[0];
      color_cal_blue_g = v[1];
      color_cal_blue_b = v[2];
      color_cal_blue_l = v[3];

      color_save_settings_internal();
      update_calibrated_white_black_thresholds();
      color_auto_calibration_on = 1;
      color_memory_valid = 1;
      return true;
    }

    color_memory_valid = 0;
    return false;
  }

  function color_ensure_settings_loaded(): void {
    // Flashは使用しない。RAMの初期値またはこの実行中の調整値を使う。
    color_memory_loaded = 1;
  }

  function color_remove_settings_internal(): void {
    flashstorage.remove("c01");
    flashstorage.remove("c02");
    flashstorage.remove("c03");
    flashstorage.remove("cK");
    flashstorage.remove("cW");
    flashstorage.remove("cR");
    flashstorage.remove("cG");
    flashstorage.remove("cB");
    color_remove_old_storage_keys();
    color_memory_valid = 0;
    color_memory_loaded = 1;
  }

  /**
   * 現在センサーの下にある色を基準として記録します。
   * 白・黒・赤・緑・青を1回ずつ読ませると、個体差と照明差を補正できます。
   */
  //% color="#ffa500" weight=87 blockId=color_calibrate_scene
  //% block="calibrate color sensor for %scene" group="6 Color sensor"
  //% blockHidden=true
  export function color_calibrate(scene: color_scene): void {
    // 既に保存されている他の色を消さないよう、先に読み込む
    color_ensure_settings_loaded();
    let v = normalized_color_average(8);
    switch (scene) {
      case color_scene.Red:
        color_cal_red_r = v[0];
        color_cal_red_g = v[1];
        color_cal_red_b = v[2];
        color_cal_red_l = v[3];
        color_cal_red_ok = 1;
        break;
      case color_scene.Green:
        color_cal_green_r = v[0];
        color_cal_green_g = v[1];
        color_cal_green_b = v[2];
        color_cal_green_l = v[3];
        color_cal_green_ok = 1;
        break;
      case color_scene.Blue:
        color_cal_blue_r = v[0];
        color_cal_blue_g = v[1];
        color_cal_blue_b = v[2];
        color_cal_blue_l = v[3];
        color_cal_blue_ok = 1;
        break;
      case color_scene.Black:
        color_cal_black_r = v[0];
        color_cal_black_g = v[1];
        color_cal_black_b = v[2];
        color_cal_black_l = v[3];
        color_cal_black_ok = 1;
        break;
      case color_scene.White:
        color_cal_white_r = v[0];
        color_cal_white_g = v[1];
        color_cal_white_b = v[2];
        color_cal_white_l = v[3];
        color_cal_white_ok = 1;
        break;
    }
    update_calibrated_white_black_thresholds();
    color_save_settings_internal();
  }

  /**
   * Aボタンを押すたびに、黒・白・赤・緑・青の順で色を記録します。
   * LEDには、次に記録する色を K / W / R / G / B で表示します。
   */
  //% color="#ffa500" weight=88 blockId=color_calibrate_five_colors
  //% block="calibrate five colors" group="6 Color sensor"
  export function color_calibrate_five_colors(): void {
    // このブロックをもう一度実行した場合は、Kからやり直す。
    color_guided_calibration_step = 0;
    color_guided_calibration_busy = 0;

    // Aボタンの処理は、同じプログラム内で重複登録しない。
    if (color_guided_calibration_handler_registered == 0) {
      color_guided_calibration_handler_registered = 1;
      input.onButtonPressed(Button.A, function () {
        // 測定中の連打と、5色完了後の追加操作を無視する。
        if (color_guided_calibration_busy == 1) return;
        if (
          color_guided_calibration_step < 0 ||
          color_guided_calibration_step > 4
        )
          return;

        color_guided_calibration_busy = 1;

        if (color_guided_calibration_step == 0) {
          color_calibrate(color_scene.Black);
        } else if (color_guided_calibration_step == 1) {
          color_calibrate(color_scene.White);
        } else if (color_guided_calibration_step == 2) {
          color_calibrate(color_scene.Red);
        } else if (color_guided_calibration_step == 3) {
          color_calibrate(color_scene.Green);
        } else {
          color_calibrate(color_scene.Blue);
        }

        color_guided_calibration_step += 1;
        basic.showIcon(IconNames.Yes);

        if (color_guided_calibration_step < 5) {
          basic.pause(500);
          if (color_guided_calibration_step == 1) {
            basic.showString("W");
          } else if (color_guided_calibration_step == 2) {
            basic.showString("R");
          } else if (color_guided_calibration_step == 3) {
            basic.showString("G");
          } else {
            basic.showString("B");
          }
        }

        color_guided_calibration_busy = 0;
      });
    }

    // デバッグ時は、Bボタンを押すと現在値・保存値・判定内容をシリアルへ表示する。
    // 同じプログラム内でイベントを重複登録しない。
    if (color_debug_handler_registered == 0) {
      color_debug_handler_registered = 1;
      input.onButtonPressed(Button.B, function () {
        if (color_guided_calibration_busy == 1) return;
        color_serial_values();
      });
    }

    basic.showString("K");
  }

  /**
   * キャリブレーション済みの値をプログラム内に設定します。
   * filesystemを使わず、起動時にこのブロックを並べれば毎回キャリブレーション不要です。
   */
  //% color="#ffa500" weight=94 blockId=color_calibration_set_values
  //% block="set color calibration %scene R %r G %g B %b L %l" group="6 Color sensor"
  //% r.min=0 r.max=65535 r.defl=0
  //% g.min=0 g.max=65535 g.defl=0
  //% b.min=0 b.max=65535 b.defl=0
  //% l.min=0 l.max=65535 l.defl=0
  //% blockHidden=true
  export function color_calibration_set_values(
    scene: color_scene,
    r: number,
    g: number,
    b: number,
    l: number,
  ): void {
    // 既に保存されている他の色を消さないよう、先に読み込む。
    color_ensure_settings_loaded();
    if (r < 0) r = 0;
    if (g < 0) g = 0;
    if (b < 0) b = 0;
    if (l < 0) l = 0;
    if (r > 65535) r = 65535;
    if (g > 65535) g = 65535;
    if (b > 65535) b = 65535;
    if (l > 65535) l = 65535;

    switch (scene) {
      case color_scene.Red:
        color_cal_red_r = r;
        color_cal_red_g = g;
        color_cal_red_b = b;
        color_cal_red_l = l;
        color_cal_red_ok = 1;
        break;
      case color_scene.Green:
        color_cal_green_r = r;
        color_cal_green_g = g;
        color_cal_green_b = b;
        color_cal_green_l = l;
        color_cal_green_ok = 1;
        break;
      case color_scene.Blue:
        color_cal_blue_r = r;
        color_cal_blue_g = g;
        color_cal_blue_b = b;
        color_cal_blue_l = l;
        color_cal_blue_ok = 1;
        break;
      case color_scene.Black:
        color_cal_black_r = r;
        color_cal_black_g = g;
        color_cal_black_b = b;
        color_cal_black_l = l;
        color_cal_black_ok = 1;
        break;
      case color_scene.White:
        color_cal_white_r = r;
        color_cal_white_g = g;
        color_cal_white_b = b;
        color_cal_white_l = l;
        color_cal_white_ok = 1;
        break;
    }
    color_auto_calibration_on = 1;
    update_calibrated_white_black_thresholds();
    // プリセット値を入れた場合も保存できるようにする
    color_save_settings_internal();
  }

  /**
   * micro:bit本体のFlash Storageに保存されている色キャリブレーション値を読み込みます。
   * 通常は色判定時に自動で読み込みますが、確認用に手動でも実行できます。
   */
  //% color="#ffa500" weight=94 blockId=color_calibration_load_flash
  //% block="load color calibration from flash" group="6 Color sensor"
  //% blockHidden=true
  export function color_calibration_load_flash(): void {
    color_load_settings_internal(true);
  }

  /**
   * 現在の色キャリブレーション値をmicro:bit本体のFlash Storageへ保存します。
   * color sensor calibrate 実行時にも自動保存されます。
   */
  //% color="#ffa500" weight=94 blockId=color_calibration_save_flash
  //% block="save color calibration to flash" group="6 Color sensor"
  //% blockHidden=true
  export function color_calibration_save_flash(): void {
    color_save_settings_internal();
  }

  /**
   * キャリブレーション判定を使うかどうかを設定します。
   */
  //% color="#ffa500" weight=94 blockId=color_calibration_mode
  //% block="turn color calibration %mode" group="6 Color sensor"
  //% blockHidden=true
  export function color_calibration_mode(mode: onoff): void {
    if (mode == onoff.ON) color_auto_calibration_on = 1;
    else color_auto_calibration_on = 0;
  }

  /**
   * 記録した色基準を消します。
   */
  //% color="#ffa500" weight=86 blockId=color_calibration_reset
  //% block="clear color sensor calibration" group="6 Color sensor"
  export function color_calibration_reset(): void {
    color_cal_red_ok = 0;
    color_cal_green_ok = 0;
    color_cal_blue_ok = 0;
    color_cal_black_ok = 0;
    color_cal_white_ok = 0;
    color_auto_calibration_on = 1;
    white_t = 150;
    black_t = 30;
    color_remove_settings_internal();
  }

  //% color="#ffa500" weight=85 blockId=color_calibration_ready
  //% block="color sensor is calibrated" group="6 Color sensor"
  export function color_calibration_ready(): boolean {
    color_ensure_settings_loaded();
    return (
      color_cal_red_ok == 1 &&
      color_cal_green_ok == 1 &&
      color_cal_blue_ok == 1 &&
      color_cal_black_ok == 1 &&
      color_cal_white_ok == 1
    );
  }

  //% color="#ffa500" weight=88 blockId=selectcycle
  //% block="set color sensor measurement time to %cycle" group="6 Color sensor"
  export function selectcycle(cycle: colorcycle): void {
    switch (cycle) {
      case colorcycle.cycle1:
        color_cycle = 1;
        smbus.writeByte(0x81, 0xff);
        break;
      case colorcycle.cycle10:
        color_cycle = 10;
        smbus.writeByte(0x81, 0xf6);
        break;
      case colorcycle.cycle42:
        color_cycle = 42;
        smbus.writeByte(0x81, 0xd5);
        break;
      case colorcycle.cycle64:
        color_cycle = 64;
        smbus.writeByte(0x81, 0xc0);
        break;
    }
  }

  //% color="#ffa500" weight=86
  //% block="set thresholds white: %white_threshold or above black: %black_threshold or below" group="6 Color sensor"
  //% white_threshold.min=100 white_threshold.max=400 white_threshold.defl=250
  //% black_threshold.min=0 black_threshold.max=200 black_threshold.defl=30
  //% blockHidden=true
  export function WB_threshold(
    white_threshold: number,
    black_threshold: number,
  ): void {
    white_t = white_threshold;
    black_t = black_threshold;
  }

  /**
   * 緑と青の境界をB/Gの百分率で設定します。初期値は90です。
   * 青紙を緑と誤判定する場合は小さく、緑紙を青と誤判定する場合は大きくします。
   * キャリブレーション後も有効です。設定はRAMのみで、再起動すると90に戻ります。
   */
  //% color="#ffa500" weight=85 blockId=color_green_blue_threshold
  //% block="set green/blue threshold B/G to %percent percent" group="6 Color sensor"
  //% percent.min=50 percent.max=150 percent.defl=90
  export function color_green_blue_threshold(percent: number): void {
    if (percent != percent) return;
    color_green_blue_threshold_percent = Math.round(Math.max(50, Math.min(150, percent)));
  }

  function determineGreenBlueID(G: number, B: number): number {
    // 除算を避け、Gが0でも判定できる。同値は青とする。
    return B * 100 >= G * color_green_blue_threshold_percent ? 3 : 2;
  }

  /**
   * 赤・緑の境界をR/Gの百分率で設定します。初期値は150です。
   * 赤紙が緑になる場合は小さく、緑紙が赤になる場合は大きくします。
   * 青との区別にはRがBの150％以上という条件も使います。
   * 設定後はキャリブレーションのRGB最短距離より手動の境界を優先します。
   * 再起動で150％に戻るため、設定ブロックは「最初だけ」に置いてください。
   */
  //% color="#ffa500" weight=85 blockId=color_red_green_threshold
  //% block="set red/green threshold R/G to $percent percent" group="6 Color sensor"
  //% percent.min=50 percent.max=300 percent.defl=150
  export function color_red_green_threshold(percent: number): void {
    if (percent != percent) return;
    color_red_green_threshold_percent = Math.round(Math.max(50, Math.min(300, percent)));
    color_red_green_manual_override = true;
  }

  //% color="#ffa500" weight=83 blockId=color_ID block="color ID" group="6 Color sensor"
  export function color_ID(): number {
    color_ensure_settings_loaded();
    // 複数回平均して、色境界や走行中の瞬間的な読み取り揺れを抑える。
    let v = normalized_color_average(color_judgement_samples);
    let R = v[0],
      G = v[1],
      B = v[2],
      L = v[3];
    if (color_auto_calibration_on == 1 && has_color_calibration()) {
      return determineColorIDCalibrated(R, G, B, L);
    }
    return determineColorID(R, G, B, L);
  }

  function determineColorID(
    R: number,
    G: number,
    B: number,
    L: number,
  ): number {
    if (L <= black_t) return 0;
    else if (color_is_white(R, G, B, L)) return 4;
    else {
      if (R * 100 >= G * color_red_green_threshold_percent && R >= 1.5 * B) return 1;
      return determineGreenBlueID(G, B);
    }
  }

  //% color="#ffa500" block="detected color" weight=90 group="6 Color sensor"
  export function getColorName(): string {
    return color_name_from_id(color_ID());
  }

  //% color="#ffa500" weight=89 block="color is %scene" group="6 Color sensor"
  export function color1(scene: color_scene): boolean {
    switch (scene) {
      case color_scene.Red:
        return color_ID() == 1;
      case color_scene.Green:
        return color_ID() == 2;
      case color_scene.Blue:
        return color_ID() == 3;
      case color_scene.Black:
        return color_ID() == 0;
      case color_scene.White:
        return color_ID() == 4;
    }
  }

  namespace smbus {
    export function writeByte(register: number, value: number): void {
      let temp = pins.createBuffer(2);
      temp[0] = register;
      temp[1] = value;
      pins.i2cWriteBuffer(0x29, temp, false);
    }
    export function readBuffer(register: number, len: number): Buffer {
      let temp = pins.createBuffer(1);
      temp[0] = register;
      // ★安定化：repeated start を使う（相手によっては必須）
      pins.i2cWriteBuffer(0x29, temp, true);
      return pins.i2cReadBuffer(0x29, len, false);
    }
    export function unpack(fmt: string, buf: Buffer): number[] {
      let le: boolean = true;
      let offset: number = 0;
      let result: number[] = [];
      let num_format: NumberFormat = 0;
      for (let c = 0; c < fmt.length; c++) {
        switch (fmt.charAt(c)) {
          case "<":
            le = true;
            continue;
          case ">":
            le = false;
            continue;
          case "c":
          case "B":
            num_format = le ? NumberFormat.UInt8LE : NumberFormat.UInt8BE;
            break;
          case "b":
            num_format = le ? NumberFormat.Int8LE : NumberFormat.Int8BE;
            break;
          case "H":
            num_format = le ? NumberFormat.UInt16LE : NumberFormat.UInt16BE;
            break;
          case "h":
            num_format = le ? NumberFormat.Int16LE : NumberFormat.Int16BE;
            break;
        }
        result.push(buf.getNumber(num_format, offset));
        offset += pins.sizeOf(num_format);
      }
      return result;
    }
  }

  // ===== NeoPixel =====
  //% color="#808080" weight=80 blockId=neopixel_white block="turn white LED %mode at brightness %BR" group="7 Color LEDs"
  //% BR.min=0 BR.max=255 BR.defl=15
  export function neopixel_white_block(mode: onoff, BR: number) {
    switch (mode) {
      case onoff.ON:
        io_neo.setBrightness(BR);
        for (let n = 0; n < 2; n++) {
          io_neo.setPixelColor(n, neopixel.colors(NeoPixelColors.White));
        }
        io_neo.show();
        break;
      case onoff.OFF:
        for (let n = 0; n < 2; n++) {
          io_neo.setPixelColor(n, neopixel.colors(NeoPixelColors.Black));
        }
        io_neo.show();
        break;
    }
  }

  //% color="#000000" weight=79 blockId=neopixel_erace block="turn off color LEDs" group="7 Color LEDs"
  export function neopixel_erace_block() {
    for (let n = 0; n < 2; n++) {
      io_neo.setPixelColor(n, neopixel.colors(NeoPixelColors.Black));
    }
    io_neo.show();
  }

  //% color="#708090" weight=78 blockId=neopixel_Brightness block="set color LED brightness to %neo_Brightness" group="7 Color LEDs"
  //% neo_Brightness.min=0 neo_Brightness.max=255 neo_Brightness.defl=15
  export function neopixel_Brightness_block(neo_Brightness: number) {
    io_neo.setBrightness(neo_Brightness);
  }

  //% color.shadow="colorNumberPicker"
  //% color="#708090" weight=77 blockId=neopixel_set block="set %Side LED to $color" group="7 Color LEDs"
  export function neopixel_set_block(Side: LEDSide, color: number) {
    switch (Side) {
      case LEDSide.Left:
        io_neo.setPixelColor(0, color);
        io_neo.show();
        break;
      case LEDSide.Right:
        io_neo.setPixelColor(1, color);
        io_neo.show();
        break;
      case LEDSide.Both:
        io_neo.setPixelColor(0, color);
        io_neo.setPixelColor(1, color);
        io_neo.show();
        break;
    }
  }

  //% color.shadow="colorNumberPicker"
  //% color="#708090" weight=76 blockId=neopixel_select block="light %neo_number LEDs in $color" group="7 Color LEDs"
  //% neo_number.min=0 neo_number.max=2 neo_number.defl=1
  export function neopixel_select_block(neo_number: number, color: number) {
    for (let n = 0; n < 2; n++)
      io_neo.setPixelColor(n, neopixel.colors(NeoPixelColors.Black));
    io_neo.show();
    for (let n = 0; n < neo_number; n++) io_neo.setPixelColor(n, color);
    io_neo.show();
  }

  // ===== 人感センサー =====
  //% color="#a0522d" weight=82 block="human motion detected" group="8 Motion sensor"
  export function IO_humanDetection(): boolean {
    pins.setPull(DigitalPin.P14, PinPullMode.PullNone);
    return pins.digitalReadPin(DigitalPin.P14) == 1;
  }
  //% color="#a0522d" weight=81 blockId=IO_human block="human sensor value" group="8 Motion sensor"
  export function IO_human(): number {
    pins.setPull(DigitalPin.P14, PinPullMode.PullNone);
    return pins.digitalReadPin(DigitalPin.P14);
  }
  //% color="#a0522d" weight=80 blockId=IO_human_DISP block="show human sensor value" group="8 Motion sensor"
  export function IO_human_DISP() {
    pins.setPull(DigitalPin.P14, PinPullMode.PullNone);
    basic.showNumber(pins.digitalReadPin(DigitalPin.P14));
  }

  // ===== micro:bit光センサー =====
  //% color="#009A00" weight=82 blockId=microbit2_decideLight block="brightness compared with %limit is %brightness" group="9 micro:bit light sensor"
  //% block.loc.ja="明るさが %limit より %brightness"
  //% limit.min=0 limit.max=100
  export function microbit2_decideLight(
    limit: number,
    brightness: BT,
  ): boolean {
    switch (brightness) {
      case BT.darker:
        return (input.lightLevel() / 254) * 100 <= limit;
      case BT.lighter:
        return (input.lightLevel() / 254) * 100 >= limit;
    }
  }

  //% color="#009A00" weight=81 blockId=microbit2_denkitemp block="brightness" group="9 micro:bit light sensor"
  export function microbit2_denkitemp(): number {
    return Math.round((input.lightLevel() / 254) * 100);
  }
  //% color="#228b22" weight=80 blockId=microbit2_denkiLED block="show brightness" group="9 micro:bit light sensor"
  export function microbit2_denkiLED() {
    basic.showNumber(Math.round((input.lightLevel() / 254) * 100));
  }

  // ===== リレー制御 =====
  //% color="#4682b4" weight=82 blockId=IO_relay block="turn relay %mode"
  //% blockHidden=true
  export function IO_relay(mode: onoff) {
    switch (mode) {
      case onoff.ON:
        return pins.digitalWritePin(DigitalPin.P8, 1);
      case onoff.OFF:
        return pins.digitalWritePin(DigitalPin.P8, 0);
    }
  }
  //% color="#4682b4" weight=81 blockId=IO_relay_2 block="output analog value %syuturyoku to relay"
  //% syuturyoku.min=0 syuturyoku.max=1023 syuturyoku.defl=1023
  //% blockHidden=true
  export function IO_relay_2(syuturyoku: number) {
    return pins.analogWritePin(AnalogPin.P8, syuturyoku);
  }

  // ===== 周期処理 =====
  // 起動時に旧Color保存領域を解放し、モーターバランスを先に読み込む。
  // これにより最初の走行命令から保存済みの最適値が使用される。
  motor_balance_load_internal();

  // 超音波センサーは裏側で連続読み取りする。
  // 読み取りブロック側では保存済みの u_dis を見るだけにして、走行判定の遅れを減らす。
  control.inBackground(function () {
    basic.pause(500); // 電源投入直後のI2C超音波センサー起動待ち
    while (true) {
      let d = serp.sonar_ping_2();
      // sonar_ping_2 が正常実測値を返したときだけ予測更新する。
      // 読み取り失敗中は前回の予測値を維持する。
      if (sonar_fail_count == 0) {
        update_sonar_prediction(d);
      } else if (d == 999) {
        update_sonar_prediction(999);
      }
      basic.pause(1); // 反応速度優先。sonar_ping_2内の120ms待ちが実質周期になる。
    }
  });

  basic.forever(function () {
    // ---- 切替直後1フレームは固定出力して補正をスキップ ----
    if (rotation_fault_latched) { motion_outputs_off(); return; }
    if (just_switched) {
      let p_base = p / 100;
      if (drive_mode == 1) {
        // 前進
        pins.servoWritePin(AnalogPin.P16, 90 + 90 * p_base);
        pins.servoWritePin(AnalogPin.P15, 90 - 90 * p_base);
      } else if (drive_mode == 2) {
        // 後進
        pins.servoWritePin(AnalogPin.P16, 90 - 90 * p_base);
        pins.servoWritePin(AnalogPin.P15, 90 + 90 * p_base);
      } else if (drive_mode == 3) {
        // 左回転
        pins.servoWritePin(AnalogPin.P16, 90 - 90 * p_base);
        pins.servoWritePin(AnalogPin.P15, 90 - 90 * p_base);
      } else if (drive_mode == 4) {
        // 右回転
        pins.servoWritePin(AnalogPin.P16, 90 + 90 * p_base);
        pins.servoWritePin(AnalogPin.P15, 90 + 90 * p_base);
      } else if (drive_mode == 5) {
        // 左カーブ（右のみ）
        pins.servoWritePin(AnalogPin.P16, 90);
        pins.servoWritePin(AnalogPin.P15, 90 - 90 * p_base);
      } else if (drive_mode == 6) {
        // 右カーブ（左のみ）
        pins.servoWritePin(AnalogPin.P16, 90 + 90 * p_base);
        pins.servoWritePin(AnalogPin.P15, 90);
      }
      just_switched = false;
      return;
    }

    // 前進・後進は中心探索型の共通処理を使用する。
    // 時間指定ブロックも同じ cont_forward / cont_backward を通るため、ここで対応される。
    if (drive_mode == 1 || drive_mode == 2) {
      update_continuous_straight();
      return;
    }

    // ---- 右・左・右回転・左回転は従来の補正駆動 ----
    let diff = Math.abs(L_counter - R_counter);
    let maxDiff = 3; // この差以上では最大補正
    if (diff > maxDiff) diff = maxDiff;

    // 差分に応じて補正量を変える（差が大きいほど n1/n2 の差が大きくなる）
    let correction = diff / maxDiff; // 0〜1
    let n1 = 1.0 - correction * 0.8;
    let n2 = 1.0 + correction * 0.8;

    // 左回転モード
    if (drive_mode == 3) {
      if (L_counter < R_counter) {
        pins.servoWritePin(AnalogPin.P16, 90 - (90 * p * n2) / 100);
        pins.servoWritePin(AnalogPin.P15, 90 - (90 * p * n1) / 100);
      } else {
        pins.servoWritePin(AnalogPin.P16, 90 - (90 * p * n1) / 100);
        pins.servoWritePin(AnalogPin.P15, 90 - (90 * p * n2) / 100);
      }
    }
    // 右回転モード
    if (drive_mode == 4) {
      if (L_counter < R_counter) {
        pins.servoWritePin(AnalogPin.P16, 90 + (90 * p * n2) / 100);
        pins.servoWritePin(AnalogPin.P15, 90 + (90 * p * n1) / 100);
      } else {
        pins.servoWritePin(AnalogPin.P16, 90 + (90 * p * n1) / 100);
        pins.servoWritePin(AnalogPin.P15, 90 + (90 * p * n2) / 100);
      }
    }
    // （カーブ 5/6 は即時出力のみ／必要なら同様に補正可）
  });
} // namespace serp
