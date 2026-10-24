# SERP programming car / イーオプログラミングカー

micro:bit 用の Microsoft MakeCode 拡張機能です。車の走行、サーボ、ラインセンサー、超音波距離センサー、カラーセンサー、カラーLEDなどをブロックで操作します。

## 拡張機能の追加

リポジトリは `https://github.com/itibanbosi/serp` です。次の手順で追加します。

1. https://makecode.microbit.org/ でプロジェクトを開きます。
2. 「拡張機能」を開きます。
3. `https://github.com/itibanbosi/serp` を検索欄に貼り付けて追加します。

URLによる追加と、拡張機能の一般検索への掲載は別です。一般検索への掲載には MakeCode の承認が必要です。

## 対象と接続

SERPの車体・対応回路を接続したmicro:bitで使用します。実機での動作確認が必要です。

| 機器 | 接続 |
| --- | --- |
| 左モーター | P16 |
| 右モーター | P15 |
| 左ラインセンサー | P1 |
| 右ラインセンサー | P0 |
| 左ホイールセンサー | P13 |
| 右ホイールセンサー | P2 |
| 超音波距離センサー | I2C、アドレス 0x57 |
| カラーセンサー | I2C |

モーターバランス補正値はFlashへ保存します。カラーセンサーの調整値は実行中のRAMだけに保持されます。

## 使用例

```typescript
// 停止ブロックの呼び出し例
serp.car_stop()
```

## API一覧

ブロック表示に対応する公開関数と型を以下に示します。引数の選択肢はMakeCodeのブロック上で選択できます。

| 関数 | 説明 |
| --- | --- |
| `serp.driveForwards(second: number)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.set_motor_balance(balance: number)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.car_move(sinkou_houkou: direction3, power: number)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.car_stop()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.car_derection1(sinkou_houkou: direction, Power: number,)` | 次の命令が来るまで、連続動作します。スピードは、0~100で、100で最速です。 |
| `serp.car_derection2(sinkou_houkou: direction, time_sec: number, Power: number,)` | 時間指定分動作します。スピードは0~100の範囲で、100が最速です。 |
| `serp.car_stepmove6(sinkou_houkou: direction2, step: number)` | ステップ数で動かします。28ステップで1回転、1ステップは約4.8mm |
| `serp.car_stepmove7(sinkou_houkou: direction3, step: number)` | 長さ指定で動かします。 |
| `serp.car_stepmove8(sinkou_houkou: direction4, step: number)` | 角度指定で、回転します。 |
| `serp.car_pivot_90_basic(sinkou_houkou: direction7, angle: number,)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.car_stepmove5(L_p: number, R_p: number)` | 左右のサーボ出力をコントロールします。マイナスを入れると、逆転できます。 |
| `serp.blocklength(square: number)` | 1マスの長さをへんこうできます |
| `serp.wheel_rotation_adjust(leftPercent: number, rightPercent: number,)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.step_speed_adjust(straightPercent: number, rotationPercent: number,)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.straight_recovery_adjust(threshold: number, overshoot: number, slowPercent: number,)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.straight_auto_balance_adjust(sw: onoff, sampleCount: number, strength: number, maxTrim: number,)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.straight_auto_balance_reset()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.straight_center_learning_adjust(sw: onoff, windowCount: number, strength: number, deadband: number,)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.straight_path_correction_adjust(sw: onoff, startCount: number, strength: number, maxDiff: number,)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.straight_short_correction_adjust(sw: onoff, sampleCount: number, windowCount: number, pathStrength: number,)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.car_square1(sinkou_houkou: direction6, F_block: number,)` | まえ（うしろ）に、きめられたマスの数だけうごきます |
| `serp.car_square2(sinkou_houkou: direction5)` | 直角（ちょっかく）方向へまわります |
| `serp.car_pivot_90(sinkou_houkou: direction7, angle: number,)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.rotation_debug(enable: boolean)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.rotation_fault_print()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.rotation_has_fault()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.rotation_log_print()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.straight_debug_start()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.straight_debug_print()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.sonar_ping_2()` | 障がい物までの距離を返します |
| `serp.sonar_ping_3(limit: number, nagasa: kyori)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.obstacle_near(limit: number)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.setPhotoSikii(threshold: number)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.phto_R()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.phto_L()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.photoSensor(pos: EventSide, wb: whiteblack)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.line_detected(pos: EventSide)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.onEvent(side: EventSide, wb: whiteblack, handler: () => void,)` | ラインセンサーの左右と白黒を判断させるブロック |
| `serp.setServoAngle(pin: ServoPin, angle: number)` | Set the angle of the servo motor connected to the selected pin. |
| `serp.colorsensor_value(color: color_sensor)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.rgb()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.raw()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.color_serial_values()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.color_calibrate(scene: color_scene)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.color_calibrate_five_colors()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.color_calibration_set_values(scene: color_scene, r: number, g: number, b: number, l: number,)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.color_calibration_load_flash()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.color_calibration_save_flash()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.color_calibration_mode(mode: onoff)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.color_calibration_reset()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.color_calibration_ready()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.selectcycle(cycle: colorcycle)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.WB_threshold(white_threshold: number, black_threshold: number,)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.color_green_blue_threshold(percent: number)` | 緑と青の境界をB/Gの百分率で設定します。初期値は90です。青紙が緑になる場合は小さく、緑紙が青になる場合は大きくします。キャリブレーション後も有効です。再起動すると90に戻ります。 |
| `serp.color_red_green_threshold(percent: number)` | 赤・緑の境界をR/Gの百分率で設定します。初期値は150です。赤紙が緑になる場合は小さく、緑紙が赤になる場合は大きくします。青との区別にはRがBの150％以上という条件も使います。設定後はキャリブレーションのRGB最短距離より手動の境界を優先します。再起動すると150％に戻るため「最初だけ」に置いてください。 |
| `serp.color_ID()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.getColorName()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.color1(scene: color_scene)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.writeByte(register: number, value: number)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.readBuffer(register: number, len: number)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.unpack(fmt: string, buf: Buffer)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.neopixel_white_block(mode: onoff, BR: number)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.neopixel_erace_block()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.neopixel_Brightness_block(neo_Brightness: number)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.neopixel_set_block(Side: LEDSide, color: number)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.neopixel_select_block(neo_number: number, color: number)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.IO_humanDetection()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.IO_human()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.IO_human_DISP()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.microbit2_decideLight(limit: number, brightness: BT,)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.microbit2_denkitemp()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.microbit2_denkiLED()` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.IO_relay(mode: onoff)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |
| `serp.IO_relay_2(syuturyoku: number)` | 引数・動作はMakeCodeのブロック表示を参照してください。 |

## 依存拡張機能

- MakeCode標準の core / radio / microphone
- microsoft/pxt-neopixel（v0.7.5）
- bsiever/microbit-pxt-flashstorage

依存拡張機能にはそれぞれのライセンスが適用されます。

## 開発と公開

`pxt.json`、`serp.ts`、`_locales`、`LICENSE.txt` など、このフォルダーの内容をリポジトリのルートに配置します。
`main.ts`、`main.blocks`、`test.ts` は拡張機能自体を開く場合だけ読み込むテスト用ファイルです。

公開前にMakeCodeでインポート・コンパイル・実機確認を行い、GitHubリリースとバージョンタグ（例: `v0.0.1`）を作成します。初回バージョンは `0.0.1` です。

一般検索への承認申請前には、公開リポジトリの説明、リリース、アイコン、APIドキュメントとコンパイル結果を確認してください。現在の公開API名は互換性のため維持しており、承認時に命名規約の確認が必要です。

## ライセンス

MIT License。著作権表記は `Copyright (c) 2026 itibanbosi` です。詳細は [LICENSE.txt](LICENSE.txt) を参照してください。

## MakeCode メタデータ

for PXT/microbit
