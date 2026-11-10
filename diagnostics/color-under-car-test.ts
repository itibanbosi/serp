// MakeCodeでSERP拡張機能を追加し、このコードをJavaScript画面に貼り付ける。
// 車体下に取り付けたセンサーで実測した基準値。別の車体では測り直す。
serp.color_calibration_set_values(serp.color_scene.Black, 194, 221, 256, 39)
serp.color_calibration_set_values(serp.color_scene.White, 233, 256, 230, 254)
serp.color_calibration_set_values(serp.color_scene.Red, 256, 194, 228, 95)
serp.color_calibration_set_values(serp.color_scene.Green, 181, 256, 254, 81)
serp.color_calibration_set_values(serp.color_scene.Blue, 151, 213, 256, 77)

// 基準値の登録後に、実機で確認した白・黒と緑・青のしきい値を設定する。
serp.WB_threshold(175, 58)
serp.color_green_blue_threshold(112)

input.onButtonPressed(Button.A, function () {
    serp.color_serial_values()
})

basic.forever(function () {
    basic.showString(serp.getColorName())
})
