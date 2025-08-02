# ZMK IQS7211E ドライバ

このドライバは、Azoteq IQS7211E静電容量式タッチセンサーをZMKフレームワークで使用できるようにします。

## 概要

IQS7211Eは、トラックパッドアプリケーション向けに設計された低消費電力(1.5mA,データシートより)のタッチコントローラーです。このドライバはI2Cインターフェースを介してIQS7211Eセンサーと通信し、ZMKにマウス移動入力を提供します。

## 機能

- 指位置検出による静電容量式タッチトラッキング
- 2本指までのマルチタッチ
  - 1本指の移動でカーソル移動
  - 1本指のタップで左クリック
  - 2本指の移動でスクロール
  - 2本指のタップで右クリック

## インストール

1. ZMKモジュールとして追加：

```yaml
manifest:
  remotes:
    - name: zmkfirmware
      url-base: https://github.com/zmkfirmware
    - name: sekigon-gonnoc
      url-base: https://github.com/sekigon-gonnoc
  projects:
    - name: zmk
      remote: zmkfirmware
      revision: main
      import: app/west.yml
    - name: zmk-driver-iqs7211e
      remote: sekigon-gonnoc
```

## デバイスツリー設定

シールドまたはボード設定ファイル（.overlayまたは.dtsi）で設定：

```dts
&pinctrl {
    i2c0_default: i2c0_default {
        group1 {
            psels = <NRF_PSEL(TWIM_SDA, 0, 17)>,
                    <NRF_PSEL(TWIM_SCL, 0, 20)>;
            bias-pull-up;
        };
    };

    i2c0_sleep: i2c0_sleep {
        group1 {
            psels = <NRF_PSEL(TWIM_SDA, 0, 17)>,
                    <NRF_PSEL(TWIM_SCL, 0, 20)>;
            bias-pull-up;
            low-power-enable;
        };
    };
};

&i2c0 {
    status = "okay";
    compatible = "nordic,nrf-twim";
    pinctrl-0 = <&i2c0_default>;
    pinctrl-1 = <&i2c0_sleep>;
    pinctrl-names = "default", "sleep";
    clock-frequency = <I2C_BITRATE_FAST>;
    zephyr,concat-buf-size = <32>;

    trackpad: trackpad@56 {
        status = "okay";
        compatible = "azoteq,iqs7211e";
        reg = <0x56>;
        irq-gpios = <&gpio0 21 GPIO_ACTIVE_LOW>;
        power-gpios = <&gpio0 22 GPIO_ACTIVE_HIGH>;
    };
};
```

## キーボードのKconfigファイルでモジュールを有効化

キーボードの `Kconfig.defconfig` に以下を追加：

```kconfig
if ZMK_KEYBOARD_YOUR_KEYBOARD

config ZMK_POINTING
    default y

config IQS7211E
    default y

endif
```

## プロパティ

- `irq-gpios`: RDY/モーションピンに接続されたGPIO（必須、アクティブロー）
- `power-gpios`: 電源制御ピンに接続されたGPIO（任意、アクティブハイ）

## ハードウェアセットアップ

### I2Cアドレス
IQS7211Eはデフォルトで I2Cアドレス `0x56` を使用します。

### ピン接続
- **VDD**: 3.3V電源供給
- **GND**: グランド
- **SDA**: I2Cデータライン
- **SCL**: I2Cクロックライン
- **RDY**: Ready/Motionピン（アクティブロー、IRQ GPIOに接続）

### 電源制御（任意）
電源制御GPIOを使用する場合：
- センサーへの電源供給を制御するGPIOピンに接続
- スリープモード中の完全電源遮断に有用

## 設定

ドライバは `iqs7211e_init.h` の事前定義された設定を使用します：
- タッチ感度閾値
- ATI（自動調整実装）パラメータ
- 電源モード設定
- レポートレートとタイミング

これらの設定は使用するトラックパッドの構成に合わせて変更してください。デフォルトでは[低消費電力円形トラックパッド](https://nogikes.booth.pm/items/7254791)用の設定になっています。

## トラブルシューティング

### デバイスが見つからない
- I2C配線を確認
- 電源供給（3.3V）を確認
- I2Cアドレス（デフォルト0x56）を確認

### 移動検出しない
- IRQ GPIO接続と設定を確認
- RDYピンが接続されているか確認（アクティブロー）

### 初期化失敗
- 十分な電源供給電流容量を確保
- I2Cバス競合を確認
- タイミング要件が満たされているか確認

### タッチ感度が悪い
- 初期化ファイルでATIパラメータの調整を検討

## 使用例

設定が完了すると、トラックパッドは自動的に移動を相対マウス入力としてZMK入力システムにレポートします。キーマップでの追加設定は不要です。
