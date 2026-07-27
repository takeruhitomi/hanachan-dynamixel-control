# Hanachan DYNAMIXEL Control

OpenRB-150 に接続した DYNAMIXEL を、Python と DYNAMIXEL SDK から制御するためのツールです。サーボの検出・ID変更・関節マッピング・手動教示・CPG歩容の調整を行えます。

このリポジトリは PC から OpenRB-150 のシリアルポートへ接続する運用を対象にしています。シミュレータは含みません。

## 前提

- Python 3.11 以降
- OpenRB-150 と DYNAMIXEL 用の外部電源
- DYNAMIXEL Protocol 2.0 / 1 Mbps
- X シリーズ互換の Control Table

PC から直接操作するには、OpenRB-150 に USB-DYNAMIXEL ブリッジ用のファームウェアが書き込まれている必要があります。OpenRB-150 は、そのままでは USB2DYNAMIXEL アダプタではありません。

## セットアップ

```sh
uv sync
```

`uv` を使わない場合:

```sh
python -m venv .venv
source .venv/bin/activate
pip install dynamixel-sdk
```

## 接続確認

macOS では `/dev/tty.*` ではなく `/dev/cu.*` を使います。接続中のポートを確認します。

```sh
ls /dev/cu.usbmodem* /dev/cu.usbserial*
```

例として OpenRB が `/dev/cu.usbmodem101` の場合、接続されているDYNAMIXELを確認します。

```sh
uv run python main.py --device /dev/cu.usbmodem101 scan --ids 1-28
```

通信条件が不明な場合:

```sh
uv run python main.py --device /dev/cu.usbmodem101 discover --ids 1-252
```

## 安全上の注意

最初は機体を支持し、可動範囲に手や配線を置かないでください。トルクを入れる操作、位置指令、CPG開始はいずれも実機を動かします。物理的に電源を遮断できる状態で試してください。

同じDYNAMIXEL IDが複数ある状態では、個別に識別やID変更はできません。対象のサーボだけを通信できる状態にしてからIDを変更してください。

## 基本操作

```sh
# トルクのON/OFF
uv run python main.py --device /dev/cu.usbmodem101 torque on --ids 1
uv run python main.py --device /dev/cu.usbmodem101 torque off --ids 1

# 現在位置の読み取り
uv run python main.py --device /dev/cu.usbmodem101 read --ids 1-3

# 位置指令
uv run python main.py --device /dev/cu.usbmodem101 move --ids 1 --position 2048 --wait

# サーボ本体のID変更。対象サーボは単体接続で実行すること
uv run python main.py --device /dev/cu.usbmodem101 change-id 1 9
```

検出されたサーボを直接動かすスライダーGUI:

```sh
uv run python main.py --device /dev/cu.usbmodem101 gui --ids 1-28
```

## 関節マッピング

関節スロットへのID割り当て、ゼロ位置、正方向、使用する関節をGUIで設定します。

```sh
uv run python main.py --device /dev/cu.usbmodem101 map \
  --config robot_config.json \
  --output robot_config.json \
  --ids 1-28
```

`Dir` は関節座標系での正方向です。`-` / `0` / `+` ボタンで小さく動かし、期待と逆なら `1` と `-1` を切り替えます。テスト移動量をさらに小さくするには `--test-delta 5` を指定してください。

共有用の雛形は `robot_config.example.json` です。このリポジトリには現在のテスト機体用 `robot_config.json` も含まれます。別機体へ適用する前に、必ずマッピングGUIでID、ゼロ位置、方向、使用関節を確認してください。

```sh
cp robot_config.example.json robot_config.json
```

## CPG歩容

CPGをターミナルから実行します。

```sh
uv run python main.py --device /dev/cu.usbmodem101 cpg --config robot_config.json
```

- `s`: 開始
- `x`: 停止してニュートラル姿勢へ移動
- `t`: トルクON
- `q`: トルクOFFして終了

パラメータ調整用GUI:

```sh
uv run python main.py --device /dev/cu.usbmodem101 cpg-gui --config robot_config.json
```

GUIには低速の `Safe slow`、設計値の `Forward (design)`、初期姿勢へ2秒かけて移動する `Initial position (2 s)` があります。通常は `Easy tuning` で速度・歩幅・足上げ・膝・推進位相を調整します。`Show advanced controls` を有効にすると12個のCPGパラメータ、胴体ID 1〜3の個別位置、歩容テスト、記名プリセットを操作できます。

`Save Config` は調整した歩容を `robot_config.tuned.json` に保存します。保存済みの歩容値を起動時に使うには次を指定します。

```sh
uv run python main.py --device /dev/cu.usbmodem101 cpg-gui \
  --config robot_config.tuned.json \
  --initial-preset config
```

## 手動教示

トルクを切って手で動かした姿勢を、モーションシーケンスとして保存します。

```sh
uv run python main.py --device /dev/cu.usbmodem101 teach \
  --config robot_config.json \
  --output motion_sequence.json
```

教示GUIは起動時に対象関節のトルクをOFFにします。再生時は実機が動くため、必ず機体を支持した状態で確認してください。

## 主な設定項目

`robot_config.json` は次を保持します。

- `ids`: 28個の関節スロットに対応するDYNAMIXEL ID
- `zero_ticks`: 各関節のゼロ位置
- `directions`: 各関節の正方向。`1` または `-1`
- `enabled_indices`: 現在の機体で制御する関節スロット
- `gait_params`: 12個のCPGパラメータ
- `reverse_legs`: 脚の前後スイープ反転
- `sweep_phase_offset_rad`: 脚上げに対する前後スイープの位相差

`three-segment` レイアウトでは leg4 と tail/body joint を既定で無効にします。全28関節構成を使う場合は `map --layout full` または `cpg --layout full` を指定してください。

## 制限

このツールは X シリーズ互換のControl Tableを前提にしています。XL-320などアドレスやデータ長が異なるモデルでは、そのまま利用できません。
