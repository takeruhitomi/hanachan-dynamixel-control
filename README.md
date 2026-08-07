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
source .venv/bin/activate  # Windows は .venv\Scripts\Activate.ps1
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

Windows では COM ポート名を使います。接続中のポートは次で確認できます。

```powershell
uv run python -c "from serial.tools.list_ports import comports; [print(p.device, p.description) for p in comports()]"
```

例として OpenRB が `COM5` の場合:

```powershell
uv run python main.py --device COM5 scan --ids 1-28
```

以降の例はすべて `--device /dev/cu.usbmodem101` の部分を `--device COM5` に読み替えてください。シリアルポートが1つだけの場合は `--device` を省略でき、複数ある場合は候補一覧が表示されます。

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
uv run python main.py --device /dev/cu.usbmodem101 map --ids 1-28
```

読み書きとも既定で `robot_config.json` を使います。別ファイルを扱うときだけ `--config` と `--output` を指定してください。

`Dir` は関節座標系での正方向です。`-` / `0` / `+` ボタンで小さく動かし、期待と逆なら `1` と `-1` を切り替えます。このボタンは現在位置からの相対移動ではなく、`Zero Tick` 欄の値 ± テスト移動量の絶対位置へ移動します。移動量は `--test-delta` で変更できます（既定 20）。

`Save Config` が書き換えるのは `ids` / `zero_ticks` / `directions` / `enabled_indices` だけで、`gait_params` などそれ以外の項目は読み込んだ設定から引き継がれます。

共有用の雛形は `robot_config.example.json` です。このリポジトリには現在のテスト機体用 `robot_config.json` も含まれます。別機体へ適用する前に、必ずマッピングGUIでID、ゼロ位置、方向、使用関節を確認してください。

```sh
cp robot_config.example.json robot_config.json
```

## CPG歩容

CPGをターミナルから実行します。

```sh
uv run python main.py --device /dev/cu.usbmodem101 cpg
```

- `s`: 開始
- `x`: 停止してニュートラル姿勢へ移動
- `t`: トルクON
- `q`: トルクOFFして終了

パラメータ調整用GUI:

```sh
uv run python main.py --device /dev/cu.usbmodem101 cpg-gui
```

GUIには低速の `Safe slow`、設計値の `Forward (design)`、初期姿勢へ2秒かけて移動する `Initial position (2 s)` があります。

パラメータのスライダーは `Advanced gait parameters` に全項目がまとまっており、既定で表示されます。`Timing and tests` にはパラメータではない項目だけが残っています（`Traction timing`、`Step test`、`Reverse propulsion`）。

`Body positions` は既定で折りたたまれています。下段のパラメータ一覧に画面を使うためで、胴体を個別に動かすときだけ `Show body position sliders` を有効にしてください。折りたたんでいる間もスライダーの値は保持されます。

`Save Config` は調整した歩容を `robot_config.json` に上書き保存します。保存済みの歩容値を起動時に使うには次を指定します。

```sh
uv run python main.py --device /dev/cu.usbmodem101 cpg-gui --initial-preset config
```

上書きを避けたい場合は `--output` で別ファイルを指定してください。

## 歩容モデル v2

`cpg-gui` の12パラメータ（v1）は、lift が半波整流で片側にしか動きません。前後の振り出しは yaw だけが担う前提のモデルです。v2 は yaw と lift を共通の前後ストロークから駆動し、膝を腰に逆連動させて足裏の角度を保ちます。

```sh
uv run python main.py --device /dev/cu.usbmodem101 cpg-gui-v2
```

v1 はそのまま `cpg-gui` で使えます。パラメータは設定ファイル内で別々に保持され、v1 は `gait_params`、v2 は `gait_params_v2` に保存されます。どちらのGUIで保存しても両方の値が残ります。

v2 の17パラメータ（v1 は12個）:

| # | 名前 | 役割 |
|---|---|---|
| 1 | Frequency | 歩行周期の速さ |
| 2 | Stride | yaw の前後振幅（水平面内の歩幅）。**負で yaw だけ反転** |
| 3 | Stride bias | yaw の中心位置 |
| 4 | Leg swing | lift の前後振幅。yaw と同位相で脚全体を振る。**負で lift/knee がまとめて反転** |
| 5 | Foot level | 膝と lift の連動係数。`1.0` で足裏が地面と平行に保たれる |
| 6 | Foot clearance | 遊脚中だけ加算される持ち上げ量。**持ち上げ向きが逆なら負にする** |
| 7 | Stance duty | 1周期に占める接地期の割合。接地と遊脚の**速度比**を決める |
| 8 | Knee bias | 膝の固定オフセット |
| 9 | Lift bias | lift の固定オフセット |
| 10 | Turn | 左右の Stride 差。脚による旋回 |
| 11 | Segment phase | 前後セグメント間の脚位相差 |
| 12 | Left/right phase | 左脚群と右脚群の位相差 |
| 13 | Body wave | 胴体関節ID 1〜3の周期的な曲げ振幅 |
| 14 | Body phase | 胴体関節間の位相差 |
| 15 | Body turn | 胴体関節ID 1〜3を同じ向きに曲げる固定量。胴体による旋回 |
| 16 | Knee phase | 膝が lift から遅れる位相。足裏が平行にならないときのタイミング補正 |
| 17 | Body rate | 胴体波の周波数倍率。`1.0` で脚と同じ周期、`2.0` で脚1周期あたり胴体2周期 |

旋回は脚（`Turn`）と胴体（`Body turn`）の2系統があり、併用できます。

`Knee phase` は膝の追従タイミングだけをずらします。`Foot level` で連動の量を、`Knee phase` で連動の遅れを決める、という分担です。lift が前から後ろへ戻る間に足裏が平行にならない場合はここを調整してください。

yaw と lift/knee の位相差は、GUIの **`Traction timing`** スライダー（設定の `sweep_phase_offset_rad`）で ±180° 調整できます。yaw だけが位相シフトし、lift と knee は動きません。足が前に出たときに yaw が後ろへ行く場合は、`Stride` を負にするのが確実です（位相シフトは接地期と遊脚期の長さが違うため、180° ずらしても単純な反転にはなりません）。

`Foot level` の符号は機体のリンク構成に依存します。膝が腰の回転を打ち消す向きが逆の場合は負の値を使ってください。

名前付きプリセットは `gait_model` で v1 / v2 のどちらの値かを記録します。GUIには現在のモデルと一致するプリセットだけが表示されます。

### 速度の決め方

ストロークは接地期に `+1 → -1`、遊脚期に `-1 → +1` と同じ振幅を動くため、2つの速度は次のようになります。

```
接地の速さ = 2 / (Stance duty × 周期)
遊脚の速さ = 2 / ((1 - Stance duty) × 周期)
速度比     = Stance duty / (1 - Stance duty)
```

**`Frequency` が絶対速度、`Stance duty` が接地と遊脚の速度比**を決めます。接地はほどほどに、空中では素早く足を前に出したい場合は `Stance duty` を上げてください。`0.75` で 3倍、`0.90` で 9倍です。

### パラメータ追加時の互換性

パラメータはモデルの末尾にのみ追加されます。項目が増えた後も古い設定ファイルやプリセットはそのまま読め、不足分は既定値で補われます。既定値は挙動を変えない値を選んであるので、調整済みの歩容は維持されます。

一方で**既存パラメータの範囲変更は正規化値の意味を変える**ため、保存済みファイルの値も同時に書き換える必要があります。範囲を変えたコミットでは、`gait_presets.json` と `robot_config.json` も併せて移行しています。git 履歴から古い版のプリセットを取り出す場合は、その時点の範囲で解釈されている点に注意してください。

v2 は PC 側で計算するため、`--onboard-cpg` とは併用できません。

## 無線接続

OpenRB-150 の4ピンUARTポート（`Serial2`）に透過型の無線モジュールを挿し、PC側は子機のシリアルポートへ接続します。標準の `usb_to_dynamixel` は USB しか見ないため、`Serial2` も中継するファームウェアが必要です。`firmware/` に用意してあります。

### 1. 通信速度を特定する

無線モジュールは両端とも透過なので、問い合わせても何も答えません。速度は「パターンが往復できる組み合わせ」を総当たりして見つけます。まず `firmware/openrb_radio_probe` を書き込み、次を実行します。

```sh
uv run python firmware/radio_scan.py --usb COM6 --radio COM7
```

OpenRB 側の速度を USB 経由で切り替えながら、PC 側の速度も変えてパターンを送り、無傷で届いた組み合わせを報告します。DYNAMIXEL バスには触れないので、サーボは動きません。

モジュールの速度が既知なら、この手順は不要です。

### 2. ブリッジを書き込む

`firmware/openrb_wireless_bridge/openrb_wireless_bridge.ino` の `RADIO_BAUD` を特定した値にして書き込みます。USB は従来どおり使えるので、有線での調整は影響を受けません。

### 3. 無線で接続する

```sh
uv run python main.py --device COM7 --baudrate 57600 scan --ids 1-28
uv run python main.py --device COM7 --baudrate 57600 cpg-gui-v2
```

`--baudrate` が 115200 以下のとき、リトライ回数の増加とパケットタイムアウトの延長が自動で有効になります。制御周期は既定で 10 Hz です（有線は 50 Hz）。`--control-hz` で変更できます。

### 無線でパケットが失われる条件

ブリッジがバイト単位で中継すると、パケットは無線の速度のままDYNAMIXELバスへ流れます。57600 bps では 1バイトあたり 174 µs なので、大きなパケットほどバス上に長く伸び、サーボの受信タイムアウトに引っかかって破棄されます。実測した閾値です。

| パケット長 | バス上での所要時間 | 結果 |
|---|---|---|
| 19バイト（1関節の sync write） | 3.3 ms | 通る |
| 29バイト（3関節） | 5.0 ms | **失われる** |
| 119バイト（21関節） | 21 ms | **失われる** |

トルクON/OFF（13バイト）と位置読み取り（14バイト）だけが通り、歩容の指令が一切届かない、という症状になります。sync write は応答を返さないため、送信側からは成功に見える点に注意してください。

`openrb_wireless_bridge` はパケットを受け切ってから 1 Mbps のバスへ送出するため、この問題は起きません。バイト単位で中継する実装にしないでください。

無線モジュールはペイロードを分割して送ることがあり、断片の間隔は数十ミリ秒に達します。ブリッジはプロトコル1.0/2.0の長さフィールドを解釈し、宣言された長さが揃うまで待ちます。無通信でパケット境界を判定したり、その間にホストの所有権を手放したりすると、受信途中のパケットを捨てて断続的な取りこぼしになります（実測で2割が失われ、動きがかくつきます）。

稼働中のビルドは `@VER` で確認できます。DYNAMIXELパケットは必ず `FF FF` で始まるので、この照会がバス通信と衝突することはありません。

```sh
uv run python -c "import serial,time; s=serial.Serial('COM7',57600,timeout=2); s.write(b'@VER\n'); time.sleep(1); print(s.read(120))"
```

### 帯域の目安

57600 bps は名目 5760 バイト/秒ですが、無線プロトコル自身のフレーミングと再送があるため**実効は約 1700 バイト/秒**です。21関節の sync write は1回119バイトなので、上限は概ね 14 Hz になります。

| 要求周期 | 実際に出る周期 | 実行直後の読み取り |
|---|---|---|
| 10 Hz | 9.9 Hz | 40 ms（アイドル時と同等。遅延の蓄積なし） |
| 20 Hz | 頭打ち | 応答なし（送信バッファが溜まり続ける） |

既定の 10 Hz はこの実測に基づく値です。要求を上げても実際の周期は上がらず、指令の遅延だけが蓄積します。

読み取りは無線では高価です（21関節の全読み取りで約 800 ms、1関節あたり約 24 ms）。CPG は毎周期では読まず初期化時のみなので、歩容の実行には影響しません。

## 手動教示

トルクを切って手で動かした姿勢を、モーションシーケンスとして保存します。

```sh
uv run python main.py --device /dev/cu.usbmodem101 teach \
  --output motion_sequence.json
```

教示GUIは起動時に対象関節のトルクをOFFにします。再生時は実機が動くため、必ず機体を支持した状態で確認してください。

## 主な設定項目

設定は `robot_config.json` の1ファイルに集約されています。`map` と `cpg-gui` はどちらもこのファイルを読み、`Save Config` でこのファイルへ書き戻します。`--config` を省略した場合もこのパスが使われ、ファイルが無いときだけ内蔵の既定値で起動します。

`robot_config.json` は次を保持します。

- `ids`: 28個の関節スロットに対応するDYNAMIXEL ID
- `zero_ticks`: 各関節のゼロ位置
- `directions`: 各関節の正方向。`1` または `-1`
- `enabled_indices`: 現在の機体で制御する関節スロット
- `gait_params`: v1 歩容モデルの12個のCPGパラメータ
- `gait_params_v2`: v2 歩容モデルの12個のパラメータ
- `reverse_legs`: 脚の前後スイープ反転
- `sweep_phase_offset_rad`: 脚上げに対する前後スイープの位相差

`three-segment` レイアウトでは leg4 と tail/body joint を既定で無効にします。全28関節構成を使う場合は `map --layout full` または `cpg --layout full` を指定してください。

## 制限

このツールは X シリーズ互換のControl Tableを前提にしています。XL-320などアドレスやデータ長が異なるモデルでは、そのまま利用できません。
