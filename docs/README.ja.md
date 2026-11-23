# ROS 2 PointCloud Crop

[English](../README.md) · 日本語

PointCloud2を複数の箱で切り取り、自機反射・近接点を除去する独立パッケージです。
元のPyLoTの箱判定を再利用し、点群のレイアウト処理を汎用化しました。

- 箱の和集合を除去、または箱の内側だけを残す。
- 箱を別フレームで定義し、点群の取得時刻のTFで判定。
- XYZはFLOAT32/FLOAT64、big/little endian、行パディングに対応。
- 強度・時刻・リングなど、残す点の全バイトとフィールド定義を保持。
- メッセージごとにレイアウトを検証し、前の点群のoffsetを使い回さない。

## 使い方

```bash
ros2 run pointcloud_crop pointcloud_crop_node --ros-args \
  -p input_topic:=/points -p output_topic:=/points/cropped \
  -p box_frame:=base_link -p boxes:='[-0.5,0.5,-0.4,0.4,-0.2,1.0]'
```

`boxes` は `[min_x,max_x,min_y,max_y,min_z,max_z]` を箱の個数分並べます（m）。
`remove_inside:=false` で内側だけ残します。`box_frame` が空なら点群フレームです。
出力は高さ1のコンパクトな点群となり、入力の行パディングは除去されます。
出力座標とheader.frame_idは元のままです。TFは判定にだけ使います。
NaN座標は自機除去モードでは残り、内側抽出モードでは除去されます。

TF取得失敗や不正レイアウトのメッセージは警告して破棄します。元実装の素通しから
変更しています。TFが必要な場合、入力stamp=0の「最新TF」へのフォールバックは避け、
正しい取得時刻を入力してください。

ROSなしでNx3の `.npy` を処理できます（座標系は入力と同じ）。

```bash
ros2-pointcloud-crop crop points.npy cropped.npy --boxes -1 1 -1 1 -1 1
```

テストは変換・複数箱・NaN・行パディング・追加フィールド・両エンディアン・FLOAT64を
確認します。ROS結合テストは合成TFと点群を使い、座標と強度が保持されることを確認します。

## インストールと検証

Python 3.10+。リポジトリのルートで実行します。通常のPython環境ではROSは不要です。

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[test]'
python -m pytest -q
python -m pointcloud_crop.cli demo
```

ROS 2 Jazzyで使う場合は通常のワークスペースのsrcに配置し、ROS環境でビルドします。

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select pointcloud_crop
source install/setup.bash
```

`config/params.yaml` を自分の入出力に合わせて変更してください。パラメータは起動時設定です。
`ros2 launch pointcloud_crop pointcloud_crop.launch.py params_file:=/absolute/path/to/params.yaml` で起動できます。
CIはPython 3.10/3.12でコアテストと合成デモを実行します。ROS結合テストはROS環境で
実行し、それ以外ではスキップします。モデル・実センサ・実アームの性能は認証しません。

## ライセンスと出典

現在はprivateリポジトリです。Apache-2.0。元のコミット・ファイルと変更範囲は
[PROVENANCE.md](../PROVENANCE.md) を参照してください。元モノレポへの依存はありません。
