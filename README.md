# montecarlo-ros2
ロボットシステム学の`課題2`用のROS2パッケージを保存するリポジトリです。
***

## パッケージの概要
- モンテカルロ法を用いて円周率の近似値を求めます。
- 円周率はfloat32にて計算されます。

***

### モンテカルロ法について
モンテカルロ法は、確率論を利用した数値計算手法です。このプロジェクトでは、単位正方形内にランダムな点を生成し、その中で単位円に含まれる点の割合を用いて円周率を近似します。

***

## ノードについて
このリポジトリには、モンテカルロ法を用いて円周率（π）を近似するための2つのROS 2ノードが含まれています。

### 構成

1.`monte_carlo_publisher.py`:
- `0.3秒`ごとに、1回ずつランダムな点を生成し、モンテカルロ法を使用して円周率(π)を計算してその結果をトピック`random_pi_estimator`に`渡すパブリッシャーノードです。

2.`result.py`:
- トピック`random_pi_estimator`を購読し、試行回数と近似したπの値をログ出力する。
- パブリッシャーノード`monte_carlo_publisher.py`の起動確認・テスト用の簡易的なサブスクライバーノードです。

***
## ファイル構成

- `montecarlo-ros2/mypkg/monte_carlo_publisher.py`: `0.3秒`ごとにモンテカルロ法を使用して円周率（π）を近似し、結果を`random_pi_estimator`トピックにパブリッシュします。
- `montecarlo-ros2/mypkg/result.py`: `random_pi_estimator`トピックを購読し、円周率の近似値と試行回数を表示します。
- `montecarlo-ros2/launch/monte_carlo_publisher-result.launch.py`: 両方のノードを起動するためのROS2,Launchファイルです。

## 動作の仕組み

1.`monte_carlo_publisher.py`:
- 範囲`[-1, 1]`内でランダムな`(x, y)`座標を生成。
- 点が単位円内`x^2 + y^2 <= 1`に含まれるかを判定。
- 単位円内の点の割合を基に、円周率を近似計算。
- 近似した円周率(π)をトピック`random_pi_estimator`に渡す。

2.`result.py`:
- トピック`random_pi_estimator`から円周率(π)を受取り、`試行回数:16 円周率: 3.5`のような形で出力する。
- 試行回数と近似した円周率をログに記録する。

***
## 使い方

### 前提条件
- ROS2がインストールされていること。
- ROS 2のワークスペースが適切にセットアップされていること。

### ビルドと実行

1.リポジトリをクローン:
```
git clone https://github.com/ChumaNaoki/montecarlo-ros2.git
```

2.ワークスペースをビルド：
```
colcon build
```

3.ワークスペースをソース：
```
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/install/local_setup.bash
```

4.以下のコマンドでローンチファイルを使用して両方のノードを実行：
```
ros2 launch mypkg monte_carlo_publisher-result.launch.py
```

## 実行結果
以下のように、試行回数が増えるにつれて円周率の近似値が収束していきます。
```
[result-2] [INFO] [1735913806.957600458] [result]: 試行回数:1 円周率: 4.0
[result-2] [INFO] [1735913807.245400350] [result]: 試行回数:2 円周率: 4.0
[result-2] [INFO] [1735913807.544752108] [result]: 試行回数:3 円周率: 4.0
[result-2] [INFO] [1735913807.844935089] [result]: 試行回数:4 円周率: 4.0
[result-2] [INFO] [1735913808.145887324] [result]: 試行回数:5 円周率: 3.200000047683716
[result-2] [INFO] [1735913808.445441048] [result]: 試行回数:6 円周率: 3.3333332538604736
[result-2] [INFO] [1735913808.745286715] [result]: 試行回数:7 円周率: 3.4285714626312256
[result-2] [INFO] [1735913809.044998629] [result]: 試行回数:8 円周率: 3.0
[result-2] [INFO] [1735913809.344730888] [result]: 試行回数:9 円周率: 3.1111111640930176
[result-2] [INFO] [1735913809.644867888] [result]: 試行回数:10 円周率: 3.200000047683716
[result-2] [INFO] [1735913809.945105956] [result]: 試行回数:11 円周率: 3.2727272510528564
[result-2] [INFO] [1735913810.245333986] [result]: 試行回数:12 円周率: 3.0
[result-2] [INFO] [1735913810.545793165] [result]: 試行回数:13 円周率: 3.076923131942749
[result-2] [INFO] [1735913810.845062179] [result]: 試行回数:14 円周率: 2.857142925262451
[result-2] [INFO] [1735913811.145346799] [result]: 試行回数:15 円周率: 2.933333396911621
[result-2] [INFO] [1735913811.445169080] [result]: 試行回数:16 円周率: 3.0
[result-2] [INFO] [1735913811.745816775] [result]: 試行回数:17 円周率: 3.058823585510254
```

***
# 注意事項
- ノード`monte_carlo_publisher.py`の結果はランダムに生成されるため、1回目の出力`試行回数:1 円周率: 4.0`か
- launchファイルを使用し実行した結果出力される試行回数は、`result.py`ノードが起動された回数に依存します。`monte_carlo_publisher.py`が発行する円周率の近似値とは独立して動作します。

***
## テスト環境
- OS: Ubuntu 20.04 LTS
- ROS 2 バージョン: Foxy Fitzroy
- Python バージョン: Python 3.8.10

# ライセンス
- このソフトウェアパッケージは３条BSDライセンスの下、再頒布および使用が許可されます。
- このパッケージのtest.ymlにて, [こちらのコンテナ](https://hub.docker.com/r/ryuichiueda/ubuntu22.04-ros2/tags)（by Ryuichi Ueda）が使用されています.
- © 2025 Chuma Naoki
