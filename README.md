# montecarlo-ros2
ロボットシステム学の課題２用のROS2パッケージを保存するリポジトリです。
***

## パッケージの概要
- モンテカルロ法を用いて円周率の近似値を求めます。
- 円周率はfloat32にて計算されます。

### モンテカルロ法について
モンテカルロ法は、確率論を利用した数値計算手法です。このプロジェクトでは、単位正方形内にランダムな点を生成し、その中で単位円に含まれる点の割合を用いて円周率を近似します。

****

## ノードについて
このリポジトリには、モンテカルロ法を用いて円周率（π）を近似するための2つのROS 2ノードが含まれています。

### 構成

1.`monte_carlo_publisher.py`:
- モンテカルロ法を使用して円周率(π)を計算し、その結果をトピック`random_pi_estimator`に配信するパブリッシャーノード。

2.`result.py`:
- トピック`random_pi_estimator`を購読し、試行回数と近似したπの値をログ出力する。
- パブリッシャーノード`monte_carlo_publisher.py`の起動確認・テスト用の簡易的なサブスクライバーノード。

***

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
cd ~/ros2_ws
colcon build
```

3.ワークスペースをソース：
```
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/install/local_setup.bash
```

4.ノードを個別に実行：
```
ros2 run mypkg monte_carlo_publisher
ros2 run mypkg result
```
または、以下のコマンドでローンチファイルを使用して両方のノードを実行：
```
ros2 launch mypkg monte_carlo_pi.launch.py
```
***
## テスト環境
- ROS 2 Jazzy（Ubuntu 24.04 LTSで, 自身のノートPCでテスト）
- ROS 2 Humble（Ubuntu 22.04 LTSで, GitHub Actionsでテスト）

# ライセンス
- このパッケージのtest.ymlでは, こちらのコンテナ（by Ryuichi Ueda）を利用しています.
- © 2025 Chuma Naoki
