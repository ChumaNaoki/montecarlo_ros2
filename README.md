# montecarlo_ros2
ロボットシステム学の`課題2`用のROS2パッケージを保存するリポジトリです。
***

## パッケージの概要
モンテカルロ法を用いて円周率（π）を近似するROS2パッケージです。単位正方形内にランダムな点を生成し、そのうち単位円内に含まれる点の割合を基に円周率（π）の近似値を計算します。試行回数を増やすことで、近似値の精度が向上します。

***

### モンテカルロ法について
モンテカルロ法は、確率論を利用した数値計算手法です。
数学的な仕組みが難しいものやランダムな現象を、実際に確率モデルとして計算するとき、コンピュータで乱数を発生させ、それを用いてシミュレーションを繰り返します。

***

# ノードとトピックの概要
このリポジトリには、モンテカルロ法を用いて円周率（π）を近似し、結果を確認するための2つのROS2ノードが含まれています。

## ノード一覧
| ノード名               | 概要                                   | パブリッシュ         | サブスクライブ     |
|------------------------|----------------------------------------|---------------------|--------------------|
| monte_carlo_publisher | モンテカルロ法で円周率（π）を近似計算するノード  | `random_pi_estimator` | なし               |
| result                | 円周率（π）の近似値をログ出力するノード       | なし                | `random_pi_estimator` |

## トピック一覧
| トピック名            | データ型             | 説明                                   |
|-----------------------|---------------------|---------------------------------------|
| random_pi_estimator   | std_msgs/msg/Float32 | 円周率の近似値をパブリッシュします。    |



# ノード詳細

- ### monte_carlo_publisher
  - **概要**: モンテカルロ法でランダムな点を生成し、単位円内に含まれる割合を基に円周率（π）を近似計算するノード。
  - **トピック**:
    - **パブリッシュ**: `random_pi_estimator` (型: `std_msgs/msg/Float32`)
  - **動作**:
    - 0.3秒ごとに円周率（π）の近似値を計算し、上記のトピックに送信します。

- ### result
  - **概要**: パブリッシャーノード`monte_carlo_publisher.py`の起動確認・テスト用の簡易的なサブスクライバーノードです。
  - **動作**: トピック`random_pi_estimator`から円周率(π)を受取り、`試行回数:16 円周率: 3.5`のような形でログに出力する。
***

## 動作の仕組み

1.`monte_carlo_publisher.py`:
- 範囲`[-1, 1]`内でランダムな`(x, y)`座標を生成。
- 点が単位円内`x^2 + y^2 <= 1`に含まれるかを判定。
- 単位円内の点の割合を基に、円周率を近似計算。
- 近似した円周率(π)をトピック`random_pi_estimator`に渡す。

***

## ビルド時の前提条件
- ROS2がインストールされていること。
- ROS 2のワークスペースが適切にセットアップされていること。

## ros2 launchでの実行結果
以下のコマンドで両ノードを同時実行した際の結果です。
```
ros2 launch montecarlo_ros2 monte_carlo_publisher-result.launch.py
```
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

---

[result-2] [INFO] [1735913815.745816775] [result]: 試行回数:1000 円周率: 3.1410372257232666
```

***
# 注意事項
- ノード`monte_carlo_publisher.py`の結果はランダムに生成されるため、出力結果が2パターンしかない1回目しかgithubにおけるテストを行っていません。(自身のノートPCではで1000回まで動作を確認しています。)
- ノード`monte_carlo_publisher.py`にて生成される近似値は徐々に精度が向上します。初期の段階では正確ではない場合があることを理解したうえで使用してください。
- launchファイルを使用し実行した結果出力される試行回数は、`result.py`ノードが起動された回数に依存します。`monte_carlo_publisher.py`が発行する円周率の近似値とは独立して動作します。

***
## 動作確認済み環境
- **ROS 2 Foxy**  
  - **OS**: Ubuntu 20.04 LTS  
  - **環境**: 自身のノートPCで動作確認とテストを実施  

- **ROS 2 Humble**  
  - **OS**: Ubuntu 22.04 LTS  
  - **環境**: GitHub Actionsで自動テストを実施 

# ライセンス
- このソフトウェアパッケージは３条BSDライセンスの下、再頒布および使用が許可されます。
- このパッケージのtest.ymlにて, [こちらのコンテナ](https://hub.docker.com/r/ryuichiueda/ubuntu22.04-ros2/tags)（by Ryuichi Ueda）が使用されています.
- © 2025 Chuma Naoki
