# 第2回自動運転AIチャレンジで作成したソースコード
これは，公益社団法人 自動車技術会が開催した[第2回AI自動運転チャレンジ](https://www.jsae.or.jp/jaaic/index.html)に出場した際に，作成したソースコードである．

コンペティションの結果としては，[予選](https://www.jsae.or.jp/jaaic/online_qualifying.html)は4位（信号に従って走行する部門では1位）で突破したが，[決勝](https://www.jsae.or.jp/jaaic/online_final.html)では上位に食い込めなかった．しかし多くの有名企業が参加している中，よく頑張ったと自分を褒めたい．

* effdet: EfficientDetの学習．物体検出用．

* effnet: EfficientNetの学習．信号機の色判定用．

* aichallenge_ws: 車の制御．学習した結果や様々な手法を使って制御する．

このソフトウェアは、Apache2.0ライセンスで配布されている制作物が含まれる．対象のものはプログラム内でApache2.0ライセンスである旨を記載する．また，運営側から公開の承諾を受けたもののみを公開する．

# 認識結果・走行動画
* 認識結果
  * [昼間](https://www.youtube.com/watch?v=OBqL6TZWBD8)
  * [夜間・悪天候](https://www.youtube.com/watch?v=SHkWbPRfkTE)
* 予選の走行結果
  * [アクセル制御](https://www.youtube.com/watch?v=VtZzHryqTMk)
  * [路駐車両の回避](https://www.youtube.com/watch?v=wPtme09xt9g)
  * [信号機認識，発進（昼間）](https://www.youtube.com/watch?v=RV9q26KSrZ0)
  * [信号機認識，発進（夜間・悪天候）](https://www.youtube.com/watch?v=Jv9UJKOby9s)
* [決勝の走行結果](https://www.youtube.com/watch?v=DbUKRlwMPi0)
