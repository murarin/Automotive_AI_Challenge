# 提出したソースコード
ここのフォルダ内にあるコードを大会運営に提出し，テストされたソースコードである．

* aichallenge_bringup: 大会運営が配布しているシナリオ再生用のデータ．参加者はここのlaunchファイルを修正して自分の制御用のソースコードを呼び出す．
* feat_proj_lanelet2_v2: デジタル地図の表示．
* mpc_follower2: モデル予測制御によるwaypointの追従．
* region_tlr_effnet: 信号機の色識別．手法はEfficientNet．使用ライブラリはLibtorch。
* vision_effdet_detect: 物体検出．手法はEfficientDet．使用ライブラリはLibtorch。
* twist_filter2: 目標速度，角速度から想定される横加速度の計算．ローパスフィルタによるノイズ除去．
* twist_gate3: twist_filterから得られた情報からスロットル値とステアリング値を出力．

今回のコンペティションでは，オープンソースの自動運転ソフトウェアであるAutowareを活用することが前提となっている．そのため，このフォルダ内の一部のプログラムは，Autowareのプログラムをベースに作成したものや，一部そのまま使用しているものが存在する．それらのものはプログラム内で示す．
