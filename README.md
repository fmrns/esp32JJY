# esp32JJY

ESP32で電波時計を設定するというありがちなやつ。ボードマネージャの設定(Arduino core for the ESP32)以外に追加のライブラリが必要にならないように、時間も自前で処理しました。


## モード

#### ノーマル

40kHz 7分 -> 60kHz 5分 -> 次の毎時58分までディープスリープ

#### ロング1

40kHz 12分 -> 60kHz 10分 -> 次の毎時58分までディープスリープ

#### ロング2

60kHz 12分 -> 40kHz 10分 -> 次の毎時58分までディープスリープ


## 結線

OUTPINで指定したESP32のピンとGNDの間に、アンテナ(丸めたイヤホンで充分)と発光ダイオードと抵抗を直列につなぎます。

オプションとして、タクトスイッチをSWPINとGNDの間につなげればモード切替ができます。


## 設定

OUTPIN: JJY出力
SWPIN : モード切替スイッチ
LEDPIN: 消灯: ノーマル, 点滅: ロング1, 点灯: ロング2

MY_TIME_ADJUST_IN_SECONDS: 10分前行動させるときは 10 x 60 = 600

WAKEUP_MINUTES: ディープスリープから再起動する時間。毎時:xx分 -1でディープスリープなしで、各モードはそれぞれの繰り返しに。

FREQ1_DURATION_MINUTES_NORMAL: ノーマル40kHzの時間。NTP同期込み。

FREQ2_DURATION_MINUTES_NORMAL: ノーマル60kHzの時間。

FREQ1_DURATION_MINUTES_LONG: ロング1,ロング2 40kHzの時間。

FREQ2_DURATION_MINUTES_LONG: ロング1,ロング2 60kHzの時間。

_ssids[] _psks[]: SSIDと事前共有鍵それぞれのリスト。

_ntp_hosts[]: 同期させるNTPサーバのリスト。


## JJY

[標準電波の出し方](http://jjy.nict.go.jp/jjy/trans/)
