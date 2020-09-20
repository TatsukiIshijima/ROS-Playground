### 使用機器
- RaspberryPi Zero WH
- Arduino Pro mini 5V

### 開発環境
- Raspbian Buster Lite
```
No LSB modules are available.
Distributor ID:	Raspbian
Description:	Raspbian GNU/Linux 10 (buster)
Release:	10
Codename:	buster
```

- ROS (melodic)

### ROS インストール方法
ほぼ、参考文献の手順通りでOK。ただし、まっさらな Raspian の環境だと入っていない依存ライブラリなどがあるためインストールする必要があった。

1. ROS をビルドするときに `setuptools` が入っていないためにエラーが発生したため、以下でインストール
```
sudo apt-get install python-pip
```
2. `roscore` は実行時にインストールされていないパッケージがあり、エラーが発生したため、以下二つをインストール
```
pip install defusedxml
```
```
pip install netifaces
```

ROS のインストールは手順に従っても RaspberriPi Zero WH では4時間ほどかかった

#### メモ
ros○○ のコマンドの実行にはワークスペースの中で
```
source ./devel/setup.bash
```
の実行が必要

チュートリアルの[ROSのmsgとsrvを作る](http://wiki.ros.org/ja/ROS/Tutorials/CreatingMsgAndSrv)で `rosmgs` 実行時に `ImportError: No module named Cryptodome.Cipher` が発生したため `pip install pycryptodome` でインストールしたところ、ダメ。なので `pip install pycryptodomex` をインストールしたところ、次は `ImportError: No module named gnupg` 。なので、`pip install gnupg` したところ、成功。

#### 参考文献
- [ROS(melodic)をRaspberry Pi + Raspbian(stretch)にインストールする方法](https://asukiaaa.blogspot.com/2018/06/raspberry-pi-raspbianstretchrosmelodic.html)

### ROSSerial セットアップ（できなかった）
以下手順ではインストールできなかったので、ソースコードをビルドする
```
sudo apt-get install ros-indigo-rosserial-arduino
sudo apt-get install ros-indigo-rosserial
```

ソースからのインストールでも `ros-melodic-geometry-msgs` がないためにビルド失敗
```
sudo apt-get install ros-melodic-geometry-msgs
```
でもパッケージがないためとエラーでインストールできず。Ubuntu ではインストールできるらしい

### PyFirmata or PyMata
インストール（RaspberryPi）
```
pip install pyfirmata
```
```
pip install pymata
```

#### Arduino 準備
1. Arduino IDE のインストール
2. ツール -> ボード を "Arduino Pro or Pro mini" へ変更
3. ツール -> プロセッサ が ATmega328P（5V, 16MHz） になっていることを確認
4. ツール -> シリアルポートを USB のものへ変更
5. ファイル -> スケッチ例 -> Firmata -> StandardFirmata を開く
6. 5.をArduino へ書き込み

Arduino の RXD と RaspberryPi の TXD、Arduino の TXD と RaspberryPi の RXD を繋ぐ（レベル変換必須）。RaspberryPi 側で以下コードを実行し、Arduino の LED が点滅するか確認

##### PyFirmata 版
```
import pyfirmata
import time

port = "/dev/ttyS0"
board = pyfirmata.Arduino(port)
led_pin = board.get_pin("d:13:o")

while True:
    led_pin.write(1)
    time.sleep(1.0)
    led_pin.write(0)
    time.sleep(1.0)
```

##### PyMata 版
```
import time
import sys
import signal

from PyMata.pymata import PyMata

BOARD_LED = 13

board = PyMata("/dev/ttyS0", verbose=True)

def signal_handler(sig, frame):
    print 'You pressed Ctrl+C'
    if board is not None:
        board.reset()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

board.set_pin_mode(BOARD_LED, board.OUTPUT, board.DIGITAL)

time.sleep(2)
print "Blinking LED on pin 13 for 10 times ..."
for x in range(10):
    print '%s' % (x+1)
    board.digital_write(BOARD_LED, 1)
    time.sleep(.5)
    board.digital_write(BOARD_LED, 0)
    time.sleep(.5)

board.close()
```
