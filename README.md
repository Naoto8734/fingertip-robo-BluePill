# fingertip-robo-BluePill
STM32F103(BluePill)用の、、STM32CubeIDEのワークスペース。

### グリッパーロボ用のリポジトリ一覧

- [ロボのグリッパー部のリポジトリ](https://github.com/Naoto8734/gripper-robo-BluePill)
- [ロボのXY直動部のリポジトリ](https://github.com/Naoto8734/xy-axis-robo-BluePill)
- [ロボの指先部のリポジトリ(これ)](https://github.com/Naoto8734/fingertip-robo-BluePill)

## 書き込み方法
OSはUbuntu。[J-Link EDU](https://www.embitek.co.jp/product/jlink-edu.html)を使用し、STM32CubeIDEで生成したbinファイルを書き込み。
**BluePill側の電源(3.3v)を入れておくこと。**
参考：[J-LinkでコマンドラインからマイコンのFlashに書き込む](http://idken.net/posts/2019-07-14-jlinkflash/)

### BluePillとJ-Link EDUとのピン接続

| SWD(JLink) | Pin# | | BluePill |
 ---- | ---- | ---- | ---- | ---- 
| VTref | 1 | |3V3(SWD-Connector) |
| GND | 4 | | GND(SWD-Connector) |
| SWDIO | 7 | | DIO(SWD-Connector) |
| SWCLK | 9 | | DCLK(SWD-Connector) |
| RESET | 15 | | R |

## モータ駆動
### 使用モータ
[DC 12V高トルクウォームギヤ付きモータ減速機 エンコーダ付きセルフロック (10rpm)](https://www.amazon.co.jp/gp/product/B073S5GM6Q)
信号は、11pulse/rotate。モータが577回転すると、減速機の出力軸が1回転する。（多分）

> 仕様：
> 定格電圧：DC 12V
> 無負荷回転速度 ：10rpm
> 10RPM定格トルク22.5kg.cm最大トルク25kg.cm減速比600
> 重量：165g（約）
> 配線方法：
> 赤：モーター出力+（交換は回転と反転を制御できます）
> 黒：符号化電力 - 負（3.3-5V）極性は間違ってはいけません
> 黄：シグナルフィードバック
> 緑色：信号のフィードバック
> 青：コーディング力+正（3.3-5V）極性は間違ってはいけません
> 白：モーター力 - （交換は回転と逆転を制御できます）

### 使用モータドライバ
[Cytron 4V～16V DCモータードライバ（2チャンネル、連続3A） - スイッチサイエンス](https://www.switch-science.com/catalog/5521/)

> 二つのブラシ付DCモーターの双方向制御
> 一つのユニポーラ / バイポーラステッピングモーターを制御
> 動作電圧：DC 4 V〜16 V
> 最大モータ電流：3 A（連続）、5 A（ピーク）
> 5 V出力（最大200 mA）を生成する昇降圧レギュレータ
> 簡単なテスト用ボタン
> モーター出力状態を示すLED
> 1.8 V、3.3 V、5 V、12 Vロジック（Arduino、Raspberry Pi、PLCなど）と互換性のある入力
> 最大20 kHzのPWM周波数（出力周波数は入力周波数と同じ）
> 逆極性保護

## I2C Slave
- Slaveのアドレスを、`0x34`とする。
- レジスタアドレス：`0x68` (WHO_AM_I)
 - 自身のSlaveアドレス`0x34`を返す

## C++に対応させる方法
参考：[How to Use C++ with STM32CubeIDE - Shawn Hymel](https://shawnhymel.com/1941/how-to-use-c-with-stm32cubeide/)

1. `main.c`を`main.cpp`に名前を変更。
2. `Project Explorer`の画面で、該当のプロジェクトを右クリック。`Convert C++`を選択。
3. Build (これで終了。HALの関数の引数がいくつか変更されるので、Errorに注意。)

### CubeMXで設定を変更した場合の対処法
`main.cpp`に変更しても、CubeMXで生成されるのは`main.c`のままとなる。
そこで、`main.cpp`を`main.c`に改名し、CubeMXからコードを生成する。
これにより、UserCodeの部分がそのまま引き継がれるので、再度`main.cpp`に変更し直す。
面倒くさい。

### 自作クラスの追加方法
1. `Project Explorer`の画面で、該当のプロジェクトを右クリック。`New -> (C++) Class`を選択。
2. Class Name`の欄に、適当な名前を入力して、Finish。
3. Coreフォルダに`***.h`と`***.cpp`ができるので、それぞれIncとSrcフォルダに突っ込む。
4. 作られた`***.h`ファイルで`#include "main.h"`する。
5. `main.cpp`ファイルで`#include "DriveMotor.h"`する。

クラスなどの命名規則：[ja/CppStyleGuide - ROS Wiki](http://wiki.ros.org/ja/CppStyleGuide)