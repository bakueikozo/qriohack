# qriohack
Qrio Lock 初代(Q-SL1)をハックするための知見と  
内蔵マイコンを書き換えるためのファームを生成するためのソースコードです

# 現状の実装
設定に追加のシリアルポートが必要
鍵を施錠位置に合わせて取り付けて'P'コマンドで初期位置入力  
続けて鍵を開錠位置に合わせて取り付けて'P'コマンドで第二位置入力すると記憶される  
開錠したいときはシリアルから'U' 、施錠は'L'を送る  
もしくは、BLE送信できる奴から特定のところに'U'か'L'を書き込むと動く  

これをアプリから送るようにUIを作ると今までのQrioのように作れます。
が、まだ作ってませんし、私が作るとは言ってません。
# どうやる
Windowsの場合
WICED-SDKをダウンロードしてきて、Apps以下にこのリポジトリ内のApps/obaqフォルダをコピーします
コマンドプロンプトから
> make obaq-BCM920737TAG_Q32

でビルドできるか確認

OKなら
> make obaq-BCM920737TAG_Q32 download UART=COMxx

で書き込み

WICED-SDKは
https://community.infineon.com/t5/Public-Archive/WICED-Smart-SDK-2-2-3-7z-Archive/td-p/247688

ここにある奴で確認済み
そのほかのバージョンやIDEにインポートするなら各自

![書き込みコネクタ](Apps/obaq/pin-fpc.jpg "書き込みコネクタ")

![ピンアサイン](Apps/obaq/pinassign.png  "ピン番号の見方")

![変換基板に取り付けた場合](Apps/obaq/pin-connected.jpg "変換基板に取り付けた場合")

# ピンアサイン
下面接続のFPC基板に取り付けた場合、この説明の番号と一致します  
使ったやつはこれ　 https://amzn.to/43Kj0qI  
|  No. |  Function  |
| ---- | ---- |
|  1,2  |  6V  |
|  3  |  3.3V  |
| 4 | RESET(NC:基板上ジャンパ)  
| 5 |SCL(NC:基板上ジャンパ)  |
| 6 | SDA(NC:基板上ジャンパ) |
| 7 |peripheral UART TXD in  |
| 8 |peripheral UART RXD out  |
| 9 |Programming RXD  |
| 10 |Programing TXD  |
| 11,12 |GND  |

1,2および3　は基板上の電源パターン直結のため、電池が入っているときの挙動に注意  
4,5,6は基板まで届いているが、基板上でジャンパがNCになっているのでそのままでは使用不可   
9,10と3,11,12の接続で書き込み可能。電源投入時に9をHighにしておくとProgrammingモードに入る   

![書き込み回路](Apps/obaq/programming.png "書き込み回路")

追加のシリアルポートが欲しい場合は7,8を使う  

※参考※  
https://community.infineon.com/t5/Resource-Library/Programming-the-TAG2-TAG3-Board-using-command-line-tools/ta-p/246905  
書き込み中に電源を落としてしまった場合正常に起動しなくなる。その場合downloadではなくrecoveryモードに入る必要がある。  
電源投入orRESETの際にeepromのSDAをVCCに吊るとソフトが読み込めない→リカバリモードになる  
ここでrecoveryで書き込む  




