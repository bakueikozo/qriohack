"# qriohack" 
# qriohack
Qrio Lock 初代(Q-SL1)をハックするための知見と
内蔵マイコンを書き換えるためのファームを生成するためのソースコードです

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
