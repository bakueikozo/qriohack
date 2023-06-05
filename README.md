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
