# Unity Setup Guide for FalconBridge

このガイドでは、UnityプロジェクトでNovint Falconハプティックデバイスを使用するためのセットアップ方法を説明します。

## プロジェクト構成

### 作成済みファイル

```
novint-falcon-unity-examples/
├── Assets/
│   ├── Plugins/
│   │   ├── FalconBridge/
│   │   │   ├── FalconBridge.h         # C++ヘッダー
│   │   │   ├── FalconBridge.cpp       # C++実装
│   │   │   ├── CMakeLists.txt         # ビルド設定
│   │   │   └── BUILD.md               # ビルド手順
│   │   └── macOS/
│   │       └── FalconBridge.bundle    # コンパイル後のプラグイン（要ビルド）
│   ├── Scripts/
│   │   ├── FalconBridge.cs            # C# P/Invokeラッパー
│   │   ├── FalconController.cs        # 基本的な使用例
│   │   └── FalconCollisionHandler.cs  # 高度な衝突検出例
│   └── Scenes/
│       └── SampleScene.unity          # テストシーン（セットアップ済み）
```

## 現在のUnityシーンセットアップ

### SampleScene の構成

1. **FalconManager** (GameObject)
   - コンポーネント: `FalconController`
   - 位置: (0, 0, 0)
   - 機能:
     - Falconデバイスの初期化
     - ツール位置の取得と表示
     - シンプルな平面との衝突検出
     - デバッグログ出力

2. **TestPlane** (GameObject)
   - プリミティブ: Plane
   - 位置: (0, 0, 0)
   - スケール: (2, 1, 2)
   - 目的: 視覚的な基準平面

3. **TestCube** (GameObject)
   - プリミティブ: Cube
   - 位置: (0, 0.5, 0)
   - スケール: (0.5, 0.5, 0.5)
   - 目的: テスト用オブジェクト

## セットアップ手順

### ステップ 1: ネイティブプラグインのビルド

ネイティブプラグインをビルドする必要があります。詳細は [Assets/Plugins/FalconBridge/BUILD.md](Assets/Plugins/FalconBridge/BUILD.md) を参照してください。

```bash
# libnifalconのインストール
brew install cmake pkg-config libusb
git clone https://github.com/libnifalcon/libnifalcon.git
cd libnifalcon
mkdir build && cd build
cmake ..
make
sudo make install

# FalconBridgeのビルド
cd /path/to/novint-falcon-unity-examples/Assets/Plugins/FalconBridge
mkdir build && cd build
cmake ..
make
```

成功すると、`Assets/Plugins/macOS/FalconBridge.bundle` が作成されます。

### ステップ 2: Unityで実行

1. Unity エディタでプロジェクトを開く
2. `Assets/Scenes/SampleScene.unity` を開く
3. Novint Falcon デバイスを接続
4. Play ボタンをクリック

### ステップ 3: 動作確認

#### プラグインがビルドされていない場合

コンソールに以下のようなエラーメッセージが表示されます：

```
FalconBridge plugin not found. Please build the native plugin first.
See Assets/Plugins/FalconBridge/BUILD.md for instructions.
```

これは正常な動作です。プラグインをビルドしてから再度実行してください。

#### プラグインがビルドされている場合

Falconデバイスが正しく接続されている場合：
- コンソールに "Falcon device initialized successfully" と表示される
- Scene ビューに赤い球体（FalconToolCursor）が表示される
- Falconを動かすと、カーソルが連動して移動する
- Y=0 より下にツールを動かすと、上向きの力を感じる

## 使用方法

### 基本的な使い方（FalconController）

`FalconController` は最もシンプルな使用例です：

1. 空のGameObjectを作成
2. `FalconController` コンポーネントをアタッチ
3. Inspectorで設定を調整：
   - **Debug Log**: デバッグログの有効/無効
   - **Position Scale**: Falconの位置のスケール（メートル→Unity単位）
   - **Enable Haptics**: ハプティックフィードバックの有効/無効
   - **Test Plane Y**: テスト平面のY座標

### 高度な使い方（FalconCollisionHandler）

`FalconCollisionHandler` は3Dオブジェクトとの衝突を自動的に検出します：

1. 空のGameObjectを作成
2. `FalconCollisionHandler` コンポーネントをアタッチ
3. シーン内にColliderを持つオブジェクトを配置
4. Playモードで実行すると、オブジェクトに触れたときに力を感じる

Inspector設定：
- **Position Scale**: 位置のスケール係数
- **Tool Radius**: ツールの衝突判定半径
- **Stiffness Multiplier**: 剛性の乗数（0.1〜2.0）
- **Min Depth Threshold**: 最小の貫通深さ
- **Visualize Contact**: 接触時の視覚的フィードバック

## トラブルシューティング

### エラー: "FalconBridge plugin not found"

**原因**: ネイティブプラグインがビルドされていない

**解決策**:
1. [BUILD.md](Assets/Plugins/FalconBridge/BUILD.md) の手順に従ってプラグインをビルド
2. `Assets/Plugins/macOS/FalconBridge.bundle` が存在することを確認
3. Unityエディタを再起動

### エラー: "Failed to initialize Falcon device"

**原因**:
- Falconデバイスが接続されていない
- USBドライバが正しくインストールされていない
- USB権限の問題

**解決策**:
1. Falconデバイスが電源ONでUSB接続されていることを確認
2. macOSの場合: システム環境設定 → セキュリティとプライバシー → プライバシー → 入力監視 で Unity を許可
3. libnifalconが正しくインストールされていることを確認: `pkg-config --libs libnifalcon`

### カーソルが表示されない

**原因**: FalconControllerが初期化されていない

**解決策**:
1. コンソールでエラーメッセージを確認
2. `FalconManager` GameObjectが存在することを確認
3. `FalconController` コンポーネントが正しくアタッチされているか確認

## API使用例

### 最小限の例

```csharp
using UnityEngine;

public class MinimalFalconExample : MonoBehaviour
{
    void Start()
    {
        // Falconを初期化
        if (FalconBridge.InitFalcon())
        {
            Debug.Log("Falcon ready!");
        }
    }

    void Update()
    {
        // 位置を取得
        Vector3 pos;
        if (FalconBridge.GetToolPosition(out pos))
        {
            // Y=0より下なら上向きの力を適用
            if (pos.y < 0)
            {
                FalconBridge.SetContact(Vector3.up, -pos.y);
            }
            else
            {
                FalconBridge.ClearContact();
            }
        }
    }

    void OnDestroy()
    {
        FalconBridge.ShutdownFalcon();
    }
}
```

## 次のステップ

1. **カスタムシーンの作成**: 独自のハプティックインタラクションを設計
2. **力モデルのカスタマイズ**: 異なる材質感を表現
3. **複数オブジェクトの対応**: より複雑な環境を構築
4. **VRとの統合**: VRヘッドセットと組み合わせる

## 参考リソース

- [FalconBridge BUILD.md](Assets/Plugins/FalconBridge/BUILD.md) - ビルド詳細手順
- [README.md](README.md) - プロジェクト概要とAPI リファレンス
- [libnifalcon GitHub](https://github.com/libnifalcon/libnifalcon) - ライブラリ本家
- [novint-falcon-examples](https://github.com/afjk/novint-falcon-examples) - C++ サンプル

## サポート

問題が発生した場合は、GitHubのIssuesページで報告してください。
