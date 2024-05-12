`actuator_custom_msgs`はNHK_ROSパッケージ全体で利用するカスタムメッセージ用のパッケージ

`R1/cpp1_nhk`はR1のノードのうちC++で記述されたものを集めたパッケージ。また，R2との共通ノードもここにある。

`R1/py1_nhk`はR1のノードのうちPythonで記述されたものを集めたパッケージ。また，R1のlaunchファイルもここにある。

`R2/cpp2_nhk`はR1のノードのうちC++で記述されたものを集めたパッケージ。

`R2/py2_nhk`はR1のノードのうちPythonで記述されたものを集めたパッケージ。また，R2のlaunchファイルもここにある。

setuptoolsのバージョンを下げないと警告が出ます。
pip install setuptools==58.2.0
pip install lapx