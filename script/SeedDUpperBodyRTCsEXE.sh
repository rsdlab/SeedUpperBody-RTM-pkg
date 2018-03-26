#!/bin/sh

COM="gnome-terminal -e"

echo "ネーミングサーバーを実行します。"
sudo rtm-naming

echo ""
echo "SEED-Noid上半身制御コンポーネント群を起動します。"
echo "デフォルトではシリアルポートは /dev/ttyUSB0 に設定されます。"
echo "変更する場合RTSystemEditorを起動して変更して下さい"
echo ""

#コンポーネント実行
$COM "../RTC/SeedUpperBodyRTC/build/src/SeedUpperBodyRTCComp -f ../RTC/SeedUpperBodyRTC/rtc.conf" -t SeedUpperBodyRTC
$COM "../RTC/SeedUpperBodyController/build/src/SeedUpperBodyControllerComp -f ../RTC/SeedUpperBodyController/rtc.conf" -t SeedUpperBodyController
$COM "../RTC/IkSolvers_SeedWaistRightArm/build/src/IkSolvers_SeedWaistRightArmComp -f ../RTC/IkSolvers_SeedWaistRightArm/rtc.conf" -t IkSolvers_SeedWaistRightArm
$COM "../RTC/IkSolvers_SeedWaistLeftArm/build/src/IkSolvers_SeedWaistLeftArmComp -f ../RTC/IkSolvers_SeedWaistLeftArm/rtc.conf" -t IkSolvers_SeedWaistLeftArm

rtconf $HOST/SeedUpperBodyRTC0.rtc set port_name "/dev/ttyUSB0"
