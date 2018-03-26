#!/bin/sh

COM="gnome-terminal -e"
export HOST="localhost"

echo "ネーミングサーバーを実行します。"
sudo rtm-naming

#===========================SeedUpperBodyRTCsEXE.sh============================
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

#===========================SeedUpperBodyRTCsConnect.sh============================

echo ""
echo "SEED-Noid上半身制御コンポーネントを接続します" 
echo ""


rtcon $HOST/SeedUpperBodyRTC0.rtc:RightManipulatorCommonInterface_Common $HOST/SeedUpperBodyController0.rtc:RightManipulatorCommonInterface_Common
rtcon $HOST/SeedUpperBodyRTC0.rtc:RightManipulatorCommonInterface_Middle $HOST/SeedUpperBodyController0.rtc:RightManipulatorCommonInterface_Middle
rtcon $HOST/SeedUpperBodyRTC0.rtc:LeftManipulatorCommonInterface_Common $HOST/SeedUpperBodyController0.rtc:LeftManipulatorCommonInterface_Common
rtcon $HOST/SeedUpperBodyRTC0.rtc:LeftManipulatorCommonInterface_Middle $HOST/SeedUpperBodyController0.rtc:LeftManipulatorCommonInterface_Middle
rtcon $HOST/SeedUpperBodyRTC0.rtc:SeedWaistInterface $HOST/SeedUpperBodyController0.rtc:SeedWaistInterface
rtcon $HOST/SeedUpperBodyRTC0.rtc:SeedNeckInterface $HOST/SeedUpperBodyController0.rtc:SeedNeckInterface

rtcon $HOST/IkSolvers_SeedWaistRightArm0.rtc:WaustRightArmIk $HOST/SeedUpperBodyController0.rtc:WaustRightArmKinematics
rtcon $HOST/IkSolvers_SeedWaistLeftArm0.rtc:WaustLeftArmIk $HOST/SeedUpperBodyController0.rtc:WaustLeftArmKinematics
