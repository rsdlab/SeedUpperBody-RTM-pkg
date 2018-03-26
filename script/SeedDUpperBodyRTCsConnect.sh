#!bin/sh

export HOST="localhost"

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
