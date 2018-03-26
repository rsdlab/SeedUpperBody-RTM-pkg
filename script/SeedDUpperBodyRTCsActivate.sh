#!bin/sh

export HOST="localhost"

rtact $HOST/SeedUpperBodyRTC0.rtc
rtact $HOST/IkSolvers_SeedWaistLeftArm0.rtc
rtact $HOST/IkSolvers_SeedWaistRightArm0.rtc
rtact $HOST/SeedUpperBodyController0.rtc
