//ReturnID.h
#ifndef __WaistRigtArmIkReturnID_H__
#define __WaistRigtArmIkReturnID_H__

static RIGHT_IK_FAST::RETURN_ID* RETURN_CODE(int id, const char *comment)
{
  RIGHT_IK_FAST::RETURN_ID_var RETURNCODE = new RIGHT_IK_FAST::RETURN_ID;
  RETURNCODE->id = id;
  RETURNCODE->comment = comment;
  return RETURNCODE._retn(); 
}

#define RETURNID(id,comment) { return RETURN_CODE(id,comment);}

#define RETURNID_OK RETURNID(RIGHT_IK_FAST::OK,"オペレーションを正常に受け付け.")
#define RETURNID_NG RETURNID(RIGHT_IK_FAST::NG,"オペレーション拒否.")
#define RETURNID_STATUS_ERR RETURNID(RIGHT_IK_FAST::STATUS_ERR,"オペレーションを受け付け可能な状態でない.")
#define RETURNID_VALUE_ERR RETURNID(RIGHT_IK_FAST::VALUE_ERR,"引数が不正.")
#define RETURNID_NOT_SV_ON_ERR RETURNID(RIGHT_IK_FAST::NOT_SV_ON_ERR,"全ての軸のサーボが入っていない.")
#define RETURNID_FULL_MOTION_QUEUE_ERR RETURNID(RIGHT_IK_FAST::FULL_MOTION_QUEUE_ERR,"バッファが一杯.")
#define RETURNID_NOT_IMPLEMENTED RETURNID(RIGHT_IK_FAST::NOT_IMPLEMENTED,"未実装のコマンド.")

#endif//__WaistRigtArmIkReturnID_H__
