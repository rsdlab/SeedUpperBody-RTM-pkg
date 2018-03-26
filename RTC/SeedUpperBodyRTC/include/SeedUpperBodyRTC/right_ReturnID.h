//RightReturnID.h
#ifndef __RightReturnID_H__
#define __RightReturnID_H__

static JARA_ARM::RETURN_ID* RETURN_CODE(int id, const char *comment)
{
  JARA_ARM::RETURN_ID_var RETURNCODE = new JARA_ARM::RETURN_ID;
  RETURNCODE->id = id;
  RETURNCODE->comment = comment;
  return RETURNCODE._retn(); 
}

#define RETURNID(id,comment) { return RETURN_CODE(id,comment);}

#define RETURNID_OK RETURNID(JARA_ARM::OK,"オペレーションを正常に受け付け.")
#define RETURNID_NG RETURNID(JARA_ARM::NG,"オペレーション拒否.")
#define RETURNID_STATUS_ERR RETURNID(JARA_ARM::STATUS_ERR,"オペレーションを受け付け可能な状態でない.")
#define RETURNID_VALUE_ERR RETURNID(JARA_ARM::VALUE_ERR,"引数が不正.")
#define RETURNID_NOT_SV_ON_ERR RETURNID(JARA_ARM::NOT_SV_ON_ERR,"全ての軸のサーボが入っていない.")
#define RETURNID_FULL_MOTION_QUEUE_ERR RETURNID(JARA_ARM::FULL_MOTION_QUEUE_ERR,"バッファが一杯.")
#define RETURNID_NOT_IMPLEMENTED RETURNID(JARA_ARM::NOT_IMPLEMENTED,"未実装のコマンド.")

#endif//__RightReturnID_H__
