//NeckReturnID.h
#ifndef __NeckReturnID_H__
#define __NeckReturnID_H__

static SeedNeck::RETURN_ID* RETURN_CODE(int id, const char *comment)
{
  SeedNeck::RETURN_ID_var RETURNCODE = new SeedNeck::RETURN_ID;
  RETURNCODE->id = id;
  RETURNCODE->comment = comment;
  return RETURNCODE._retn(); 
}

#define RETURNID(id,comment) { return RETURN_CODE(id,comment);}

#define RETURNID_OK RETURNID(SeedNeck::OK,"オペレーションを正常に受け付け.")
#define RETURNID_NG RETURNID(SeedNeck::NG,"オペレーション拒否.")
#define RETURNID_STATUS_ERR RETURNID(SeedNeck::STATUS_ERR,"オペレーションを受け付け可能な状態でない.")
#define RETURNID_VALUE_ERR RETURNID(SeedNeck::VALUE_ERR,"引数が不正.")
#define RETURNID_NOT_SV_ON_ERR RETURNID(SeedNeck::NOT_SV_ON_ERR,"全ての軸のサーボが入っていない.")
#define RETURNID_FULL_MOTION_QUEUE_ERR RETURNID(SeedNeck::FULL_MOTION_QUEUE_ERR,"バッファが一杯.")
#define RETURNID_NOT_IMPLEMENTED RETURNID(SeedNeck::NOT_IMPLEMENTED,"未実装のコマンド.")

#endif//__NeckReturnID_H__
