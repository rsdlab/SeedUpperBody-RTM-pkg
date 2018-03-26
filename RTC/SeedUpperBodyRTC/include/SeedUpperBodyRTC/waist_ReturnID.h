//WaistReturnID.h
#ifndef __WaistReturnID_H__
#define __WaistReturnID_H__

static SeedWaist::RETURN_ID* RETURN_CODE(int id, const char *comment)
{
  SeedWaist::RETURN_ID_var RETURNCODE = new SeedWaist::RETURN_ID;
  RETURNCODE->id = id;
  RETURNCODE->comment = comment;
  return RETURNCODE._retn(); 
}

#define RETURNID(id,comment) { return RETURN_CODE(id,comment);}

#define RETURNID_OK RETURNID(SeedWaist::OK,"オペレーションを正常に受け付け.")
#define RETURNID_NG RETURNID(SeedWaist::NG,"オペレーション拒否.")
#define RETURNID_STATUS_ERR RETURNID(SeedWaist::STATUS_ERR,"オペレーションを受け付け可能な状態でない.")
#define RETURNID_VALUE_ERR RETURNID(SeedWaist::VALUE_ERR,"引数が不正.")
#define RETURNID_NOT_SV_ON_ERR RETURNID(SeedWaist::NOT_SV_ON_ERR,"全ての軸のサーボが入っていない.")
#define RETURNID_FULL_MOTION_QUEUE_ERR RETURNID(SeedWaist::FULL_MOTION_QUEUE_ERR,"バッファが一杯.")
#define RETURNID_NOT_IMPLEMENTED RETURNID(SeedWaist::NOT_IMPLEMENTED,"未実装のコマンド.")

#endif//__WaistReturnID_H__
