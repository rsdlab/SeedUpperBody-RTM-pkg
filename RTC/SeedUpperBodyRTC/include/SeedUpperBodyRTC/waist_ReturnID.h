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

#define RETURNID_OK RETURNID(SeedWaist::OK,"�I�y���[�V�����𐳏�Ɏ󂯕t��.")
#define RETURNID_NG RETURNID(SeedWaist::NG,"�I�y���[�V��������.")
#define RETURNID_STATUS_ERR RETURNID(SeedWaist::STATUS_ERR,"�I�y���[�V�������󂯕t���\�ȏ�ԂłȂ�.")
#define RETURNID_VALUE_ERR RETURNID(SeedWaist::VALUE_ERR,"�������s��.")
#define RETURNID_NOT_SV_ON_ERR RETURNID(SeedWaist::NOT_SV_ON_ERR,"�S�Ă̎��̃T�[�{�������Ă��Ȃ�.")
#define RETURNID_FULL_MOTION_QUEUE_ERR RETURNID(SeedWaist::FULL_MOTION_QUEUE_ERR,"�o�b�t�@����t.")
#define RETURNID_NOT_IMPLEMENTED RETURNID(SeedWaist::NOT_IMPLEMENTED,"�������̃R�}���h.")

#endif//__WaistReturnID_H__
