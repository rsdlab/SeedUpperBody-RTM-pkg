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

#define RETURNID_OK RETURNID(SeedNeck::OK,"�I�y���[�V�����𐳏�Ɏ󂯕t��.")
#define RETURNID_NG RETURNID(SeedNeck::NG,"�I�y���[�V��������.")
#define RETURNID_STATUS_ERR RETURNID(SeedNeck::STATUS_ERR,"�I�y���[�V�������󂯕t���\�ȏ�ԂłȂ�.")
#define RETURNID_VALUE_ERR RETURNID(SeedNeck::VALUE_ERR,"�������s��.")
#define RETURNID_NOT_SV_ON_ERR RETURNID(SeedNeck::NOT_SV_ON_ERR,"�S�Ă̎��̃T�[�{�������Ă��Ȃ�.")
#define RETURNID_FULL_MOTION_QUEUE_ERR RETURNID(SeedNeck::FULL_MOTION_QUEUE_ERR,"�o�b�t�@����t.")
#define RETURNID_NOT_IMPLEMENTED RETURNID(SeedNeck::NOT_IMPLEMENTED,"�������̃R�}���h.")

#endif//__NeckReturnID_H__
