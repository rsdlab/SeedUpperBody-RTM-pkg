//LeftReturnID.h
#ifndef __LeftReturnID_H__
#define __LeftReturnID_H__

static JARA_ARM_LEFT::RETURN_ID* RETURN_CODE(int id, const char *comment)
{
  JARA_ARM_LEFT::RETURN_ID_var RETURNCODE = new JARA_ARM_LEFT::RETURN_ID;
  RETURNCODE->id = id;
  RETURNCODE->comment = comment;
  return RETURNCODE._retn(); 
}

#define RETURNID(id,comment) { return RETURN_CODE(id,comment);}

#define RETURNID_OK RETURNID(JARA_ARM_LEFT::OK,"�I�y���[�V�����𐳏�Ɏ󂯕t��.")
#define RETURNID_NG RETURNID(JARA_ARM_LEFT::NG,"�I�y���[�V��������.")
#define RETURNID_STATUS_ERR RETURNID(JARA_ARM_LEFT::STATUS_ERR,"�I�y���[�V�������󂯕t���\�ȏ�ԂłȂ�.")
#define RETURNID_VALUE_ERR RETURNID(JARA_ARM_LEFT::VALUE_ERR,"�������s��.")
#define RETURNID_NOT_SV_ON_ERR RETURNID(JARA_ARM_LEFT::NOT_SV_ON_ERR,"�S�Ă̎��̃T�[�{�������Ă��Ȃ�.")
#define RETURNID_FULL_MOTION_QUEUE_ERR RETURNID(JARA_ARM_LEFT::FULL_MOTION_QUEUE_ERR,"�o�b�t�@����t.")
#define RETURNID_NOT_IMPLEMENTED RETURNID(JARA_ARM_LEFT::NOT_IMPLEMENTED,"�������̃R�}���h.")

#endif//__LEFTReturnID_H__
