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

#define RETURNID_OK RETURNID(JARA_ARM::OK,"�I�y���[�V�����𐳏�Ɏ󂯕t��.")
#define RETURNID_NG RETURNID(JARA_ARM::NG,"�I�y���[�V��������.")
#define RETURNID_STATUS_ERR RETURNID(JARA_ARM::STATUS_ERR,"�I�y���[�V�������󂯕t���\�ȏ�ԂłȂ�.")
#define RETURNID_VALUE_ERR RETURNID(JARA_ARM::VALUE_ERR,"�������s��.")
#define RETURNID_NOT_SV_ON_ERR RETURNID(JARA_ARM::NOT_SV_ON_ERR,"�S�Ă̎��̃T�[�{�������Ă��Ȃ�.")
#define RETURNID_FULL_MOTION_QUEUE_ERR RETURNID(JARA_ARM::FULL_MOTION_QUEUE_ERR,"�o�b�t�@����t.")
#define RETURNID_NOT_IMPLEMENTED RETURNID(JARA_ARM::NOT_IMPLEMENTED,"�������̃R�}���h.")

#endif//__RightReturnID_H__