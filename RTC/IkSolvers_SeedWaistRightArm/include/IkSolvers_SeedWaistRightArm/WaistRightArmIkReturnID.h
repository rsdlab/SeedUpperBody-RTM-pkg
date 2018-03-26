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

#define RETURNID_OK RETURNID(RIGHT_IK_FAST::OK,"�I�y���[�V�����𐳏�Ɏ󂯕t��.")
#define RETURNID_NG RETURNID(RIGHT_IK_FAST::NG,"�I�y���[�V��������.")
#define RETURNID_STATUS_ERR RETURNID(RIGHT_IK_FAST::STATUS_ERR,"�I�y���[�V�������󂯕t���\�ȏ�ԂłȂ�.")
#define RETURNID_VALUE_ERR RETURNID(RIGHT_IK_FAST::VALUE_ERR,"�������s��.")
#define RETURNID_NOT_SV_ON_ERR RETURNID(RIGHT_IK_FAST::NOT_SV_ON_ERR,"�S�Ă̎��̃T�[�{�������Ă��Ȃ�.")
#define RETURNID_FULL_MOTION_QUEUE_ERR RETURNID(RIGHT_IK_FAST::FULL_MOTION_QUEUE_ERR,"�o�b�t�@����t.")
#define RETURNID_NOT_IMPLEMENTED RETURNID(RIGHT_IK_FAST::NOT_IMPLEMENTED,"�������̃R�}���h.")

#endif//__WaistRigtArmIkReturnID_H__
