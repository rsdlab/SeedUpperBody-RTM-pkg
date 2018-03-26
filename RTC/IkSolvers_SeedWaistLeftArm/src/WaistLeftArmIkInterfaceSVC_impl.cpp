// -*-C++-*-
/*!
 * @file  WaistLeftArmIkInterfaceSVC_impl.cpp
 * @brief Service implementation code of WaistLeftArmIkInterface.idl
 *
 */

#include "WaistLeftArmIkInterfaceSVC_impl.h"
#include "WaistLeftArmIkReturnID.h"

/*
 * Example implementational code for IDL interface LEFT_IK_FAST::WaistLeftArmIkInterface
 */
LEFT_IK_FAST_WaistLeftArmIkInterfaceSVC_impl::LEFT_IK_FAST_WaistLeftArmIkInterfaceSVC_impl()
{
  // Please add extra constructor code here.
}


LEFT_IK_FAST_WaistLeftArmIkInterfaceSVC_impl::~LEFT_IK_FAST_WaistLeftArmIkInterfaceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
LEFT_IK_FAST::RETURN_ID* LEFT_IK_FAST_WaistLeftArmIkInterfaceSVC_impl::solveLeftArmIkSetWaistJoint(const LEFT_IK_FAST::Rotation& eerot, const LEFT_IK_FAST::FramePos& eetrans, const LEFT_IK_FAST::WaistFrame& WaistPos, const LEFT_IK_FAST::JointPos& nowJointPos, LEFT_IK_FAST::JointPos_out solLeftJointPos)
{
  std::cout << "solveLeftArmIkSetWaistJoint" << std::endl;
  int ret; 
  double sol_eerot[9];
  double sol_eetrans[3];
  std::vector<IkReal> free_joint(GetNumFreeParameters());
  int NumSolutions = 0;
  int phase = 0;
  std::vector<int> LimitSolNum;
  std::vector<std::vector<double> > sol;
  int l = 0;//可動範囲内の解の数
  std::vector<double> min;
  LimitSolNum.push_back(-1);
  free_joint[0] = WaistPos.yaw*M_PI/180;
  free_joint[1] = WaistPos.pitch*M_PI/180;
  free_joint[2] = WaistPos.roll*M_PI*180;
  free_joint[3] = 0;
  
  for(int n=0;n<9;n++){
    sol_eerot[n] = eerot[n];
  }
  for(int m=0;m<3;m++){
    sol_eetrans[m] = eetrans[m];
  }
  
  solLeftJointPos = new LEFT_IK_FAST::JointPos;
  solLeftJointPos->length(10);

  //================================================================================
  //FreeJoint0~(-135)で解を探す
    for(int j=0;j>-136;j--){
      ret = all_left_ik_solve(sol_eerot,sol_eetrans,sol,free_joint,&NumSolutions);
      if(!ret){
	//printf("IK計算完了0~-135\n");
	for(int k=0;k<NumSolutions;k++){
	  if(sol[k][0]*180/M_PI>WaistYaw_LimitMax||sol[k][0]*180/M_PI<WaistYaw_LimitMin||
	     sol[k][1]*180/M_PI>WaistPitch_LimitMax||sol[k][1]*180/M_PI<WaistPitch_LimitMin||
	     sol[k][2]*180/M_PI>WaistRoll_LimitMax||sol[k][2]*180/M_PI<WaistRoll_LimitMin||
	     sol[k][3]*180/M_PI>Angle1_LimitMax||sol[k][3]*180/M_PI<Angle1_LimitMin||
	     sol[k][4]*180/M_PI>Angle2_LimitMax||sol[k][4]*180/M_PI<Angle2_LimitMin||
	     sol[k][5]*180/M_PI>Angle3_LimitMax||sol[k][5]*180/M_PI<Angle3_LimitMin||
	     sol[k][6]*180/M_PI>Angle4_LimitMax||sol[k][6]*180/M_PI<Angle4_LimitMin||
	     sol[k][7]*180/M_PI>Angle5_LimitMax||sol[k][7]*180/M_PI<Angle5_LimitMin||
	     sol[k][8]*180/M_PI>Angle6_LimitMax||sol[k][8]*180/M_PI<Angle6_LimitMin||
	     sol[k][9]*180/M_PI>Angle7_LimitMax||sol[k][9]*180/M_PI<Angle7_LimitMin){
	    continue;
	  }//end if
	  else{
	    LimitSolNum[l] = k;
	    l++;
	  }//end else
	}//end for(int k=0;k<NumSolutions;k++)
	if(l==0){//可動範囲内の解がない
	  free_joint[3] = (j-1)*(M_PI/180);	
	  if(j==Angle3_LimitMin){//freeが0~-90は最後まで可動範囲外
	    ret = -1;
	  }
	  continue;
	}
	if(l!=0){//可動範囲内の解があった
	  phase = 2;
	  break;
	}
      }//end if(!ret)
      else{
	free_joint[3] = j*M_PI/180;
	continue; 
      }    
    }//end for(int j=0;j>-90;j--)

  //=============================================================================
  //FreeJoint0~135で解を探す
    if(ret)//FreeJoint0~-90で解が見つかってない
      {
	for(int i=0;i<136;i++){
	  ret = all_left_ik_solve(sol_eerot,sol_eetrans,sol,free_joint,&NumSolutions);
	  if(!ret){
	    //printf("IK計算完了0~135\n");
	    for(int k=0;k<NumSolutions;k++){
	      if(sol[k][0]*180/M_PI>WaistYaw_LimitMax||sol[k][0]*180/M_PI<WaistYaw_LimitMin||
		 sol[k][1]*180/M_PI>WaistPitch_LimitMax||sol[k][1]*180/M_PI<WaistPitch_LimitMin||
		 sol[k][2]*180/M_PI>WaistRoll_LimitMax||sol[k][2]*180/M_PI<WaistRoll_LimitMin||
		 sol[k][3]*180/M_PI>Angle1_LimitMax||sol[k][3]*180/M_PI<Angle1_LimitMin||
		 sol[k][4]*180/M_PI>Angle2_LimitMax||sol[k][4]*180/M_PI<Angle2_LimitMin||
		 sol[k][5]*180/M_PI>Angle3_LimitMax||sol[k][5]*180/M_PI<Angle3_LimitMin||
		 sol[k][6]*180/M_PI>Angle4_LimitMax||sol[k][6]*180/M_PI<Angle4_LimitMin||
		 sol[k][7]*180/M_PI>Angle5_LimitMax||sol[k][7]*180/M_PI<Angle5_LimitMin||
		 sol[k][8]*180/M_PI>Angle6_LimitMax||sol[k][8]*180/M_PI<Angle6_LimitMin||
		 sol[k][9]*180/M_PI>Angle7_LimitMax||sol[k][9]*180/M_PI<Angle7_LimitMin){
		continue;//可動範囲外
	      }//end if
	      else{
		LimitSolNum[l] = k;
		l++;
	      }//end else
	    }//end for(int k=0;k<NumSolutions;k++)
	    if(l==0){//可動範囲内の解がない
	      free_joint[3] = (i+1)*(M_PI/180);
	      if(i==Angle3_LimitMax){//freeが0~90は最後まで可動範囲外
		ret = -1;
	      }
	      continue;
	    }
	    if(l!=0){//可動範囲内の解があった
	      phase = 2;
	      break;
	    }
	    
	  }
	  else{//if(!ret)
	    //printf("i = %d\n",i);
	    free_joint[3] = i*M_PI/180;
	    continue; 
	  } 
	}//end  for(int i=0;i<91;i++)
      }//end if(ret)
    //=======================================================================================
    
    
    //現在値から近い解を選択する
    int MinAveNum=0;
    int MinAve;
    if(phase == 2){
      double diffJointPos[l][10];
      for(int n=0;n<l;n++){
	for(int a=0;a<ARM_FREEDOM+3;a++){
	  diffJointPos[n][a] = (nowJointPos[a]*M_PI/180) - sol[LimitSolNum[n]][a];
	}//end for(int a=0;a<ARM_FREEDOM;a++)
      }//end for(int n=0;n<l;n++)
    double Average[l];
    for(int av=0;av<l;av++){
      Average[av] = (diffJointPos[av][0] + diffJointPos[av][1] + diffJointPos[av][2] + diffJointPos[av][3] + diffJointPos[av][4] + diffJointPos[av][5] + diffJointPos[av][6] + diffJointPos[av][7] + diffJointPos[av][8] + diffJointPos[av][9])/ARM_FREEDOM+3;//ARM_FREEDOM=7
      Average[av] = fabs(Average[av]);
      //std::cout << "Average["<<av<<"] = "<< Average[av] <<std::endl;  
    }
    MinAveNum = 0;
    MinAve = Average[0];
    for(int mi=1;mi<l;mi++){
      if(MinAve > Average[mi]){
	MinAve = Average[mi];
	MinAveNum = mi;
      }
    }
    }//end if(phase == 2)
    
    
    if(!ret&&LimitSolNum[MinAveNum]!=-1){
      //std::cout << "弧度法の角度[rad]" << std::endl;
      for(int end=0;end<ARM_FREEDOM+3;end++){
	(*solLeftJointPos)[end] = sol[LimitSolNum[MinAveNum]][end]*180/M_PI;
	/*
	if(end==4){
	  std::cout << -sol[LimitSolNum[MinAveNum]][end]+M_PI/2 << std::endl;
	}
	else if(end==5){
	  std::cout << -sol[LimitSolNum[MinAveNum]][end] << std::endl;
	}
	else if(end==7){
	  std::cout << -sol[LimitSolNum[MinAveNum]][end] << std::endl;
	}
	else if(end==8){
	  std::cout << -sol[LimitSolNum[MinAveNum]][end] << std::endl;
	}
	else if(end==9){
	  std::cout << -sol[LimitSolNum[MinAveNum]][end] << std::endl;
	}
	else{
	  std::cout << sol[LimitSolNum[MinAveNum]][end]<<std::endl;
	  //std::cout << LimitSolNum[MinAveNum] <<std::endl;
	}
	*/
      }
      std::cout << "IK計算完了" << std::endl;
      RETURNID_OK;
    }
    else{
      std::cout << "与えられた位置姿勢ではIKが解けません" << std::endl;
      RETURNID_NG;
    }  
}

LEFT_IK_FAST::RETURN_ID* LEFT_IK_FAST_WaistLeftArmIkInterfaceSVC_impl::solveLeftArmIk(const LEFT_IK_FAST::Rotation& eerot, const LEFT_IK_FAST::FramePos& eetrans, const LEFT_IK_FAST::JointPos& nowJointPos, LEFT_IK_FAST::JointPos_out solLeftJointPos)
{
  std::cout << "solveLeftArmIk" << std::endl;

  LEFT_IK_FAST::WaistFrame WaistPos;
  LEFT_IK_FAST::RETURN_ID_var m_rid;
  WaistPos.yaw = 0;
  WaistPos.pitch = 0;
  WaistPos.roll = 0;
  int i=0;


	  //============================pitch_plus_START======================================
	  for(int pitch_plus=0;pitch_plus<WaistPitch_LimitMax;pitch_plus++)
	    {
	      WaistPos.yaw = 0;
	      WaistPos.pitch = pitch_plus;
	      WaistPos.roll = 0;
	      m_rid=solveLeftArmIkSetWaistJoint(eerot,eetrans,WaistPos,nowJointPos,solLeftJointPos);
	      if(m_rid->id == 0){//IK計算完了
		std::cout << "solveRightArmIk_IK計算完了" <<std::endl;
		RETURNID_OK;
	      }
	      if(m_rid->id == -1){//IK計算できず
		//std::cout << "IK計算できずー続行" <<std::endl;
		//std::cout << "i = " << i << std::endl;
		//i++;
		continue;
	      }
	    }
	  for(int pitch_minus=0;pitch_minus>WaistPitch_LimitMin;pitch_minus--)
	    {
	      WaistPos.yaw = 0;
	      WaistPos.pitch = pitch_minus;
	      WaistPos.roll = 0;
	      m_rid=solveLeftArmIkSetWaistJoint(eerot,eetrans,WaistPos,nowJointPos,solLeftJointPos);
	      if(m_rid->id == 0){//IK計算完了
		std::cout << "solveRightArmIk_IK計算完了" <<std::endl;
		RETURNID_OK;
	      }
	      if(m_rid->id == -1){//IK計算できず
		//std::cout << "IK計算できずー続行" <<std::endl;
		//std::cout << "i = " << i << std::endl;
		//i++;
		continue;
	      }
	    }
	  //=============================pitch_minus_END====================================

  std::cout << "solveLeftArmIk_IK計算できず" <<std::endl;
  RETURNID_NG;
  //RETURNID_OK;
}

LEFT_IK_FAST::RETURN_ID* LEFT_IK_FAST_WaistLeftArmIkInterfaceSVC_impl::solveLeftArmFk(const LEFT_IK_FAST::JointPos& LeftJointPos, LEFT_IK_FAST::Rotation_out eerot, LEFT_IK_FAST::FramePos_out eetrans)
{
  std::cout << "solveLeftArmFK" <<std::endl;
  eerot = new LEFT_IK_FAST::Rotation;
  eerot->length(9);
  eetrans = new LEFT_IK_FAST::FramePos;
  eetrans->length(3);
  double ikjoint[10];
  double calceerot[9];
  double calceetrans[3];
  ikjoint[0] = LeftJointPos[0]*M_PI/180;
  ikjoint[1] = LeftJointPos[1]*M_PI/180;
  ikjoint[2] = LeftJointPos[2]*M_PI/180;
  ikjoint[3] = -LeftJointPos[3]*M_PI/180;
  ikjoint[4] = LeftJointPos[4]*M_PI/180-M_PI/2;
  ikjoint[5] = LeftJointPos[5]*M_PI/180;
  ikjoint[6] = -LeftJointPos[6]*M_PI/180;
  ikjoint[7] = LeftJointPos[7]*M_PI/180;
  ikjoint[8] = LeftJointPos[8]*M_PI/180;
  ikjoint[9] = LeftJointPos[9]*M_PI/180; 

  ComputeFk(ikjoint,calceetrans,calceerot);
  //for(int j=0;j<7;j++)
  //  {
  //    std::cout << "right_ik_joint["<<j<<"] = "<<ikjoint[j]<<std::endl;
  //  }
  for(int i=0;i<9;i++)
    {
      eerot[i] = calceerot[i];
      //std::cout << "eerot["<<i<<"] = "<<eerot[i]<<std::endl;
    }
  for(int k=0;k<3;k++)
    {
      eetrans[k] = calceetrans[k]*1000;
      //std::cout << "eetrans["<<k<<"] = "<<eetrans[k]<<std::endl;
  }
  std::cout << "Success" <<std::endl;
  RETURNID_OK;
}



// End of example implementational code



