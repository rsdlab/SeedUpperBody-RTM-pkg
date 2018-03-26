
#include "LibSeednoidUpper.h"

//グローバル宣言
#ifdef _USE_SERIALCOM_CLASS
SerialCom sc;
#endif
Seednoid noid;


Seednoid::Seednoid()
{
  for (int i = 0; i<SEED_FREEDOM; i++){
	  noidJointPos[i] = 32767;
	  ReadAngle[i] = 0;
  }
  SeedArmJointLimit[0].Upper = Angle1_LimitMax;
  SeedArmJointLimit[0].Lower = Angle1_LimitMin;
  SeedArmJointLimit[1].Upper = Angle2_LimitMax;
  SeedArmJointLimit[1].Lower = Angle2_LimitMin;
  SeedArmJointLimit[2].Upper = Angle3_LimitMax;
  SeedArmJointLimit[2].Lower = Angle3_LimitMin;
  SeedArmJointLimit[3].Upper = Angle4_LimitMax;
  SeedArmJointLimit[3].Lower = Angle4_LimitMin;
  SeedArmJointLimit[4].Upper = Angle5_LimitMax;
  SeedArmJointLimit[4].Lower = Angle5_LimitMin;
  SeedArmJointLimit[5].Upper = Angle6_LimitMax;
  SeedArmJointLimit[5].Lower = Angle6_LimitMin;
  SeedArmJointLimit[6].Upper = Angle7_LimitMax;
  SeedArmJointLimit[6].Lower = Angle7_LimitMin;
  SeedNeckJointLimit[0].Upper = NeckRoll_LimitMax;
  SeedNeckJointLimit[0].Lower = NeckRoll_LimitMin;
  SeedNeckJointLimit[1].Upper = NeckPitch_LimitMax;
  SeedNeckJointLimit[1].Lower = NeckPitch_LimitMin;
  SeedNeckJointLimit[2].Upper = NeckYaw_LimitMax;
  SeedNeckJointLimit[2].Lower = NeckYaw_LimitMin;
  SeedWaistJointLimit[0].Upper = WaistRoll_LimitMax;
  SeedWaistJointLimit[0].Lower = WaistRoll_LimitMin;
  SeedWaistJointLimit[1].Upper = WaistPitch_LimitMax;
  SeedWaistJointLimit[1].Lower = WaistPitch_LimitMin;
  SeedWaistJointLimit[2].Upper = WaistYaw_LimitMax;
  SeedWaistJointLimit[2].Lower = WaistYaw_LimitMin;
  
  for(int j=0;j<7;j++){
    if(j==3)
      RightArmSetJointPos[j] = 180;
    else
      RightArmSetJointPos[j] = 0;
  }
  for(int k=0;k<7;k++){
    if(k==3)
      LeftArmSetJointPos[k] = 180;
    else
      LeftArmSetJointPos[k] = 0;
  }
  
  NeckSetJointPos[0] = 0;
  NeckSetJointPos[1] = 0;
  NeckSetJointPos[2] = 0;
  WaistSetJointPos[0] = 0;
  WaistSetJointPos[1] = 0;
  WaistSetJointPos[2] = 0;

  NeedCancelScriptRightHand = 0;
  NeedCancelScriptLeftHand = 0;

#ifndef _USE_SERIALCOM_CLASS
  fd = 0;
  bzero(&newtio, sizeof(newtio)); //initialize
#endif
}

/************************************************

	int OpenSerialPort(const char *SERIAL_PORT)

	概要：シリアル通信を開始する

	引数：const char *SERIAL_PORT・・・ポート名
	
	戻り値：なし

*************************************************/
int Seednoid::OpenSerialPort(const char *SERIAL_PORT)
{
#ifdef _USE_SERIALCOM_CLASS
  if(!sc.Initialize(SERIAL_PORT,1000000)){
    std::cout<<"error"<<endl;
    return false;
  }
  return true;
#else
  dev = (char*)SERIAL_PORT;
  fd = open(dev, O_RDWR | O_NOCTTY );
  if(fd < 0){
    std::cout<<"Cannot Open expected COM Port!"<<std::endl;
    return false;
  }
  else{
    std::cout<<"Open COM Port:"<<dev<<std::endl;
  }
  
  newtio.c_cflag = B1000000 | CS8 | CREAD | CLOCAL;
  
  newtio.c_cc[VTIME] = 10;
  newtio.c_cc[VMIN] = 10;
  newtio.c_lflag = 0;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  
  tcflush(fd, TCIOFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);
  
  return true;

#endif
}

/************************************************

	int CloseSerialPort()

	概要：シリアル通信を終了する

	引数：なし
	
	戻り値：なし

*************************************************/
void Seednoid::CloseSerialPort()
{
#ifdef _USE_SERIALCOM_CLASS
  sc.Finalize();
#else
  std::cout<<"Close COM Port:"<<dev<<std::endl;
  close(fd);
#endif
}

/************************************************

	void serialWrite(uchar *buf, int length)

	概要：シリアル通信でデータを送信する

	引数：
              *buf・・・送るデータの配列
              length・・・データの長さ
	
	戻り値：なし

*************************************************/
void Seednoid::serialWrite(uchar *buf, int length)
{
#ifdef _USE_SERIALCOM_CLASS
  sc.SendMessage(buf, length);
#else
  write(fd, buf, length);
  tcflush(fd, TCOFLUSH);
#endif
}


/************************************************

	void serialRead(uchar *recv)

	概要：シリアル通信でデータを送信する

	引数：
              *recv・・・受け取るデータの配列
            
	
	戻り値：なし

*************************************************/
void Seednoid::serialRead(uchar *recv, int length)
{
#ifdef _USE_SERIALCOM_CLASS
  sc.ReceiveMessage(recv);
#else
  read(fd, recv, length);
#endif
}



/************************************************

	char calcCheckSum(char *buf, int Datasize)

	概要：チェックサムを計算する

	引数：
              *buf・・・送るデータの配列
              Datasize・・・データの数
	
	戻り値：計算結果

*************************************************/
uchar Seednoid::calcCheckSum(uchar *buf, int Datasize)
{
  int checksum = 0;

  buf += 2;	//skip 0xFD 0xDF

  for(int i=2;i<Datasize-1;i++)
    {
      checksum += *buf;
      buf += 1;
      //printf("byte %d   checksum = %x\n",i,checksum);
    }
  return 0xff & (~checksum);
}

/************************************************

       void ReadServoAngle(int id, double ReadAngle)

       概要：サーボのAngle値を受け取る

       引数：
             id・・・サーボのID

       戻り値：なし

*************************************************/
void Seednoid::ReadServoAngle(double ReadAngle[])
{
  int i = 5;  //skip4byte
  int j = 0;
  uchar buf[6] = {0};
  uchar recv[68] = {0};
  uchar Hex_high;
  uchar Hex_low;
  buf[0] = 0xfd;
  buf[1] = 0xdf;
  buf[2] = 0x02;
  buf[3] = 0x41;
  buf[4] = 0x00;//全軸指令
  buf[5] = calcCheckSum(buf, sizeof(buf));

  serialWrite(buf,sizeof(buf));
  usleep(10000);
  serialRead(recv,sizeof(recv));

  for(i=5;i<65;i=i+2)
    {
      Hex_high = recv[i];
      Hex_low = recv[i+1];
      ReadAngle[j] = (Hex_high<<8)|(Hex_low);
      j++;
    }
  //for(int h=0;h<68;h++){
  //  printf("recv[%d] = %x\n",h,recv[h]);
  //}
  for(int l=0;l<30;l++){
    if(ReadAngle[l]>50000)//noidJointPosがマイナスだが下位8ビットしかreadできないため
      {
	ReadAngle[l] = ReadAngle[l]-65536;
      }
    //printf("ReadAngle[%d] = %f\n",l,ReadAngle[l]);
  }


}


/************************************************

	void initPosition()

	概要：全身を初期位置に移動する

	引数：なし
	
	戻り値：なし

*************************************************/
void Seednoid::initPosition()
{
  for (int k = 0; k<SEED_FREEDOM; k++){
    noidJointPos[k] = 0;
    if(k==6)
      {
	noidJointPos[k] = 42.4260 * 100;//180°　右肘ピッチ
      }
    if(k==11)
      {
	noidJointPos[k] = 32767;//0x7fff RightHand
      }
    if(k==21)
      {
	noidJointPos[k] = 42.4260 * 100;//180° 左肘ピッチ
      }
    if(k==26)
      {
	noidJointPos[k] = 32767;//0x7fff LeftHand
      }
  }
  std::cout << "init position" << std::endl;
}

/************************************************

      void ServoOnOff(int torque)

      概要：Seednoid全身のサーボONOFFさせる

      引数：
            int truque・・・トルク(1…ON,0…OFF)

      戻り値：なし

*************************************************/
void Seednoid::ServoOnOff(int torque)
{

  int i = 5;  //skip4byte
  uchar buf[68] = {0};
  buf[0] = 0xfd;
  buf[1] = 0xdf;
  buf[2] = 0x40;
  buf[3] = 0x21;
  buf[4] = 0x00;
  for(i=5;i<65;i=i+2)
    {
      buf[i] = 0x00;
      buf[i+1] = torque & 0xff;
    }
  buf[65] = 0x00;
  buf[66] = 0x00;
  buf[67] = calcCheckSum(buf, sizeof(buf));
  
  /*
  for(int j=0;j<68;j++)
    {
      printf("byte:%d  ",j);
      printf("buf[%d] = %x\n",j,buf[j]);
    }
  */
    
  serialWrite(buf, sizeof(buf)); 
  

}

/************************************************

      int setRightJointAngle(double JointPos[])

      概要：関節角度を入力するとSeednoid用の値に変換してnoidJointPos[]に書き込む(右腕のみ)

      引数：
            double JointPos[]・・・関節角度
                       JointPos[0]:肩ピッチ
                       JointPos[1]:肩ロール
                       JointPos[2]:肘ヨー
                       JointPos[3]:肘ピッチ
                       JointPos[4]:手首ヨー
                       JointPos[5]:手首ピッチ
                       JointPos[6]:手首ロール
      戻り値：なし

*************************************************/
int Seednoid::setRightJointAngle(double JointPos[])
{
  //回転軸は指令値＝角度[deg]*100　，　直動軸は指令値＝ストローク100
  //JointPos[5]を手首ピッチ、JointPos[6]を手首ロールとする
  //ikfastで解いているモデルではJoint6がピッチ方向、Joint7がロール方向
  double right_tekubi_pitch;
  double right_tekubi_roll;

  //可動範囲の確認
  for(int i=0;i<7;i++){
    if(JointPos[i]>SeedArmJointLimit[i].Upper||JointPos[i]<SeedArmJointLimit[i].Lower){
      std::cout << "ERROR : アーム可動範囲外の値が入力されました" << std::endl;     
      return -1;
    }
  }

  for(int j=0;j<7;j++){
    RightArmSetJointPos[j] = JointPos[j];
  }

  right_tekubi_pitch = -0.0000000000323147360649714*pow(JointPos[5], 6) - 0.0000000011960639551261*pow(JointPos[5], 5) + 0.000000136757912853652*pow(JointPos[5], 4) - 0.0000137610219349258*pow(JointPos[5], 3) - 0.000567080887725977*pow(JointPos[5], 2) + 0.381931145938457*JointPos[5] - 0.000259899835136242; 
  
  right_tekubi_roll = -0.00000000000272433095433587*pow(JointPos[6], 6) - 0.000000000189055910446297*pow(JointPos[6], 5) + 0.0000000433652906966435*pow(JointPos[6], 4) - 0.00000501857786795387*pow(JointPos[6], 3) - 0.000439121122909961*pow(JointPos[6], 2) + 0.164370317437567*JointPos[6] - 0.0000402047331640176;
  
  //右腕
  //肩ピッチ 
  noidJointPos[3] = (int)((-0.0000000000933971256927911*pow(JointPos[0], 6) + 0.0000000369034355317591*pow(JointPos[0], 5) - 0.00000574010042392163*pow(JointPos[0], 4) + 0.000374322804691228*pow(JointPos[0], 3) - 0.00458523350688013*pow(JointPos[0], 2) - 0.813300679027264*JointPos[0] - 0.210097776915891) * 100);
  //肩ロール
  noidJointPos[4] = (int)((-0.0000000000018923323872504*pow(JointPos[1], 6) + 0.000000000313119551120788*pow(JointPos[1], 5) + 0.000000105806520878104*pow(JointPos[1], 4) - 0.0000494102868628943*pow(JointPos[1], 3) + 0.00456617193276543*pow(JointPos[1], 2) + 0.452293247948546*JointPos[1] + 0.000399823795305565) * 100);
  //肘ヨー
  noidJointPos[5] = (int)(JointPos[2]*100);
  //肘ピッチ
  noidJointPos[6] = (int)((0.00000000000049111019804115*pow(JointPos[3], 6) - 0.000000000390702276724364*pow(JointPos[3], 5) + 0.000000130488756156302*pow(JointPos[3], 4) - 0.0000234826685847622*pow(JointPos[3], 3) + 0.00161963481365035*pow(JointPos[3], 2) + 0.261403325221181*JointPos[3] - 42.4326561880471) * 100);
  //手首ヨー
  noidJointPos[7] = (int)(JointPos[4]*100);
  //手首ピッチ＆ロール
  noidJointPos[8] = (int)((right_tekubi_roll + right_tekubi_pitch) * 100);
  noidJointPos[9] = (int)((right_tekubi_roll - right_tekubi_pitch) * 100);
  //printf("JointPos[0] = %f[°]\n", JointPos[0]);
  //printf("JointPos[1] = %f[°]\n", JointPos[1]);
  //printf("JointPos[2] = %f[°]\n", JointPos[2]);
  //printf("JointPos[3] = %f[°]\n", JointPos[3]);
  //printf("JointPos[4] = %f[°]\n", JointPos[4]);
  //printf("JointPos[5] = %f[°]\n", JointPos[5]);
  //printf("JointPos[6] = %f[°]\n", JointPos[6]);
  //printf("noidJointPos[3] = %d\n", noidJointPos[3]);
  //printf("noidJointPos[4] = %d\n", noidJointPos[4]);
  //printf("noidJointPos[5] = %d\n", noidJointPos[5]);
  //printf("noidJointPos[6] = %d\n", noidJointPos[6]);
  //printf("noidJointPos[7] = %d\n", noidJointPos[7]);
  //printf("noidJointPos[8] = %d\n", noidJointPos[8]);
  //printf("noidJointPos[9] = %d\n", noidJointPos[9]);

  return 0;
}


/************************************************

      int setLeftJointAngle(double JointPos[])

      概要：関節角度を入力するとSeednoid用の値に変換してnoidJointPos[]に書き込む(左腕のみ)

      引数：
            double JointPos[]・・・関節角度
                       JointPos[0]:肩ピッチ
                       JointPos[1]:肩ロール
                       JointPos[2]:肘ヨー
                       JointPos[3]:肘ピッチ
                       JointPos[4]:手首ヨー
                       JointPos[5]:手首ピッチ
                       JointPos[6]:手首ロール
      戻り値：なし

*************************************************/
int Seednoid::setLeftJointAngle(double JointPos[])
{
  //回転軸は指令値＝角度[deg]*100　，　直動軸は指令値＝ストローク100
  //JointPos[5]を手首ピッチ、JointPos[6]を手首ロールとする
  //ikfastで解いているモデルではJoint6がピッチ方向、Joint7がロール方向
  double left_tekubi_pitch;
  double left_tekubi_roll;

  //可動範囲の確認
  for(int i=0;i<7;i++){
    if(JointPos[i]>SeedArmJointLimit[i].Upper||JointPos[i]<SeedArmJointLimit[i].Lower){
      std::cout << "ERROR : アーム可動範囲外の値が入力されました" << std::endl;     
      return -1;
    }
  }

  for(int j=0;j<7;j++){
    LeftArmSetJointPos[j] = JointPos[j];
  }

  left_tekubi_pitch = -0.0000000000323147360649714*pow(JointPos[5], 6) - 0.0000000011960639551261*pow(JointPos[5], 5) + 0.000000136757912853652*pow(JointPos[5], 4) - 0.0000137610219349258*pow(JointPos[5], 3) - 0.000567080887725977*pow(JointPos[5], 2) + 0.381931145938457*JointPos[5] - 0.000259899835136242; 
  
  left_tekubi_roll = -0.00000000000272433095433587*pow(JointPos[6], 6) - 0.000000000189055910446297*pow(JointPos[6], 5) + 0.0000000433652906966435*pow(JointPos[6], 4) - 0.00000501857786795387*pow(JointPos[6], 3) - 0.000439121122909961*pow(JointPos[6], 2) + 0.164370317437567*JointPos[6] - 0.0000402047331640176;
  
  //左腕
  //肩ピッチ 
  noidJointPos[18] = (int)((-0.0000000000933971256927911*pow(JointPos[0], 6) + 0.0000000369034355317591*pow(JointPos[0], 5) - 0.00000574010042392163*pow(JointPos[0], 4) + 0.000374322804691228*pow(JointPos[0], 3) - 0.00458523350688013*pow(JointPos[0], 2) - 0.813300679027264*JointPos[0] - 0.210097776915891) * 100);
  //肩ロール
  noidJointPos[19] = (int)((-0.0000000000018923323872504*pow(JointPos[1], 6) + 0.000000000313119551120788*pow(JointPos[1], 5) + 0.000000105806520878104*pow(JointPos[1], 4) - 0.0000494102868628943*pow(JointPos[1], 3) + 0.00456617193276543*pow(JointPos[1], 2) + 0.452293247948546*JointPos[1] + 0.000399823795305565) * 100);
	//肘ヨー
  noidJointPos[20] = (int)(JointPos[2]*100);
  //肘ピッチ
  noidJointPos[21] = (int)((0.00000000000049111019804115*pow(JointPos[3], 6) - 0.000000000390702276724364*pow(JointPos[3], 5) + 0.000000130488756156302*pow(JointPos[3], 4) - 0.0000234826685847622*pow(JointPos[3], 3) + 0.00161963481365035*pow(JointPos[3], 2) + 0.261403325221181*JointPos[3] - 42.4326561880471) * 100);
  //手首ヨー
  noidJointPos[22] = (int)(JointPos[4]*100);
  //手首ピッチ＆ロール
  noidJointPos[23] = (int)((left_tekubi_roll + left_tekubi_pitch) * 100);
  noidJointPos[24] = (int)((left_tekubi_roll - left_tekubi_pitch) * 100);
  //printf("JointPos[0] = %f[°]\n", JointPos[0]);
  //printf("JointPos[1] = %f[°]\n", JointPos[1]);
  //printf("JointPos[2] = %f[°]\n", JointPos[2]);
  //printf("JointPos[3] = %f[°]\n", JointPos[3]);
  //printf("JointPos[4] = %f[°]\n", JointPos[4]);
  //printf("JointPos[5] = %f[°]\n", JointPos[5]);
  //printf("JointPos[6] = %f[°]\n", JointPos[6]);
  //printf("noidJointPos[18] = %d\n", noidJointPos[18]);
  //printf("noidJointPos[19] = %d\n", noidJointPos[19]);
  //printf("noidJointPos[20] = %d\n", noidJointPos[20]);
  //printf("noidJointPos[21] = %d\n", noidJointPos[21]);
  //printf("noidJointPos[22] = %d\n", noidJointPos[22]);
  //printf("noidJointPos[23] = %d\n", noidJointPos[23]);
  //printf("noidJointPos[24] = %d\n", noidJointPos[24]);
 
  return 0;
}


/************************************************

        void setNeckJointAngle(double roll,double pitch,double yaw)

	概要：関節角度を入力するとSeednoid用の値に変換してnoidJointPos[]に書き込む(首のみ)

	引数：
              roll・・・顔ロール方向
              pitch・・・顔ピッチ方向
              yaw・・・	顔ヨー方向
  
	戻り値：なし

*************************************************/
int Seednoid::setNeckJointAngle(Frame pos)
{
  double neck_roll;
  double neck_pitch;

  //可動範囲の確認
  if(pos.roll>SeedNeckJointLimit[0].Upper||pos.roll<SeedNeckJointLimit[0].Lower||
     pos.pitch>SeedNeckJointLimit[1].Upper||pos.pitch<SeedNeckJointLimit[1].Lower||
     pos.yaw>SeedNeckJointLimit[2].Upper||pos.yaw<SeedNeckJointLimit[2].Lower){
    std::cout << "ERROR : アーム可動範囲外の値が入力されました" << std::endl;     
    return -1;
  }
  
  NeckSetJointPos[0] = pos.roll;
  NeckSetJointPos[1] = pos.pitch;
  NeckSetJointPos[2] = pos.yaw;
  
  neck_roll = -0.0000000000400086550261108*pow(pos.roll, 6) - 0.00000000163908662510759*pow(pos.roll, 5) + 0.000000218863322087176*pow(pos.roll, 4) - 0.0000226736637258251*pow(pos.roll, 3) + 0.000274072986861718*pow(pos.roll, 2) + 0.385430061655752*pos.roll - 0.0000533826671356152;
  neck_pitch = -0.00000000000275160912121836*pow(pos.pitch, 6) + 0.000000000355176744460707*pow(pos.pitch, 5) + 0.0000000294452607359653*pow(pos.pitch, 4) - 0.0000111256314356441*pow(pos.pitch, 3) + 0.000411998083208204*pow(pos.pitch, 2) + 0.169074232321956*pos.pitch - 0.000185818175493513;

  noidJointPos[0] = (int)(pos.yaw*100);
  noidJointPos[1] = (int)((neck_pitch+neck_roll)*100);
  noidJointPos[2] = (int)((neck_pitch-neck_roll)*100);

  return 0;
}

/************************************************

        void setWaistJointAngle(double roll,double pitch,double yaw)

	概要：関節角度を入力するとSeednoid用の値に変換してnoidJointPos[]に書き込む(腰のみ)

	引数：
              roll・・・腰ロール方向
              pitch・・・腰ピッチ方向
              yaw・・・	腰ヨー方向
  
	戻り値：なし

*************************************************/
int Seednoid::setWaistJointAngle(Frame pos)
{
  double waist_roll;
  double waist_pitch;

  //可動範囲の確認
  if(pos.roll>SeedWaistJointLimit[0].Upper||pos.roll<SeedWaistJointLimit[0].Lower||
     pos.pitch>SeedWaistJointLimit[1].Upper||pos.pitch<SeedWaistJointLimit[1].Lower||
     pos.yaw>SeedWaistJointLimit[2].Upper||pos.yaw<SeedWaistJointLimit[2].Lower){
    std::cout << "ERROR : アーム可動範囲外の値が入力されました" << std::endl;     
    return -1;
  }

  WaistSetJointPos[0] = pos.roll;
  WaistSetJointPos[1] = pos.pitch;
  WaistSetJointPos[2] = pos.yaw;

  waist_roll=0.0000000662755569313944*pow(pos.roll,6) - 0.000000669306194467367*pow(pos.roll,5) + 0.00000348293585972215*pow(pos.roll,4) - 0.000128728565186975*pow(pos.roll,3) + 0.00519581250091505*pow(pos.roll,2) + 0.78075913026441*pos.roll - 0.000190689301689417;
  waist_pitch=0.000000000174315032894223*pow(pos.pitch,6) - 0.0000000306996127789894*pow(pos.pitch,5) + 0.00000257454371696726*pow(pos.pitch,4) - 0.000159480493435595*pow(pos.pitch,3) + 0.00671690503845524*pow(pos.pitch,2) + 0.668272456505489*pos.pitch - 0.0000551434847361771;  

  noidJointPos[10] = (int)((waist_pitch+waist_roll)*100);
  noidJointPos[15] = (int)(pos.yaw*100);
  noidJointPos[25] = (int)((waist_pitch-waist_roll)*100);

  return 0;

}

/************************************************

      int getRightJointAngle(double JointPos[])

      概要：Seednoidクラスに格納されている右腕の関節値を入手する

      引数：
            double JointPos[]・・・関節角度
                       JointPos[0]:肩ピッチ
                       JointPos[1]:肩ロール
                       JointPos[2]:肘ヨー
                       JointPos[3]:肘ピッチ
                       JointPos[4]:手首ヨー
                       JointPos[5]:手首ピッチ
                       JointPos[6]:手首ロール
      戻り値：なし

*************************************************/
int Seednoid::getRightJointAngle(double JointPos[])
{
  JointPos[0] = RightArmSetJointPos[0];
  JointPos[1] = RightArmSetJointPos[1];
  JointPos[2] = RightArmSetJointPos[2];
  JointPos[3] = RightArmSetJointPos[3];
  JointPos[4] = RightArmSetJointPos[4];
  JointPos[5] = RightArmSetJointPos[5];
  JointPos[6] = RightArmSetJointPos[6];

  return 0;
}

/************************************************

      int getLeftJointAngle(double JointPos[])

      概要：Seednoidクラスに格納されている左腕の関節値を入手する

      引数：
            double JointPos[]・・・関節角度
                       JointPos[0]:肩ピッチ
                       JointPos[1]:肩ロール
                       JointPos[2]:肘ヨー
                       JointPos[3]:肘ピッチ
                       JointPos[4]:手首ヨー
                       JointPos[5]:手首ピッチ
                       JointPos[6]:手首ロール
      戻り値：なし

*************************************************/
int Seednoid::getLeftJointAngle(double JointPos[])
{
  JointPos[0] = LeftArmSetJointPos[0];
  JointPos[1] = LeftArmSetJointPos[1];
  JointPos[2] = LeftArmSetJointPos[2];
  JointPos[3] = LeftArmSetJointPos[3];
  JointPos[4] = LeftArmSetJointPos[4];
  JointPos[5] = LeftArmSetJointPos[5];
  JointPos[6] = LeftArmSetJointPos[6];

  return 0;
}

/************************************************

      Frame Seednoid::getNeckJointAngle()

      概要：Seednoidクラスに格納されている首の関節値を入手する

      引数：なし

      戻り値：首の関節値（ロールピッチヨー）

*************************************************/
Frame Seednoid::getNeckJointAngle()
{
  Frame pos;
  pos.roll = NeckSetJointPos[0];
  pos.pitch = NeckSetJointPos[1];
  pos.yaw = NeckSetJointPos[2];

  return pos;
}

/************************************************

      Frame Seednoid::getWaistJointAngle()

      概要：Seednoidクラスに格納されている腰の関節値を入手する

      引数：なし

      戻り値：腰の関節値（ロールピッチヨー）

*************************************************/
Frame Seednoid::getWaistJointAngle()
{
  Frame pos;
  pos.roll = WaistSetJointPos[0];
  pos.pitch = WaistSetJointPos[1];
  pos.yaw = WaistSetJointPos[2];

  return pos;
}


/************************************************

        void readRightJointAngle(double JointPos[])

	概要：SeedNoidの右腕の関節値を読み取り、度数法にしてJointPos[]に格納する

	引数：
             double JointPos[]・・・関節角度を格納する配列
  
	戻り値：なし

*************************************************/
void Seednoid::readRightJointAngle(double JointPos[])
{
  ReadServoAngle(ReadAngle);
  double SeedPos[7];  
  double tekubi_roll;
  double tekubi_pitch;
  for(int i=0;i<7;i++)
    {
      SeedPos[i] = ReadAngle[i+3]/100;
    }

  tekubi_roll = (SeedPos[5]+SeedPos[6])/2;
  tekubi_pitch = (SeedPos[5]-SeedPos[6])/2;

  JointPos[0] = -0.0000000360742099675144*pow(SeedPos[0],6) - 0.00000335029464179319*pow(SeedPos[0],5) - 0.00011218800655908*pow(SeedPos[0],4) - 0.00233475809141627*pow(SeedPos[0],3) - 0.0186432996555008*pow(SeedPos[0],2) - 1.19872714942747*SeedPos[0] + 0.0949926824675345;
  JointPos[1] = -0.00000000069708744505903*pow(SeedPos[1],6) + 0.000000252742056838784*pow(SeedPos[1],5) - 0.0000245454691594205*pow(SeedPos[1],4) + 0.00128199329537892*pow(SeedPos[1],3) - 0.0370077199622756*pow(SeedPos[1],2) + 2.17693328140012*SeedPos[1] + 0.020900383242406;
  JointPos[2] = SeedPos[2];
  JointPos[3] = 0.000000043014640453698*pow(SeedPos[3],6) + 0.0000070130668059876*pow(SeedPos[3],5) + 0.000453246553025867*pow(SeedPos[3],4) + 0.0157916727949889*pow(SeedPos[3],3) + 0.351768178778908*pow(SeedPos[3],2) + 8.53645519252888*SeedPos[3] + 179.751943668045;
  JointPos[4] = SeedPos[4];
  JointPos[5] = 0.0000000338503162167214*pow(tekubi_pitch,6) + 0.00000102621995010632*pow(tekubi_pitch,5) - 0.00000402410074364352*pow(tekubi_pitch,4) + 0.000710999018203095*pow(tekubi_pitch,3) + 0.0101874649864333*pow(tekubi_pitch,2) + 2.61843261952329*tekubi_pitch + 0.000658034453859403;
  JointPos[6] = -0.0000133460213396219*pow(tekubi_roll,6) - 0.000103540465346869*pow(tekubi_roll,5) + 0.00090058056473841*pow(tekubi_roll,4) + 0.01701227143659*pow(tekubi_roll,3) + 0.0941837692117802*pow(tekubi_roll,2) + 6.02215998169597*tekubi_roll + 0.0128766044073103;

  for(int i=0;i<7;i++)
    {
      std::cout << "RightArmJointPos["<<i<<"] = "<<JointPos[i] << "[°]" <<std::endl;
    }

}


/************************************************

        void readLeftJointAngle(double JointPos[])

	概要：SeedNoidの左腕の関節値を読み取り、度数法にしてJointPos[]に格納する

	引数：
             double JointPos[]・・・関節角度を格納する配列
  
	戻り値：なし

*************************************************/
void Seednoid::readLeftJointAngle(double JointPos[])
{
  ReadServoAngle(ReadAngle);
  double SeedPos[7];  
  double tekubi_roll;
  double tekubi_pitch;
  for(int i=0;i<7;i++)
    {
      SeedPos[i] = ReadAngle[i+18]/100;
    }

  tekubi_roll = (SeedPos[5]+SeedPos[6])/2;
  tekubi_pitch = (SeedPos[5]-SeedPos[6])/2;

  JointPos[0] = -0.0000000360742099675144*pow(SeedPos[0],6) - 0.00000335029464179319*pow(SeedPos[0],5) - 0.00011218800655908*pow(SeedPos[0],4) - 0.00233475809141627*pow(SeedPos[0],3) - 0.0186432996555008*pow(SeedPos[0],2) - 1.19872714942747*SeedPos[0] + 0.0949926824675345;
  JointPos[1] = -0.00000000069708744505903*pow(SeedPos[1],6) + 0.000000252742056838784*pow(SeedPos[1],5) - 0.0000245454691594205*pow(SeedPos[1],4) + 0.00128199329537892*pow(SeedPos[1],3) - 0.0370077199622756*pow(SeedPos[1],2) + 2.17693328140012*SeedPos[1] + 0.020900383242406;
  JointPos[2] = SeedPos[2];
  JointPos[3] = 0.000000043014640453698*pow(SeedPos[3],6) + 0.0000070130668059876*pow(SeedPos[3],5) + 0.000453246553025867*pow(SeedPos[3],4) + 0.0157916727949889*pow(SeedPos[3],3) + 0.351768178778908*pow(SeedPos[3],2) + 8.53645519252888*SeedPos[3] + 179.751943668045;
  JointPos[4] = SeedPos[4];
  JointPos[5] = 0.0000000338503162167214*pow(tekubi_pitch,6) + 0.00000102621995010632*pow(tekubi_pitch,5) - 0.00000402410074364352*pow(tekubi_pitch,4) + 0.000710999018203095*pow(tekubi_pitch,3) + 0.0101874649864333*pow(tekubi_pitch,2) + 2.61843261952329*tekubi_pitch + 0.000658034453859403;
  JointPos[6] = -0.0000133460213396219*pow(tekubi_roll,6) - 0.000103540465346869*pow(tekubi_roll,5) + 0.00090058056473841*pow(tekubi_roll,4) + 0.01701227143659*pow(tekubi_roll,3) + 0.0941837692117802*pow(tekubi_roll,2) + 6.02215998169597*tekubi_roll + 0.0128766044073103;

  for(int i=0;i<7;i++)
    {
      std::cout << "LeftArmJointPos["<<i<<"] = "<<JointPos[i] << "[°]" <<std::endl;
    }

}


/************************************************

        Frame readNeckJointAngle()

	概要：SeedNoidの首の関節値を読み取り、度数法にしてFrame構造体に格納する

	引数：
             なし
  
	戻り値：首の関節角度(roll,pitch,yaw)

*************************************************/
Frame Seednoid::readNeckJointAngle()
{
  ReadServoAngle(ReadAngle);
  Frame pos;  
  double SeedPos[3];
  double neck_roll;
  double neck_pitch;
  for(int i=0;i<3;i++)
    {
      SeedPos[i] = ReadAngle[i]/100;
    }
  neck_roll = (SeedPos[1]-SeedPos[2])/2;
  neck_pitch = (SeedPos[1]+SeedPos[2])/2;
  
  pos.roll = -0.0000000967906101934091*pow(neck_roll,6) + 0.00000275344283291901*pow(neck_roll,5) - 0.0000345131388448596*pow(neck_roll,4) + 0.00102581137967788*pow(neck_roll,3) - 0.00479308003951673*pow(neck_roll,2) + 2.59463403218453*neck_roll + 0.00014693282153511;
  pos.pitch = -0.00000221465847261015*pow(neck_pitch,6) + 0.000100142608257954*pow(neck_pitch,5) - 0.00135773530086106*pow(neck_pitch,4) + 0.0171252717519687*pow(neck_pitch,3) - 0.0862191168583828*pow(neck_pitch,2) + 5.91142349775356*neck_pitch + 0.00275500404667073; 
  pos.yaw = SeedPos[0];

  std::cout << "NeckPos.roll = " <<pos.roll << "[°]" << std::endl;
  std::cout << "NeckPos.pitch = " <<pos.pitch << "[°]"<< std::endl;
  std::cout << "NeckPos.yaw = " <<pos.yaw << "[°]" << std::endl;

  return pos;

}


/************************************************

        Frame readWaistJointAngle()

	概要：SeedNoidの腰の関節値を読み取り、度数法にしてFrame構造体に格納する

	引数：
             なし
  
	戻り値：腰の関節角度(roll,pitch,yaw)

*************************************************/
Frame Seednoid::readWaistJointAngle()
{
  ReadServoAngle(ReadAngle);
  Frame pos;  
  double SeedPos[3];
  double waist_roll;
  double waist_pitch;
  
  SeedPos[0] = ReadAngle[10]/100;
  SeedPos[1] = ReadAngle[15]/100;
  SeedPos[2] = ReadAngle[25]/100;
    
  waist_roll = (SeedPos[0]-SeedPos[2])/2;
  waist_pitch = (SeedPos[0]+SeedPos[2])/2;
  
  pos.roll = -0.000000884979920900975*pow(waist_roll,6) + 0.00000876698606313831*pow(waist_roll,5) - 0.0000291619590011077*pow(waist_roll,4) + 0.000447649797955574*pow(waist_roll,3) - 0.0108892557527606*pow(waist_roll,2) + 1.28118533903592*waist_roll + 0.000161740014248277;
  pos.pitch = -0.0000000187560450677637*pow(waist_pitch,6) + 0.00000183293729446015*pow(waist_pitch,5) - 0.0000717423204159852*pow(waist_pitch,4) + 0.00156523105429672*pow(waist_pitch,3) - 0.0241571731447025*pow(waist_pitch,2) + 1.49856262998118*waist_pitch + 0.00835796796454664; 
  pos.yaw = SeedPos[1];

  std::cout << "WaistPos.roll = " <<pos.roll << "[°]" << std::endl;
  std::cout << "WaistPos.pitch = " <<pos.pitch << "[°]" << std::endl;
  std::cout << "WaistPos.yaw = " <<pos.yaw << "[°]" << std::endl;

  return pos;
}


/************************************************

      void SeedAction()

      概要：Seednoid全身の関節を動作させる

      引数：なし

      戻り値：なし

*************************************************/
void Seednoid::SeedAction(int msectime)
{
  int i = 5;  //skip4byte
  int j = 0;
  uchar buf[68] = {0};
  uchar recv[68] = {0};
  int movetime;
  movetime = msectime / 10;
  buf[0] = 0xfd;
  buf[1] = 0xdf;
  buf[2] = 0x40;
  buf[3] = 0x14;
  buf[4] = 0x00;
  for(i=5;i<65;i=i+2)
    {
      buf[i] = (noidJointPos[j]>>8) & 0xff;
      buf[i+1] = (noidJointPos[j]) & 0xff;
      j++;
    }
  buf[65] = (movetime>>8) & 0xff;
  buf[66] = (movetime) & 0xff;//動作時間
  buf[67] = calcCheckSum(buf, sizeof(buf));

  
  //for(int j=0;j<68;j++)
  //  {
  //    printf("byte:%d  ",j);
  //    printf("buf[%d] = %x\n",j,buf[j]);
  //  }
  
    serialWrite(buf, sizeof(buf));
    usleep(1000*10);
    serialRead(recv,sizeof(recv));
  
}

/************************************************

      void OpenRightGripper()

      概要：Seednoid右腕のハンドを開く

      引数：なし

      戻り値：なし

*************************************************/
void Seednoid::OpenRightGripper()
{
  noidJointPos[11]=32767;//0x7fff

  uchar buf[8] = { 0 };
  buf[0] = 0xfd;
  buf[1] = 0xdf;
  buf[2] = 0x04;
  buf[3] = 0x22;
  buf[4] = 0x0c;
  buf[5] = 0x00;
  buf[6] = 0x03;
  buf[7] = calcCheckSum(buf, sizeof(buf));
  
  serialWrite(buf, sizeof(buf));

  NeedCancelScriptRightHand = 1;
  
}

/************************************************

      void CloseRightGripper()

      概要：Seednoid右腕のハンドを閉じる

      引数：なし

      戻り値：なし

*************************************************/
void Seednoid::CloseRightGripper()
{
  noidJointPos[11]=32767;//0x7fff
  uchar buf[8] = { 0 };
  buf[0] = 0xfd;
  buf[1] = 0xdf;
  buf[2] = 0x04;
  buf[3] = 0x22;
  buf[4] = 0x0c;
  buf[5] = 0x00;
  buf[6] = 0x02;
  buf[7] = calcCheckSum(buf, sizeof(buf));
  
  serialWrite(buf, sizeof(buf));

  NeedCancelScriptRightHand = 1;
  
}

/************************************************

      void MoveRightGripper()

      概要：Seednoid右腕のハンドを閉じる

      引数：なし

      戻り値：なし

*************************************************/
void Seednoid::MoveRightGripper(int angleRatio)
{

  if(NeedCancelScriptRightHand)
    {
      cancelScriptRightHand();
      sleep(1);
      NeedCancelScriptRightHand = 0;
    }
  double calc = (HAND_LIMITMIN*(100-angleRatio))/100;
  noidJointPos[11]=(int)(calc);
  SeedAction(3000); 
}

/************************************************

      void OpenLeftGripper()

      概要：Seednoid右腕のハンドを開く

      引数：なし

      戻り値：なし

*************************************************/
void Seednoid::OpenLeftGripper()
{
  noidJointPos[26]=32767;//0x7fff
  uchar buf[8] = { 0 };
  buf[0] = 0xfd;
  buf[1] = 0xdf;
  buf[2] = 0x04;
  buf[3] = 0x22;
  buf[4] = 0x1b;
  buf[5] = 0x00;
  buf[6] = 0x03;
  buf[7] = calcCheckSum(buf, sizeof(buf));
  
  serialWrite(buf, sizeof(buf));

  NeedCancelScriptLeftHand = 1;
}

/************************************************

      void CloseLeftGripper()

      概要：Seednoid右腕のハンドを閉じる

      引数：なし

      戻り値：なし

*************************************************/
void Seednoid::CloseLeftGripper()
{
  noidJointPos[26]=32767;//0x7fff
  uchar buf[8] = { 0 };
  buf[0] = 0xfd;
  buf[1] = 0xdf;
  buf[2] = 0x04;
  buf[3] = 0x22;
  buf[4] = 0x1b;
  buf[5] = 0x00;
  buf[6] = 0x02;
  buf[7] = calcCheckSum(buf, sizeof(buf));
  
  serialWrite(buf, sizeof(buf));

  NeedCancelScriptLeftHand = 1;
}

/************************************************

      void MoveLeftGripper()

      概要：Seednoid左腕のハンドを操作する

      引数：なし

      戻り値：なし

*************************************************/
void Seednoid::MoveLeftGripper(int angleRatio)
{

  if(NeedCancelScriptLeftHand)
    {
      cancelScriptLeftHand();
      sleep(1);
      NeedCancelScriptLeftHand = 0;
    }
  double calc = (HAND_LIMITMIN*(100-angleRatio))/100;
  noidJointPos[26]=(int)(calc);
  SeedAction(3000); 
}



/************************************************

	void TransRot(double eerot[],double roll,double pitch,double yaw)
	
	概要：ロールピッチヨー[deg]から先端姿勢の回転行列を求める

	引数：
　　　　　　　double eerot[]・・・計算した行列を格納する配列
                                 |0 1 2|
                                 |3 4 5|
                                 |6 7 8|
             double roll・・・ロール回転角度
             double pitch・・・ピッチ回転角度
             double yaw・・・ヨー回転角度

	戻り値：なし

*************************************************/
void Seednoid::TransRot(double eerot[],double roll,double pitch,double yaw)
{

  //ロールピッチヨー　ロボット工学（遠山茂樹著）p25
  double Rrotx;
  double Rroty;
  double Rrotz;
  Rrotx = yaw * M_PI / 180;
  Rroty = pitch * M_PI / 180;// -M_PI/2;
  Rrotz = roll * M_PI / 180;
  eerot[0] = cos(Rroty)*cos(Rrotz);
  eerot[1] = cos(Rrotz)*sin(Rroty)*sin(Rrotx)-sin(Rrotz)*cos(Rrotx);
  eerot[2] = cos(Rrotz)*sin(Rroty)*cos(Rrotx)+sin(Rrotz)*sin(Rrotx);
  eerot[3] = sin(Rrotz)*cos(Rroty);
  eerot[4] = sin(Rrotz)*sin(Rroty)*sin(Rrotx)+cos(Rrotz)*cos(Rrotx);
  eerot[5] = sin(Rrotz)*sin(Rroty)*cos(Rrotx)-cos(Rrotz)*sin(Rrotx);
  eerot[6] = -sin(Rroty);
  eerot[7] = cos(Rroty)*sin(Rrotx);
  eerot[8] = cos(Rroty)*cos(Rrotx);
  std::cout << "Rrotx = " << Rrotx << std::endl;
  std::cout << "Rroty = " << Rroty << std::endl;
  std::cout << "Rrotz = " << Rrotz << std::endl;

  //オイラー角　ロボット工学（遠山茂樹著）p25
  //double z_phi;//φ
  //double y_theta;//θ
  //double z_psi;//ψ
  //z_phi = yaw * M_PI / 180;
  //y_theta = pitch * M_PI / 180;
  //z_psi = roll * M_PI / 180;
  //eerot[0] = cos(z_phi)*cos(y_theta)*cos(z_psi)-sin(z_phi)*sin(z_psi);
  //eerot[1] = -cos(z_psi)*cos(y_theta)*sin(z_psi)-sin(z_phi)*cos(z_psi);
  //eerot[2] = cos(z_phi)*sin(y_theta);
  //eerot[3] = sin(z_phi)*cos(y_theta)*cos(z_psi)+cos(z_phi)*sin(z_psi);
  //eerot[4] = -sin(z_phi)*cos(y_theta)*sin(z_psi)+cos(z_phi)*cos(z_psi);
  //eerot[5] = sin(z_phi)*sin(y_theta);
  //eerot[6] = -sin(y_theta)*cos(z_psi);
  //eerot[7] = sin(y_theta)*sin(z_psi);
  //eerot[8] = cos(y_theta);
  
  //片桐さん
  //double Rrotx;
  //double Rroty;
  //double Rrotz;
  //Rrotx = yaw * M_PI / 180;
  //Rroty = pitch * M_PI / 180 -M_PI/2;
  //Rrotz = roll * M_PI / 180;
  //eerot[0] = cos(Rroty)*cos(Rrotz) - sin(Rrotx)*sin(Rroty)*sin(Rrotz);
  //eerot[1] = -cos(Rrotx)*sin(Rrotz);
  //eerot[2] = sin(Rroty)*cos(Rrotz) + sin(Rrotx)*cos(Rroty)*sin(Rrotz);
  //eerot[3] = cos(Rroty)*sin(Rrotz) + sin(Rrotx)*sin(Rroty)*cos(Rrotz);
  //eerot[4] = cos(Rrotx)*cos(Rrotz);
  //eerot[5] = sin(Rrotz)*sin(Rroty) - sin(Rrotx)*cos(Rroty)*cos(Rrotz);
  //eerot[6] = -cos(Rrotx)*sin(Rroty);
  //eerot[7] = sin(Rrotx);
  //eerot[8] = cos(Rrotx)*cos(Rroty);

  // for(int j=0;j<9;j++){
  //  printf("eerot[%d] = %f\n",j,eerot[j]);
  //}
}

/************************************************

        Frame Solve_Rot(double eerot[]);
	
	概要：先端姿勢の回転行列から、ロールピッチヨー[deg]を求める

	引数：
　　　　　　　double eerot[]・・・姿勢行列を格納する配列
                                 |0 1 2|
                                 |3 4 5|
                                 |6 7 8|

	戻り値：姿勢（ロールピッチヨー）

*************************************************/
Frame Seednoid::Solve_Rot(double eerot[])
{
  Frame rotation;
  double phi;//φ
  double theta;//θ
  double psi;//ψ

  for(int i=0;i<9;i++)
    {
      if(eerot[i]>1)
	eerot[i] = 1;
      if(eerot[i]<-1)
	eerot[i] = -1;
    }

  phi = atan2(eerot[3], eerot[0]);
  theta =  atan2(-eerot[6], sqrt(eerot[7]*eerot[7]+eerot[8]*eerot[8]));
  psi = atan2(eerot[7], eerot[8]);  

  //phi = atan2(eerot[3], eerot[0]);
  //theta =  asin(-eerot[6]);
  //psi = atan2(eerot[7], eerot[8]);  

  rotation.roll = phi*180/M_PI;
  rotation.pitch = theta*180/M_PI;
  rotation.yaw = psi*180/M_PI;

  return rotation;
}


/************************************************

	int Solve_RightArmIk(double eerot[9],double eetrans[3], double iksol[])
	
	概要：Seednoid右腕の逆運動学を計算する

	引数：
             double eerot[9]・・・先端姿勢の行列
             double eetrans[3]・・・先端座標(x,y,z)
             double iksol[]・・・計算角度を格納する配列

	戻り値：IKが計算できれば0，計算できなければ-1

*************************************************/
int Seednoid::Solve_RightArmIk(double eerot[9],double eetrans[3], double iksol[])
{
  int ret;
  std::vector<IkReal> free_joint;
  double chest = 1.00;
  int NumSolutions = 0;
  int phase = 0;
  std::vector<int> LimitSolNum;
  std::vector<std::vector<double> > sol;
  int l = 0;//可動範囲内の解の数
  //double Average[l];
  std::vector<double> min;
  LimitSolNum.push_back(-1);
  free_joint.push_back(chest*M_PI/180.0);
  free_joint[0] = 0;
  //free_joint[1] = 0;

  //================================================================================
  //FreeJoint0~(-90)で解を探す
    for(int j=0;j>Angle3_LimitMin-1;j--){
      ret = right_ik_solve(eerot,eetrans,sol,free_joint,&NumSolutions);
      if(!ret){
	//printf("IK計算完了0~Angle3_LimitMin-1\n");
	for(int k=0;k<NumSolutions;k++){
	  if(sol[k][0]*180/M_PI>Angle1_LimitMax||sol[k][0]*180/M_PI<Angle1_LimitMin||
	     sol[k][1]*180/M_PI>Angle2_LimitMax||sol[k][1]*180/M_PI<Angle2_LimitMin||
	     sol[k][2]*180/M_PI>Angle3_LimitMax||sol[k][2]*180/M_PI<Angle3_LimitMin||
	     sol[k][3]*180/M_PI>Angle4_LimitMax||sol[k][3]*180/M_PI<Angle4_LimitMin||
	     sol[k][4]*180/M_PI>Angle5_LimitMax||sol[k][4]*180/M_PI<Angle5_LimitMin||
	     sol[k][5]*180/M_PI>Angle6_LimitMax||sol[k][5]*180/M_PI<Angle6_LimitMin||
	     sol[k][6]*180/M_PI>Angle7_LimitMax||sol[k][6]*180/M_PI<Angle7_LimitMin){
	    //std::cout << "Seednoid補正なしの角度" <<std::endl; 
	    //std::cout << sol[k][0] <<std::endl;
	    //std::cout << -sol[k][1]+M_PI/2 <<std::endl;
	    //std::cout << sol[k][2] <<std::endl;
	    //std::cout << -sol[k][3] <<std::endl;
	    //std::cout << sol[k][4] <<std::endl;
	    //std::cout << sol[k][5] <<std::endl;
	    //std::cout << "free = "<<sol[k][6] <<std::endl;
	    //std::cout << sol[k][0]*180/M_PI <<std::endl;
	    //std::cout << sol[k][1]*180/M_PI <<std::endl;
	    //std::cout << sol[k][2]*180/M_PI <<std::endl;
	    //std::cout << sol[k][3]*180/M_PI <<std::endl;
	    //std::cout << sol[k][4]*180/M_PI <<std::endl;
	    //std::cout << sol[k][5]*180/M_PI <<std::endl;
	    //std::cout << "free = "<<sol[k][6]*180/M_PI <<std::endl;
	    //printf("可動範囲外のため続行\n");
	    continue;//可動範囲外
	  }//end if
	  else{
	    LimitSolNum[l] = k;
	    //std::cout << "可動範囲内の解はsol"<<LimitSolNum[l]<< std::endl;
	    //std::cout << sol[k][0]*180/M_PI <<std::endl;
	    //std::cout << sol[k][1]*180/M_PI <<std::endl;
	    //std::cout << sol[k][2]*180/M_PI <<std::endl;
	    //std::cout << sol[k][3]*180/M_PI <<std::endl;
	    //std::cout << sol[k][4]*180/M_PI <<std::endl;
	    //std::cout << sol[k][5]*180/M_PI <<std::endl;
	    //std::cout << "free = "<<sol[k][6]*180/M_PI <<std::endl;
	    l++;
	  }//end else
	}//end for(int k=0;k<NumSolutions;k++)
	if(l==0){//可動範囲内の解がない
	  free_joint[0] = (j-1)*(M_PI/180);
	  if(j==Angle3_LimitMin){//freeが0~-90は最後まで可動範囲外
	    ret = -1;
	  }
	  continue;
	}
	if(l!=0){//可動範囲内の解があった
	  phase = 2;
	  //std::cout << "可動範囲内の解の数は"<<l<<"個"<<std::endl;
	  break;
	}
      }//end if(!ret)
      else{
	//printf("j = %d\n",j);
	free_joint[0] = j*M_PI/180;
	continue; 
      }    
    }//end for(int j=0;j>Angle3_LimitMin-1;j--)

  //=============================================================================
  //FreeJoint0~Angle3_LimitMaxで解を探す
    if(ret)//FreeJoint0~-90で解が見つかってない
      {
	for(int i=0;i<Angle3_LimitMax+1;i++){
	  ret = right_ik_solve(eerot,eetrans,sol,free_joint,&NumSolutions);
	  if(!ret){
	    //printf("IK計算完了0~Angle3_LimitMax\n");
	    
	    for(int k=0;k<NumSolutions;k++){
	      if(sol[k][0]*180/M_PI>Angle1_LimitMax||sol[k][0]*180/M_PI<Angle1_LimitMin||
		 sol[k][1]*180/M_PI>Angle2_LimitMax||sol[k][1]*180/M_PI<Angle2_LimitMin||
		 sol[k][2]*180/M_PI>Angle3_LimitMax||sol[k][2]*180/M_PI<Angle3_LimitMin||
		 sol[k][3]*180/M_PI>Angle4_LimitMax||sol[k][3]*180/M_PI<Angle4_LimitMin||
		 sol[k][4]*180/M_PI>Angle5_LimitMax||sol[k][4]*180/M_PI<Angle5_LimitMin||
		 sol[k][5]*180/M_PI>Angle6_LimitMax||sol[k][5]*180/M_PI<Angle6_LimitMin||
		 sol[k][6]*180/M_PI>Angle7_LimitMax||sol[k][6]*180/M_PI<Angle7_LimitMin){
		//std::cout << "Seednoid補正なしの角度" <<std::endl; 
		//std::cout << sol[k][0] <<std::endl;
		//std::cout << -sol[k][1]+M_PI/2 <<std::endl;
		//std::cout << sol[k][2] <<std::endl;
		//std::cout << -sol[k][3] <<std::endl;
		//std::cout << sol[k][4] <<std::endl;
		//std::cout << sol[k][5] <<std::endl;
		//std::cout << "free = "<<sol[k][6] <<std::endl;
		
		//std::cout << sol[k][0]*180/M_PI <<std::endl;
		//std::cout << sol[k][1]*180/M_PI <<std::endl;
		//std::cout << sol[k][2]*180/M_PI <<std::endl;
		//std::cout << sol[k][3]*180/M_PI <<std::endl;
		//std::cout << sol[k][4]*180/M_PI <<std::endl;
		//std::cout << sol[k][5]*180/M_PI <<std::endl;
		//std::cout << "free = "<<sol[k][6]*180/M_PI <<std::endl;
		continue;//可動範囲外
	      }//end if
	      else{
		LimitSolNum[l] = k;
		//std::cout << "可動範囲内の解はsol"<<LimitSolNum[l]<< std::endl;
		//std::cout << sol[k][0]*180/M_PI <<std::endl;
		//std::cout << sol[k][1]*180/M_PI <<std::endl;
		//std::cout << sol[k][2]*180/M_PI <<std::endl;
		//std::cout << sol[k][3]*180/M_PI <<std::endl;
		//std::cout << sol[k][4]*180/M_PI <<std::endl;
		//std::cout << sol[k][5]*180/M_PI <<std::endl;
		//std::cout << "free = "<<sol[k][6]*180/M_PI <<std::endl;
		l++;
	      }//end else
	    }//end for(int k=0;k<NumSolutions;k++)
	    if(l==0){//可動範囲内の解がない
	      free_joint[0] = (i+1)*(M_PI/180);
	      if(i==Angle3_LimitMax){//freeが0~Angle3_LimitMaxは最後まで可動範囲外
		ret = -1;
	      }
	      continue;
	    }
	    if(l!=0){//可動範囲内の解があった
	      phase = 2;
	      //std::cout << "可動範囲内の解の数は"<<l<<"個"<<std::endl;
	      break;
	    }
	    
	  }
	  else{//if(!ret)
	    //printf("i = %d\n",i);
	    free_joint[0] = i*M_PI/180;
	    continue; 
	  } 
	}//end  for(int i=0;i<Angle3_LimitMax+1;i++)
      }//end if(ret)
    //=======================================================================================
    
    
    //現在値から近い解を選択する
    int MinAveNum=0;
    int MinAve;
    if(phase == 2){
      //std::vector<std::vector<double> > diffJointPos;
      double diffJointPos[7][7];
      double nowJointPos[7];
      
      getRightJointAngle(nowJointPos);
      //std::cout <<"diffJointPos宣言完了"<<std::endl;
      //diffJointPos.push_back(100);
      //diffJointPos.push_back(100);
      for(int n=0;n<l;n++){
	//std::cout <<"diffJointPos宣言完了1"<<std::endl;
	for(int a=0;a<ARM_FREEDOM;a++){
	  //std::cout <<"diffJointPos宣言完了2"<<std::endl;
	  diffJointPos[n][a] = (nowJointPos[a]*M_PI/180) - sol[LimitSolNum[n]][a];
	  //std::cout << "nowpos["<<a<<"] = "<<nowJointPos[a]<<std::endl;
	  //std::cout <<"diffJointPos宣言完了3"<<std::endl;
	}//end for(int a=0;a<ARM_FREEDOM;a++)
      }//end for(int n=0;n<l;n++)
    double Average[l];
    for(int av=0;av<l;av++){
      Average[av] = (diffJointPos[av][0] + diffJointPos[av][1] +diffJointPos[av][2] +diffJointPos[av][3] +diffJointPos[av][4] +diffJointPos[av][5] +diffJointPos[av][6])/ARM_FREEDOM;//ARM_FREEDOM=7
      //Average[av] = fabs(Average[av]);
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
      for(int end=0;end<ARM_FREEDOM;end++){
	iksol[end] = sol[LimitSolNum[MinAveNum]][end]*180/M_PI;
	if(end==1){
	  //std::cout << -sol[LimitSolNum[MinAveNum]][end]+M_PI/2 << std::endl;
	}
	else if(end==3){
	  //std::cout << sol[LimitSolNum[MinAveNum]][end]-M_PI/2 << std::endl;
	}
	else{
	  //std::cout << sol[LimitSolNum[MinAveNum]][end]<<std::endl;
	  //std::cout << LimitSolNum[MinAveNum] <<std::endl;
	}
      }
      std::cout << "IK計算完了" << std::endl;
      return 0;
    }
    else{
      std::cout << "与えられた位置姿勢ではIKが解けません" << std::endl;
      return -1;
    }  
}


/************************************************

	int Solve_LeftArmIk(double eerot[9],double eetrans[3], double iksol[])
	
	概要：Seednoid左腕の逆運動学を計算する

	引数：
             double eerot[9]・・・先端姿勢の行列
             double eetrans[3]・・・先端座標(x,y,z)
             double iksol[]・・・計算角度を格納する配列

	戻り値：IKが計算できれば0，計算できなければ-1

*************************************************/
int Seednoid::Solve_LeftArmIk(double eerot[9],double eetrans[3], double iksol[])
{
  int ret;
  std::vector<IkReal> free_joint;
  double chest = 1.00;
  int NumSolutions = 0;
  int phase = 0;
  std::vector<int> LimitSolNum;
  std::vector<std::vector<double> > sol;
  int l = 0;//可動範囲内の解の数
  //double Average[l];
  std::vector<double> min;
  LimitSolNum.push_back(-1);
  free_joint.push_back(chest*M_PI/180.0);
  free_joint[0] = 0;
  //free_joint[1] = 0;

  //=====================================================================================
  //FreeJoint0~(-90)で解を探す
    for(int j=0;j>Angle3_LimitMin-1;j--){
      ret = left_ik_solve(eerot,eetrans,sol,free_joint,&NumSolutions);
      if(!ret){
	//printf("IK計算完了0~Angle3_LimitMin\n");
	for(int k=0;k<NumSolutions;k++){
	  if(sol[k][0]*180/M_PI>Angle1_LimitMax||sol[k][0]*180/M_PI<Angle1_LimitMin||
	     sol[k][1]*180/M_PI>Angle2_LimitMax||sol[k][1]*180/M_PI<Angle2_LimitMin||
	     sol[k][2]*180/M_PI>Angle3_LimitMax||sol[k][2]*180/M_PI<Angle3_LimitMin||
	     sol[k][3]*180/M_PI>Angle4_LimitMax||sol[k][3]*180/M_PI<Angle4_LimitMin||
	     sol[k][4]*180/M_PI>Angle5_LimitMax||sol[k][4]*180/M_PI<Angle5_LimitMin||
	     sol[k][5]*180/M_PI>Angle6_LimitMax||sol[k][5]*180/M_PI<Angle6_LimitMin||
	     sol[k][6]*180/M_PI>Angle7_LimitMax||sol[k][6]*180/M_PI<Angle7_LimitMin){
	    //std::cout << sol[k][0]*180/M_PI <<std::endl;
	    //std::cout << sol[k][1]*180/M_PI <<std::endl;
	    //std::cout << sol[k][2]*180/M_PI <<std::endl;
	    //std::cout << sol[k][3]*180/M_PI <<std::endl;
	    //std::cout << sol[k][4]*180/M_PI <<std::endl;
	    //std::cout << sol[k][5]*180/M_PI <<std::endl;
	    //std::cout << "free = "<<sol[k][6]*180/M_PI <<std::endl;
	    //printf("可動範囲外のため続行\n");
	    continue;//可動範囲外
	  }//end if
	  else{
	    LimitSolNum[l] = k;
	    //std::cout << "可動範囲内の解はsol"<<LimitSolNum[l]<< std::endl;
	    //std::cout << sol[k][0]*180/M_PI <<std::endl;
	    //std::cout << sol[k][1]*180/M_PI <<std::endl;
	    //std::cout << sol[k][2]*180/M_PI <<std::endl;
	    //std::cout << sol[k][3]*180/M_PI <<std::endl;
	    //std::cout << sol[k][4]*180/M_PI <<std::endl;
	    //std::cout << sol[k][5]*180/M_PI <<std::endl;
	    //std::cout << "free = "<<sol[k][6]*180/M_PI <<std::endl;
	    l++;
	  }//end else
	}//end for(int k=0;k<NumSolutions;k++)
	if(l==0){//可動範囲内の解がない
	  free_joint[0] = (j-1)*(M_PI/180);
	  if(j==Angle3_LimitMin){//freeが0~Angle3_LimitMinは最後まで可動範囲外
	    ret = -1;
	  }
	  continue;
	}
	if(l!=0){//可動範囲内の解があった
	  phase = 2;
	  //std::cout << "可動範囲内の解の数は"<<l<<"個"<<std::endl;
	  break;
	}
      }//end if(!ret)
      else{
	//printf("j = %d\n",j);
	free_joint[0] = j*M_PI/180;
	continue; 
      }    
    }//end for(int j=0;j>Angle3_LimitMin-1;j--)

  //===============================================================================
  //FreeJoint0~90で解を探す
    if(ret)
      {
	for(int i=0;i<Angle3_LimitMax+1;i++){
	  ret = left_ik_solve(eerot,eetrans,sol,free_joint,&NumSolutions);
	  if(!ret){
	    //printf("IK計算完了0~Angle3_LimitMax\n");
	    
	    for(int k=0;k<NumSolutions;k++){
	      if(sol[k][0]*180/M_PI>Angle1_LimitMax||sol[k][0]*180/M_PI<Angle1_LimitMin||
		 sol[k][1]*180/M_PI>Angle2_LimitMax||sol[k][1]*180/M_PI<Angle2_LimitMin||
		 sol[k][2]*180/M_PI>Angle3_LimitMax||sol[k][2]*180/M_PI<Angle3_LimitMin||
		 sol[k][3]*180/M_PI>Angle4_LimitMax||sol[k][3]*180/M_PI<Angle4_LimitMin||
		 sol[k][4]*180/M_PI>Angle5_LimitMax||sol[k][4]*180/M_PI<Angle5_LimitMin||
		 sol[k][5]*180/M_PI>Angle6_LimitMax||sol[k][5]*180/M_PI<Angle6_LimitMin||
		 sol[k][6]*180/M_PI>Angle7_LimitMax||sol[k][6]*180/M_PI<Angle7_LimitMin){
		//std::cout << "Seednoid補正なしの角度" <<std::endl; 
		//std::cout << sol[k][0] <<std::endl;
		//std::cout << -sol[k][1]+M_PI/2 <<std::endl;
		//std::cout << sol[k][2] <<std::endl;
		//std::cout << -sol[k][3] <<std::endl;
		//std::cout << sol[k][4] <<std::endl;
		//std::cout << sol[k][5] <<std::endl;
		//std::cout << "free = "<<sol[k][6] <<std::endl;
		
		//std::cout << sol[k][0]*180/M_PI <<std::endl;
		//std::cout << sol[k][1]*180/M_PI <<std::endl;
		//std::cout << sol[k][2]*180/M_PI <<std::endl;
		//std::cout << sol[k][3]*180/M_PI <<std::endl;
		//std::cout << sol[k][4]*180/M_PI <<std::endl;
		//std::cout << sol[k][5]*180/M_PI <<std::endl;
		//std::cout << "free = "<<sol[k][6]*180/M_PI <<std::endl;
		continue;//可動範囲外
	      }//end if
	      else{
		LimitSolNum[l] = k;
		//std::cout << "可動範囲内の解はsol"<<LimitSolNum[l]<< std::endl;
		//std::cout << sol[k][0]*180/M_PI <<std::endl;
		//std::cout << sol[k][1]*180/M_PI <<std::endl;
		//std::cout << sol[k][2]*180/M_PI <<std::endl;
		//std::cout << sol[k][3]*180/M_PI <<std::endl;
		//std::cout << sol[k][4]*180/M_PI <<std::endl;
		//std::cout << sol[k][5]*180/M_PI <<std::endl;
		//std::cout << "free = "<<sol[k][6]*180/M_PI <<std::endl;
		l++;
	      }//end else
	    }//end for(int k=0;k<NumSolutions;k++)
	    if(l==0){//可動範囲内の解がない
	      free_joint[0] = (i+1)*(M_PI/180);
	      if(i==Angle3_LimitMax){//freeが0~-90は最後まで可動範囲外
		ret = -1;
	      }
	      continue;
	    }
	    if(l!=0){//可動範囲内の解があった
	      phase = 2;
	      //std::cout << "可動範囲内の解の数は"<<l<<"個"<<std::endl;
	      break;
	    }
	    
	  }
	  else{//if(!ret)
	    //printf("i = %d\n",i);
	    free_joint[0] = i*M_PI/180;
	    continue; 
	  } 
	}//end  for(int i=0;i<Angle3_LimitMax+1;i++)
      }//end if(ret)
    
    
    //現在値から近い解を選択する
    int MinAveNum=0;
    int MinAve;
    if(phase == 2){
      //std::vector<std::vector<double> > diffJointPos;
      double diffJointPos[7][7];
      double nowJointPos[7];
      
      getLeftJointAngle(nowJointPos);
      //std::cout <<"diffJointPos宣言完了"<<std::endl;
      //diffJointPos.push_back(100);
      //diffJointPos.push_back(100);
      for(int n=0;n<l;n++){
	//std::cout <<"diffJointPos宣言完了1"<<std::endl;
	for(int a=0;a<ARM_FREEDOM;a++){
	  //std::cout <<"diffJointPos宣言完了2"<<std::endl;
	  diffJointPos[n][a] = (nowJointPos[a]*M_PI/180) - sol[LimitSolNum[n]][a];
	  //std::cout << "nowpos[a] = "<<nowJointPos[a]<<std::endl;
	  //std::cout <<"diffJointPos宣言完了3"<<std::endl;
	}//end for(int a=0;a<ARM_FREEDOM;a++)
      }//end for(int n=0;n<l;n++)
      double Average[l];
      for(int av=0;av<l;av++){
	Average[av] = (diffJointPos[av][0] + diffJointPos[av][1] +diffJointPos[av][2] +diffJointPos[av][3] +diffJointPos[av][4] +diffJointPos[av][5] +diffJointPos[av][6])/ARM_FREEDOM;//ARM_FREEDOM=7
	//Average[av] = fabs(Average[av]);
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
      for(int end=0;end<ARM_FREEDOM;end++){
	iksol[end] = sol[LimitSolNum[MinAveNum]][end]*180/M_PI;
	if(end==1){
	  //std::cout << -sol[LimitSolNum[MinAveNum]][end]+M_PI/2 << std::endl;
	}
	else if(end==3){
	  //std::cout << sol[LimitSolNum[MinAveNum]][end]-M_PI/2 << std::endl;
	}
	else{
	  //std::cout << sol[LimitSolNum[MinAveNum]][end]<<std::endl;
	  //std::cout << LimitSolNum[MinAveNum] <<std::endl;
	}
      }
      std::cout <<"IK計算完了" <<std::endl;
      return 0;
    }
    else{
      std::cout <<"与えられた位置姿勢ではIKが解けません" <<std::endl;
      return -1;
    }  
}


/************************************************

        int Solve_RightArmFK(double eerot[9],double eetrans[3],double joint[])

	概要：Seednoid右腕の順運動学を計算する

	引数：
           　double eerot[]・・・計算した行列を格納する配列
                                 |0 1 2|
                                 |3 4 5|
                                 |6 7 8|
             double eetrans[3]・・・先端座標(x,y,z)が格納される
             double joint[]・・・入力関節角度を格納する配列

	戻り値：FKが計算できれば0，計算できなければ-1

*************************************************/
int Seednoid::Solve_RightArmFK(double eerot[9],double eetrans[3],double joint[])
{
  double ikjoint[7];
  ikjoint[0] = joint[0]*M_PI/180;
  ikjoint[1] = -joint[1]*M_PI/180+M_PI/2;
  ikjoint[2] = joint[2]*M_PI/180;
  ikjoint[3] = -joint[3]*M_PI/180;
  ikjoint[4] = joint[4]*M_PI/180;
  ikjoint[5] = joint[5]*M_PI/180;
  ikjoint[6] = -joint[6]*M_PI/180; 

  ComputeFk(ikjoint,eetrans,eerot);
  //for(int j=0;j<7;j++)
  //  {
  //    std::cout << "right_ik_joint["<<j<<"] = "<<ikjoint[j]<<std::endl;
  //  }
  //for(int i=0;i<9;i++)
  //  {
  //    std::cout << "eerot["<<i<<"] = "<<eerot[i]<<std::endl;
  //  }
  //for(int k=0;k<3;k++)
  //  {
  //  std::cout << "eetrans["<<k<<"] = "<<eetrans[k]<<std::endl;
  //}
  
  return 0;
}


/************************************************

        int Solve_LeftArmFK(double eerot[9],double eetrans[3],double joint[])

	概要：Seednoid左腕の順運動学を計算する

	引数：
           　double eerot[]・・・計算した行列を格納する配列
                                 |0 1 2|
                                 |3 4 5|
                                 |6 7 8|
             double eetrans[3]・・・先端座標(x,y,z)が格納される
             double joint[]・・・入力関節角度を格納する配列列

	戻り値：FKが計算できれば0，計算できなければ-1

*************************************************/
int Seednoid::Solve_LeftArmFK(double eerot[9],double eetrans[3],double joint[])
{

  double ikjoint[7];
  ikjoint[0] = joint[0]*M_PI/180;
  ikjoint[1] = -joint[1]*M_PI/180+M_PI/2;
  ikjoint[2] = -joint[2]*M_PI/180;
  ikjoint[3] = -joint[3]*M_PI/180;
  ikjoint[4] = -joint[4]*M_PI/180;
  ikjoint[5] = joint[5]*M_PI/180;
  ikjoint[6] = -joint[6]*M_PI/180; 

  ComputeFk(ikjoint,eetrans,eerot);
  //for(int j=0;j<7;j++)
  //  {
  //    std::cout << "left_ik_joint["<<j<<"] = "<<ikjoint[j]<<std::endl;
  //  }
  //for(int i=0;i<9;i++)
  //  {
  //    std::cout << "eerot["<<i<<"] = "<<eerot[i]<<std::endl;
  //  }
  //for(int k=0;k<3;k++)
  //  {
  //    std::cout << "eetrans["<<k<<"] = "<<eetrans[k]<<std::endl;
  //  }

  return 0;
}


/************************************************

        void setRightHandCurrent(int CurrentRate)

	概要：ハンドの最大電流を設定する

	引数：
              int CurrentRate・・・最大電流0~100[%] 100%で1.5A
	
	戻り値：なし

*************************************************/
void Seednoid::setRightHandCurrent(int CurrentRate)
{
  if(0>CurrentRate||CurrentRate>100){
    std::cout << "Error CurrentRate Wrong Value" <<std::endl;
  }
  else{
  uchar buf[8] = {0};
  buf[0] = 0xfd;
  buf[1] = 0xdf;
  buf[2] = 0x04;
  buf[3] = 0x01;
  buf[4] = 0x0c;
  buf[5] = CurrentRate & 0xff;
  buf[6] = CurrentRate & 0xff;
  buf[7] = calcCheckSum(buf, sizeof(buf));
  
  serialWrite(buf, sizeof(buf)); 
  }
}

/************************************************

        void setLeftHandCurrent(int CurrentRate)

	概要：ハンドの最大電流を設定する

	引数：
              int CurrentRate・・・最大電流0~100[%] 100%で1.5A
	
	戻り値：なし

*************************************************/
void Seednoid::setLeftHandCurrent(int CurrentRate)
{
  if(0>CurrentRate||CurrentRate>100){
    std::cout << "Error CurrentRate Wrong Value" <<std::endl;
  }
  else{
  uchar buf[8] = {0};
  buf[0] = 0xfd;
  buf[1] = 0xdf;
  buf[2] = 0x04;
  buf[3] = 0x01;
  buf[4] = 0x1b;
  buf[5] = CurrentRate & 0xff;
  buf[6] = CurrentRate & 0xff;
  buf[7] = calcCheckSum(buf, sizeof(buf));
  
  serialWrite(buf, sizeof(buf)); 
  }
}

/************************************************

        void cancelScriptRightHand()

	概要：スクリプトをキャンセルする

	引数：なし
	
	戻り値：なし

*************************************************/
void Seednoid::cancelScriptRightHand()
{
  uchar buf[8] = { 0 };
  buf[0] = 0xfd;
  buf[1] = 0xdf;
  buf[2] = 0x04;
  buf[3] = 0x22;
  buf[4] = 0x0c;
  buf[5] = 0x00;
  buf[6] = 0x04;
  buf[7] = calcCheckSum(buf, sizeof(buf));
  std::cout << "cancelScript"<< std::endl;
  serialWrite(buf, sizeof(buf));
}


/************************************************

        void cancelScriptLeftHand()

	概要：スクリプトをキャンセルする

	引数：なし
	
	戻り値：なし

*************************************************/
void Seednoid::cancelScriptLeftHand()
{
  uchar buf[8] = { 0 };
  buf[0] = 0xfd;
  buf[1] = 0xdf;
  buf[2] = 0x04;
  buf[3] = 0x22;
  buf[4] = 0x1b;
  buf[5] = 0x00;
  buf[6] = 0x04;
  buf[7] = calcCheckSum(buf, sizeof(buf));
  std::cout << "cancelScript" << std::endl;
  serialWrite(buf, sizeof(buf));
}
