// SeekFiber.cpp : Defines the class behaviors for the application.
//
#include <Eigen/Dense>
//using Eigen::MatrixXd;
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

#include "stdafx.h"
#include "LSeekFiber.h"

#include <iostream>
#include "math.h"
#include <string>
#include <fstream>
#include <vector> 
#include <set>
#include <algorithm>
#include<numeric>


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


#ifndef FFLILUNPATH		
#define FFLILUNPATH "F:\\Paohe\\bin\\FFLilun.txt"	 //參考光纖理論位置
#endif	
#ifndef FFCIRCLEPATH		
#define FFCIRCLEPATH "F:\\Paohe\\bin\\FFCircle.txt" //FFCircle file path	 //參考光纖像素位置
#endif	
#ifndef FFOFFSETPATH		
#define FFOFFSETPATH "F:\\Paohe\\bin\\config_offset.ini" //config_offset file path	 //光纤偏移量
#endif	

/////////////////////////////////////////////////////////////////////////////
// LSeekFiber construction

LSeekFiber::LSeekFiber()
{
	intUnitNum=0;
	intSolidNum=0;
	intLightNum=0;//实际找到的点数
	intMemAlloc=0;//从配置文件读的点数，用于分配内存
	intUnitTotal=0;
	intRepeatTimes=0;

	for(int i=0;i<5;i++)
	{
		dblCenterX[i]=0;
		dblCenterY[i]=0;
		dblXiangX[i]=0;
		dblXiangY[i]=0;
		dblXiangA[i]=0;
		dblBiliX[i]=0;
		dblBiliY[i]=0;
	}
	intImageWidth=0;
	intImageHeight=0;
	intWndSize=0;
	intBKThreshold=0;
	intLightThreshold=0;
}

LSeekFiber::~LSeekFiber()
{
}

void LSeekFiber::Close()
{
	MemRelease();
}

/////////////////////////////////////////////////////////////////////////////
// LSeekFiber initialization

BOOL LSeekFiber::Init()
{
	if (!GetConfig())
	{
		AfxMessageBox(_T("Failed to read config file."));
		return FALSE;
	}
	if (!MemAlloc())
	{
		AfxMessageBox(_T("Failed to allocate memory."));
		return FALSE;
	}
	//if (!Py_IsInitialized())
	//{
	//  AfxMessageBox(_T("Failed to Py_IsInitialized."));
	//	return FALSE;
	//}
	if (!GetFFLilun())
	{
		AfxMessageBox(_T("Failed to read FFLilun file."));
		return FALSE;
	}
	if (!GetConfigOffset())
	{
		AfxMessageBox(_T("Failed to read FFoffsetFile file."));
		return FALSE;
	}
	if (!GetFFCircle())
	{
		AfxMessageBox(_T("Failed to read FFCircle file."));
		return FALSE;
	}

	
	
	return TRUE;
}
BOOL LSeekFiber::GetFFCircle() //放入相機初始化中
{
	FILE* FFCFile;
	if ((FFCFile = fopen(FFCIRCLEPATH, "rt")) <= 0)//open file
	{
		//Sleep(1500);
		//printf("FF文件无法打开！");
		return false;
	}
	char unuse[100];
	intQNum = 0;
	//fscanf(FFCFile, "%lf", &FFNum);
	while (!feof(FFCFile))
	{
		fscanf(FFCFile, "%lf %lf", &FFCircleX[intQNum], &FFCircleY[intQNum]);//读取Q的pixcel坐标
		FFCircleX[intQNum] += offSetX;
		FFCircleY[intQNum] += offSetY;
		//fgets(unuse,1000, FFCFile);
		intQNum++;
	}
	fclose(FFCFile);
	return true;
}

BOOL LSeekFiber::GetFFLilun() //放入相機初始化中
{
	

	//char* FFLiLunPath;
	//FFLiLunPath = "F:\\Paohe\\bin\\FFLilun.txt";//FFCircle file path
	FILE* FFLiLunFile;
	if ((FFLiLunFile = fopen(FFLILUNPATH, "rt")) <= 0)//open file
	{
		return false;
	}
	//fscanf(FFCFile, "%lf", &FFNum);
	char c;
	char buf[100] = { 0 };
	int temp = 0;
	while (!feof(FFLiLunFile))
	{
		fscanf_s(FFLiLunFile, "%lf %lf", &FFLilunX[temp], &FFLilunY[temp]);//读取Q的理论坐标
		temp++;
		//fgets(buf, 100, FFLiLunFile);
		c = fgetc(FFLiLunFile);//防止多读一次

	}
	fclose(FFLiLunFile);

	//double test1;
	//double test2;
	//for (int i = 0; i < numQL; i++)
	//{
	//	test1 = FFLilunX[i];
	//	test2 = FFLilunY[i];
	//}
	return true;
}

BOOL LSeekFiber::GetConfigOffset()
{
	
	FILE* FFoffsetFile;
	if ((FFoffsetFile = fopen(FFOFFSETPATH, "rt")) <= 0)//open file
	{
		return false;
	}

	while (!feof(FFoffsetFile))
	{
		fscanf_s(FFoffsetFile, "%lf %lf", &offSetX, &offSetY);//读取Q的像素偏移
		break;
		//c = fgetc(FFoffsetFile);//防止多读一次

	}
	fclose(FFoffsetFile);
	return true;
}

BOOL LSeekFiber::MemAlloc()
{
	lpRawBuffer = new unsigned short[intImageWidth*intImageHeight];
	if (!lpRawBuffer)return FALSE;
	dblX = new double[intMemAlloc];
	if (!dblX)return FALSE;
	dblY = new double[intMemAlloc];
	if (!dblY)return FALSE;
	dblX1 = new double[intMemAlloc];
	if (!dblX1)return FALSE;
	dblY1 = new double[intMemAlloc];
	if (!dblY1)return FALSE;
	dblCoorX = new double[intMemAlloc];
	if (!dblCoorX)return FALSE;
	dblCoorY = new double[intMemAlloc];
	if (!dblCoorY)return FALSE;
	dblCoorX1 = new double[intMemAlloc];
	if (!dblCoorX1)return FALSE;
	dblCoorY1 = new double[intMemAlloc];
	if (!dblCoorY1)return FALSE;
	dblDist=new double[intMemAlloc*intMemAlloc];
	if (!dblDist)return FALSE;
	dblDist1=new double[intMemAlloc*intMemAlloc];
	if (!dblDist1)return FALSE;
	dblDistM=new int[intMemAlloc*intMemAlloc];
	if (!dblDistM)return FALSE;
	dblDistN=new int[intMemAlloc*intMemAlloc];
	if (!dblDistN)return FALSE;
	bFlag1=new BOOL[intMemAlloc];
	if (!bFlag1)return FALSE;
	bFlag2=new BOOL[intMemAlloc];
	if (!bFlag2)return FALSE;
	intMM = new int[intMemAlloc];
	if (!intMM)return FALSE;
	intMM1 = new int[intMemAlloc];
	if (!intMM1)return FALSE;
	intNum = new int[intMemAlloc];
	if (!intNum)return FALSE;
	intNum1 = new int[intMemAlloc];
	if (!intNum1)return FALSE;

	dblZBCoorX = new double[intMemAlloc];
	if (!dblZBCoorX)return FALSE;
	dblZBCoorY = new double[intMemAlloc];
	if (!dblZBCoorY)return FALSE;
	dblZBCenA = new double[intMemAlloc];
	if (!dblZBCenA)return FALSE;
	dblZBEccA = new double[intMemAlloc];
	if (!dblZBEccA)return FALSE;
	//dsp add python
	pixelPCoorX = new double[intMemAlloc];
	if (!pixelPCoorX)return FALSE;
	pixelPCoorY = new double[intMemAlloc];
	if (!pixelPCoorY)return FALSE;
	pixelPCoorX1 = new double[intMemAlloc];
	if (!pixelPCoorX1)return FALSE;
	pixelPCoorY1 = new double[intMemAlloc];
	if (!pixelPCoorY1)return FALSE;

	micronPCoorX = new double[intMemAlloc];
	if (!micronPCoorX)return FALSE;
	micronPCoorY = new double[intMemAlloc];
	if (!micronPCoorY)return FALSE;

	dblQCoorX = new double[intMemAlloc];
	if (!dblQCoorX)return FALSE;
	dblQCoorY = new double[intMemAlloc];
	if (!dblQCoorY)return FALSE;
	dblQCenA = new double[intMemAlloc];
	if (!dblQCenA)return FALSE;
	dblQEccA = new double[intMemAlloc];
	if (!dblQEccA)return FALSE;
	pixelQCoorX = new double[intMemAlloc];
	if (!pixelQCoorX)return FALSE;
	pixelQCoorY = new double[intMemAlloc];
	if (!pixelQCoorY)return FALSE;
	temp = new double[intMemAlloc];
	if (!temp)return FALSE;

	//FF
	FFCircleX = new double[intMemAlloc];
	if (!FFCircleX)return FALSE;
	FFCircleY = new double[intMemAlloc];
	if (!FFCircleY)return FALSE;
	FFSingeX = new double[intMemAlloc];
	if (!FFSingeX)return FALSE;
	FFSingeY = new double[intMemAlloc];
	if (!FFSingeY)return FALSE;
	centerX = new double[intMemAlloc];
	if (!centerX)return FALSE;
	centerY = new double[intMemAlloc];
	if (!centerY)return FALSE;
	radius = new double[intMemAlloc];
	if (!radius)return FALSE;
	centerUmX = new double[intMemAlloc];
	if (!centerUmX)return FALSE;
	centerUmY = new double[intMemAlloc];
	if (!centerUmY)return FALSE;

	paramX = new double[intMemAlloc];
	if (!paramX)return FALSE;
	paramY = new double[intMemAlloc];
	if (!paramY)return FALSE;

	FFLilunX = new double[intMemAlloc];
	if (!FFLilunX)return FALSE;
	FFLilunY = new double[intMemAlloc];
	if (!FFLilunY)return FALSE;
	micronQCoorX = new double[intMemAlloc];
	if (!micronQCoorX)return FALSE;
	micronQCoorY = new double[intMemAlloc];
	if (!micronQCoorY)return FALSE;

	//Py_SetPythonHome(L"D:\\anaconda");  //系统的python路径
	//Py_Initialize();			//使用python之前，要调用Py_Initialize();这个函数进行初始化 

	//double* pixelPCoorX;					//工作光纤点角度坐标，intPNum个，预存
	//double* pixelPCoorY;					//工作光纤点角度坐标，intPNum个，预存
	//double* dblQCoorX;				//参考光纤理论坐标，intQNum个，预存
	//double* dblQCoorY;				//参考光纤理论坐标，intQNum个，预存
	//double* dblQCenA;					//参考光纤点角度坐标，intQNum个，预存
	//double* dblQEccA;					//参考光纤点角度坐标，intQNum个，预存
	//double* pixelQCoorX;			//参考光纤像素坐标，intQNum个，预存
	//double* pixelQCoorY;			//参考光纤像素坐标，intQNum个，预存
	//vector<string> sCellNameQ;				//CellName，intUnitNum个，预存
	//vector<string> sCellNameP;				//CellName，intUnitNum个，预存

//	sCellName = new char[20*intMemAlloc];
//	if (!sCellName)return FALSE;
	return TRUE;
}

void LSeekFiber::MemRelease()
{
//	if (sCellName)delete [] sCellName;
	if (dblZBCenA)delete [] dblZBCenA;
	if (dblZBEccA)delete [] dblZBEccA;
	if (dblZBCoorX)delete [] dblZBCoorX;
	if (dblZBCoorY)delete [] dblZBCoorY;
	if (intNum1)delete [] intNum1;
	if (intNum)delete [] intNum;
	if (intMM1)delete [] intMM1;
	if (intMM)delete [] intMM;
	if (bFlag1)delete [] bFlag1;
	if (bFlag2)delete [] bFlag2;
	if (dblDist)delete [] dblDist;
	if (dblDist1)delete [] dblDist1;
	if (dblDistM)delete [] dblDistM;
	if (dblDistN)delete [] dblDistN;
	if (dblCoorX1)delete [] dblCoorX1;
	if (dblCoorY1)delete [] dblCoorY1;
	if (dblCoorX)delete [] dblCoorX;
	if (dblCoorY)delete [] dblCoorY;
	if (dblX1)delete [] dblX1;
	if (dblY1)delete [] dblY1;
	if (dblX)delete [] dblX;
	if (dblY)delete [] dblY;
	if (lpRawBuffer)delete [] lpRawBuffer;

	if (pixelPCoorX)delete[] pixelPCoorX;
	if (pixelPCoorY)delete[] pixelPCoorY;
	if (pixelPCoorX1)delete[] pixelPCoorX1;
	if (pixelPCoorY1)delete[] pixelPCoorY1;
	if (micronPCoorX)delete[] micronPCoorX;
	if (micronPCoorY)delete[] micronPCoorY;

	if (dblQCoorX)delete[] dblQCoorX;
	if (dblQCoorY)delete[] dblQCoorY;
	if (dblQCenA)delete[] dblQCenA;
	if (dblQEccA)delete[] dblQEccA;
	if (pixelQCoorX)delete[] pixelQCoorX;
	if (pixelQCoorY)delete[] pixelQCoorY;
	if (temp)delete[] temp;
	
	//Py_Finalize(); //释放资源 (已转移)
	//Py_Finalize(); //释放资源 (已转移)


	//FF
	if (FFCircleX)delete[] FFCircleX;
	if (FFCircleY)delete[] FFCircleY;
	if (FFSingeX)delete[] FFSingeX;
	if (FFSingeY)delete[] FFSingeY;
	if (centerX)delete[] centerX;
	if (centerY)delete[] centerY;
	if (radius)delete[] radius;
	if (centerUmX)delete[] centerUmX;
	if (centerUmY)delete[] centerUmX;
	if (paramX)delete[] paramX;
	if (paramY)delete[] paramY;

	if (FFLilunX)delete[] FFLilunX;
	if (FFLilunY)delete[] FFLilunY;
	if (micronQCoorX)delete[] micronQCoorX;
	if (micronQCoorY)delete[] micronQCoorY;
	
}


BOOL LSeekFiber::GetConfig()
{
	//得到配置文件名
	CString strFilename;
	strFilename=CFG_DIR;
	strFilename+="\\config.ini";

	//读配置
	char lpBuffer[50];
	TCHAR LpBuffer[100];
	GetPrivateProfileString(L"SYSTEM", L"CCD_No", L"0", LpBuffer, 100, strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer,50,NULL,FALSE);
	intCCDNo=atoi(lpBuffer);
	CString strCaliFactorFilename;		//lwm标定文件名
	GetPrivateProfileString(L"SYSTEM",L"cali_factors",L"",LpBuffer,100,strFilename);
	strCaliFactorFilename.Format(LpBuffer,intCCDNo);
	GetPrivateProfileString(L"SYSTEM",L"allunit",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	intAllUnit=atoi(lpBuffer);
	GetPrivateProfileString(L"SYSTEM",L"needfile",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	intNeedFile=atoi(lpBuffer);
	GetPrivateProfileString(L"SYSTEM",L"max_light_point_num",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	intMemAlloc=atoi(lpBuffer);
	GetPrivateProfileString(L"SYSTEM",L"all_unit_num",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	intUnitTotal=atoi(lpBuffer);
	GetPrivateProfileString(L"SYSTEM",L"repeat_times",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	intRepeatTimes=atoi(lpBuffer);
	GetPrivateProfileString(L"SYSTEM",L"center_x0",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	dblCenterX[0]=atof(lpBuffer);
	GetPrivateProfileString(L"SYSTEM",L"center_y0",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	dblCenterY[0]=atof(lpBuffer);
	GetPrivateProfileString(L"SYSTEM",L"xiang_x0",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	dblXiangX[0]=atof(lpBuffer);
	GetPrivateProfileString(L"SYSTEM",L"xiang_y0",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	dblXiangY[0]=atof(lpBuffer);
	GetPrivateProfileString(L"SYSTEM",L"xiang_a0",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	dblXiangA[0]=atof(lpBuffer)*3.1415926/60/180;
	GetPrivateProfileString(L"SYSTEM",L"bili_x0",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	dblBiliX[0]=atof(lpBuffer);
	GetPrivateProfileString(L"SYSTEM",L"bili_y0",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	dblBiliY[0]=atof(lpBuffer);
	GetPrivateProfileString(L"SYSTEM",L"center_x1",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	dblCenterX[1]=atof(lpBuffer);
	GetPrivateProfileString(L"SYSTEM",L"center_y1",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	dblCenterY[1]=atof(lpBuffer);
	GetPrivateProfileString(L"SYSTEM",L"center_a1",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	dblXiangA[1]=atof(lpBuffer)*3.1415926/60/180;
	GetPrivateProfileString(L"SYSTEM",L"center_x2",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	dblCenterX[2]=atof(lpBuffer);
	GetPrivateProfileString(L"SYSTEM",L"center_y2",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	dblCenterY[2]=atof(lpBuffer);
	GetPrivateProfileString(L"SYSTEM",L"center_a2",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	dblXiangA[2]=atof(lpBuffer)*3.1415926/60/180;
	GetPrivateProfileString(L"SYSTEM",L"center_x3",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	dblCenterX[3]=atof(lpBuffer);
	GetPrivateProfileString(L"SYSTEM",L"center_y3",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	dblCenterY[3]=atof(lpBuffer);
	GetPrivateProfileString(L"SYSTEM",L"center_a3",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	dblXiangA[3]=atof(lpBuffer)*3.1415926/60/180;
	GetPrivateProfileString(L"SYSTEM",L"center_x4",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	dblCenterX[4]=atof(lpBuffer);
	GetPrivateProfileString(L"SYSTEM",L"center_y4",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	dblCenterY[4]=atof(lpBuffer);
	GetPrivateProfileString(L"SYSTEM",L"center_a4",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	dblXiangA[4]=atof(lpBuffer)*3.1415926/60/180;
	GetPrivateProfileString(L"CALC",L"image_pixel_w",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	intImageWidth=atoi(lpBuffer);
	GetPrivateProfileString(L"CALC",L"image_pixel_h",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	intImageHeight=atoi(lpBuffer);
	GetPrivateProfileString(L"CALC",L"window_size",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	intWndSize=atoi(lpBuffer);
	GetPrivateProfileString(L"CALC",L"image_background_threshold_value",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	intBKThreshold=atoi(lpBuffer);
	GetPrivateProfileString(L"CALC",L"light_threshold",L"",LpBuffer,100,strFilename);
	WideCharToMultiByte(CP_OEMCP, 0, LpBuffer, -1, lpBuffer, 50, NULL, FALSE);
	intLightThreshold=atoi(lpBuffer);

	//读标定系数
	CFile file;
	CFileException e;
	char chrBuffer[1024];
	if(!file.Open(strCaliFactorFilename,CFile::modeRead,&e))
	{
		AfxMessageBox(_T("Failed to read calibration factor file."));
		return FALSE;
	}
	file.Seek(0L,CFile::begin);
	if(!file.Read(chrBuffer,1024))
	{
		AfxMessageBox(_T("Failed to read calibration factor file."));
		file.Close();
		return FALSE;
	}
	sscanf_s(chrBuffer," %le  %le  %le  %le  %le  %le  %le  %le  %le  %le  %le  %le  %le  %le  %le\r\n %le  %le  %le  %le  %le  %le  %le  %le  %le  %le  %le  %le  %le  %le  %le\r\n",
		&dblCaliFactor[0][0],&dblCaliFactor[0][1],&dblCaliFactor[0][2],&dblCaliFactor[0][3],&dblCaliFactor[0][4],&dblCaliFactor[0][5],&dblCaliFactor[0][6],&dblCaliFactor[0][7],&dblCaliFactor[0][8],&dblCaliFactor[0][9],&dblCaliFactor[0][10],&dblCaliFactor[0][11],&dblCaliFactor[0][12],&dblCaliFactor[0][13],&dblCaliFactor[0][14],
		&dblCaliFactor[1][0],&dblCaliFactor[1][1],&dblCaliFactor[1][2],&dblCaliFactor[1][3],&dblCaliFactor[1][4],&dblCaliFactor[1][5],&dblCaliFactor[1][6],&dblCaliFactor[1][7],&dblCaliFactor[1][8],&dblCaliFactor[1][9],&dblCaliFactor[1][10],&dblCaliFactor[1][11],&dblCaliFactor[1][12],&dblCaliFactor[1][13],&dblCaliFactor[1][14]);
	file.Close();

	return TRUE;
}
BOOL LSeekFiber::SeekPoints(CString fn)  //计算所有光点的像素坐标
{
#define MAP(i,j) lpRawBuffer[(i)*intImageHeight+ (j)]

	
	if (!ReadZBdata(fn))	return FALSE;

	int p,q,g;
	int i,j,m,m1,m2,n,n1,n2,mm;
	double sumRow,sumCol,SumGray;
	int intCounter=-1;
	CString rawfn,ss,str;
	for(int rep=0;rep<intRepeatTimes;rep++)
	{
		ss.Format(_T("%02d.raw"),rep);
		rawfn=fn.Left(fn.GetLength() - 6)+ss;
		if (!ReadRawData(rawfn))	return FALSE;

		ZeroMemory(dblCoorX,intMemAlloc*8);//it's essential
		ZeroMemory(dblCoorY,intMemAlloc*8);//it's essential
		ZeroMemory(dblX,intMemAlloc*8);//it's essential
		ZeroMemory(dblY,intMemAlloc*8);//it's essential
		ZeroMemory(intMM,intMemAlloc*4);//it's essential
		ZeroMemory(intMM1,intMemAlloc*4);//it's essential
		ZeroMemory(intNum,intMemAlloc*4);//it's essential
		ZeroMemory(intNum1,intMemAlloc*4);//it's essential

		intCounter=0;

		//阈值筛选
		long long meanGray = 0;
		for (int row = 0; row < intImageWidth; ++row)
		{
			for (int col = 0; col < intImageHeight; ++col)
			{
				int t = MAP(row, col);
				meanGray += MAP(row, col);
			}
		}
		meanGray /= (intImageWidth * intImageHeight);
		//int meanGray1 = std::accumulate(lpRawBuffer.begin(), lpRawBuffer.end(), 0)/ (intImageWidth * intImageHeight);

		intBKThreshold = max((int)(meanGray * 1.2), intBKThreshold);
		intLightThreshold = max(intBKThreshold,min((int)(meanGray * 4), intLightThreshold));
		

		for (i = 0; i < intImageWidth; i++)
		{
			for (j = 0; j<intImageHeight; j++)
			{
				if (MAP(i, j)>intLightThreshold)//可能存在一个亮点
				{
					sumRow = 0; sumCol = 0; SumGray = 0;

					m_nTop = 0;
					m_nBottom = 0;
					mm = 0;
					m_nDui[0][m_nBottom] = i;
					m_nDui[1][m_nBottom] = j;
					m_nDui[2][m_nBottom] = MAP(i, j);
					MAP(i, j) = 0;
					m_nBottom++; if (m_nBottom >= 2000)m_nBottom = 0;

					while (m_nTop != m_nBottom)
					{
						p = m_nDui[0][m_nTop];
						q = m_nDui[1][m_nTop];
						g = m_nDui[2][m_nTop];
						if (mm < g)mm = g;
						sumRow += p * g;
						sumCol += q * g;
						SumGray += g;
						m_nTop++; if (m_nTop >= 2000)m_nTop = 0;

						m1 = p - 1;	if (m1 < 0)m1 = 0;
						m2 = p + 1; if (m2 >= intImageWidth)m2 = intImageWidth - 1;
						n1 = q - 1;	if (n1<0)n1 = 0;
						n2 = q + 1;	if (n2 >= intImageHeight)n2 = intImageHeight - 1;
						for (m = m1; m <= m2; m++)
						for (n = n1; n <= n2; n++)
						if (MAP(m, n)>intBKThreshold)
						{
							m_nDui[0][m_nBottom] = m;
							m_nDui[1][m_nBottom] = n;
							m_nDui[2][m_nBottom] = MAP(m, n);
							MAP(m, n) = 0;
							m_nBottom++; if (m_nBottom >= 2000)m_nBottom = 0;
						}
					}
					//ASSERT(SumGray > 0);//正常情况下分母不会为0
					if (m_nTop > 6)
					{
						// + 1是为了和matlab一致，坐标从1开始
						dblCoorY[intCounter] = intImageWidth/2-(sumRow / SumGray + 1);//y
						dblCoorX[intCounter] = sumCol / SumGray + 1 - intImageHeight/2;//x
						intMM[intCounter] = mm;//mm
						intNum[intCounter] = m_nTop ;//mm
						intCounter++;
					}
					
				}

				//就要超出内存分配限制了，中止循环
				if (intCounter >= intMemAlloc)
				{
					i = intImageWidth;
					j = intImageHeight;
				}
			}
		}
		intLightNum = intCounter;//实际的点数


		FindQ();//从 circle txt文件找到像素坐标，进行圆心拟合，结果存放在变量 &centerX[i], &centerY[i] 中
		NiheParamCpp(biaoding);//计算参数文件
		PixelToMicron(biaoding);//所有光纤与FF坐标转换为微米坐标
		SortAllPoints(); //匹配计算
		str =(LPCSTR)(rawfn.Left(rawfn.GetLength() - 4) + ".txt");
		WriteMicronTxt(str);//写最终文件

	}

	return TRUE;
}



//计算参数文件
void LSeekFiber::NiheParamCpp(int biaoding)
{
	int Num = intQNum;
	//进一步筛选Q值，防止参数拟合错误
	//vector<double> dis(intQNum, 0);
	//double pixcelDisThreshold = 50.0;
	for (int i = 0; i < intQNum; i++)
	{
		//dis[i]=sqrt((FFCircleX[i]- pixelQCoorX[i])* (FFCircleX[i] - pixelQCoorX[i])+ (FFCircleY[i]- pixelQCoorY[i])* (FFCircleY[i] - pixelQCoorY[i]));
		if (radius[i] == -1.0)
		{
			Num--;
		}
	}
	
	MatrixXd x(Num, 1);
	MatrixXd y(Num, 1);
	MatrixXd lilun_x(Num, 1);
	MatrixXd lilun_y(Num, 1);
	MatrixXd paramX_array(Num, 1);
	MatrixXd paramY_array(Num, 1);

	MatrixXd A(biaoding/2, biaoding/2);
	MatrixXd BX(biaoding/2, 1);
	MatrixXd BY(biaoding/2, 1);

	for (int i = 0,j=0; i < intQNum; i++)
	{
		if (radius[i] == -1.0)
		{
			continue;
		}
		else
		{
			
			x(j, 0) = centerX[i];
			y(j, 0) = centerY[i];
			lilun_x(j, 0) = FFLilunX[i];
			lilun_y(j, 0) = FFLilunY[i];
			j++;
		}
		
		
		//lilun_x(i, 0) = dblQCoorX[i];
		//lilun_y(i, 0) = dblQCoorY[i];
		
		//double test3 = dblQCoorX[i];
		//double test4 = dblQCoorY[i];
		
		//double test1 = FFLilunX[i];
		//double test2 = FFLilunY[i];
	}

	if (biaoding == 6)
	{
	
		A << Num, x.sum(), y.sum(),
			x.sum(), (x.array() * x.array()).sum(), (x.array() *y.array()).sum(),
			y.sum(), (y.array() *x.array()).sum(), (y.array() *y.array()).sum();

		BX <<(lilun_x).sum(),
			(lilun_x.array() *x.array()).sum(),
			(lilun_x.array() *y.array()).sum();

		BY <<(lilun_y).sum(),
			(lilun_y.array() *x.array()).sum(),
			(lilun_y.array() *y.array()).sum();
		
	}
	if (biaoding == 20)
	{
		//MatrixXd test1(1, 1);
		//test1 << (lilun_x.array() * x.array() * x.array()).sum() ;
		//MatrixXd test2(1, 1);
		//test2 << (lilun_x.array() * (x.array().cube()).array()).sum();
		//MatrixXd test3(1, 1);
		//test3 << (lilun_x.array() * x.array() * x.array() * y.array()).sum();
		//MatrixXd test4(1, 1);
		//test4 << (lilun_x.array() * (y.array().cube()).array());
		//MatrixXd test5(1, 1);
		//test5 << (lilun_x.array() * (x.array().cube()).array()).sum();

		BX << (lilun_x).sum(),
			(lilun_x.array() * x.array()).sum(),
			(lilun_x.array() * y.array()).sum(),
			(lilun_x.array() * x.array() * x.array()).sum(),
			(lilun_x.array() * x.array() * y.array()).sum(),
			(lilun_x.array() * y.array() * y.array()).sum(),
			(lilun_x.array() * (x.array().cube()).array()).sum(),
			(lilun_x.array() * x.array() * x.array() * y.array()).sum(),
			(lilun_x.array() * x.array() * y.array() * y.array()).sum(),
			(lilun_x.array() * (y.array().cube()).array()).sum();

		BY << (lilun_y).sum(),
			(lilun_y.array() * x.array()).sum(),
			(lilun_y.array() * y.array()).sum(),
			(lilun_y.array() * x.array() * x.array()).sum(),
			(lilun_y.array() * x.array() * y.array()).sum(),
			(lilun_y.array() * y.array() * y.array()).sum(),
			(lilun_y.array() * (x.array().cube()).array()).sum(),
			(lilun_y.array() * x.array() * x.array() * y.array()).sum(),
			(lilun_y.array() * x.array() * y.array() * y.array()).sum(),
			(lilun_y.array() * (y.array().cube()).array()).sum();

		A << Num, x.sum(), y.sum(), (x.array()*x.array()).sum(), (x.array()*y.array()).sum(), (y.array() *y.array()).sum(), (x.array().cube() ).sum(), (x.array().square().array() * y.array()).sum(), (y.array().square().array() * x.array()).sum(), (y.array().cube() ).sum(),
			x.sum(), (x.array() *x.array()).sum(), (x.array() *y.array()).sum(), (x.array().cube() ).sum(), ((x.array().square()).array() *y.array()).sum(), ((y.array().square()).array() *x.array()).sum(), (x.array().pow(4)).sum(), (x.array().cube().array() * y.array()).sum(), (x.array().square().array() * y.array().square().array()).sum(), (x.array() *y.array().cube().array()).sum(),
			y.sum(), (y.array() *x.array()).sum(), (y.array() *y.array()).sum(), ((x.array().square()).array() *y.array()).sum(), (x.array() *(y.array().square()).array()).sum(), (y.array().cube()).sum(), (x.array().cube().array() * y.array()).sum(), (x.array().square().array() * y.array().square().array()).sum(), (x.array() *(y.array().cube()).array()).sum(), (y.array().pow(4)).sum(),
			(x.array() *x.array()).sum(), (x.array().cube() ).sum(), ((x.array().square()).array() *y.array()).sum(), (x.array().pow(4)).sum(), ((x.array().cube() ).array() *y.array()).sum(), ((x.array().square()).array() *(y.array().square()).array()).sum(), (x.array().pow(5)).sum(), (x.array().pow(4).array() * y.array()).sum(), ((x.array().cube()).array() * (y.array().square()).array()).sum(), (x.array().square().array() * (y.array().cube()).array()).sum(),
			(x.array() *y.array()).sum(), ((x.array().square()).array() *(y.array()).array()).sum(), (x.array() *(y.array().square()).array()).sum(), ((x.array().cube() ).array() *y.array()).sum(), ((x.array().square()).array() *(y.array().square()).array()).sum(), (x.array() *(y.array().cube() ).array()).sum(), (x.array().pow(4).array() * y.array()).sum(), (x.array().cube().array() * (y.array().square()).array()).sum(), (x.array().square().array() * (y.array().cube()).array()).sum(), (x.array() *(y.array().pow(4)).array()).sum(),
			(y.array() *y.array()).sum(), ((y.array().square()).array() * x.array()).sum(), (y.array().cube() ).sum(), ((x.array().square()).array() *(y.array().square()).array()).sum(), ((y.array().cube() ).array() *x.array()).sum(), (y.array().pow(4)).sum(), (x.array().cube().array() * (y.array().square()).array()).sum(), (x.array().square().array() * (y.array().cube()).array()).sum(), (x.array() *(y.array().pow(4)).array()).sum(), (y.array().pow(5)).sum(),
			(x.array().cube() ).sum(), (x.array().pow(4)).sum(), ((x.array().cube() ).array() *y.array()).sum(), (x.array().pow(5)).sum(), (x.array().pow(4).array() * y.array()).sum(), ((x.array().cube()).array() * (y.array().square()).array()).sum(), (x.array().pow(6)).sum(), (x.array().pow(5).array() * y.array()).sum(), ((x.array().pow(4)).array() * (y.array().square()).array()).sum(), ((x.array().cube()).array() * (y.array().cube()).array()).sum(),
			((x.array().square()).array() *(y.array())).sum(), ((x.array().cube() ).array() *y.array()).sum(), ((x.array().square()).array() *(y.array().square()).array()).sum(), (x.array().pow(4).array() * y.array()).sum(), (x.array().cube().array() * (y.array().square()).array()).sum(), (x.array().square().array() * (y.array().cube()).array()).sum(), (x.array().pow(5).array() * y.array()).sum(), (x.array().pow(4).array() * (y.array().square()).array()).sum(), (x.array().cube().array() * (y.array().cube()).array()).sum(), (x.array().square().array() * (y.array().pow(4)).array()).sum(),
			(x.array() *(y.array().square()).array()).sum(), ((x.array().square()).array() *(y.array().square()).array()).sum(), (x.array() *(y.array().cube() ).array()).sum(), (x.array().cube().array() * y.array().square().array()).sum(), (x.array().square().array() * (y.array().cube()).array()).sum(), (x.array() *(y.array().pow(4)).array()).sum(), (x.array().pow(4).array() * (y.array().square()).array()).sum(), (x.array().cube().array() * (y.array().cube()).array()).sum(), (x.array().square().array() * (y.array().pow(4)).array()).sum(), (x.array() *(y.array().pow(5)).array()).sum(),
			(y.array().cube() ).sum(), (x.array().array() *(y.array().cube()).array()).sum(), (y.array().pow(4)).sum(), (x.array().square().array() * (y.array().cube()).array()).sum(), (x.array() *(y.array().pow(4)).array()).sum(), (y.array().pow(5)).sum(), (x.array().cube().array() * (y.array().cube()).array()).sum(), (x.array().square().array() * (y.array().pow(4)).array()).sum(), (x.array() *(y.array().pow(5)).array()).sum(), (y.array().pow(6)).sum();
	}

	paramX_array = A.inverse() * BX;
	paramY_array = A.inverse() * BY;

	for (int i = 0; i < biaoding / 2; i++)
	{
		double a = paramX_array(i, 0);
		double b = paramY_array(i, 0);
		paramX[i] = paramX_array(i,0);
		paramY[i] = paramY_array(i, 0);
	}
	return;
	
}

//P坐标转换为微米坐标
void LSeekFiber::PixelToMicron(int biaoding)
{
	for (int i = 0; i < intQNum; ++i)
	{
		pixelQCoorX[i] = centerX[i];
		pixelQCoorY[i] = centerY[i];
	}
	for (int i = 0; i < intLightNum; ++i)
	{
		pixelPCoorX[i] = dblCoorX[i];
		pixelPCoorY[i] = dblCoorY[i];
	}

	/*三十个参数 */
	if (biaoding == 30)
	{
		/*
		*X=paramX[0]+paramX[1]*x+paramX[2]*y+paramX[3]*x*x+paramX[4]*x*y\
			+paramX[5]*y*y+paramX[6]*pow(x,3)+paramX[7]*pow(x,2)*y+paramX[8]*x*pow(y,2)\
			+paramX[9]*pow(y,3)+paramX[10]*pow(x,4)+paramX[11]*pow(x,3)*y+paramX[12]*pow(x,2)*pow(y,2)+paramX[13]*x*pow(y,3)+paramX[14]*pow(y,4);
		*Y=paramY[0]+paramY[1]*x+paramY[2]*y+paramY[3]*x*x+paramY[4]*x*y\
			+paramY[5]*y*y+paramY[6]*pow(x,3)+paramY[7]*pow(x,2)*y+paramY[8]*x*pow(y,2)\
			+paramY[9]*pow(y,3)+paramY[10]*pow(x,4)+paramY[11]*pow(x,3)*y+paramY[12]*pow(x,2)*pow(y,2)+paramY[13]*x*pow(y,3)+paramY[14]*pow(y,4);
		*X/=1000.0;
		*Y/=1000.0;
		*/
	}	
	//20个参数 输入：centerX，centerY
	else if (biaoding == 20)
	{
		for (int i = 0; i < intLightNum; i++)
		{
			micronPCoorX[i] = (paramX[0] + paramX[1] * pixelPCoorX[i] + paramX[2] * pixelPCoorY[i] + paramX[3] * pixelPCoorX[i] * pixelPCoorX[i] + paramX[4] * pixelPCoorX[i] * pixelPCoorY[i]\
				+ paramX[5] * pixelPCoorY[i] * pixelPCoorY[i] + paramX[6] * pow(pixelPCoorX[i], 3) + paramX[7] * pow(pixelPCoorX[i], 2) * pixelPCoorY[i] + paramX[8] * pixelPCoorX[i] * pow(pixelPCoorY[i], 2)\
				+ paramX[9] * pow(pixelPCoorY[i], 3));
			micronPCoorY[i] = (paramY[0] + paramY[1] * pixelPCoorX[i] + paramY[2] * pixelPCoorY[i] + paramY[3] * pixelPCoorX[i] * pixelPCoorX[i] + paramY[4] * pixelPCoorX[i] * pixelPCoorY[i]\
				+ paramY[5] * pixelPCoorY[i] * pixelPCoorY[i] + paramY[6] * pow(pixelPCoorX[i], 3) + paramY[7] * pow(pixelPCoorX[i], 2) * pixelPCoorY[i] + paramY[8] * pixelPCoorX[i] * pow(pixelPCoorY[i], 2)\
				+ paramY[9] * pow(pixelPCoorY[i], 3)) ;
		}
		for (int i = 0; i < intQNum; i++)
		{
			micronQCoorX[i] = (paramX[0] + paramX[1] * pixelQCoorX[i] + paramX[2] * pixelQCoorY[i] + paramX[3] * pixelQCoorX[i] * pixelQCoorX[i] + paramX[4] * pixelQCoorX[i] * pixelQCoorY[i]\
				+ paramX[5] * pixelQCoorY[i] * pixelQCoorY[i] + paramX[6] * pow(pixelQCoorX[i], 3) + paramX[7] * pow(pixelQCoorX[i], 2) * pixelQCoorY[i] + paramX[8] * pixelQCoorX[i] * pow(pixelQCoorY[i], 2)\
				+ paramX[9] * pow(pixelQCoorY[i], 3));
			micronQCoorY[i] = (paramY[0] + paramY[1] * pixelQCoorX[i] + paramY[2] * pixelQCoorY[i] + paramY[3] * pixelQCoorX[i] * pixelQCoorX[i] + paramY[4] * pixelQCoorX[i] * pixelQCoorY[i]\
				+ paramY[5] * pixelQCoorY[i] * pixelQCoorY[i] + paramY[6] * pow(pixelQCoorX[i], 3) + paramY[7] * pow(pixelQCoorX[i], 2) * pixelQCoorY[i] + paramY[8] * pixelQCoorX[i] * pow(pixelQCoorY[i], 2)\
				+ paramY[9] * pow(pixelQCoorY[i], 3));
		}
	}
	else if (biaoding == 6)
	{
		for (int i = 0; i < intPNum; i++)
		{
			micronPCoorX[i] = (paramX[0] + paramX[1] * pixelPCoorX[i] + paramX[2] * pixelPCoorY[i] + paramX[3] * pixelPCoorX[i] * pixelPCoorX[i] + paramX[4] * pixelPCoorX[i] * pixelPCoorY[i]\
				+ paramX[5] * pixelPCoorY[i] * pixelPCoorY[i]);
			micronPCoorY[i] = (paramY[0] + paramY[1] * pixelPCoorX[i] + paramY[2] * pixelPCoorY[i] + paramY[3] * pixelPCoorX[i] * pixelPCoorX[i] + paramY[4] * pixelPCoorX[i] * pixelPCoorY[i]\
				+ paramY[5] * pixelPCoorY[i] * pixelPCoorY[i] );
		}
	}
	return;
}


void LSeekFiber::FindQ()
{

	int nn;
	intNoUse = 0;
	intLightNumRaw = intLightNum;
	for (int i = 0; i < intQNum; i++)
	{
		ZeroMemory(FFSingeX, intMemAlloc * sizeof(double));
		ZeroMemory(FFSingeY, intMemAlloc * sizeof(double));
		nn = 0;
		for (int j = 0; j < intLightNum; j++)
		{
			dblDist[i * intLightNum + j] = (dblCoorX[j] - FFCircleX[i]) * (dblCoorX[j] - FFCircleX[i])
				+ (dblCoorY[j] - FFCircleY[i]) * (dblCoorY[j] - FFCircleY[i]);
			if (dblDist[i * intLightNum + j] < FFCMax * FFCMax && dblDist[i * intLightNum + j] > FFCMin * FFCMin)
			{
				FFSingeX[nn] = dblCoorX[j];
				FFSingeY[nn] = dblCoorY[j];
				nn++;
			}
		}
		CircleFitting(FFSingeX, FFSingeY, &centerX[i], &centerY[i], &radius[i]); //CircleFitting
		//加入Q的距离判断，如果大于阈值pixcelDisThreshold ，则不参与匹配
		double disRight = sqrt((centerX[i] - FFCircleX[i]) * (centerX[i] - FFCircleX[i])+ (centerY[i] - FFCircleY[i]) * (centerY[i] - FFCircleY[i]));
		if (_isnan(disRight) || disRight > pixcelDisThreshold)
		{
			//centerX[i] = -1.0; centerY[i] = -1.0; 
			radius[i] = -1.0;
		}

		//删除FF周边光纤点（intLightNum--）
		for (int j = intLightNum-1, k = intLightNum - 1; j >= 0; j--)
		{
			//double a = dblCoorX[i];
			double disDel= sqrt((centerX[i] - dblCoorX[j]) * (centerX[i] - dblCoorX[j]) + (centerY[i] - dblCoorY[j]) * (centerY[i] - dblCoorY[j]));
			if (disDel< (FFCMax+10))
			{
				double tempX = dblCoorX[k];
				double tempY = dblCoorY[k];
				dblCoorX[k] = dblCoorX[j];
				dblCoorY[k] = dblCoorY[j];
				dblCoorX[j] = tempX;
				dblCoorY[j] = tempY;
				intLightNum--;
				k--;
				intNoUse++;
			}
		}
	}
	return;
}


void LSeekFiber::CircleFitting(double* FFSingeX, double* FFSingeY, double* centerX, double* centerY, double* radius)
{
	std::set<int> deleteIndexSet; //剔除光点下标集合
	double distanceSigma = DBL_MAX;
	double distanceSigmaThreshold = 0.5;  //方差的阈值
	do
	{
		//1.先拟合圆心数据
		double sumX = 0, sumY = 0;
		double sumXX = 0, sumYY = 0, sumXY = 0;
		double sumXXX = 0, sumXXY = 0, sumXYY = 0, sumYYY = 0;
		int i = 0;
		while (FFSingeX[i] != 0)
		{
			if (deleteIndexSet.count(i))
			{
				i++;
				continue;
			}
			double p0 = FFSingeY[i];
			double p1 = FFSingeX[i];
			sumX += p0;
			sumY += p1;
			sumXX += p0 * p0;
			sumYY += p1 * p1;
			sumXY += p0 * p1;
			sumXXX += p0 * p0 * p0;
			sumXXY += p0 * p0 * p1;
			sumXYY += p0 * p1 * p1;
			sumYYY += p1 * p1 * p1;
			i++;
		}
		int pCount = i - deleteIndexSet.size();
		double M1 = pCount * sumXY - sumX * sumY;
		double M2 = pCount * sumXX - sumX * sumX;
		double M3 = pCount * (sumXXX + sumXYY) - sumX * (sumXX + sumYY);
		double M4 = pCount * sumYY - sumY * sumY;
		double M5 = pCount * (sumYYY + sumXXY) - sumY * (sumXX + sumYY);

		double a = (M1 * M5 - M3 * M4) / (M2 * M4 - M1 * M1);
		double b = (M1 * M3 - M2 * M5) / (M2 * M4 - M1 * M1);
		double c = -(a * sumX + b * sumY + sumXX + sumYY) / pCount;

		//圆心XY 半径
		double xCenter = -0.5 * a;
		double yCenter = -0.5 * b;
		double ra = 0.5 * sqrt(a * a + b * b - 4 * c);
		*radius = ra;
		*centerX = yCenter; 
		*centerY = xCenter;

		//2.计算均值和方差
		int k = 0;
		std::vector<double> distanceList;
		while (FFSingeX[k] != 0)
		{
			if (deleteIndexSet.count(k))
			{
				k++;
				continue;
			}
			double dist = sqrt((FFSingeX[k] - yCenter) * (FFSingeX[k] - yCenter) + (FFSingeY[k] - xCenter) * (FFSingeY[k] - xCenter));
			distanceList.push_back(dist);
			k++;
		}
		k = distanceList.size();
		double sum = std::accumulate(std::begin(distanceList), std::end(distanceList), 0.0);
		double mean = sum / k; //均值
		double accum = 0.0;
		std::for_each(std::begin(distanceList), std::end(distanceList), [&](const double d) {
			accum += (d - mean) * (d - mean);
		});
		distanceSigma = sqrt(accum / k); //方差

		//3.根据方差去剔除
		if (distanceSigma <= distanceSigmaThreshold) break;
		for (int i = 0; i < distanceList.size(); i++)
		{
			if (distanceList[i] < mean - 3 * distanceSigma || distanceList[i] > mean + 3 * distanceSigma)
			{
				deleteIndexSet.insert(i);
			}
		}
		
	} 
	while (distanceSigma > distanceSigmaThreshold);
	return;
}

BOOL LSeekFiber::ReadZBdata(CString strFileName)
{
	FILE* fid;
	errno_t err;
	char cell;
	USES_CONVERSION;
	LPCSTR StrFileName = (LPCSTR)T2A(strFileName);//转化CString到const char*
	err = fopen_s(&fid,StrFileName, "rt");
	if(fid==NULL)
	{
		AfxMessageBox(strFileName+" : Failed to read ZB file.");
		return FALSE;
	}
	
	char buf[100] = {0};
	intUnitNum=0;
	intSolidNum=0;
	intPNum = 0;
	sCellName = {};
	char c;
	c = fgetc(fid);
	while (!feof(fid))
	{
		char cellName[11];
		fscanf_s(fid, "%10c", cellName);
		cellName[10] = '\0';
		//string s(cellName);
		//s = s.substr(3, 6);
		//sCellName.push_back(s);
		for (int i =0; i <11; i++)
		{
			if (cellName[i] != ' ')
			{
				cell = cellName[i];
				break;
			}
		}
		if (intCCDNo == 0)
		{
			if (cell == 'P' || cell == 'Q')
			{
				string s(cellName);
				sCellName.push_back(s);
				if (cell == 'P')
				{
					sCellNameP.push_back(s);
				}
				else
				{
					sCellNameQ.push_back(s);
				}
				fscanf_s(fid,"%2c",buf);
				fscanf_s(fid,"%lf %lf",&dblZBCoorX[intUnitNum],&dblZBCoorY[intUnitNum]);
				fscanf_s(fid,"%lf %lf",&dblZBCenA[intUnitNum],&dblZBEccA[intUnitNum]);
				intUnitNum++;
				intSolidNum++;
				if (cell == 'P')
				{
					intPNum++;
				}
			}

		}
		else if(intAllUnit||(cell=='Q')||(cell=='D'+intCCDNo))
		{
			fscanf_s(fid,"%2c",buf);
			fscanf_s(fid,"%lf %lf",&dblZBCoorX[intUnitNum],&dblZBCoorY[intUnitNum]);
			fscanf_s(fid,"%lf %lf",&dblZBCenA[intUnitNum],&dblZBEccA[intUnitNum]);
			intUnitNum++;
			intSolidNum++;
		}
		/*else if(i<12)
		{lpRawBuffer
			if (((intCCDNo==0))||
				((intCCDNo==1)&&(i==3||i==7||i==11||i==0||i==4||i==8 ))||
				((intCCDNo==2)&&(i==0||i==4||i==8 ||i==1||i==5||i==9 ))||
				((intCCDNo==3)&&(i==1||i==5||i==9 ||i==2||i==6||i==10))||
				((intCCDNo==4)&&(i==2||i==6||i==10||i==3||i==7||i==11)))
			{
				fscanf_s(fid,"%2c",buf);
				fscanf_s(fid,"%lf %lf",&dblZBCoorX[intUnitNum],&dblZBCoorY[intUnitNum]);
				fscanf_s(fid,"%lf %lf",&dblZBCenA[intUnitNum],&dblZBEccA[intUnitNum]);
				intUnitNum++;
			}
		}*/
		fgets(buf,100,fid);
		c = fgetc(fid);//防止多读一次
	}	
	fclose(fid);

}

BOOL LSeekFiber::ReadRawData(CString strFileName)
{
	CFile file;
	CFileException e;
	if(!file.Open(strFileName,CFile::modeRead,&e))
	{
//		AfxMessageBox("Failed to read raw file.");
		return FALSE;
	}
	
	file.Seek(0L,CFile::begin);
	if(!file.Read(lpRawBuffer,intImageWidth*intImageHeight* sizeof(unsigned short)))
	{
		AfxMessageBox(_T("Failed to read raw data."));
		file.Close();
		return FALSE;
	}

	file.Close();

	if(intNeedFile==0)
		DeleteFile(strFileName);
	return TRUE;
}


void LSeekFiber::SortAllPoints()
{
	//dblCoorX  dblCoorY   micronPCoorX  micronPCoorY
	for (int allPoint = 0; allPoint < intLightNum; allPoint++)
	{
		dblCoorX[allPoint] = micronPCoorX[allPoint];
		dblCoorY[allPoint] = micronPCoorY[allPoint];
		pixelPCoorX1[allPoint] = pixelPCoorX[allPoint];
		pixelPCoorY1[allPoint] = pixelPCoorY[allPoint];
	}
	

	int i, j, k, m, n;
	double dblMinDist, dblMaxDouble, dblMaxDouble1;

	ZeroMemory(bFlag1, intMemAlloc * sizeof(BOOL));
	ZeroMemory(bFlag2, intMemAlloc * sizeof(BOOL));
	ZeroMemory(dblCoorX1, intMemAlloc * sizeof(double));
	ZeroMemory(dblCoorY1, intMemAlloc * sizeof(double));
	ZeroMemory(dblX1, intMemAlloc * sizeof(double));
	ZeroMemory(dblY1, intMemAlloc * sizeof(double));

	dblMaxDouble = double(MAXLONGLONG);
	dblMaxDouble1 = double(33000 * 33000);

	//calc dblDist and get dblDist1
	int nn = 0;
	for (i = 0; i < intUnitNum; i++)
	{
		for (j = 0; j < intLightNum; j++)
		{
			dblDist[i * intLightNum + j] = (dblCoorX[j] - dblZBCoorX[i]) * (dblCoorX[j] - dblZBCoorX[i])
				+ (dblCoorY[j] - dblZBCoorY[i]) * (dblCoorY[j] - dblZBCoorY[i]);
			if (dblDist[i * intLightNum + j] < dblMaxDouble1)
			{
				dblDist1[nn] = dblDist[i * intLightNum + j];
				dblDistM[nn] = i; //i表示单元
				dblDistN[nn] = j; //j表示光点
				nn++;
			}
		}
	}
	//sort dblDist1  //这个就是根据dblDist1排序
	double dd;
	for (i = 0; i < nn - 1; i++)
	{
		k = i;
		for (j = i + 1; j < nn; j++)
			//对于当前i 找到距离最小的 和他交换
			if (dblDist1[j] < dblDist1[k])
				k = j;
		if (k != i)
		{
			//将最小的值依次给i
			dd = dblDist1[i]; dblDist1[i] = dblDist1[k]; dblDist1[k] = dd;
			m = dblDistM[i]; dblDistM[i] = dblDistM[k]; dblDistM[k] = m;
			n = dblDistN[i]; dblDistN[i] = dblDistN[k]; dblDistN[k] = n;
		}
	}
	//proc dblDist1
	for (i = 0; i < nn; i++)
	{
		//dblDistM里面存的单元  dblDistN存的光点  都已经按照dblDist1排序了
		m = dblDistM[i];
		n = dblDistN[i];
		if (!(bFlag1[m] || bFlag2[n]))
		{
			//为什么把n的给了m
			dblCoorX1[m] = dblCoorX[n];
			dblCoorY1[m] = dblCoorY[n];

			pixelPCoorX1[m] = pixelPCoorX[n];
			pixelPCoorY1[m] = pixelPCoorY[n];

			dblX1[m] = dblX[n];
			dblY1[m] = dblY[n];
			intMM1[m] = intMM[n];
			intNum1[m] = intNum[n];
			bFlag1[m] = 1;
			bFlag2[n] = 1;
		}
	}
	for (j = 0, k = intUnitNum; j < intLightNum; j++)
	{ 
		if (!bFlag2[j])
		{
			intMM1[k] = intMM[j];
			intNum1[k] = intNum[j];
			dblX1[k] = dblX[j];
			dblY1[k] = dblY[j];

			pixelPCoorX1[k] = pixelPCoorX[j];
			pixelPCoorY1[k] = pixelPCoorY[j];

			dblCoorX1[k] = dblCoorX[j];
			dblCoorY1[k++] = dblCoorY[j];
		}
	}
	intLightNum = k;
}

void LSeekFiber::WriteMicronTxt(CString str)
{
	USES_CONVERSION;
	CString path_Micron = (str.Left(str.GetLength() - 4) + "_xy.txt");
	LPCSTR Path_Micron = (LPCSTR)T2A(path_Micron);//转化CString到const char*

	int i;
	FILE* ff;
	errno_t err;
	err = fopen_s(&ff, Path_Micron, "w+");

	for (i = 0; i < intPNum; i++)
	{
		if (pixelPCoorX1[i] == 0.0 && pixelPCoorY1[i] == 0.0)
		{
			fprintf_s(ff, "%s\t%12.6f\t%12.6f\t%12.6f\t%12.6f\n", sCellNameP[i].c_str(), 0.0,0.0,0.0,0.0);
		}
		else
		{
			fprintf_s(ff, "%s\t%12.6f\t%12.6f\t%12.6f\t%12.6f\n", sCellNameP[i].c_str(), dblCoorX1[i], dblCoorY1[i], pixelPCoorX1[i], pixelPCoorY1[i]);
		}

	}
	for (i = 0; i < intQNum; i++)
	{
		double dis = sqrt((micronQCoorX[i] - FFLilunX[i]) * (micronQCoorX[i] - FFLilunX[i]) + (micronQCoorY[i] - FFLilunY[i]) * (micronQCoorY[i] - FFLilunY[i]));
		if (dis < 3000.0)
		{
			fprintf_s(ff, "%s\t%12.6f\t%12.6f\t%12.6f\t%12.6f\n", sCellNameQ[i].c_str(), micronQCoorX[i], micronQCoorY[i], centerX[i], centerY[i]);
		}
		else
		{
			fprintf_s(ff, "%s\t%12.6f\t%12.6f\t%12.6f\t%12.6f\n", sCellNameQ[i].c_str(), FFLilunX[i], FFLilunY[i], centerX[i], centerY[i]);
		}

	}
	for (i = intPNum; i < intLightNum; i++)
	{
		if (pixelPCoorX1[i] == 0.0 || pixelPCoorY1[i] == 0.0 || dblCoorX1[i] ==0.0 || dblCoorY1[i]==0.0)
		{
			//fprintf_s(ff, "%s\t%12.6f\t%12.6f\t%12.6f\t%12.6f\n", "nouse", 0.0, 0.0, 0.0, 0.0);
		}
		else
		{
			fprintf_s(ff, "%s\t%12.6f\t%12.6f\t%12.6f\t%12.6f\n", "nouse", dblCoorX1[i], dblCoorY1[i], pixelPCoorX1[i], pixelPCoorY1[i]);
		}

	}
	for (i = intLightNumRaw-1; i >= intLightNumRaw - intNoUse; i--)
	{
		//if (pixelPCoorX1[i] == 0.0 || pixelPCoorY1[i] == 0.0 || dblCoorX1[i] == 0.0 || dblCoorY1[i] == 0.0)
		//{
		//	//fprintf_s(ff, "%s\t%12.6f\t%12.6f\t%12.6f\t%12.6f\n", "nouse", 0.0, 0.0, 0.0, 0.0);
		//}
		//else
		//{
			fprintf_s(ff, "%s\t%12.6f\t%12.6f\t%12.6f\t%12.6f\n", "delpoints", 0.0, 0.0, dblCoorX[i], dblCoorY[i]);
		//}

	}
	
	fclose(ff);
	return;

}