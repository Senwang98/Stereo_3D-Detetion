//#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include "mytest/IiBasicDef.h"

//only for Pseduo-color displaying
unsigned char stclut_b[2049];
unsigned char stclut_g[2049];
unsigned char stclut_r[2049];

/////////////////////////////////////////////////////////////////////////////////////////
//This is just a subfunction of the Pseduo-color displaying function void ShowDispImg()
/////////////////////////////////////////////////////////////////////////////////////////
void ST_setCLUT_sub()
{
	int i;
	//B=8cm
	stclut_r[0] = 0;
	stclut_g[0] = 0;
	stclut_b[0] = 0;
	for (i = 1; i <= 24; i++)
	{ //white(255,255,255) // disparity 0.03125(1/32) -> 0.75(160m) thereafter // 160m thereafter are whilte
		stclut_r[i] = 255;
		stclut_g[i] = 255;
		stclut_b[i] = 255;
	}
	for (i = 25; i <= 40; i++)
	{ //whilte (255,255,255)->grey(128,128,128) // disparity 0.75(160m)  -> to 1.25(100m)
		stclut_r[i] = 255 - ((255.0 - 128.0) / (40.0 - 24.0)) * (i - 24);
		stclut_g[i] = 255 - ((255.0 - 128.0) / (40.0 - 24.0)) * (i - 24);
		stclut_b[i] = 255 - ((255.0 - 128.0) / (40.0 - 24.0)) * (i - 24);
	}
	for (i = 41; i <= 64; i++)
	{ //grey(128,128,128)-> purple(255,0,255) // disparity 1.25(100m) -> to 2.0(60m)
		stclut_r[i] = 128 + int(5.291668 * (i - 41));
		stclut_g[i] = 128 - int(5.291668 * (i - 41));
		stclut_b[i] = 128 + int(5.291668 * (i - 41));
	}
	for (i = 65; i <= 120; i++)
	{ //purple(255,0,255)-> blue(0,0,255) // disparity 2.0(60m) -> to 4.0(30m)
		stclut_r[i] = 255 - int(4.553571 * (i - 64));
		stclut_g[i] = 0;
		stclut_b[i] = 255;
	}
	for (i = 121; i <= 176; i++)
	{ //blue(0,0,255) -> magenta(0,255,255) // disparity 4.0(30m) -> to 7.0(17m)
		//			stclut_r[i]=0+(255/(176-128))*(i-128);
		stclut_r[i] = 0;
		stclut_g[i] = 0 + int(4.553571 * (i - 120));
		stclut_b[i] = 255;
	}
	for (i = 177; i <= 320; i++)
	{ //magenta(0,255,255) -> green(0,255,0) // disparity 7.0(17m) -> to 15.0(8m)
		//			stclut_r[i]=0+(255/(320-176))*(i-176);
		stclut_r[i] = 0;
		stclut_g[i] = 255;
		stclut_b[i] = 255 - int(1.770833 * (i - 176));
	}
	for (i = 321; i <= 800; i++)
	{ //green(0,255,0)-> yellow(255,255,0) // disparity 10.0(12m) -> to 25.0(8m)
		//			stclut_r[i]=int(0+double((255/(800-320))*(i-320)));
		stclut_r[i] = 0 + int(0.53125 * (i - 320));
		stclut_g[i] = 255;
		stclut_b[i] = 0;
	}
	for (i = 801; i <= 2048; i++)
	{ //yellow (0,255,255)-> read(255,0,0) // disparity 25.0(8m) -> to 64.0(2m)
		//			stclut_r[i]=int(0+double((255/(2048-800))*(i-800)));
		stclut_r[i] = 255;
		stclut_g[i] = 255 - int(0.204327 * (i - 800));
		stclut_b[i] = 0;
	}
}

void generatePseudoColorImageForDisparity(float *pDisparityMap, int width, int height, cv::Mat *pColorImage)
{

	ST_setCLUT_sub();

	for (int i = 0; i < height; i++)
	{
		int iStep = i * pColorImage->step;

		for (int j = 0; j < width; j++)
		{

			float tmp = *(pDisparityMap + i * width + j);

			//tuning parameters
			//int d = int(tmp*32) * 1.2; //1290*960 setting
			//int d = int(tmp*16); //Zed 1080 setting
			int d = int(tmp * 32) * 1.5; //Zed 720 setting

			if (d > 2048)
			{
				d = 2048;
			}

			*(pColorImage->data + iStep + 0 + j * pColorImage->channels()) = stclut_b[d];
			*(pColorImage->data + iStep + 1 + j * pColorImage->channels()) = stclut_g[d];
			*(pColorImage->data + iStep + 2 + j * pColorImage->channels()) = stclut_r[d];

			//*(pColorImage->ptr<uchar>(i,j) + 0) = stclut_b[d];
			//*(pColorImage->ptr<uchar>(i,j) + 1) = stclut_g[d];
			//*(pColorImage->ptr<uchar>(i,j) + 2) = stclut_r[d];
		}
	}

	return;
}
