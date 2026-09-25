/**
 * @file    image2.c
 * @brief   图像二值化模块：OTSU大津法自适应阈值计算、双阈值二值化
 * @author  RRZ
 * @date    2026-05-16
 */

#include "image2.h"

uint8 base_image[MT9V03X_H][MT9V03X_W];
uint8 Threshold;

/**
 * @brief  OTSU大津法计算自适应二值化阈值
 * @param  mt9v03x_image_first  原始灰度图像指针
 * @return uint8  计算出的最佳阈值
 * @note   遍历灰度级[Pixel_Min, Pixel_Max]，找使类间方差最大的阈值
 *         纯黑/纯白图直接返回Pixel_Min; 灰度偏移量提前终止; 历史限幅保护
 */
uint8 otsuThreshold(uint8 *mt9v03x_image_first)   //注意计算阈值一定要用原图像
{
    int pixelCount[GrayScale]={0};	//可见光图像中每个灰度值的个数
    float pixelPro[GrayScale]={0};	//可见光图像中每个灰度值的比例
    int Pixel_Max=0;		//可见光最大灰度值
    int Pixel_Min=255;	//可见光最小灰度值
    int offset;//灰度值偏移量,在防止局部最优时使用
    uint16 width = MT9V03X_W;
    uint16 height = MT9V03X_H;
    uint32 gray_sum=0;	//总像素点灰度值之和

        // 优化点2：规范命名，初始化顺序统一，初始化内存零值
        int i, j, pixelSum = width * height;	//总像素点
        uint8 threshold = 0;	//阈值
        uint8* date_mt9v03x_image ;
        float w0=0, w1=0, u0tmp=0, u1tmp=0, u0=0, u1=0, u=0, deltaTmp=0, deltaMax=-0.5;

    date_mt9v03x_image= mt9v03x_image_first;  //指向图像数据的指针

        // 统计灰度级中每个像素在整幅图像中的个数
        // 优化点6：内层循环i改为uint16，匹配width/height类型，避免隐式转换开销
        for ( i = 0; (uint16)i < height; i++)
        {
            for (j = 0; (uint16)j < width; j++)
            {
                pixelCount[(int)date_mt9v03x_image[i * width + j]]++;  //当前的像素值作为计数数组的下标
                gray_sum+=(int)date_mt9v03x_image[i * width + j];       //灰度值总和
                        //更新最大最小灰度值
                if(date_mt9v03x_image[i * width + j]>Pixel_Max)   Pixel_Max=date_mt9v03x_image[i * width + j];
                if(date_mt9v03x_image[i * width + j]<Pixel_Min)   Pixel_Min=date_mt9v03x_image[i * width + j];
            }
        }

        // 优化点1：边界提前退出 - 图像全黑/全白，提前返回，避免浮点计算异常
        if (Pixel_Min == Pixel_Max)
        {
            threshold = Pixel_Min;
            return threshold;
        }

        //计算每个像素值的点在图像中的比例
        for (i = Pixel_Min; i <=Pixel_Max; i++)
        {
            pixelPro[i] = (float)pixelCount[i] / pixelSum;
        }

        //遍历灰度级[0,255]
        for (j = Pixel_Min; j <=Pixel_Max; j++)
        {
            w0 += pixelPro[j];  //目标像素占图像比例，即前景部分的比例
            u0tmp += j * pixelPro[j];  //目标平均灰度值: 每个灰度值点的比例*灰度值

            w1=1-w0;
            u1tmp=(float)gray_sum/pixelSum-u0tmp;
                //防止某类像素数量太少,导致计算结果不稳定
            if(w0<1e-6f||w1<1e-6f)
                continue;

            u0 = u0tmp / w0;              //前景平均灰度
            u1 = u1tmp / w1;              //背景平均灰度
            u = u0tmp + u1tmp;            //全局平均灰度
            deltaTmp = (float)(w0 *w1* (u0 - u1)* (u0 - u1)) ;//方差,方差越大,对应阈值效果越好
            if (deltaTmp > deltaMax)//找到最大方差,记录最佳阈值
            {
                deltaMax = deltaTmp;
                threshold = j;
            }
                    offset=(Pixel_Max-Pixel_Min)/10;//灰度值偏移量
                    //判断偏离最优阈值情况,避免阈值在达到最优后开始下降
            // 优化点3：加入灰度偏移，j > Pixel_Min + 10 后才检测局部最优提前终止，避免全局最优
            if (deltaTmp < deltaMax && j > Pixel_Min + offset)
            {
                break;//跳出for循环
            }
        }

        // 优化点4+5：通过宏开关控制限幅 + 将last_threshold改为static静态变量，保留历史值
    #if OTSU_USE_LIMIT               // 1=启用OTSU阈值限幅，0=关闭
        if(threshold>OTSU_LIMIT_MIN && threshold<OTSU_LIMIT_MAX)//限幅
            last_threshold = threshold;
        else
            threshold = last_threshold;
    #endif
        return threshold;
}

/**
 * @brief  图像二值化：根据阈值将灰度图转为黑白
 * @param  Threshold  OTSU计算出的阈值
 * @retval 无 (原地修改 base_image[][])
 * @note   0~39行: 阈值+10(更宽容的暗色判定)
 *         40~119行: 直接使用阈值
 */
void base_image_threshold(uint8 Threshold)
{
    int i,j;
    for(i=0;i<40;i++)               // 0~39行：阈值+10，更宽容的暗色判定
    {
        for(j=0;j<MT9V03X_W;j++)
        {
            if(base_image[i][j]<Threshold+10)
                base_image[i][j]=0;
            else
                base_image[i][j]=255;
        }
    }

        for(i=40;i<MT9V03X_H;i++)   // 40行起：直接使用OTSU阈值
    {
        for(j=0;j<MT9V03X_W;j++)
        {
            if(base_image[i][j]<Threshold)
                base_image[i][j]=0;
            else
                base_image[i][j]=255;
        }
    }

}
