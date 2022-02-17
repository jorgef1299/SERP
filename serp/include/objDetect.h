#pragma once
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

const double pi = 3.14159265358979323846;

using namespace std;
using namespace cv;


//contantes para dete��o de objetos
const int minThreshold = 140;
const int threshMultiplier = 3;
const int kernelSize = 3;
const int minRadiusFilter = 40;

//defeni��o dos pixeis limite para a segmenta��o da imagem
// 
//frente
const int frontBeginning = 90;
const int frontEnd = 420;

//tr�s
const int backBeginning = 1200;
const int backEnd = 300;

//direita
const int rightEnd = 330;

//esquerda
const int leftBeginning = 880;

//limites pilares

const int leftPilar = 470;
const int rightPilar = leftPilar + 450;

//limites robo
const int rFront = frontBeginning + frontEnd;
const int rRear = backBeginning;



//inicializa��es vari�veis de sensores
uint8_t N = 0, NE = 0, NW = 0, S = 0, SE = 0, SW = 0;
// N=Frente
// NE=Frente Direita
// NW=Frente Esquerda
// S=Tr�s
// SE=Tr�s Esquerda
// SW=Tr�s Direita

//fun��o que calcula o n�vel de per�go de objeto dependendo da sua dist�ncia ao robo. 0 min, 3 max
int levelDanger(float n) 
{
    if ((n <= 420) && (n > 200))
        return 1;
    
    else if ((n <= 200) && (n > 100))
        return 2;
    
    else if ((n <= 100) && (n > 0))
        return 3;

    return 0;
    
}

//filtra os objetos detetados, e identifica qual o mais pr�ximo, onde calcula o seu perigo
void sortDetectedObjects(String pos, Mat img, vector<Point2f> centers, vector<float> radius)
{
    int height = img.rows;
    int width = img.cols;

    float distQuant = 0;

    if (pos.compare("N") == 0) {

        for (size_t i = 0; i < centers.size(); i++)
        {

            if ((float)radius[i] > minRadiusFilter) {

                if (levelDanger(abs(centers[i].x - rFront)) > N) {
                    N = levelDanger(abs(centers[i].x - rFront));
                }
                
            }
        }

    }
    else if (pos.compare("S") == 0) {

        for (size_t i = 0; i < centers.size(); i++)
        {
            if ((float)radius[i] > minRadiusFilter) {

                if (levelDanger(abs(centers[i].x - 0)) > S) {
                    S = levelDanger(abs(centers[i].x - 0)+50);
                }

            }
        }

    }
    else if (pos.compare("NE") == 0) {

        for (size_t i = 0; i < centers.size(); i++)
        {
            if ((float)radius[i] > minRadiusFilter) {

                distQuant = sqrt(pow(abs(centers[i].x - leftPilar), 2) + pow(abs(centers[i].y - 400), 2));

                if (levelDanger(distQuant) > NE) {
                    NE = levelDanger(distQuant);
                }

            }
        }

    }
    else if (pos.compare("NW") == 0) {

        for (size_t i = 0; i < centers.size(); i++)
        {
            if ((float)radius[i] > minRadiusFilter) {

                distQuant = sqrt(pow(abs(centers[i].x - leftPilar), 2) + pow(abs(centers[i].y - 400), 2));

                if (levelDanger(distQuant) > NW) {
                    NW = levelDanger(distQuant);
                }

            }
        }

    }
    else if (pos.compare("SE") == 0) {

        for (size_t i = 0; i < centers.size(); i++)
        {
            if ((float)radius[i] > minRadiusFilter) {

                distQuant = sqrt(pow(abs(centers[i].x - 0), 2) + pow(abs(centers[i].y - 350), 2));

                if (levelDanger(distQuant) > SE) {
                    SE = levelDanger(distQuant);
                }

            }
        }

    }
    else if (pos.compare("SW") == 0) {

        for (size_t i = 0; i < centers.size(); i++)
        {
            if ((float)radius[i] > minRadiusFilter) {

                distQuant = sqrt(pow(abs(centers[i].x - 0), 2) + pow(abs(centers[i].y - 350), 2));

                if (levelDanger(distQuant) > SW) {
                    SW = levelDanger(distQuant);
                }

            }
        }

    }

}

//trabalho incacabado de elemina��o de circulos dentro de circulos
bool inRadious(Point2f centerOrigin, Point2f center2, int r) {

    float distBetweenCenters = sqrt(pow(abs((double)centerOrigin.x- center2.x), 2) + pow(abs((double)centerOrigin.y - center2.y), 2));

    if (distBetweenCenters <= r)
        return true;
    else
        return false;
}


//Encontrar objetos na imagem, e guardar os centros e os raios em vatores
void detectObj(Mat img, int lowThreshold, int threshMult, int kernel_size, String pos)
{
    Mat equalized, detected_edges;

    float sumX = 0, sumY = 0, sumR = 0;
    int num = 0;

    equalizeHist(img, equalized);
    Canny(equalized, detected_edges, lowThreshold, lowThreshold * threshMult, kernel_size);

    vector<vector<Point> > contours;
    findContours(detected_edges, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> boundRect(contours.size());
    vector<Point2f> centers(contours.size());
    vector<float> radius(contours.size());
    for (size_t j = 0; j < contours.size(); j++)
    {
        approxPolyDP(contours[j], contours_poly[j], 3, true);
        boundRect[j] = boundingRect(contours_poly[j]);
        minEnclosingCircle(contours_poly[j], centers[j], radius[j]);
    }
    //Mat drawing = Mat::zeros(detected_edges.size(), CV_8UC3);

    int i = 0;
    for (size_t i = 0; i < contours.size(); i++) {

        sumX = centers[i].x;
        sumY = centers[i].y;
        sumR = radius[i];
        num = 1;

        for (size_t j = 0; j < contours.size(); j++)
        {
            
            
            if ((i != j)) {

                if ((radius[j] != 0) && (inRadious(centers[i], centers[j], radius[i]))) {
                    sumX += centers[j].x;
                    sumY += centers[j].y;
                    sumR += radius[j];
                    radius[j] = 0;
                    num += 1;
                }
                
            }

        }

    }



    /* for (size_t j = 0; j < contours.size(); j++)
    {

        if ((int)radius[j] > minRadiusFilter) {
            rectangle(img, boundRect[j].tl(), boundRect[j].br(), color, 2);
            circle(img, centers[j], (int)radius[j], color, 2);
        }
    }*/

    sortDetectedObjects(pos, img, centers, radius);

    /*cv::imshow("Image ", img);
    cv::waitKey(0);*/
}


//fun��o que pega em cada frame do video, aplica pre processamento, segmenta a imagem e deteta os objetos em cada segmento da imagem
std::vector<uint8_t> frameProcessing(Mat frame) {



    N = 0;
    NW = 0;
    NE = 0;
    SE = 0;
    SW = 0;
    S = 0;

    int width = frame.cols;
    int height = frame.rows;
    Mat img_grey;
    Mat detected_edges;
    //preprocessing on image
    cvtColor(frame, img_grey, COLOR_BGR2GRAY);
    blur(img_grey, detected_edges, Size(3, 3));

    //segmentacao dos frames
    //ROI(pixelHorizontalInicio, PixelVerticalInicio, QuantidadePixeisHorizontal, QuantidadePixeisVertical)
    Rect ROI_NE(frontBeginning, 1, leftPilar, 400);
    Mat image_NE = img_grey(ROI_NE);

    Rect ROI_N(frontBeginning, 400, frontEnd, 400);
    Mat image_N = img_grey(ROI_N);

    Rect ROI_NW(frontBeginning, 800, leftPilar, 400);
    Mat image_NW = img_grey(ROI_NW);

    Rect ROI_SE(rightPilar, 1, 600, 350);
    Mat image_SE = img_grey(ROI_SE);

    Rect ROI_S(backBeginning, 350, backEnd, 450);
    Mat image_S = img_grey(ROI_S);

    Rect ROI_SW(rightPilar, 850, 600, 250);
    Mat image_SW = img_grey(ROI_SW);

    //deteta os objetos com os thresholds, e com o identificador de que parte �.
    detectObj(image_NE, minThreshold, threshMultiplier, kernelSize, "NE");
    detectObj(image_N, minThreshold, threshMultiplier, kernelSize, "N");
    detectObj(image_NW, minThreshold, threshMultiplier, kernelSize, "NW");
    //detectObj(image_SE, minThreshold, threshMultiplier, kernelSize, "SE");
    detectObj(image_S, minThreshold, threshMultiplier, kernelSize, "S");
    //detectObj(image_SW, minThreshold, threshMultiplier, kernelSize, "SW");

    //vetor para retornar valores dos sensores
    std::vector<uint8_t> retorna_valores{NE , N , S, NW};


    /*ROS_INFO("direta: %d", retorna_valores[0]);
    ROS_INFO("frente: %d", retorna_valores[1]);
    ROS_INFO("tras: %d", retorna_valores[2]);
    ROS_INFO("esquerda: %d", retorna_valores[3]);
    ROS_INFO("----------------------");*/


    return retorna_valores;

}
