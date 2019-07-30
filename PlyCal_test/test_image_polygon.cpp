/**
  ******************************************************************************
  * @file	test_imagepolygon.cpp
  * @author	Nick.Liao
  * @version V1.0.0
  * @date	2017-12-25
  * @brief	test_imagepolygon.cpp
  ******************************************************************************
  * @attention
  *
  * Copyright (C) 2017 Nick.Liao <simplelife_nick@hotmail.com>
  * Distributed under terms of the MIT license.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "image_polygon.h"

#include <iostream>
#include <fstream>

#include <Eigen/Dense>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
  * @brief
  * @param
  * @note
  * @return None
  */
int main(int argc, char** argv)
{
	std::string config_path, img_path;

	if(argc >= 3)
	{
		config_path = std::string(argv[1]);
		img_path = std::string(argv[2]);
	}
	else
	{
		std::cerr << "Usage: " << argv[0] << " config_path image_path"
				  << std::endl;
		return 1;
	}


	std::cout << "----------------------------------------"<< std::endl
			  << "config path: " << config_path << std::endl
			  << "image  path: " << img_path << std::endl
			  << "----------------------------------------" << std::endl;

	// parse config file
	std::ifstream f (config_path);
	if ( !f.good() )
	{
		std::cerr << "[ERROR]\tconfig file doesn't exist" << std::endl;
		return 2;
	}

	std::string content{ std::istreambuf_iterator<char>(f),
						 std::istreambuf_iterator<char>()  };
	f.close();

	auto js= nlohmann::json::parse(content);
	auto& corners = js["img"]["init"];
	Eigen::Matrix3Xd points;
	points.resize(3,4);
	points.row(2).setOnes();

	for(uint8_t i=0; i<4; i++)
	{
		points(0,i) = corners[i][0].get<double>();
		points(1,i) = corners[i][1].get<double>();
	}

    cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);
	cv::Mat img_out;

	lqh::ImagePolygon cvx(js["img"], 4);
	cvx.Init(img, img_out, points);
	lqh::ImagePolygon::Polygon2D::ConstPtr ply = cvx.Add(img, img_out);
	cvx.SaveMarkedImage("ExtractPolygon.jpg", img, ply);

	std::cout << "-----------------END----------------\n";

	return 0;
}


/*****************************END OF FILE**************************************/
