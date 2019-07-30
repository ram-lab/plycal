/**
  ******************************************************************************
  * @file	test_image_polygons.cpp
  * @author	Nick.Liao
  * @version V1.0.0
  * @date	2018-01-02
  * @brief	test_image_polygons.cpp
  ******************************************************************************
  * @attention
  *
  * Copyright (C) 2018 Nick.Liao <simplelife_nick@hotmail.com>
  * Distributed under terms of the MIT license.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "image_polygon.h"

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
	std::string config_path, img_root_path;
	uint32_t num = 0;

	if(argc >= 4)
	{
		config_path = std::string(argv[1]);
		img_root_path = std::string(argv[2]);
		num = std::stoi(std::string(argv[3]));
	}
	else
	{
		std::cerr << "Usage: " << argv[0] << " config_path image_root_path image_num"
				  << std::endl;
		return 1;
	}


	std::cout << "\n----------------------------------------"
			  << "\nconfig path: " << config_path
			  << "\nimage  path: " << img_root_path
			  << "\nimage   num: " << num
			  << "\n----------------------------------------" << std::endl;

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

	std::string img_path = img_root_path + "/0.jpeg";

	cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);
	cv::Mat img_out;

	lqh::ImagePolygon cvx(js["img"], 4);
	cvx.Init(img, img_out, points);


	for(uint32_t i=0; i<num; i++)
	{
		std::cout << "Process: " << i << "\n";
		img_path = img_root_path + "/" + std::to_string(i) + ".jpeg";
		std::string fn_out =  std::to_string(i) + ".png";

		cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);
		lqh::ImagePolygon::Polygon2D::ConstPtr ply = cvx.Add(img, img_out);
		if(ply != nullptr)
		{
			cvx.SaveMarkedImage(fn_out, img, ply);
			std::cout << "-------------OK\n";
		}
		else
		{
			std::cout << "-------------Fail\n";
		}
	}

	std::cout << "-----------------END----------------\n";

	return 0;
}




/*****************************END OF FILE**************************************/
