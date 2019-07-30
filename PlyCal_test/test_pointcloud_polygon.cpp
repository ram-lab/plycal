/**
  ******************************************************************************
  * @file	test_pointcloud_polygon.cpp
  * @author	Nick.Liao
  * @version V1.0.0
  * @date	2017-12-26
  * @brief	test_pointcloud_polygon.cpp
  ******************************************************************************
  * @attention
  *
  * Copyright (C) 2017 Nick.Liao <simplelife_nick@hotmail.com>
  * Distributed under terms of the MIT license.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string>
#include <iostream>
#include <fstream>


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "json/json.hpp"
#include "pointcloud_polygon.h"

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
	std::string config_path, pc_path;
	std::string edge_prefix;

	if(argc >= 3)
	{
		config_path = std::string(argv[1]);
		pc_path = std::string(argv[2]);
		edge_prefix = std::string(argv[3]);
	}
	else
	{
		std::cerr << "Usage: " << argv[0] << " config_path pointlcoud_path"
				  << std::endl;
		return 1;
	}


	std::cout << "----------------------------------------"<< std::endl
			  << "config      path: " << config_path << std::endl
			  << "pointcloud  path: " << pc_path << std::endl
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
	pcl::PointCloud<pcl::PointXYZI> pc;
	pcl::PointCloud<pcl::PointXYZRGB> pcc;
	if(pcl::io::loadPCDFile(pc_path, pc) == -1)
	{
		std::cerr << "[ERROR]\tfail to read pointcoud" << std::endl;
		return 3;
	}

	lqh::PointcloudPolygon pp(js["pc"], 4);
	pp.Add(pc, pcc);
	pp.SaveEdgePoints(edge_prefix);

	std::cout << "-----------------END----------------\n";

	return 0;
}



/*****************************END OF FILE**************************************/
