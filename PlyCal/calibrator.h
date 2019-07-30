/**
  ******************************************************************************
  * @file	calibrator.h
  * @author	Nick.Liao
  * @version V1.0.0
  * @date	2017-12-28
  * @brief	calibrator.h
  ******************************************************************************
  * @attention
  *
  * Copyright (C) 2017 Nick.Liao <simplelife_nick@hotmail.com>
  * Distributed under terms of the MIT license.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CALIBRATOR_H
#define __CALIBRATOR_H

/* Includes ------------------------------------------------------------------*/

#include "stdint.h"
#include <string>
#include <memory>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "image_polygon.h"
#include "pointcloud_polygon.h"

#include "json/json.hpp"

#include "utils.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

namespace lqh
{
class Calibrator
{
public:
	explicit Calibrator(const nlohmann::json& js);
    uint32_t Add(const cv::Mat& img, const pcl::PointCloud<pcl::PointXYZI>& pc,
                 cv::Mat& img_out, pcl::PointCloud<pcl::PointXYZRGB>& pcc);

    bool RefineImage(const cv::Mat& img, cv::Mat& img_out, const std::vector<cv::Point2d>& pts);
    bool RefinePointcloud(const pcl::PointCloud<pcl::PointXYZI>& pc,
                          pcl::PointCloud<pcl::PointXYZRGB>& pcc,
                          const Eigen::Vector4d& param );
    uint32_t SavePolygonData(const std::string& dir);
    void SetCameraK(const Eigen::Matrix3d& k)
    {
        K_ = k;
    }
    void SetTranformation(const Eigen::Matrix4d& tf);
    bool Remove(uint32_t id);
    bool Remove();
    bool Compute(Eigen::Matrix4d& tf);
    bool Compute()
    {
        return Compute(T_);
    }
    const Eigen::Matrix4d& GetTransformation()
    {
        return T_;
    }

    bool ImageGood(uint32_t id)
    {
        if(id >= polygons_v_.size())
        {
            return false;
        }
        return (polygons_v_[id]->img != nullptr);
    }
    bool PointcloudGood(uint32_t id)
    {
        if(id >= polygons_v_.size())
        {
            return false;
        }
        return (polygons_v_[id]->pc != nullptr);
    }

	bool Good()
	{
		return is_valid_;
	}

    bool Project(pcl::PointCloud<pcl::PointXYZRGB>& pc, cv::Mat& img)
    {
        Project(pc, img, T_);
    }
    bool Project(pcl::PointCloud<pcl::PointXYZRGB>& pc, cv::Mat& img,
                 const Eigen::Matrix4d& tf);

private:
	struct Polygon
	{
		/**
		 * ids: corroesponences of 2d/3d Polygon
		 * same edge: $i$th 3d edge  <---> ids[i] 2d edge
         */
		std::vector<uint32_t> ids;
		ImagePolygon::Polygon2D::ConstPtr img;
		PointcloudPolygon::Polygon3D::ConstPtr pc;

        Polygon(uint32_t size):img(nullptr), pc(nullptr)
		{
			ids.resize(size);
			for(uint32_t i=0; i<size; i++)
			{
				ids[i] = i;
			}
		}
	};

	bool is_valid_;
	uint32_t size_;
	double track_error_threshold_;
    uint32_t img_width_;
    uint32_t img_height_;
	Eigen::Matrix3d K_;
	Eigen::Matrix4d T_;
	std::unique_ptr<ImagePolygon> imgply_;
	std::unique_ptr<PointcloudPolygon> pcply_;
    std::vector<Polygon*> polygons_v_;
	std::list<Polygon> polygons_;

	void TrackLines(const Polygon& ply_prev, Polygon& ply) const;
	void MatchLines(Polygon& ply) const;
    void Optimize(Eigen::Matrix4d& tf);
};
}

/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#endif /* !__CALIBRATOR_H */

/*****************************END OF FILE**************************************/
