/**
  ******************************************************************************
  * @file	image_polygon.h
  * @author	Nick.Liao
  * @version V1.0.0
  * @date	2017-12-22
  * @brief	image_polygon.h
  ******************************************************************************
  * @attention
  *
  * Copyright (C) 2017 Nick.Liao <simplelife_nick@hotmail.com>
  * Distributed under terms of the MIT license.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IMAGE_POLYGON_H
#define __IMAGE_POLYGON_H

/* Includes ------------------------------------------------------------------*/

#include <string>
#include <vector>
#include <memory>
#include <stdint.h>

#include <Eigen/Dense>

#include "json/json.hpp"
#include "edlines/EDLineDetector.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

namespace lqh
{
class ImagePolygon
{
public:

	using LineFeature = Eigen::Matrix<uint8_t,3,Eigen::Dynamic>;
	struct Edge2D
	{
		double dir;
		double length;
		Eigen::Vector3d coef;
		LineFeature feature;
	};

	struct Polygon2D
	{
		using ConstPtr = std::shared_ptr<const Polygon2D>;
		using Ptr = std::shared_ptr<Polygon2D>;

        Eigen::Vector3d center;
        Eigen::Matrix3Xd vertexs;
		std::vector<Edge2D> edges;

		Polygon2D(uint32_t size)
		{
			center.setZero();
            vertexs.resize(3, size);
			edges.resize(size);
		}

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	explicit ImagePolygon(const nlohmann::json& js, uint8_t size);
    Polygon2D::ConstPtr Add(const cv::Mat& img, cv::Mat& img_out);
    Polygon2D::ConstPtr Init(const cv::Mat& img, cv::Mat& img_out, const Eigen::Matrix3Xd& pts);
    bool SaveMarkedImage(const std::string& fn, const cv::Mat& img,
								Polygon2D::ConstPtr pyg);
private:
	struct Line2D
	{
		double dir;
		Eigen::Vector3d p0, p1;
		Eigen::Vector3d coef;
		Eigen::Matrix2Xi points;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	const uint32_t size_;	// current implemention only support size_=4
	uint32_t feature_size_;
	uint32_t feature_offset_;
	double feature_threshold_;

	double merge_distance_threshold_;
	double merge_angle_threshold_;
	double merge_endpoint_distance_threshold_;

	double filter_point_line_threshold_;
	double filter_line_angle_threshold_;
	double filter_point_center_factor_;
	double filter_point_center_max_;
	double filter_line_center_min_;

	//@TODO only apply for rectangular board, size_ == 4
	double width_;
	double length_;
	double angle_width_;
	double angle_length_;

	std::unique_ptr<EDLineDetector> detector_;	// line detector
	// edge descripter
	Polygon2D::Ptr polygon_;	// last polygon

    Polygon2D::ConstPtr ExtractPolygon(const cv::Mat& img, cv::Mat& img_out,
                                       Polygon2D::ConstPtr prev);
    Polygon2D::Ptr InitPolygon(const Eigen::Matrix3Xd& pts);
	bool ExtractLines(const cv::Mat& img, std::list<Line2D>& ls);
	bool FilterLines(std::list<Line2D>& ls, Polygon2D::ConstPtr prev);
	bool MergeLines(std::list<Line2D>& ls);
	bool FilterLinesByRectangule(std::list<Line2D>& ls, Polygon2D::ConstPtr prev);
    bool SortLines(std::list<Line2D>& ls, Polygon2D::ConstPtr prev, Polygon2D::Ptr& ply);

    void UpdateParameters();
    void UpdateParameters(Polygon2D::Ptr ply);

	double LineAngleError(double l0, double l1);
	double LineAngleAverage(double l0, double l1);

    void MarkImage(cv::Mat& img_out, const std::list<Line2D>&ls);
    void MarkImage(cv::Mat& img_out, Polygon2D::ConstPtr ply);
	bool SaveMarkedImage(const std::string& fn, const cv::Mat& img,
						 const std::list<Line2D>&ls);
};

}

/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#endif /* !__IMAGE_POLYGON_H */

/*****************************END OF FILE**************************************/
