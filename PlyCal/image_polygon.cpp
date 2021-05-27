/**
  ******************************************************************************
  * @file	image_polygon.cpp
  * @author	Nick.Liao
  * @version V1.0.0
  * @date	2017-12-22
  * @brief	image_polygon.cpp
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
#include <cmath>

#include "utils.h"

using namespace lqh;

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
ImagePolygon::ImagePolygon(const nlohmann::json& js, uint8_t size):
	size_(size), polygon_(nullptr)
{
	//!!! we don't set default
	// config line detector
	auto ed = js["edlines"];
	EDLineParam para;

	para.minLineLen = ed["minLineLen"];
	para.lineFitErrThreshold = ed["lineFitErrThreshold"];
	para.ksize = ed["ksize"];
	para.sigma = ed["sigma"];
	para.gradientThreshold = ed["gradientThreshold"];
	para.scanIntervals = ed["scanIntervals"];
	para.anchorThreshold = ed["anchorThreshold"];

	detector_.reset(new EDLineDetector(para));

	feature_size_ = js["feature"]["size"];
	feature_offset_ = js["feature"]["offset"];
	feature_threshold_ = js["feature"]["threshold"];

	merge_distance_threshold_ = js["merge"]["distance_threshold"];
	merge_angle_threshold_ = js["merge"]["angle_threshold"];
	merge_endpoint_distance_threshold_ = js["merge"]["endpoint_distance_threshold"];

	filter_point_line_threshold_  = js["filter"]["point_line_threshold"];
	filter_line_angle_threshold_  = js["filter"]["line_angle_threshold"];
	filter_point_center_factor_    = js["filter"]["point_center_factor"];
	if(filter_point_center_factor_ >1 || filter_point_center_factor_ <0)
	{
		std::cout << "W: point_dis_factor in (0,1)  get " <<
				  filter_point_center_factor_ << "\n";
		filter_point_center_factor_ = 0.2;
	}
	// random
	// center to line distance
	filter_point_center_max_ = 100;
	filter_line_center_min_ = 50;

	width_ = 0;
	length_ = 0;
	angle_width_ = 0;
	angle_length_ = 0;

	Eigen::Matrix3Xd pts(3,4);
	pts.row(2).setOnes();
	for(uint8_t i=0; i<4; i++)
	{
		pts(0,i) = js["init"][i][0].get<double>();
		pts(1,i) = js["init"][i][1].get<double>();
	}
	polygon_ = InitPolygon(pts);

}

ImagePolygon::Polygon2D::Ptr ImagePolygon::InitPolygon(const Eigen::Matrix3Xd& pts)
{
	// currently, only apply to rectangular
	// @TODO
	assert(pts.cols() == 4);
	assert(pts.cols() == size_ );

	using PairUD = std::pair<uint32_t, double>;
	// sort the points(clock wise)
	Eigen::Vector3d center = pts.rowwise().mean();
	center(2) = 1;
	std::vector<PairUD> verts( size_ );

	for(uint32_t i=0; i<size_; i++)
	{
		verts[i].first = i;
		verts[i].second = std::atan2(pts(1,i)-center(1), pts(0,i)-center(0)) + MATH_PI;
	}
	std::sort(verts.begin(), verts.end(), [](PairUD& a, PairUD& b)
	{
		return a.second < b.second;
	});

	Polygon2D::Ptr ply = std::make_shared<Polygon2D>(size_);
	ply->center = center;

	auto EistimateCoef = [](const Eigen::Vector3d& p0,
							const Eigen::Vector3d& p1,
							Eigen::Vector3d& coef)
	{
		Eigen::Vector3d dir = (p0-p1).normalized();
		Eigen::Vector3d c = (p0+p1)/2;

		coef(0) = -1*dir(1);
		coef(1) = dir(0);
		coef(2) = -1*(coef(0)*c(0) + coef(1)*c(1));
	};

	for(uint32_t i=0; i<size_; i++)
	{
		uint32_t next = (i== size_-1) ? 0: i+1;
		ply->vertexs.col(i) = pts.col(verts[i].first);
		EistimateCoef(pts.col(verts[i].first), pts.col(verts[next].first),
					  ply->edges[i].coef);
		ply->edges[i].length = (pts.col(verts[i].first) - pts.col(verts[next].first)).norm();
		double dir = std::atan2(-ply->edges[i].coef(0), ply->edges[i].coef(1))/MATH_PI*180;
		ply->edges[i].dir = (dir<0)?dir+180:dir;
		//ExtractLineFeature(img, pts.col(verts[i].first), pts.col(verts[next].first),
		//center, polygon_->edges[i].feature);
	}

	UpdateParameters(ply);

	return ply;
}

ImagePolygon::Polygon2D::ConstPtr ImagePolygon::Init(const cv::Mat& img,
		cv::Mat& img_out, const Eigen::Matrix3Xd& pts)
{
	Polygon2D::Ptr ply = InitPolygon(pts);
//    SaveMarkedImage("init.png", img, ply);
	return ExtractPolygon(img, img_out,ply);
}

ImagePolygon::Polygon2D::ConstPtr ImagePolygon::Add(const cv::Mat& img, cv::Mat& img_out)
{
	if(polygon_ == nullptr)
	{
		std::cerr << "E: init first\n";
		return nullptr;
	}
	return ExtractPolygon(img, img_out, polygon_);
}

ImagePolygon::Polygon2D::ConstPtr ImagePolygon::ExtractPolygon(const cv::Mat& img,
		cv::Mat& img_out, Polygon2D::ConstPtr prev)
{
	// we require color image
	assert(img.channels() == 3);
	std::list<Line2D> lines;
	Polygon2D::Ptr ply;

	bool res = ExtractLines(img, lines);
	if(!res)
	{
		MarkImage(img_out, lines);
		return nullptr;
	}
//    SaveMarkedImage("ExtractLines.png", img, lines);

	res &= FilterLines(lines, prev);
	if(!res)
	{
		MarkImage(img_out, lines);
		return nullptr;
	}
//    SaveMarkedImage("FilterLines.png", img, lines);

	res &= MergeLines(lines);
	if(!res)
	{
		MarkImage(img_out, lines);
		return nullptr;
	}
//    SaveMarkedImage("EMergeLines.png", img, lines);

	res &= FilterLinesByRectangule(lines, prev);
	if(!res)
	{
		MarkImage(img_out, lines);
		return nullptr;
	}
//    SaveMarkedImage("FilterLinesByRectangule.png", img, lines);

	res &= SortLines(lines, prev, ply);
	if(res)
	{
		polygon_ = ply;
		UpdateParameters();
		MarkImage(img_out, ply);
		return ply;
	}
	else
	{
		return nullptr;
	}
}

bool ImagePolygon::ExtractLines(const cv::Mat& img, std::list<Line2D>& ls)
{
	//cv::Mat img_hsv;
	//cv::Mat imgs[3];

	//cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);
	//cv::split(img_hsv, imgs);
	//auto& img_h = imgs[2];

	//ls.clear();


	//// set circle
	//double x = polygon_->center(0);
	//double y = polygon_->center(1);
	//uint16_t r_max = static_cast<uint16_t>(filter_point_center_max_);
	//uint16_t r_min = static_cast<uint16_t>(filter_line_center_min_);

	//std::cout << "cx: " << x <<" cy: " << y
	//<< "\n r_max: " << r_max << "  r_min: " << r_min << std::endl;

	////cv::Mat img_mask(img_h.rows, img_h.cols, CV_8U);
	////cv::circle(img_mask, cv::Point2d(x,y), r_max, cv::Scalar(255), -1);
	////cv::Mat img_roi;
	////img_h.copyTo(img_roi, img_mask);


	//double sum = 0;
	//uint8_t p_max = 0;
	//uint8_t p_min = 255;

	//for(uint16_t i=y-r_min; i<y+r_min; i++ )
	//{
	//for(uint16_t j=x-r_min; j<x+r_min; j++)
	//{
	//const auto& pixel = img_h.at<uint8_t>(i,j);
	//sum += pixel;
	//if( p_max < pixel)
	//{
	//p_max = pixel;
	//}
	//if(p_min > pixel)
	//{
	//p_min = pixel;
	//}
	//}
	//}
	//uint8_t average = static_cast<uint8_t>(sum/(4*r_min*r_min));
	//std::cout << "average: " << static_cast<int>(average)
	//<< "  p_min: " << static_cast<int>(p_min)
	//<< " p_max: " << static_cast<int>(p_max) << std::endl;

	//p_max += 4;
	//p_min -= 4;

	//for(uint16_t i=0; i<img.rows; i++)
	//{
	//for(uint16_t j=0; j<img.cols; j++)
	//{
	//double d = (i-y)*(i-y) + (j-x)*(j-x);
	//d = std::sqrt(d);
	//auto& pixel = img_h.at<uint8_t>(i,j);

	//if(d >= filter_point_center_max_)
	//{
	//pixel = 0;
	//}
	//else
	//{
	//if(pixel  > p_max)
	//{
	//pixel = 255 - (p_max - pixel);
	////pixel = 0;
	//}
	//else if(pixel < p_min)
	//{
	//pixel = 255 - (pixel-p_min);
	////pixel = 0;
	//}
	//else
	//{
	//pixel = 255;
	//}
	//}
	//}
	//}

	//cv::threshold(img_h, img_h,0,180,cv::THRESH_BINARY+cv::THRESH_OTSU);
	//img_h = img_h*255/180;

	cv::Mat img_h;
	cv::cvtColor(img, img_h, cv::COLOR_BGR2GRAY);
//	cv::imwrite("img_gray.png", img_h);

	if(detector_->EDline(img_h) < 0 )
	{
		std::cerr << "E: EDline detector fail to detect lines in the image\n";
		return false;
	}

	for (uint16_t i =0; i<detector_->lineEquations_.size(); i++)
	{
		ls.emplace_back();
		Line2D& last = ls.back();

		// ax+by+c=0  direction vector=(-b a)
		last.dir = std::atan2(-detector_->lineEquations_[i][0],
							  detector_->lineEquations_[i][1])/MATH_PI*180;

		if(last.dir<0)
		{
			last.dir += 180;
		}

		last.p0  << detector_->lineEndpoints_[i][0],
				 detector_->lineEndpoints_[i][1],
				 1;
		last.p1 << detector_->lineEndpoints_[i][2],
				detector_->lineEndpoints_[i][3],
				1;
		last.coef << detector_->lineEquations_[i][0],
				  detector_->lineEquations_[i][1],
				  detector_->lineEquations_[i][2];
		last.coef = last.coef/(last.coef.head(2).norm());

		last.points.resize(2, detector_->lines_.sId[i+1]-detector_->lines_.sId[i]);

		uint16_t col_index = 0;
		for(uint16_t j=detector_->lines_.sId[i]; j<detector_->lines_.sId[i+1]; j++)
		{
			last.points.col(col_index++) << detector_->lines_.xCors[j],
							detector_->lines_.yCors[j];
		}
	}

	return true;
}

/**
 * filter criteria
 * 1. center to endpoint distances
 * 2. center to line distance
 * 3. endpoints to line distances
 * 4. line direction angle error
 */
bool ImagePolygon::FilterLines(std::list<Line2D>& ls, Polygon2D::ConstPtr prev)
{
	if(ls.size() < size_ )
	{
		return false;
	}
	auto l = ls.begin();
	while( l != ls.end() )
	{
		auto ln = l;
		l++;
		double center2point_0 = (prev->center - ln->p0).norm();
		double center2point_1 = (prev->center - ln->p1).norm();
		double center2line = std::abs(prev->center.dot(ln->coef));

		if( center2point_0 > filter_point_center_max_ ||
				center2point_1 > filter_point_center_max_ ||
				center2line < filter_line_center_min_  )
		{
			ls.erase( ln );
			continue;
		}

		bool is_remove = true;
		for(uint32_t i=0; i<size_; i++)
		{
			double d = std::abs(ln->p0.dot(prev->edges[i].coef));
			d += std::abs(ln->p1.dot(prev->edges[i].coef));
			double angle = std::abs(ln->dir - prev->edges[i].dir);
			if(angle > 90)
			{
				angle = 180 - angle;
			}
			if(angle < filter_line_angle_threshold_ &&
					d < filter_point_line_threshold_)
			{
				is_remove = false;
			}
		}

		if(is_remove )
		{
			ls.erase( ln );
		}
	}

	return ls.size() >= size_ ? true : false;
}

/**
 * @brief merge all paralle lines
 * @param ls
 * @return
 */
bool ImagePolygon::MergeLines(std::list<Line2D>& ls)
{
	if(ls.size() < size_)
	{
		std::cerr << "E: no enough line segments for " << __FUNCTION__
				  << "\n   get " << ls.size() << " lines\n";
		return false;
	}

	auto ln = ls.begin();
	while(ln != ls.end())
	{
		bool flag = false;
		auto ln_nx = std::next(ln);
		while(ln_nx != ls.end())
		{
			// distance error
			double d = std::abs(ln->p0.transpose()*ln_nx->coef);
			d += std::abs(ln->p1.transpose()*ln_nx->coef);
			d += std::abs(ln_nx->p0.transpose()*ln->coef);
			d += std::abs(ln_nx->p1.transpose()*ln->coef);

			Eigen::Vector4d e2e;
			e2e(0) = (ln->p0 - ln_nx->p0).norm();
			e2e(1) = (ln->p0 - ln_nx->p1).norm();
			e2e(2) = (ln->p1 - ln_nx->p0).norm();
			e2e(3) = (ln->p1 - ln_nx->p1).norm();

			// direction angle error
			double angle = std::abs(ln->dir - ln_nx->dir);
			if(angle > 90)
			{
				angle = 180 - angle;
			}

			if(angle < merge_angle_threshold_ &&
					d < merge_distance_threshold_ &&
					e2e.minCoeff() < merge_endpoint_distance_threshold_)
			{
				// merge two line segement
				flag = true;

				// find endpoints
				Eigen::Matrix3Xd endpoint(3,4);
				endpoint.col(0) = ln->p0;
				endpoint.col(1) = ln->p1;
				endpoint.col(2) = ln_nx->p0;
				endpoint.col(3) = ln_nx->p1;
				uint32_t axis = 0; // x-axis
				if(std::abs(ln->dir - 90) < 10)
				{
					axis = 1; // y-axis
				}

				uint8_t id_max =0, id_min = 1;
				for(uint8_t i=0; i<4; i++)
				{
					if(endpoint(axis, i) > endpoint(axis, id_max))
					{
						id_max = i;
					}
					else if (endpoint(axis, i) < endpoint(axis, id_min))
					{
						id_min = i;
					}
				}
				ln->p0 = endpoint.col(id_min);
				ln->p1 = endpoint.col(id_max);

				ln->points.conservativeResize(Eigen::NoChange,
											  ln->points.cols()+ln_nx->points.cols());
				ln->points.topRightCorner(2,ln_nx->points.cols()) = ln_nx->points;

				// erase node
				ln_nx++;
				ls.erase(std::prev(ln_nx));
			}
			else
			{
				ln_nx++;
			}
		}

		if(flag)
		{
			ln->dir = -1;
			ln->coef.setZero();
		}

		ln++;
	}


	if(ls.size() < size_)
	{
		std::cerr << "E: no enough line segments after merging\n"
				  << "   get " << ls.size() << " lines\n";
		return false;
	}

	// re-fit lines
	for(auto&ln : ls)
	{
		if(ln.dir >=0 )
		{
			continue;
		}

		std::vector<cv::Point2d> pts;
		pts.reserve(ln.points.cols());
		for(uint32_t i=0; i<ln.points.cols(); i++)
		{
			pts.emplace_back(ln.points(0,i), ln.points(1,i));
		}

		std::vector<double> coef(4, 0);
		cv::fitLine(pts, coef, cv::DIST_L1,0, 0.01, 0.01);

		ln.coef(0) = coef[1];
		ln.coef(1) = -coef[0];
		ln.coef(2) = -1*(ln.coef(0)*coef[2] + ln.coef(1)*coef[3]);
		ln.coef = ln.coef/(ln.coef.head(2).norm());
		ln.dir = std::atan2(coef[1], coef[0])/MATH_PI*180;
		if(ln.dir < 0)
		{
			ln.dir += 180;
		}
	}

	return true;
}

//@TODO only apply to rectangular board
bool ImagePolygon::FilterLinesByRectangule(std::list<Line2D>& ls, Polygon2D::ConstPtr prev)
{
	assert(ls.size()>=4 );
	if(ls.size() < size_)
	{
		std::cerr << "E: no enough lines for " << __FUNCTION__
				  << "\n get " << ls.size() << "lines \n";
		return false;
	}

	struct RectangleEdges
	{
		double error;
		uint32_t id0;
		uint32_t id1;

		RectangleEdges(double error_, uint32_t id0_, uint32_t id1_):
			error(error_), id0(id0_), id1(id1_) {}
	};

	std::list<RectangleEdges> width;
	std::list<RectangleEdges> length;

	uint32_t id0 = 0;
	for(auto l=ls.cbegin(); l!=ls.cend(); l++)
	{
		uint32_t id1 = id0+1;
		Eigen::Vector3d ep2c = prev->center - (l->p0 + l->p1)/2;
		for(auto next=std::next(l); next!=ls.cend(); next++)
		{
			double err_angle = LineAngleError(l->dir, next->dir);
			if( err_angle < 10)
			{
				// parallel line
				double dis = std::abs(next->coef.dot(l->p0));
				dis += std::abs(next->coef.dot(l->p1));
				dis += std::abs(l->coef.dot(next->p0));
				dis += std::abs(l->coef.dot(next->p1));
				dis = dis/4;

				Eigen::Vector3d c2c = (next->p0+next->p1)/2 - (l->p0 + l->p1)/2 ;
				c2c.normalize();
				double err_c2c = c2c.cross(ep2c).norm();

				double angle_width = LineAngleError(angle_width_,
													LineAngleAverage(l->dir,next->dir));
				double angle_length = LineAngleError(angle_length_,
													 LineAngleAverage(l->dir, next->dir));
				if(angle_width > angle_length)
				{
					double dis_rate = std::abs(dis - width_)/width_;
					if(dis_rate < 0.15)
					{
						length.emplace_back(dis_rate*err_angle*err_c2c, id0, id1);
					}
				}
				else
				{
					double dis_rate = std::abs(dis - length_)/length_;
					if(dis_rate < 0.15)
					{
						width.emplace_back(dis_rate*err_angle*err_c2c, id0, id1);
					}
				}
			}
			id1++;
		}
		id0++;
	}

	if(width.size()<1 || length.size() <1)
	{
		std::cerr << "E: FilterLinesByRectangule width: " << width.size()
				  << " length: " << length.size() << std::endl;
		return false;
	}

	auto RectangleEdgesSort = [](const RectangleEdges& a, const RectangleEdges& b)
	{
		return a.error < b.error;
	};

	if(width.size() > 1)
	{
		width.sort(RectangleEdgesSort);
	}
	if(length.size() > 1)
	{
		length.sort(RectangleEdgesSort);
	}

	std::set<uint32_t> edges;
	edges.insert(width.front().id0);
	edges.insert(width.front().id1);
	edges.insert(length.front().id0);
	edges.insert(length.front().id1);

	id0 = 0;
	auto ln = ls.begin();
	while(ln != ls.end())
	{
		ln++;
		if(edges.count(id0) == 0)
		{
			ls.erase(std::prev(ln));
		}
		id0++;
	}
	return true;
}

bool ImagePolygon::SortLines(std::list<Line2D>& ls, Polygon2D::ConstPtr prev, Polygon2D::Ptr& ply )
{
	if(ls.size() != size_)
	{
		std::cerr << "E: too many/fewer line segments to sort\n"
				  << "   get " << ls.size() << " lines\n";
		return false;
	}

	using PairUD = std::pair<uint32_t,double>;
	Eigen::Vector3d center;

	for(const auto& it:ls)
	{
		center += it.p0;
		center += it.p1;
	}
	center = center / (ls.size()*2);

	for(auto& it:ls)
	{
		Eigen::Vector3d mid = (it.p0 + it.p1)/2;
		it.dir = std::atan2(mid(1)-center(1), mid(0)-center(0)) + MATH_PI;
	}
	ls.sort([](Line2D& a, Line2D& b)
	{
		return a.dir < b.dir;
	});


	// find all vertexs
	std::vector<Eigen::Vector3d>vertexs(size_);
	{
		auto l = ls.cbegin();
		for(auto& it:vertexs)
		{
			Eigen::Matrix2d A;
			Eigen::Vector2d B;

			A.row(0) = l->coef.head(2);
			B(0) = -1*l->coef(2);
			if(l != ls.cbegin())
			{
				auto pre = std::prev(l);
				A.row(1) = pre->coef.head(2);
				B(1) = -1*pre->coef(2);
			}
			else
			{
				A.row(1) = ls.back().coef.head(2);
				B(1) = -1*ls.back().coef(2);
			}

			it.head(2) = A.inverse()*B;
			it(2) = 1;

			l++;
		}
	}

	// find offset, index correspondence
	uint32_t id_cors = 0;
	{
		double error = INT_MAX;
		auto& l0 = ls.front();
		double dir = std::atan2(-l0.coef(0), l0.coef(1))/MATH_PI*180;
		if(dir < 0)
		{
			dir += 180;
		}
		for(uint32_t i=0; i<size_; i++)
		{
			double err = std::abs(prev->edges[i].dir - dir)*2;
			err += std::abs(l0.p0.dot(prev->edges[i].coef));
			err += std::abs(l0.p1.dot(prev->edges[i].coef));
			if(err < error)
			{
				id_cors = i;
			}
		}
	}

	ply = std::make_shared<Polygon2D>(size_);
	uint32_t id = 0;
	for(const auto& it: ls)
	{
		uint32_t pindex = (id+id_cors)%size_;

		ply->edges[pindex].coef = it.coef;
		double dir = std::atan2(-it.coef(0), it.coef(1))/MATH_PI*180;
		if(dir < 0)
		{
			dir += 180;
		}
		ply->edges[pindex].dir = dir;
		Eigen::Vector3d p01 = vertexs[id] - vertexs[(id+1)%size_];
		ply->edges[pindex].length = p01.head(2).norm();
		ply->vertexs.col(pindex) = vertexs[id];

		id++;
	}

	return true;
}


void ImagePolygon::UpdateParameters()
{
	UpdateParameters(polygon_);
}

void ImagePolygon::UpdateParameters(Polygon2D::Ptr ply)
{
	// update threshold
//	Eigen::Vector3d center;
//	center.setZero();

//    for(auto& it: ply->vertexs)
//	{
//		center += it;
//	}
//    center = center/size_;

	Eigen::Vector3d center = ply->vertexs.rowwise().mean();

	ply->center = center;


	Eigen::VectorXd center2point(size_);
	Eigen::VectorXd center2line(size_);
	for(uint32_t i=0; i<size_; i++)
	{
		center2point(i) = (center - ply->vertexs.col(i)).norm();
		center2line(i) = std::abs( center.dot(ply->edges[i].coef) );
	}
	filter_point_center_max_ = center2point.maxCoeff()*(1+filter_point_center_factor_);
	filter_line_center_min_ = center2line.minCoeff()*(1-filter_point_center_factor_);


	//@TODO
	double dis0=0, dis1=0;
	dis0 += std::abs(ply->edges[2].coef.dot(ply->vertexs.col(0) ));
	dis0 += std::abs(ply->edges[2].coef.dot(ply->vertexs.col(1) ));
	dis0 += std::abs(ply->edges[0].coef.dot(ply->vertexs.col(2) ));
	dis0 += std::abs(ply->edges[0].coef.dot(ply->vertexs.col(3) ));
	dis0 = dis0 / 4;
	double angle1 = LineAngleAverage(ply->edges[2].dir, ply->edges[0].dir);


	dis1 += std::abs(ply->edges[1].coef.dot(ply->vertexs.col(0) ));
	dis1 += std::abs(ply->edges[1].coef.dot(ply->vertexs.col(3) ));
	dis1 += std::abs(ply->edges[3].coef.dot(ply->vertexs.col(1) ));
	dis1 += std::abs(ply->edges[3].coef.dot(ply->vertexs.col(2) ));
	dis1 = dis1 / 4;
	double angle0 = LineAngleAverage(ply->edges[1].dir, ply->edges[3].dir);

	if(dis0 < dis1)
	{
		width_ = dis0;
		length_ = dis1;
		angle_width_ = angle0;
		angle_length_ = angle1;
	}
	else
	{
		width_ = dis1;
		length_ = dis0;
		angle_width_ = angle1;
		angle_length_ = angle0;
	}


	std::cout << "width: " << width_ << " length: " << length_
			  << "\n angle_width: " << angle_width_
			  << " angle_length: " << angle_length_ << "\n";

}


double ImagePolygon::LineAngleError(double l0, double l1)
{
	double e = std::abs(l0 - l1);
	if(e >90)
	{
		e = 180 - e;
	}
	return e;
}


double ImagePolygon::LineAngleAverage(double l0, double l1)
{
	double e = (l0+l1)/2;
	if(std::abs(l0 - l1) > 90)
	{
		e = (l0+l1-180)/2;
	}
	return e;
}

void ImagePolygon::MarkImage(cv::Mat &img, const std::list<Line2D> &ls)
{
	utils::color::rgbs colors = utils::color::get_rgbs(ls.size());
	auto lines_it = ls.cbegin();

	for (std::size_t i = 0; i < ls.size(); i++)
	{
		auto& it = *lines_it;
		cv::Scalar color(colors[i][0], colors[i][1], colors[i][2]);
		cv::Point2d p0(it.p0(0), it.p0(1));
		cv::Point2d p1(it.p1(0), it.p1(1));

		cv::line(img, p0, p1, color, 2 );
		cv::circle(img, p0, 5, color);
		cv::circle(img, p1, 4, color);
		cv::putText(img, std::to_string(i),(p0+p1)/2,
					cv::FONT_HERSHEY_SIMPLEX, 2, color, 2 );
		lines_it++;
	}

	if(polygon_ != nullptr)
	{
		const auto& c = polygon_->center;
		cv::circle(img, cv::Point2d(c(0), c(1)), filter_line_center_min_,
				   cv::Scalar(0,0,255), 2);
		cv::circle(img, cv::Point2d(c(0), c(1)), filter_point_center_max_,
				   cv::Scalar(0,255,0), 2);
	}
}

void ImagePolygon::MarkImage(cv::Mat& img, Polygon2D::ConstPtr ply)
{
	utils::color::rgbs colors = utils::color::get_rgbs( size_ );
	for (uint32_t i = 0; i < size_; i++)
	{
		uint32_t next = (i== size_-1)?0:i+1;
		auto& it = ply->vertexs;
		auto& ln = ply->edges[i];

		cv::Scalar color(colors[i][2], colors[i][1], colors[i][0]);
		cv::Point2d p0( it(0,i), it(1,i) );
		cv::Point2d p1( it(0,next), it(1, next) );
		// draw id
		cv::putText(img, std::to_string(i),(p0+p1)/2,
					cv::FONT_HERSHEY_SIMPLEX, 2, color, 2 );
		// draw vertex
		if(i == 0)
		{
			cv::circle(img, p0, 8, color, 3);
		}
		else
		{
			cv::circle(img, p0, 5, color);
		}

		// draw line
		if( std::abs(ln.dir - 90 ) < 45)
		{
			p0.y = 0;
			p0.x = -1*ln.coef(2)/ln.coef(0);
			p1.y = img.rows;
			p1.x = -1*(ln.coef(2) + ln.coef(1)*p1.y)/ln.coef(0);
		}
		else
		{
			p0.x = 0;
			p0.y = -1*ln.coef(2)/ln.coef(1);
			p1.x = img.cols;
			p1.y = -1*(ln.coef(2) + ln.coef(0)*p1.x)/ln.coef(1);
		}
		cv::line(img, p0, p1, color, 1 );
	}
}


bool ImagePolygon::SaveMarkedImage(const std::string& fn,
								   const cv::Mat& img, const std::list<Line2D>&ls)
{
	cv::Mat img_line;

	if(img.channels() == 1)
	{
#if (CV_VERSION_MAJOR >= 4)
		cv::cvtColor(img, img_line, cv::COLOR_GRAY2BGR);
#else
		cv::cvtColor(img, img_line, CV_GRAY2BGR);
#endif
	}
	else
	{
		img_line = img.clone();
	}

	MarkImage(img_line, ls);

	return cv::imwrite(fn, img_line);
}


bool ImagePolygon::SaveMarkedImage(const std::string& fn, const cv::Mat& img,
								   Polygon2D::ConstPtr ply)
{
	cv::Mat img_line;

	if(img.channels() == 1)
	{
#if (CV_VERSION_MAJOR >= 4)
		cv::cvtColor(img, img_line, cv::COLOR_GRAY2BGR);
#else
		cv::cvtColor(img, img_line, CV_GRAY2BGR);
#endif

	}
	else
	{
		img_line = img.clone();
	}

	MarkImage(img_line, ply);

	return cv::imwrite(fn, img_line);

}

/*****************************END OF FILE**************************************/
