/**
  ******************************************************************************
  * @file	calibrator.cpp
  * @author	Nick.Liao
  * @version V1.0.0
  * @date	2017-12-28
  * @brief	calibrator.cpp
  ******************************************************************************
  * @attention
  *
  * Copyright (C) 2017 Nick.Liao <simplelife_nick@hotmail.com>
  * Distributed under terms of the MIT license.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "calibrator.h"

#include <fstream>
#include <iostream>

#include <pcl/registration/icp.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

using namespace lqh;
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
using PCC = pcl::PointCloud<pcl::PointXYZRGB>;
using PCI = pcl::PointCloud<pcl::PointXYZI>;

namespace
{
template <typename T>
static void ProjectPoint2Image(const T* const q_ptr, const T* const t_ptr,
							   const Eigen::Matrix<T, 3, 3>& k,
							   const Eigen::Matrix<T, 3, Eigen::Dynamic>& p3,
							   Eigen::Matrix<T, 3, Eigen::Dynamic>& p2)
{
	Eigen::Map<const Eigen::Quaternion<T> > q(q_ptr);
	Eigen::Map<const Eigen::Matrix<T, 1, 3> > t(t_ptr);

	p2 = (q.matrix()*p3);
//    p2.colwise() += t;
	for(uint32_t i=0; i<p2.cols(); i++)
	{
		p2.col(i) += t;
		p2.col(i) = p2.col(i)/p2(2,i);
	}
	p2 = k*p2;
}

/**
 * @brief Edge to edge error (point to line error)
 *        3D point P_i_j locates at 3D edge i, its index is j. 3D edge i has correspondense in
 *        image E_i(ax+by+c=0,a^2+b^2+c^2=1), project P_i_j into image with initial T(q,p) and
 *        get 2D point P'_i_j(u,v). If T(q,p) is correct or precise, P'_i_j should be on E_i.
 *        Thus, the error is the distance between P'_i_j and E_i
 */
struct Edge2EdgeError
{
	const Eigen::Matrix3d& K;       // camera matrix
	const Eigen::Matrix3Xd& pts;    // 3d points
	const Eigen::Vector3d& coef;    // 2d edge(line) coefficients

	/**
	 * @brief Edge2EdgeError constructor
	 *
	 * @param k	[in]: camera intrinsic parameter matrix K
	 * @param ps[in]: 3D edge pints set, P_i_j(j=0,1,...n), it's 3xn matrix
	 * @param cf[in]: 2D line coefficients (ax+by+c=0,a^2+b^2+c^2=1)
	 */
	Edge2EdgeError(const Eigen::Matrix3d& k, const Eigen::Matrix3Xd& ps, const Eigen::Vector3d& cf):
		K(k), pts(ps), coef(cf) {}


	/**
	 * @brief Ceres error compute function
	 *
	 * @tparam T	double or Jet
	 * @param q	[in]: Quaternion (rotation)
	 * @param p [in]: translation
	 * @param residuals[out]: error
	 *
	 * @return true: indicate success
	 */
	template<typename T>
	bool operator()(const T* const q, const T* const p, T* residuals)const
	{
		Eigen::Matrix<T, 3, Eigen::Dynamic> points;
		ProjectPoint2Image<T>(q,p, K.cast<T>(), pts.cast<T>(), points);

		Eigen::Matrix<T, Eigen::Dynamic, 1> tmp = (coef.transpose().cast<T>())*points;
		residuals[0] = tmp.cwiseAbs().sum();

		return true;	// important
	}
};

struct Point2PolygonError
{
	const Eigen::Matrix3d& K;
	const Eigen::Matrix3Xd& pts;
	const Eigen::Matrix3Xd& vertex;
	Eigen::Matrix3Xd edge_normal;

	Point2PolygonError(const Eigen::Matrix3d& k, const Eigen::Matrix3Xd& ps, const Eigen::Matrix3Xd& vx ):
		K(k), pts(ps), vertex(vx)
	{
		Eigen::Vector3d center = vx.rowwise().mean();
		edge_normal.resize(3, vx.cols());
		edge_normal.row(2).setZero();

		for(uint32_t i=0; i<vx.cols(); i++)
		{
			uint32_t next = (i == (vx.cols()-1)) ? 0 : i+1;
			Eigen::Vector3d dir = vx.col(next) - vx.col(i);
			edge_normal(0, i) = -dir(1);
			edge_normal(1, i) = dir(0);

			if(edge_normal.col(i).dot(center - vx.col(i)) < 0 )
			{
				edge_normal.col(i) *= -1;
			}
		}
	}

	template<typename T>
	bool operator()(const T* const q, const T* const t, T* residuals)const
	{
		using Matrix3XT = Eigen::Matrix<T, 3, Eigen::Dynamic>;

		Matrix3XT vtx = vertex.cast<T>();
		Matrix3XT normals = edge_normal.cast<T>();

		Matrix3XT points;
		ProjectPoint2Image<T>(q, t, K.cast<T>(), pts.cast<T>(), points);

		residuals[0] = T(0);
		for(uint32_t i=0; i<pts.cols(); i++)
		{
			bool flag = false;
			for(uint32_t j=0; j<vertex.cols(); j++)
			{
				if( normals.col(j).dot(points.col(i) - vtx.col(j)) < T(0) )
				{
					flag = true;
					break;
				}
			}
			// one point locates outside the polygon
			if(flag)
			{
				residuals[0] += T(1);
			}
		}

		return true;	// important
	}


};
}


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
Calibrator::Calibrator(const nlohmann::json& js):
	is_valid_(false),
	img_width_(0),
	img_height_(0)
{
	size_ = js["size"].get<uint32_t>();
	if(size_ < 3 )
	{
		std::cerr << "E: size must >= 3\n " << __FUNCTION__;
		return;
	}

	imgply_.reset( new ImagePolygon(js["img"], size_) );
	pcply_.reset( new PointcloudPolygon(js["pc"], size_) );
	polygons_.clear();
	polygons_v_.clear();
	polygons_v_.reserve(10);

	track_error_threshold_ = js["track_error_threshold"].get<double>();

	K_.setIdentity();
	T_.setIdentity();

	auto& k = js["cam"]["K"];
	assert( k.size() == 3 );
	for(uint8_t i=0; i<2; i++)
	{
		assert( k[i].size() == 3 );
		K_.row(i) << k[i][0], k[i][1], k[i][2];
	}
	K_.row(2) << 0,0,1;

	auto& t = js["tf"];
	assert( t.size() == 4 );
	for(uint8_t i=0; i<3; i++)
	{
		assert( t[i].size() == 4 );
		T_.row(i) << t[i][0], t[i][1], t[i][2], t[i][3];
	}
	T_.row(3) << 0,0,0,1;

	is_valid_ = true;
}

uint32_t Calibrator::Add(const cv::Mat& img, const PCI& pc,
						 cv::Mat& img_out, pcl::PointCloud<pcl::PointXYZRGB>& pcc)
{
	polygons_.emplace_back(size_);
	auto& ply = polygons_.back();
	polygons_v_.push_back(&ply);

	ply.img = imgply_->Add(img,img_out);
	ply.pc  = pcply_->Add(pc, pcc);

	if(ply.img != nullptr && ply.pc != nullptr)
	{
		MatchLines(ply);
	}

//	if( polygons_.size() == 1)
//	{
//        MatchLines(ply);
//	}
//	else
//	{
//		auto it = std::prev(polygons_.end(),2);
//        TrackLines(*it, ply);
//	}

	//return id
	return polygons_v_.size()-1;
}

bool Calibrator::RefineImage(const cv::Mat& img,
							 cv::Mat& img_out, const std::vector<cv::Point2d>& pts)
{
	assert(pts.size() == size_);

	Eigen::Matrix3Xd vtx(3, size_);
	vtx.row(2).setOnes();

	for(uint8_t i=0; i<size_; i++)
	{
		vtx(0,i) = pts[i].x;
		vtx(1,i) = pts[i].y;
	}

	auto& ply = polygons_.back();
	ply.img = imgply_->Init(img, img_out, vtx);
	if(ply.img == nullptr)
	{
		return false;
	}

	if(ply.pc == nullptr)
	{
		return true;
	}
	// both valid, match
	MatchLines(ply);
//    if( polygons_.size() == 1)
//    {
//        MatchLines(ply);
//    }
//    else
//    {
//        auto it = std::prev(polygons_.end(),2);
//        TrackLines(*it, ply);
//    }
	return true;
}

bool Calibrator::RefinePointcloud(const PCI& pc, PCC& pcc, const Eigen::Vector4d& param )
{
	auto& ply = polygons_.back();

	pcply_->SetFilterParameters(param);
	ply.pc = pcply_->Add(pc, pcc);
	if(ply.pc == nullptr)
	{
		return false;
	}

	if(ply.img == nullptr)
	{
		return true;
	}

	// both valid, match
	MatchLines(ply);
//    if( polygons_.size() == 1)
//    {
//        MatchLines(ply);
//    }
//    else
//    {
//        auto it = std::prev(polygons_.end(),2);
//        TrackLines(*it, ply);
//    }
	return true;
}

/**
 * @brief Calibrator::Remove
 * @todo: maybe we should use !!!skip
 * @param id
 * @return
 */
bool Calibrator::Remove(uint32_t id)
{
	if(id >= polygons_v_.size())
	{
		return false;
	}

	polygons_v_.erase(polygons_v_.begin()+id);
	auto it = polygons_.begin();
	std::advance(it, id);
	polygons_.erase(it);
	return true;
}

// remove last one
bool Calibrator::Remove()
{
	if(polygons_.size() == 0)
	{
		return false;
	}

	polygons_.pop_back();
	polygons_v_.pop_back();
}

bool Calibrator::Compute(Eigen::Matrix4d &tf)
{
	if(polygons_.size() == 0)
	{
		return false;
	}

	Optimize(tf);

	T_ = tf;
	return true;
}


void  Calibrator::Optimize(Eigen::Matrix4d& tf)
{
	Eigen::Matrix3d rot = T_.topLeftCorner(3,3);
	Eigen::Quaterniond q (rot);
	Eigen::Vector3d p = T_.topRightCorner(3,1);

	//std::cout << "Before \nq:\n" << q.coeffs() << "\nt:\n" << p << std::endl;

	ceres::Problem problem;
	ceres::Solver::Summary summary;
	ceres::Solver::Options options;
	ceres::LossFunction* loss_function_edge (new ceres::SoftLOneLoss(1));
	ceres::HuberLoss* loss_function_inlier (new ceres::HuberLoss(500));
	ceres::LocalParameterization* quaternion_local_parameterization =
		new ceres::EigenQuaternionParameterization;

	for(const auto& ply : polygons_)
	{
		// add edge
		for(uint32_t i=0; i<size_; i++)
		{
			if(ply.pc->edges[i].points.cols() == 0)
			{
				continue;
			}
			ceres::CostFunction* cost =
				new ceres::AutoDiffCostFunction<Edge2EdgeError, 1,4,3>(
				new Edge2EdgeError( K_, ply.pc->edges[i].points, ply.img->edges[ply.ids[i]].coef ));

			problem.AddResidualBlock(cost, loss_function_edge,
									 q.coeffs().data(), p.data() );
		}
		// add inlier points
		ceres::CostFunction* cost = new ceres::AutoDiffCostFunction<Point2PolygonError,1,4,3>(
			new Point2PolygonError(K_, ply.pc->inliers, ply.img->vertexs) );
		problem.AddResidualBlock(cost, NULL, q.coeffs().data(), p.data());
//        problem.AddResidualBlock(cost, loss_function_inlier, q.coeffs().data(), p.data());
	}
	problem.SetParameterization(q.coeffs().data(), quaternion_local_parameterization);
	//options.linear_solver_type = ceres::DENSE_SCHUR;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.max_num_iterations = 5000;
	// or use all cpu cores
	options.num_threads= boost::thread::hardware_concurrency() - 1;
#if (CERES_VERSION_MAJOR < 2)
	options.num_linear_solver_threads = options.num_threads;
#endif
	//options.minimizer_progress_to_stdout = true;

	ceres::Solve(options, &problem, &summary);

	std::cout << summary.FullReport() << std::endl;

	tf = Eigen::Matrix4d::Identity();
	tf.topLeftCorner(3,3) = q.matrix();
	tf.topRightCorner(3,1) = p;

	std::cout << "T: \n" << tf << std::endl;
}


void Calibrator::TrackLines(const Polygon& ply_prev,  Polygon& ply) const
{
	// image polygon keep orders, no futhur processing

	// tracking pointcloud edge
//    using PC = pcl::PointCloud<pcl::PointXYZ>;
//    auto Mat2PC = [](const Eigen::Matrix3Xd& mat, PC& pc)
//    {
//        pc.points.reserve(mat.cols());
//        for(uint32_t i=0; i<mat.cols(); i++)
//        {
//            pc.points.emplace_back(mat(0,i), mat(1,i), mat(2,i));
//        }
//    };

//    PC::Ptr pc_prev (new PC);
//    PC::Ptr pc (new PC);
//    PC pc_out;
//    Mat2PC(ply_prev.pc->inliers, *pc_prev);
//    Mat2PC(ply.pc->inliers, *pc);

//    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//    icp.setInputSource(pc);
//    icp.setInputTarget(pc_prev);
//    icp.setMaximumIterations(200);
//    icp.setTransformationEpsilon(1e-6);
//    icp.setEuclideanFitnessEpsilon(1);

//    icp.align(pc_out, Eigen::Matrix4d::Identity());
//    Eigen::Matrix4d T = icp.getFinalTransformation();


//    using PairUD = std::pair<uint32_t, double>;
//    std::vector<PairUD> offsets(size_);

//    for(uint32_t i=0; i<size_; i++)
//    {
//        offsets[i].first = i;
//        offsets[i].second = 0;
//        for(uint32_t j=0; j<size_; j++)
//        {
//            const auto& prev = ply_prev.pc->edges[(j+i)%size_];
//            const auto& cur = ply.pc->edges[j];
//            if(prev.coef.norm() < 0.1 || cur.points.cols() == 0)
//            {
//                continue;
//            }
//            for(uint32_t k=0; k<cur.points.cols(); k++)
//            {
//                offsets[i].second += prev.coef.cross(T*cur.points.col(k) - prev.p0).norm();
//            }
//        }
//    }

//    std::sort(offsets.begin(), offsets.end(), [](const PairUD& a, const PairUD& b)
//    {
//        return a.second < b.second;
//    });

//    uint32_t offset = offsets[0].first;
//    for(uint32_t i=0; i<size_; i++)
//    {
//        ply.ids[i] = ply_prev.ids[(i+offset)%size_];
//    }
}

/**
 * @brief match the 2/3d Polygon, since both polygon save edges
 * 		  as clock-wise, just find the offset is enough
 *
 * @param ply [in/out]
 */
void Calibrator::MatchLines(Polygon& ply) const
{
	// we find the offset based on the longest(most points) two edges
	std::vector<uint32_t> ids(size_);
	for(uint32_t i=0; i<size_; i++)
	{
		ids[i] = i;
	}

	// decrease order
	std::sort(ids.begin(), ids.end(), [&ply](uint32_t a, uint32_t b)
	{
		return ply.pc->edges[a].points.cols() >
			   ply.pc->edges[b].points.cols();
	});

	const auto& edge_0 = ply.pc->edges[ids[0]];
	const auto& edge_1 = ply.pc->edges[ids[1]];
	int32_t gap = ids[1] - ids[0];

	using PairUD = std::pair<uint32_t, double>;
	std::vector<PairUD> offsets(size_);

	Eigen::Matrix3d rot = T_.topLeftCorner(3,3);
	Eigen::Vector3d trans = T_.topRightCorner(3,1);

	auto err_proj = [this, &rot, &trans](const ImagePolygon::Edge2D& img,
										 const PointcloudPolygon::Edge3D& pc,
										 double& err)
	{
		Eigen::Vector2d dir_proj = (K_*(rot*pc.coef + trans)).head(2);
		dir_proj.normalize();
		Eigen::Vector2d dir_img(img.coef(1), -img.coef(0));
		dir_img.normalize();
		double err_angle = std::acos(dir_proj.dot(dir_img))/MATH_PI*180;


		Eigen::Matrix3Xd pt_proj = rot*pc.points;
		pt_proj.colwise() += trans;
		pt_proj = K_*pt_proj;
		for(uint32_t i=0; i<pt_proj.cols(); i++)
		{
			pt_proj.col(i) = pt_proj.col(i)/pt_proj(2,i);
		}
		double err_dis = (img.coef.transpose()*pt_proj).cwiseAbs().sum();

		err += err_angle*err_dis;
	};

	for(uint32_t i=0; i<size_; i++)
	{
		offsets[i].first = i;
		offsets[i].second = 0;
		// 3d edge_0 <----> 2d edge i
		uint32_t id1 = (static_cast<int32_t>(i)+gap) < 0 ? i+gap +size_ : (i+gap)%size_;
		err_proj(ply.img->edges[i], edge_0, offsets[i].second);
		err_proj(ply.img->edges[id1], edge_1, offsets[i].second);
	}

	// increase order, offsets[0] has smallest error
	std::sort(offsets.begin(), offsets.end(), [](const PairUD& a, const PairUD&b)
	{
		return a.second < b.second;
	});

	// ids[0] 3d edge <----> offsets[0].first 2d edge
	uint32_t id_img = offsets[0].first > ids[0] ? offsets[0].first - ids[0]:
					  offsets[0].first + size_ - ids[0];

	std::cout << "pc[0-4]: ";
	for(uint32_t i=0; i<size_; i++)
	{
		ply.ids[i] = (id_img+i)%size_;

		std::cout << ply.ids[i] << ", ";
	}
	std::cout << std::endl;


}




uint32_t Calibrator::SavePolygonData(const std::string& dir)
{
	Eigen::IOFormat csvfmt(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n", "", "", "", "");
	uint32_t i = 0;
	for(const auto& ply : polygons_)
	{
		std::ofstream pc_inlier(dir+"/pc_inlier_"+std::to_string(i)+".csv");
		std::ofstream pc_edge(dir+"/pc_edge_"+std::to_string(i)+".csv");
		std::ofstream img_coef(dir+"/img_coef_"+std::to_string(i)+".csv");
		std::ofstream img_vertex(dir+"/img_vertex_"+std::to_string(i)+".csv");
		if(!pc_inlier.good() || !pc_edge.good() ||
				!img_coef.good() || !img_vertex.good())
		{
			break;
		}

		pc_inlier << ply.pc->inliers.format(csvfmt);
		pc_inlier.close();

		uint32_t num = 0;
		for(const auto& it: ply.pc->edges)
		{
			num += it.points.cols();
		}
		Eigen::Matrix4Xd edges(4, num);
		uint32_t id =0;
		for(uint32_t j=0; j<ply.pc->edges.size(); j++)
		{
			const auto& it = ply.pc->edges[j].points;
			if(it.cols() == 0)
			{
				continue;
			}
			edges.block(0,id, 1, it.cols() ).setConstant(j);
			edges.block(1,id, 3, it.cols() ) = it;
			id += it.cols();
		}
		pc_edge << edges.format(csvfmt);
		pc_edge.close();


		Eigen::Matrix3Xd coef(3, size_);
		for(uint32_t j=0; j<ply.img->edges.size(); j++)
		{
			coef.col(j) = ply.img->edges[ply.ids[j]].coef;
		}
		img_coef << coef.format(csvfmt);
		img_coef.close();

		img_vertex << ply.img->vertexs.format(csvfmt);
		img_vertex.close();

		i++;
	}

}




bool Calibrator::Project(pcl::PointCloud<pcl::PointXYZRGB>& pc, cv::Mat& img,
						 const Eigen::Matrix4d& tf)
{
	const double resolution = 0.1;
	const double depth_min = 1.0;
	const double depth_gap = 10.0;

	if(img_width_ != img.cols || img_height_ != img.rows)
	{
		img_width_ = img.cols;
		img_height_ = img.rows;
	}
//    Eigen::Vector4f pt_min, pt_max;
//    pcl::getMinMax3D(pc, pt_min, pt_max);


//    double depth_gap = std::sqrt(pt_max(0)*pt_max(0) + pt_max(1)+pt_max(1)) - depth_gap + 0.01;
//    if(depth_gap > 5)
//    {
//        depth_gap = 5.0;
//    }
	uint32_t  num = static_cast<uint32_t>(depth_gap/resolution);
	lqh::utils::color::rgbs colors = lqh::utils::color::get_rgbs(num);

	cv::Mat im;
	img.copyTo(im);

	for(auto& p : pc.points)
	{
		Eigen::Vector3d pt = K_*(tf*Eigen::Vector4d(p.x, p.y, p.z, 1)).topRows(3);
		if(pt(2) < 0.5)
		{
			continue;
		}
		int32_t u = static_cast<int32_t>(pt(0)/pt(2));
		int32_t v = static_cast<int32_t>(pt(1)/pt(2));

		if(u < 0 || u >= img.cols || v<0 || v >= img.rows)
		{
			continue;
		}
		cv::Vec3b color = im.at<cv::Vec3b>(cv::Point(u,v));
		p.r = color[2];
		p.g = color[1];
		p.b = color[0];

		double f = std::sqrt(p.x*p.x + p.y*p.y) - depth_min;
		uint32_t idx = static_cast<uint32_t>(f/resolution);
		if(idx >= num)
		{
			idx = num -1;
		}
		auto& c = colors[idx];
		cv::circle(img, cv::Point2d(u,v), 2, cv::Scalar(c[2], c[1], c[0]), -1);
	}
}


void Calibrator::SetTranformation(const Eigen::Matrix4d& tf)
{
	T_ = tf;

	// check pointcloud filter
	if(img_height_ == 0 || img_width_ == 0)
	{
		return;
	}

	Eigen::Vector4d orig = pcply_->GetFilterParameters();
	auto getYaw = [this](double x, double y)->double
	{
		Eigen::Vector4d v (0,0,0,1);
		v.head(3) = K_.inverse()*Eigen::Vector3d(x,y,1);
		v = T_.inverse()*v;
		double angle = std::atan2(v(1), v(0))/MATH_PI*180;
		return angle <0? 360+angle : angle;
	};
	double angle0 = getYaw(0,0);
	double angle1 = getYaw(img_width_-1, img_height_-1);

	if(angle0 > angle1)
	{
		std::swap(angle0, angle1);
	}

	if(angle1-angle0 <180)
	{
		if(orig(0)< angle0 || orig(0)>angle1 )
		{
			orig(0) = angle0;
		}
		if(angle1 - orig(0) > orig(1))
		{
			orig(1) = angle1 - orig(0);
		}
	}
	else
	{
		if(orig(0)> angle0 && orig(0)<angle1 )
		{
			orig(0) = angle1;
		}
		double gap = orig(0) >= angle1 ? 360-angle1+angle0 : angle0-orig(0);
		if(orig(1) > gap)
		{
			orig(1) = gap;
		}
	}
	pcply_->SetFilterParameters(orig);
}





/*****************************END OF FILE**************************************/
