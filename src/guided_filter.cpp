#include "image_brighten/guided_filter.h"

static void boxfilter(const cv::Mat &I, cv::Mat &result, int r)
{
	cv::blur(I, result, cv::Size(r, r));
}

static void convertTo(const cv::Mat &mat, cv::Mat &result, int depth)
{
	if (mat.depth() == depth)
		result = mat.clone();
		return;
	mat.convertTo(result, depth);
}
GuidedFilterColor::GuidedFilterColor(int rows, int cols, int r, double eps, int resize_factor)
{
	m_original_image_size = cv::Size(cols, rows);
	m_resized_size = cv::Size(cols/resize_factor, rows/resize_factor);
	m_radius = r;
	m_sub_radius = m_radius / resize_factor;
	m_eps = eps;
	reformated_I	= cv::Mat(m_original_image_size, CV_32FC3);
	resized_I		= cv::Mat(m_resized_size, CV_32FC3);
	resized_p		= cv::Mat(m_resized_size, CV_32FC3);
	mean_I_r		= cv::Mat(m_resized_size, CV_32FC1);
	mean_I_g		= cv::Mat(m_resized_size, CV_32FC1);
	mean_I_b		= cv::Mat(m_resized_size, CV_32FC1);
	invrr			= cv::Mat(m_resized_size, CV_32FC1);
	invrg			= cv::Mat(m_resized_size, CV_32FC1);
	invrb			= cv::Mat(m_resized_size, CV_32FC1);
	invgg			= cv::Mat(m_resized_size, CV_32FC1);
	invgb			= cv::Mat(m_resized_size, CV_32FC1);
	invbb			= cv::Mat(m_resized_size, CV_32FC1);
	var_I_rr		= cv::Mat(m_resized_size, CV_32FC1);
	var_I_rg		= cv::Mat(m_resized_size, CV_32FC1);
	var_I_rb		= cv::Mat(m_resized_size, CV_32FC1);
	var_I_gg		= cv::Mat(m_resized_size, CV_32FC1);
	var_I_gb		= cv::Mat(m_resized_size, CV_32FC1);
	var_I_bb		= cv::Mat(m_resized_size, CV_32FC1);
	covDet			= cv::Mat(m_resized_size, CV_32FC1);
	mean_p			= cv::Mat(m_resized_size, CV_32FC1);
	mean_Ip_r		= cv::Mat(m_resized_size, CV_32FC1);
	mean_Ip_g		= cv::Mat(m_resized_size, CV_32FC1);
	mean_Ip_b		= cv::Mat(m_resized_size, CV_32FC1);
	cov_Ip_r		= cv::Mat(m_resized_size, CV_32FC1);
	cov_Ip_g		= cv::Mat(m_resized_size, CV_32FC1);
	cov_Ip_b		= cv::Mat(m_resized_size, CV_32FC1);
	a_r				= cv::Mat(m_resized_size, CV_32FC1);
	a_g				= cv::Mat(m_resized_size, CV_32FC1);
	a_b				= cv::Mat(m_resized_size, CV_32FC1);
	b				= cv::Mat(m_resized_size, CV_32FC1);
	temp 			= cv::Mat(m_resized_size, CV_32FC1);
}
void GuidedFilterColor::filter(const cv::Mat &I, const cv::Mat &p, cv::Mat &result)
{
	if (I.depth() != CV_32F)
		I.convertTo(reformated_I, CV_32F);
	else
		reformated_I = I.clone();
	cv::resize(reformated_I, resized_I, m_resized_size, 0, 0, cv::INTER_NEAREST); // NN recommended by matlab code

	m_original_I_depth = resized_I.depth();

	cv::split(resized_I, Ichannels);

	boxfilter(Ichannels[0], mean_I_r, m_sub_radius);
	boxfilter(Ichannels[1], mean_I_g, m_sub_radius);
	boxfilter(Ichannels[2], mean_I_b, m_sub_radius);

	boxfilter(Ichannels[0].mul(Ichannels[0]), var_I_rr, m_sub_radius);
	boxfilter(Ichannels[0].mul(Ichannels[1]), var_I_rg, m_sub_radius);
	boxfilter(Ichannels[0].mul(Ichannels[2]), var_I_rb, m_sub_radius);
	boxfilter(Ichannels[1].mul(Ichannels[1]), var_I_gg, m_sub_radius);
	boxfilter(Ichannels[1].mul(Ichannels[2]), var_I_gb, m_sub_radius);
	boxfilter(Ichannels[2].mul(Ichannels[2]), var_I_bb, m_sub_radius);

	var_I_rr = var_I_rr - mean_I_r.mul(mean_I_r) + m_eps;
	var_I_rg = var_I_rg - mean_I_r.mul(mean_I_g);
	var_I_rb = var_I_rb - mean_I_r.mul(mean_I_b);
	var_I_gg = var_I_gg - mean_I_g.mul(mean_I_g) + m_eps;
	var_I_gb = var_I_gb - mean_I_g.mul(mean_I_b);
	var_I_bb = var_I_bb - mean_I_b.mul(mean_I_b) + m_eps;
	// Inverse of Sigma + eps * I
	invrr = var_I_gg.mul(var_I_bb) - var_I_gb.mul(var_I_gb);
	invrg = var_I_gb.mul(var_I_rb) - var_I_rg.mul(var_I_bb);
	invrb = var_I_rg.mul(var_I_gb) - var_I_gg.mul(var_I_rb);
	invgg = var_I_rr.mul(var_I_bb) - var_I_rb.mul(var_I_rb);
	invgb = var_I_rb.mul(var_I_rg) - var_I_rr.mul(var_I_gb);
	invbb = var_I_rr.mul(var_I_gg) - var_I_rg.mul(var_I_rg);

	covDet = invrr.mul(var_I_rr) + invrg.mul(var_I_rg) + invrb.mul(var_I_rb);

	invrr /= covDet;
	invrg /= covDet;
	invrb /= covDet;
	invgg /= covDet;
	invgb /= covDet;
	invbb /= covDet;

	cv::resize(p, resized_p, m_resized_size, 0, 0, cv::INTER_NEAREST);
	
	boxfilter(resized_p, mean_p, m_sub_radius);

	boxfilter(Ichannels[0].mul(resized_p), mean_Ip_r, m_sub_radius);
	boxfilter(Ichannels[1].mul(resized_p), mean_Ip_g, m_sub_radius);
	boxfilter(Ichannels[2].mul(resized_p), mean_Ip_b, m_sub_radius);

	// covariance of (I, p) in each local patch.
	cov_Ip_r = mean_Ip_r - mean_I_r.mul(mean_p);
	cov_Ip_g = mean_Ip_g - mean_I_g.mul(mean_p);
	cov_Ip_b = mean_Ip_b - mean_I_b.mul(mean_p);

	a_r = invrr.mul(cov_Ip_r) + invrg.mul(cov_Ip_g) + invrb.mul(cov_Ip_b);
	a_g = invrg.mul(cov_Ip_r) + invgg.mul(cov_Ip_g) + invgb.mul(cov_Ip_b);
	a_b = invrb.mul(cov_Ip_r) + invgb.mul(cov_Ip_g) + invbb.mul(cov_Ip_b);

	b = mean_p - a_r.mul(mean_I_r) - a_g.mul(mean_I_g) - a_b.mul(mean_I_b); // Eqn. (15) in the paper;

	boxfilter(a_r, result, m_sub_radius);
	result = result.mul(Ichannels[0]);
	boxfilter(a_g, temp, m_sub_radius);
	result += temp.mul(Ichannels[1]);
	boxfilter(a_b, temp, m_sub_radius);
	result += temp.mul(Ichannels[2]);
	boxfilter(b, temp, m_sub_radius);
	result += temp;

	cv::resize(result, result, m_original_image_size, 0,0, CV_INTER_NN);
}
