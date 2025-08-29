#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <math.h>

/*!
 * Simple DEM class: load DEM from image, store as elevation matrix.
 */
class DEM
{
public:
  struct BBox {
    double xmin;
    double xmax;
    double ymin;
    double ymax;
    double zmin;
    double zmax;
    BBox() {}
    BBox(double xmin, double xmax, double ymin, double ymax,
         double zmin, double zmax):
        xmin(xmin), xmax(xmax), ymin(ymin), ymax(ymax), zmin(zmin), zmax(zmax) {}
  };
private:
  BBox bbox_;
  double res_;
  int width_;
  int height_;
  cv::Mat_<double> dem_;   // floating-point elevation map

public:
    //Default constructor
  DEM() {
  }
  DEM(const std::string& imgFileName, const BBox& bbox, double res): res_(res) {
    
    // Initialize the bbox
    bbox_.xmin = bbox.xmin;
    bbox_.xmax = bbox.xmax;
    bbox_.ymin = bbox.ymin;
    bbox_.ymax = bbox.ymax;
    bbox_.zmin = bbox.zmin;
    bbox_.zmax = bbox.zmax;

    // Load image as grayscale
    cv::Mat tmp = cv::imread(imgFileName, cv::IMREAD_UNCHANGED);
    assert(!tmp.empty());

    height_ = tmp.rows;
    width_  = tmp.cols;

    assert(width_>0);
    assert(height_>0);
    assert(res_>0);

    double x_length_ = bbox_.xmax - bbox.xmin;
    double y_length_ = bbox_.ymax - bbox.ymin;
    
    assert(fabs(width_ *res_ - x_length_) < res_*1e-2);
    assert(fabs(height_ *res_ - y_length_) < res_*1e-2);
    
    // Convert to greyscale image to meters, store in dem_
    size_t encoding = std::pow(2, (tmp.elemSize() * 8));
    double zscale = (bbox_.zmax - bbox_.zmin)/encoding;
    dem_ = tmp *zscale + bbox_.zmin;
    }

    //check if a point is in a BBox, limits included    
    bool isInBBox(double x, double y) const {
        if ((x < bbox_.xmin) || (x) > bbox_.xmax) return false;
        if ((y < bbox_.ymin) || (y) > bbox_.ymax) return false;
        return true;
    };
    
    //Returns pixel coordinates from metrics point
    cv::Point metersToPix(double x, double y) const {
        assert(isInBBox(x,y));
        cv::Point pix;
        pix.x = std::min(std::max(static_cast<int>((x-bbox_.xmin)/res_), 0), static_cast<int>(width_ -1));
        pix.y = (height_ -1)- std::min(std::max(static_cast<int>((y-bbox_.ymin)/res_), 0), static_cast<int>(height_-1));
        return pix;
    };

  //! Get elevation from XY position.
  double getZfromXY(double x, double y) const {
    cv::Point p = metersToPix(x,y);
    return dem_.at<double>(p);
  }

  //! Accessors
  int width() const { return width_; }
  int height() const { return height_; }
  const cv::Mat& data() const { return dem_; }
  const BBox& bbox() const { return bbox_; }
  double resolution() const { return res_; }

};
