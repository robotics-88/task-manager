#ifndef ELEVATION_2_ROS_H_
#define ELEVATION_2_ROS_H_

#include <vector>
#include <cmath>
#include <map>
#include <iostream>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <gdal/gdal_priv.h>
#include <gdal/gdal.h>
#include <gdal/gdalwarper.h>
#include <gdal/ogr_p.h>
#include <gdal/ogr_spatialref.h>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace elevation2ros
{

class Elevation2Ros
{
public:
    Elevation2Ros() {}

    bool init(const std::string &tif_name, const double utm_x, const double utm_y) 
    {
        // Load mat
        dem_cv = cv::imread(tif_name, cv::IMREAD_LOAD_GDAL | cv::IMREAD_ANYDEPTH );
        // Get coords with GDAL
        GDALAllRegister();
        GDALDataset* hSrcDS = static_cast<GDALDataset*>(GDALOpen(tif_name.c_str(), GA_ReadOnly));
        if (hSrcDS == nullptr){
            std::cout << "Topography data set is null in gdal, name was: " << tif_name << std::endl;
            return false;
        }

        std::vector<double> geotransform(6);
        if(hSrcDS->GetGeoTransform(geotransform.data())!=CE_None){
            std::cout << "Could not get a geotransform!" << std::endl;
            return false;
        }
        // Upper left coords
        double easting=geotransform[0];// + x*geotransform[1] + y*geotransform[2]; // Longitude
        double northing=geotransform[3];// + x*geotransform[4] + y*geotransform[5]; // Latitude
        std::cout << "explore topog got UL UTM east/north: " << easting << ", " << northing << std::endl;
        ul_x_utm = easting;
        ul_y_utm = northing;

        // GDALClose( hDstDS );
        GDALClose( hSrcDS );

        initializeValleys();

        double resolution = 1.0;
        double width = dem_cv.cols;
        double height = dem_cv.rows;
        double origin_x = ul_x_utm - utm_x;
        double origin_y = ul_y_utm - height - utm_y;
        // Initialize occupancy grid message
        grid.header.frame_id = "map";
        grid.info.resolution = resolution;
        grid.info.width = width;
        grid.info.height = height;
        grid.info.origin.position.x = origin_x;
        grid.info.origin.position.y = origin_y;
        grid.info.origin.position.z = 0.0;
        grid.data.resize(width * height);
        cv::Mat dem_copy  = dem_cv.clone();
        for (int y = height - 1; y >= 0; y--) {
            for (int x = 0; x < width; x++) {
                float elevation = dem_cv.at<float>(y, x);
                uint8_t occupancy_value = static_cast<uint8_t>(elevation); // Convert elevation to occupancy value
                grid.data[(height - y - 1) * width + x] = occupancy_value;
                pcl::PointXYZ point;
                point.x = origin_x + x * resolution;
                point.y = origin_y + (height - y - 1) * resolution;
                point.z = elevation;
                cloud.points.push_back(point);
            }
        }
        
        return true;
    }

    bool getElevationChunk(const double utm_x, const double utm_y, const int width, const int height, cv::Mat &chunk) {
        int pixel_r, pixel_c;
        pixel_c = utm_x - ul_x_utm;
        pixel_r = dem_cv.rows - floor(ul_y_utm - utm_y);
        bool cols_outofbounds = pixel_c < 0 || pixel_c >= dem_cv.cols;
        bool rows_outofbounds = pixel_r < 0 || pixel_r >= dem_cv.rows;
        if (cols_outofbounds || rows_outofbounds ) {
            // TODO should extend to checking if ROI is out of bounds
            std::cout << "Pixels or rows out of bounds in get tif chunk." << std::endl;
            return false;
        }
        else {
            // value = dem_cv.at<float>(pixel_r, pixel_c);
            int start_r = pixel_r - floor(height / 2);
            int start_c = pixel_c - floor(width / 2);
            int len_r = height - floor(height / 2);
            int len_c = width - floor(width / 2);
            cv::Rect roi(start_c, start_r, len_c, len_r);
            cv::Mat submat = dem_cv(roi);
            chunk = submat.clone(); // Required to protect the original mat data
            return true;
        }
    }

    double getElevation(const double utm_x, const double utm_y) {
        double value;
        getMapValue(utm_x, utm_y, dem_cv, value);
        return value;
    }

    double getSlope(const double utm_x, const double utm_y) {
        double value;
        getMapValue(utm_x, utm_y, slope_cv, value);
        return value;
    }

    nav_msgs::msg::OccupancyGrid getGrid() {
        return grid;
    }

    pcl::PointCloud<pcl::PointXYZ> getCloud() {
        return cloud;
    }

private:

    cv::Mat dem_cv;
    cv::Mat slope_cv;
    double ul_y_utm;
    double ul_x_utm;

    nav_msgs::msg::OccupancyGrid grid;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    void makeOutputFile(GDALDataset* hSrcDS, std::string filename) {
        GDALDriverH hDriver;
        GDALDataType eDT;
        GDALDatasetH hDstDS;

        // Create output with same datatype as first input band.
        eDT = GDALGetRasterDataType(GDALGetRasterBand(hSrcDS,1));

        // Get output driver (GeoTIFF format)
        hDriver = GDALGetDriverByName( "GTiff" );
        CPLAssert( hDriver != NULL );
    
        // Get Source coordinate system.
        const char *pszSrcWKT;
        char *pszDstWKT = NULL;
        pszSrcWKT = GDALGetProjectionRef( hSrcDS );
        CPLAssert( pszSrcWKT != NULL && strlen(pszSrcWKT) > 0 );

        // Setup output coordinate system that is UTM 11 WGS84.
        OGRSpatialReference oSRS;
        oSRS.SetUTM( 16, TRUE ); // TODO make map manager in TM and get correct zone
        oSRS.SetWellKnownGeogCS( "WGS84" );
        oSRS.exportToWkt( &pszDstWKT );

        // Create a transformer that maps from source pixel/line coordinates
        // to destination georeferenced coordinates (not destination
        // pixel line).  We do that by omitting the destination dataset
        // handle (setting it to NULL).
        void *hTransformArg;
        hTransformArg =
            GDALCreateGenImgProjTransformer( hSrcDS, pszSrcWKT, NULL, pszDstWKT,
                                            FALSE, 0, 1 );
        CPLAssert( hTransformArg != NULL );

        // Get approximate output georeferenced bounds and resolution for file.
        double adfDstGeoTransform[6];
        int nPixels=0, nLines=0;
        CPLErr eErr;
        eErr = GDALSuggestedWarpOutput( hSrcDS,
                                        GDALGenImgProjTransform, hTransformArg,
                                        adfDstGeoTransform, &nPixels, &nLines );
        CPLAssert( eErr == CE_None );
        GDALDestroyGenImgProjTransformer( hTransformArg );

        // Create the output file.
        hDstDS = GDALCreate( hDriver, filename.c_str(), nPixels, nLines,
                            GDALGetRasterCount(hSrcDS), eDT, NULL );
        CPLAssert( hDstDS != NULL );

        // Write out the projection definition.
        GDALSetProjection( hDstDS, pszDstWKT );
        GDALSetGeoTransform( hDstDS, adfDstGeoTransform );
        GDALClose(hDstDS);
    }

    bool getMapValue(const double utm_x, const double utm_y, const cv::Mat &mat, double &value) {
        int pixel_r, pixel_c;
        pixel_c = floor(utm_x - ul_x_utm);
        pixel_r = floor(ul_y_utm - utm_y);
        bool cols_outofbounds = pixel_c < 0 || pixel_c >= mat.cols;
        bool rows_outofbounds = pixel_r < 0 || pixel_r >= mat.rows;
        if (cols_outofbounds || rows_outofbounds ) {
            std::cout << "Pixels or rows out of bounds in get tif value" << std::endl;
            return false;
        }
        else {
            value = mat.at<float>(pixel_r, pixel_c);
            return true;
        }
    }

    void initializeValleys() {
        // Pixelwise scores of slope magnitude, ignoring slope direction
        cv::Mat sobelx, sobely;
        cv::Sobel(dem_cv, sobelx, CV_32F, 1, 0);
        cv::Sobel(dem_cv, sobely, CV_32F, 0, 1);

        cv::Mat xsqr = sobelx.mul(sobelx);
        cv::Mat ysqr = sobelx.mul(sobely);
        cv::Mat xysum = xsqr + ysqr;
        cv::sqrt(xysum, slope_cv);
    }

};

}

#endif // ELEVATION_2_ROS_H_
