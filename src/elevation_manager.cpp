/*
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com>
*/

#include "task_manager/elevation_manager.h"
#include "task_manager/decco_utilities.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/PolygonArea.hpp>

#include <boost/filesystem.hpp>

namespace elevation_manager {
ElevationManager::ElevationManager(const double flightleg_acres, const std::string mavros_map_frame)
    : mavros_map_frame_(mavros_map_frame), elevation_init_(false) {}

ElevationManager::~ElevationManager() {}

bool ElevationManager::elevationInitializer(const double utm_x, const double utm_y) {

    std::string tif_name =
        ament_index_cpp::get_package_share_directory("task_manager") + "/dem/test.tif";

    if (!boost::filesystem::exists(tif_name)) {
        RCLCPP_INFO(rclcpp::get_logger("elevation_manager"), "No elevation file found at: %s",
                    tif_name.c_str());
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("elevation_manager"), "Elevation file found at: %s",
                tif_name.c_str());

    elevation_init_ = initElevation(tif_name, utm_x, utm_y);
    return elevation_init_;
}

bool ElevationManager::getElevationChunk(const double utm_x, const double utm_y, const int width,
                                         const int height, sensor_msgs::msg::Image &chunk,
                                         double &max, double &min) {
    cv::Mat mat;
    if (!elevation_init_) {
        bool has_elev = elevationInitializer(0, 0);
        if (!has_elev)
            return false;
    }
    bool worked = getElevationChunk(utm_x, utm_y, width, height, mat);
    if (worked) {
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(mat, &min, &max, &minLoc, &maxLoc);
        std_msgs::msg::Header header;
        header.frame_id = "world";
        sensor_msgs::msg::Image::SharedPtr chunk =
            cv_bridge::CvImage(header, "bgr8", mat).toImageMsg();
    }
    return worked;
}

bool ElevationManager::getHomeElevation(double &value) {
    if (!elevation_init_) {
        bool has_elev = elevationInitializer(-1 * utm_x_offset_, -1 * utm_y_offset_);
        if (!has_elev)
            return false;
    }
    return getElevation(-1 * utm_x_offset_, -1 * utm_y_offset_, value);
}

bool ElevationManager::getElevationValue(const double utm_x, const double utm_y, double &value) {
    if (!elevation_init_) {
        bool has_elev = elevationInitializer(0, 0);
        if (!has_elev)
            return false;
    }
    return getElevation(utm_x, utm_y, value);
}

void ElevationManager::llToMap(const double lat, const double lon, double &px, double &py) {
    decco_utilities::llToMap(lat, lon, px, py, utm_x_offset_, utm_y_offset_);
}

void ElevationManager::mapToLl(const double px, const double py, double &lat, double &lon) {
    decco_utilities::mapToLl(px, py, lat, lon, utm_x_offset_, utm_y_offset_, utm_zone_);
}

bool ElevationManager::initElevation(const std::string &tif_name, const double utm_x,
                                     const double utm_y) {
    // Load mat
    dem_cv_ = cv::imread(tif_name, cv::IMREAD_LOAD_GDAL | cv::IMREAD_ANYDEPTH);
    // Get coords with GDAL
    GDALAllRegister();
    GDALDataset *hSrcDS = static_cast<GDALDataset *>(GDALOpen(tif_name.c_str(), GA_ReadOnly));
    if (hSrcDS == nullptr) {
        std::cout << "Topography data set is null in gdal, name was: " << tif_name << std::endl;
        return false;
    }

    std::vector<double> geotransform(6);
    if (hSrcDS->GetGeoTransform(geotransform.data()) != CE_None) {
        std::cout << "Could not get a geotransform!" << std::endl;
        return false;
    }
    // Upper left coords
    double easting = geotransform[0];  // + x*geotransform[1] + y*geotransform[2]; // Longitude
    double northing = geotransform[3]; // + x*geotransform[4] + y*geotransform[5]; // Latitude
    std::cout << "explore topog got UL UTM east/north: " << easting << ", " << northing
              << std::endl;
    ul_x_utm_ = easting;
    ul_y_utm_ = northing;

    // GDALClose( hDstDS );
    GDALClose(hSrcDS);

    initializeValleys();

    double resolution = 1.0;
    double width = dem_cv_.cols;
    double height = dem_cv_.rows;
    double origin_x = ul_x_utm_ - utm_x;
    double origin_y = ul_y_utm_ - height - utm_y;
    // Initialize occupancy grid message
    tif_grid_.header.frame_id = "world";
    tif_grid_.info.resolution = resolution;
    tif_grid_.info.width = width;
    tif_grid_.info.height = height;
    tif_grid_.info.origin.position.x = origin_x;
    tif_grid_.info.origin.position.y = origin_y;
    tif_grid_.info.origin.position.z = 0.0;
    tif_grid_.data.resize(width * height);
    for (int y = height - 1; y >= 0; y--) {
        for (int x = 0; x < width; x++) {
            float elevation = dem_cv_.at<float>(y, x);
            uint8_t occupancy_value =
                static_cast<uint8_t>(elevation); // Convert elevation to occupancy value
            tif_grid_.data[(height - y - 1) * width + x] = occupancy_value;
            pcl::PointXYZ point;
            point.x = origin_x + x * resolution;
            point.y = origin_y + (height - y - 1) * resolution;
            point.z = elevation;
            tif_cloud_.points.push_back(point);
        }
    }

    return true;
}

bool ElevationManager::getElevationChunk(const double utm_x, const double utm_y, const int width,
                                         const int height, cv::Mat &chunk) {
    int pixel_r, pixel_c;
    pixel_c = utm_x - ul_x_utm_;
    pixel_r = dem_cv_.rows - floor(ul_y_utm_ - utm_y);
    bool cols_outofbounds = pixel_c < 0 || pixel_c >= dem_cv_.cols;
    bool rows_outofbounds = pixel_r < 0 || pixel_r >= dem_cv_.rows;
    if (cols_outofbounds || rows_outofbounds) {
        // TODO should extend to checking if ROI is out of bounds
        std::cout << "Pixels or rows out of bounds in get tif chunk." << std::endl;
        return false;
    } else {
        // value = dem_cv.at<float>(pixel_r, pixel_c);
        int start_r = pixel_r - floor(height / 2);
        int start_c = pixel_c - floor(width / 2);
        int len_r = height - floor(height / 2);
        int len_c = width - floor(width / 2);
        cv::Rect roi(start_c, start_r, len_c, len_r);
        cv::Mat submat = dem_cv_(roi);
        chunk = submat.clone(); // Required to protect the original mat data
        return true;
    }
}

bool ElevationManager::getElevation(const double utm_x, const double utm_y, double &value) {
    return getMapValue(utm_x, utm_y, dem_cv_, value);
}

double ElevationManager::getSlope(const double utm_x, const double utm_y) {
    double value;
    getMapValue(utm_x, utm_y, slope_cv_, value);
    return value;
}

void ElevationManager::makeOutputFile(GDALDataset *hSrcDS, std::string filename) {
    GDALDriverH hDriver;
    GDALDataType eDT;
    GDALDatasetH hDstDS;

    // Create output with same datatype as first input band.
    eDT = GDALGetRasterDataType(GDALGetRasterBand(hSrcDS, 1));

    // Get output driver (GeoTIFF format)
    hDriver = GDALGetDriverByName("GTiff");
    CPLAssert(hDriver != NULL);

    // Get Source coordinate system.
    const char *pszSrcWKT;
    char *pszDstWKT = NULL;
    pszSrcWKT = GDALGetProjectionRef(hSrcDS);
    CPLAssert(pszSrcWKT != NULL && strlen(pszSrcWKT) > 0);

    // Setup output coordinate system that is UTM 11 WGS84.
    OGRSpatialReference oSRS;
    oSRS.SetUTM(16, TRUE); // TODO make map manager in TM and get correct zone
    oSRS.SetWellKnownGeogCS("WGS84");
    oSRS.exportToWkt(&pszDstWKT);

    // Create a transformer that maps from source pixel/line coordinates
    // to destination georeferenced coordinates (not destination
    // pixel line).  We do that by omitting the destination dataset
    // handle (setting it to NULL).
    void *hTransformArg;
    hTransformArg =
        GDALCreateGenImgProjTransformer(hSrcDS, pszSrcWKT, NULL, pszDstWKT, FALSE, 0, 1);
    CPLAssert(hTransformArg != NULL);

    // Get approximate output georeferenced bounds and resolution for file.
    double adfDstGeoTransform[6];
    int nPixels = 0, nLines = 0;
    CPLErr eErr;
    eErr = GDALSuggestedWarpOutput(hSrcDS, GDALGenImgProjTransform, hTransformArg,
                                   adfDstGeoTransform, &nPixels, &nLines);
    CPLAssert(eErr == CE_None);
    GDALDestroyGenImgProjTransformer(hTransformArg);

    // Create the output file.
    hDstDS = GDALCreate(hDriver, filename.c_str(), nPixels, nLines, GDALGetRasterCount(hSrcDS), eDT,
                        NULL);
    CPLAssert(hDstDS != NULL);

    // Write out the projection definition.
    GDALSetProjection(hDstDS, pszDstWKT);
    GDALSetGeoTransform(hDstDS, adfDstGeoTransform);
    GDALClose(hDstDS);
}

bool ElevationManager::getMapValue(const double utm_x, const double utm_y, const cv::Mat &mat,
                                   double &value) {
    int pixel_r, pixel_c;
    pixel_c = floor(utm_x - ul_x_utm_);
    pixel_r = floor(ul_y_utm_ - utm_y);
    bool cols_outofbounds = pixel_c < 0 || pixel_c >= mat.cols;
    bool rows_outofbounds = pixel_r < 0 || pixel_r >= mat.rows;
    if (cols_outofbounds || rows_outofbounds) {
        std::cout << "Pixels or rows out of bounds in get tif value" << std::endl;
        return false;
    } else {
        value = mat.at<float>(pixel_r, pixel_c);
        return true;
    }
}

void ElevationManager::initializeValleys() {
    // Pixelwise scores of slope magnitude, ignoring slope direction
    cv::Mat sobelx, sobely;
    cv::Sobel(dem_cv_, sobelx, CV_32F, 1, 0);
    cv::Sobel(dem_cv_, sobely, CV_32F, 0, 1);

    cv::Mat xsqr = sobelx.mul(sobelx);
    cv::Mat ysqr = sobelx.mul(sobely);
    cv::Mat xysum = xsqr + ysqr;
    cv::sqrt(xysum, slope_cv_);
}

} // namespace elevation_manager