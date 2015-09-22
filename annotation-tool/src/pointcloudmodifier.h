#ifndef POINTCLOUDMODIFIER_H
#define POINTCLOUDMODIFIER_H

#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZRGB pointT;
typedef pcl::PointCloud<pointT> pointCloud;
typedef pointCloud::Ptr pointCloudPtr;



class pointCloudModifier
{
public:
    explicit pointCloudModifier();

    /** \brief Detect automatically the table plane and moves the plane x-y to the table plane.
      * \param[in] cloud_in The input point cloud
      * \param[out] cloud_out The output point cloud
      */
    void automaticTableDetection(pcl::PointCloud<pointT>::Ptr cloud_in,
                                 pcl::PointCloud<pointT>::Ptr cloud_out);

    /** \brief Calculate manually the table plane (given three points) and moves the plane x-y to the table plane.
      * \param[in] cloud_in The input point cloud
      * \param[out] cloud_out The output point cloud
      * \param[in] points The three points belonging to the table plane
      */
    void manualTableDetection(pcl::PointCloud<pointT>::Ptr cloud_in,
                              pcl::PointCloud<pointT>::Ptr cloud_out,
                              std::vector<pointT> &points);

    /** \brief Filter one axis of the input point cloud
      * \param[in] cloud_in The input point cloud
      * \param[out] cloud_out The output point cloud
      * \param[in] axis The axis desired to filter
      * \param[in] min The minimum allowed field value
      * \param[in] max The maximum allowed field value
      */
    void filter_axis(pcl::PointCloud<pointT>::Ptr cloud_in,
                     pcl::PointCloud<pointT>::Ptr cloud_out,
                     std::string axis,
                     float min,
                     float max);

    /** \brief Translate the point cloud to have the origin of the coordinate system in the plane x-y.
      * \param[in] cloud_in The input point cloud
      * \param[out] cloud_out The output point cloud
      * \param[in] dest The point to set the coordinate origin
      */
    void translate_on_plane_x_y(pcl::PointCloud<pointT>::Ptr cloud_in,
                                pcl::PointCloud<pointT>::Ptr cloud_out,
                                pointT dest);

    /** \brief Align the x axis to the lower edge of the table.
      * \param[in] cloud_in The input point cloud
      * \param[out] cloud_out The output point cloud
      * \param[in] corner The lower right point of the table
      */
    void align_x_with_edge(pcl::PointCloud<pointT>::Ptr cloud_in,
                           pcl::PointCloud<pointT>::Ptr cloud_out,
                           pointT corner);

    /** \brief Rotate the pointcloud 180º of the z axis.
      * \param[in] The input point cloud
      * \param[out] The output point cloud
      */
    void rotate_z_180(pcl::PointCloud<pointT>::Ptr cloud_in,
                      pcl::PointCloud<pointT>::Ptr cloud_out);

private:
    // The plane coefficients. Saved as: a * x + b * y + c * z = d
    pcl::ModelCoefficients::Ptr _planeCoefficients;

    /** \brief Compute the plane coefficients given three points.
      * \param points The three points belonging to the table plane
      */
    void computePlaneCoefficients(std::vector<pointT> &points);

};

#endif // POINTCLOUDMODIFIER_H
