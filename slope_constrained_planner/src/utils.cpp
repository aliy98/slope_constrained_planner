/**
 * @file utils.cpp
 * @brief Contains the definition of ``estiamteNormals`` helper function
*/
#include "slope_constrained_planner/utils.h"

#include <grid_map_core/iterators/CircleIterator.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>



void slope_constrained_planner::estimateNormals(grid_map::GridMap& map,
                                  double estimation_radius,
                                  const std::string& input_layer_name,
                                  const std::string& output_layer_prefix) {
  const std::string normal_x = output_layer_prefix + "_x";
  const std::string normal_y = output_layer_prefix + "_y";
  const std::string normal_z = output_layer_prefix + "_z";
  const std::string plane_fit_std_dev = "plane_fit_std_dev";
  map.add(normal_x);
  map.add(normal_y);
  map.add(normal_z);
  map.add(plane_fit_std_dev);
  const int estimation_radius_cells = estimation_radius / map.getResolution();

  auto& input_layer = map.get(input_layer_name);
  auto& normal_x_layer = map.get(normal_x);
  auto& normal_y_layer = map.get(normal_y);
  auto& normal_z_layer = map.get(normal_z);
  auto& std_layer = map.get(plane_fit_std_dev);

  // Build matrix of cell positions.
  grid_map::Position pos;
  Eigen::Matrix<grid_map::DataType, 3, Eigen::Dynamic> map_3d(3, input_layer.size());
  std::vector<std::pair<Eigen::Index, Eigen::Index> > lin_to_2d_index(input_layer.size());
  std::vector<std::vector<Eigen::Index> > lin_from_2d_index(input_layer.rows(), std::vector<Eigen::Index>(input_layer.cols()));
  Eigen::Index lin_index = 0;
  for (Eigen::Index i = 0; i < input_layer.rows(); ++i) {
    for (Eigen::Index j = 0; j < input_layer.cols(); ++j) {
      map.getPosition(grid_map::Index(i,j), pos);
      map_3d(0, lin_index) = pos.x();
      map_3d(1, lin_index) = pos.y();
      map_3d(2, lin_index) = input_layer(i, j);
      lin_from_2d_index[i][j] = lin_index;
      lin_to_2d_index[lin_index++] = std::make_pair(i, j);
    }
  }

  unsigned int n_vec;
  Eigen::Matrix<grid_map::DataType, 3, 1> vec_sum;
  float max_z_diff;
  for (Eigen::Index i = 0; i < input_layer.rows(); ++i) {
    for (Eigen::Index j = 0; j < input_layer.cols(); ++j) {
      vec_sum.setZero();
      n_vec = 0;
      max_z_diff = 0;

      const auto center = map_3d.col(lin_from_2d_index[i][j]);

      for (int offset = 1; offset < estimation_radius_cells; ++offset) {
        const Eigen::Index i_offset = i + offset;
        const Eigen::Index j_offset = j + offset;
        if (i_offset >= input_layer.rows() || j_offset >= input_layer.cols()) continue;
        const auto vec_x = map_3d.col(lin_from_2d_index[i_offset][j]) - center;
        const auto vec_y = map_3d.col(lin_from_2d_index[i][j_offset]) - center;
        if (std::fabs(vec_x.z()) > max_z_diff) max_z_diff = std::fabs(vec_x.z());
        if (std::fabs(vec_y.z()) > max_z_diff) max_z_diff = std::fabs(vec_y.z());
        if (!std::isnan(vec_x.cross(vec_y).normalized().matrix().mean())) {
          vec_sum += vec_x.cross(vec_y).normalized();
          ++n_vec;
        } 
      }
      for (int offset = 1; offset < estimation_radius_cells; ++offset) {
        const Eigen::Index i_offset = i - offset;
        const Eigen::Index j_offset = j - offset;
        if (i_offset < 0 || j_offset < 0) continue;
        const auto vec_x = map_3d.col(lin_from_2d_index[i_offset][j]) - center;
        const auto vec_y = map_3d.col(lin_from_2d_index[i][j_offset]) - center;
        if (std::fabs(vec_x.z()) > max_z_diff) max_z_diff = std::fabs(vec_x.z());
        if (std::fabs(vec_y.z()) > max_z_diff) max_z_diff = std::fabs(vec_y.z());
        if (!std::isnan(vec_x.cross(vec_y).normalized().matrix().mean())) {
          vec_sum += vec_x.cross(vec_y).normalized();
          ++n_vec;
        } 
      }
      

      // Average over all vectors.
      if (n_vec > 0) {
        vec_sum.array() /= n_vec;
      }
      std_layer(i, j) = max_z_diff;
      // Set normal layers.
      vec_sum.normalize();

      normal_x_layer(i, j) = vec_sum.x();
      normal_y_layer(i, j) = vec_sum.y();
      normal_z_layer(i, j) = vec_sum.z();
      
    }
  }

}
