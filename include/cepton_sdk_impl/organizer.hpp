/*******************************************************************
 **                                                               **
 **  Copyright(C) 2019 Cepton Technologies. All Rights Reserved.  **
 **  Contact: https://www.cepton.com                              **
 **                                                               **
 *******************************************************************/

#pragma once

#include <cepton_sdk_util.hpp>

namespace cepton_sdk {

namespace util {

inline Organizer::Organizer(cepton_sdk::SensorInformation sensor_info)
{
  switch(sensor_info.model)
  {
    case HR80W:
    case VISTA_860:
    case VISTA_860_GEN2:
    case VISTA_P60:
      m_settings.horizontal_range_radians = to_radians(70.f);
      break;
    case HR80T:
    case HR80T_R2:
    default:
      m_settings.horizontal_range_radians = to_radians(40.f);

    m_settings.vertical_range_radians = to_radians(30.f);
  }

}

inline void Organizer::organize_points(
    const int num_points_in,
    const int n_returns,
    const CeptonSensorImagePoint* const unorganized_points,
    cepton_sdk::util::OrganizedCloud& organized_cloud)
{
  std::lock_guard<std::mutex> lock(m_organizer_mutex);

  //This list of points we are going to output
  auto& organized_points = organized_cloud.points;
  organized_points.clear();
  organized_cloud.info_cells.clear();
  organized_cloud.timestamp_start = std::numeric_limits<long>::max();
  organized_cloud.timestamp_end = std::numeric_limits<long>::min();
  organized_cloud.n_returns = n_returns;
  //Setup the grid sizing in case it has changed
  organized_cloud.width = static_cast<int>(
      (m_settings.horizontal_range_radians / m_settings.horizontal_bin_size_radians));
  organized_cloud.height =
      static_cast<int>(m_settings.vertical_range_radians / m_settings.vertical_bin_size_radians);
  const auto organized_cloud_size = organized_cloud.width * organized_cloud.height;

  //There are n_returned points for each location in the grid
  organized_points.resize(static_cast<size_t>(organized_cloud_size * n_returns));

  //Initialize each location in the grid as unoccupied with no valid original indice
  organized_cloud.info_cells.resize(static_cast<size_t>(organized_cloud_size * n_returns));

  for (int point_index = 0; point_index < num_points_in; point_index++)
  {
    const auto& unorganized_point = unorganized_points[point_index];
    const auto grid_index = getGridIndex(organized_cloud, unorganized_point.image_x,unorganized_point.image_z);

    int n_return = 0;
    if (unorganized_point.return_type | CEPTON_RETURN_STRONGEST)
    {
      n_return = 0;
    }
    else if (n_returns > 1)
    {
      n_return = 1;
    }

    if (grid_index.row >=0 && grid_index.col >= 0)
    {
      const size_t organized_index = static_cast<size_t>(
          organized_cloud.getIndex(grid_index.row, grid_index.col, n_return));

      if (!organized_cloud.info_cells[organized_index].occupied_cell ||
          (unorganized_point.valid &&
           unorganized_point.timestamp >
               organized_cloud.points[organized_index].timestamp)) 
      {
        organized_cloud.timestamp_start =
            std::min(organized_cloud.timestamp_start, unorganized_point.timestamp);
        organized_cloud.timestamp_end =
            std::max(organized_cloud.timestamp_end, unorganized_point.timestamp);
        organized_cloud.info_cells[organized_index].occupied_cell = true;
        organized_cloud.info_cells[organized_index].original_index = point_index;

        organized_cloud.points[organized_index] = unorganized_point;
        if (m_settings.mode == OrganizerMode::CENTER) {
          ImageXZ image_xz = getXZ(organized_cloud, grid_index.row, grid_index.col);
          organized_cloud.points[organized_index].image_x = image_xz.X;
          organized_cloud.points[organized_index].image_z = image_xz.Z;
        }
      }
    }
  }
}

inline void Organizer::mode(Organizer::OrganizerMode mode)
{
  std::lock_guard<std::mutex> lock(m_organizer_mutex);
  m_settings.mode = std::move(mode);
}

inline void Organizer::binSize(float bin_size)
{
  std::lock_guard<std::mutex> lock(m_organizer_mutex);
  m_settings.horizontal_bin_size_radians = bin_size;
  m_settings.vertical_bin_size_radians = bin_size;
}

inline Organizer::OrganizerSettings Organizer::settings()
{
  return m_settings;
}

inline void Organizer::settings(OrganizerSettings organizer_settings)
{
  std::lock_guard<std::mutex> lock(m_organizer_mutex);
  m_settings = organizer_settings;
}

inline Organizer::ImageXZ Organizer::getXZ(const OrganizedCloud& cloud, int row, int col)
{
   float angle_horizontal = (m_settings.horizontal_range_radians /cloud.width) * ((col + 0.5f) - cloud.width / 2);
   float angle_vertical = (m_settings.vertical_range_radians /cloud.height) * ((row + 0.5f) - cloud.height / 2);
   return {tanf(angle_horizontal),tanf(angle_vertical)};
}


inline Organizer::GridIndex Organizer::getGridIndex(const OrganizedCloud& cloud, float X, float Z)
{
  float angle_horizontal = atanf(X);
  float angle_vertical = atanf(Z);
  GridIndex index;

  index.col = cloud.width / 2 + static_cast<int>(cloud.width * angle_horizontal / m_settings.horizontal_range_radians);
  index.row = cloud.height / 2 + static_cast<int>(cloud.height * angle_vertical / m_settings.vertical_range_radians);
  if (index.col < 0 || index.col >= cloud.width)
  {
    // Data outside of expected range given invalid index
    index.col = -1;
  }

  if (index.row < 0 || index.row >= cloud.height)
  {
    // Data outside of expected range given invalid index
    index.row = -1;
  }
  return index;
}

} //util

} //cepton_sdk
