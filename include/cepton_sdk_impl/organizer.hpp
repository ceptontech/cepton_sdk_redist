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
  m_settings.vertical_range_radians = to_radians(30.f);
  switch(sensor_info.model)
  {
    case HR80W:
    case VISTA_860_GEN2:
    case VISTA_P60:
      m_settings.horizontal_range_radians = to_radians(70.f);
      break;
    case VISTA_X120:
    case VISTA_X15:
      m_settings.horizontal_range_radians = to_radians(130.f);
      m_settings.vertical_range_radians = to_radians(40.f);
      break;
    case HR80T:
    case HR80T_R2:
    default:
      m_settings.horizontal_range_radians = to_radians(40.f); 
  }

}

inline void Organizer::organize_points(
    const int num_points_in,
    const int n_returns,
    const CeptonSensorImagePoint* const unorganized_points,
    cepton_sdk::util::OrganizedCloud& organized_cloud)
{
  std::lock_guard<std::mutex> lock(m_organizer_mutex);

  // set start and end timestamps to numeric limits
  organized_cloud.timestamp_start = std::numeric_limits<long>::max();
  organized_cloud.timestamp_end = std::numeric_limits<long>::min();

  organized_cloud.width =
      static_cast<int>(m_settings.horizontal_range_radians /
                       m_settings.horizontal_bin_size_radians);
  organized_cloud.height = static_cast<int>(
      m_settings.vertical_range_radians / m_settings.vertical_bin_size_radians);
  organized_cloud.n_returns = n_returns;

  int cloud_size = organized_cloud.width * organized_cloud.height * n_returns;

  // initialize points
  organized_cloud.points.resize(cloud_size);

  // initialize info cells
  organized_cloud.info_cells.clear();
  organized_cloud.info_cells.resize(cloud_size);

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

      auto& organized_point = organized_cloud.points[organized_index];
      auto& info_cell = organized_cloud.info_cells[organized_index];

      // If the existing cell is unoccupied populate. If we have a valid point
      // and it's more recent we always want to populate with it. 
      if (!info_cell.occupied_cell ||
          (unorganized_point.valid &&
           organized_point.timestamp < unorganized_point.timestamp)) 
      {
        organized_cloud.timestamp_start =
            std::min(organized_cloud.timestamp_start, unorganized_point.timestamp);
        organized_cloud.timestamp_end =
            std::max(organized_cloud.timestamp_end, unorganized_point.timestamp);
        info_cell.occupied_cell = true;
        info_cell.original_index = point_index;
        
        // We always populated everything except the image_x and image_z
        organized_point.timestamp = unorganized_point.timestamp;
        organized_point.distance = unorganized_point.distance;
        organized_point.intensity = unorganized_point.intensity;
        organized_point.return_type = unorganized_point.return_type;
        organized_point.valid = unorganized_point.valid;
        organized_point.saturated = unorganized_point.saturated;

        // In RECENT mode, populate the image xy points with the unorganized
        // point for accuracy.
        if (m_settings.mode == OrganizerMode::RECENT) 
        {
          organized_point.image_x = unorganized_point.image_x;
          organized_point.image_z = unorganized_point.image_z;
        }
        else
        {
          auto image_xz = getXZ(organized_cloud.width, organized_cloud.height,
                                grid_index.row, grid_index.col);
          organized_point.image_x = image_xz.X;
          organized_point.image_z = image_xz.Z;
        }
      }
    }
  }
  for (int row = 0; row < organized_cloud.height; ++row)
  {
    for (int col = 0; col < organized_cloud.width; ++col)
    {
      for (int n_return = 0; n_return < organized_cloud.n_returns; ++n_return)
      {
        int index = organized_cloud.getIndex(row, col, n_return);
        auto& pt = organized_cloud.points[index];
        if (pt.distance == 0.f)
        {
          ImageXZ image_xz =
              getXZ(organized_cloud.width, organized_cloud.height, row, col);
          pt.image_x = image_xz.X;
          pt.image_z = image_xz.Z;
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

inline Organizer::ImageXZ Organizer::getXZ(int width, int height, int row, int col)
{
   float angle_horizontal = (m_settings.horizontal_range_radians / width) * ((col + 0.5f) - width / 2);
   float angle_vertical = (m_settings.vertical_range_radians / height) * ((row + 0.5f) - height / 2);
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
