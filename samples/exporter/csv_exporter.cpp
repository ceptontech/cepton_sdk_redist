#include "cepton_sdk_util.hpp"
#include "exporter.h"

static char buffer[4000];

void CsvExporter::ExportPoint(CeptonSensorImagePoint const &p) {
  float x, y, z;
  cepton_sdk::util::convert_image_point_to_point(p.image_x, p.image_z,
                                                 p.distance, x, y, z);
  float azim = atan(-p.image_x);
  float elev = atan2(-p.image_z, sqrt(p.image_x * p.image_x + 1));
  int strongest = ((p.return_type & CEPTON_RETURN_STRONGEST) ? 1 : 0);
  int farthest = ((p.return_type & CEPTON_RETURN_FARTHEST) ? 1 : 0);
  // clang-format off
  //*export_stream
  //  << p.timestamp << ','
  //  << p.image_x << ','
  //  << p.image_z << ','
  //  << p.distance << ','
  //  << x << ','
  //  << y << ','
  //  << z << ','
  //  << azim << ','
  //  << elev << ','
  //  << p.intensity << ','
  //  << strongest << ','
  //  << farthest << ','
  //  << (int)p.valid << ','
  //  << (int)p.saturated << ','
  //  << (int)p.segment_id << "\n";
  // clang-format on
  int sz;
  if (split_timestamp) {
    sz = snprintf(buffer, sizeof(buffer),
                  "%u,%u,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d\n",
                  (unsigned)(p.timestamp / 1000000),
                  (unsigned)(p.timestamp % 1000000), p.image_x, p.image_z,
                  p.distance, x, y, z, azim, elev, p.intensity, strongest,
                  farthest, p.valid, p.saturated, p.segment_id);
  } else {
    sz = snprintf(buffer, sizeof(buffer),
                  "%llu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d\n",
                  (unsigned long long)p.timestamp, p.image_x, p.image_z,
                  p.distance, x, y, z, azim, elev, p.intensity, strongest,
                  farthest, p.valid, p.saturated, p.segment_id);
  }
  export_stream->write(buffer, sz);
}

int CsvExporter::Export(CeptonSensorImagePoint const *points, size_t size,
                        bool include_invalid, bool append) {
  if (!append) {
    if (split_timestamp) {
      *export_stream
          << "# "
             "timestamp(sec),timestamp(us),image_x,image_z,distance,x,y,z,"
             "azimuth,"
             "elevation,reflectivity,return_strongest,return_farthest,"
             "valid,saturated,segment_id\n";
    } else {
      *export_stream
          << "# "
             "timestamp,image_x,image_z,distance,x,y,z,azimuth,"
             "elevation,reflectivity,return_strongest,return_farthest,"
             "valid,saturated,segment_id\n";
    }
  }
  for (size_t i = 0; i < size; i++) {
    if (points[i].valid || include_invalid) ExportPoint(points[i]);
  }
  return 0;
}
