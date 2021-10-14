#pragma once
#include <openvdb/openvdb.h>

void makeLineAlongX(openvdb::FloatGrid& grid, int side_size, const openvdb::Coord& start){
  openvdb::FloatGrid::Accessor accessor = grid.getAccessor();
  for (auto i=start[0]; i<=start[0]+side_size; ++i){
    openvdb::Coord xyz(i, start[1], start[2]);
    accessor.setValue(xyz, 1.0);
  }
}

void makeSquare(openvdb::FloatGrid& grid, int side_size, const openvdb::Coord& left_corner){
  openvdb::FloatGrid::Accessor accessor = grid.getAccessor();
  for (auto i=left_corner[0]; i<=left_corner[0]+side_size; ++i){
    for (auto j=left_corner[1]; j<=left_corner[1]+side_size; ++j){
        openvdb::Coord xyz(i, j, left_corner[2]);
        accessor.setValue(xyz, 1.0);
    }
  }
}

void makeCube(openvdb::FloatGrid& grid, int side_size, const openvdb::Coord& left_corner){
  openvdb::FloatGrid::Accessor accessor = grid.getAccessor();
  for (auto i=left_corner[0]; i<=left_corner[0]+side_size; ++i){
    for (auto j=left_corner[1]; j<=left_corner[1]+side_size; ++j){
      for (auto k=left_corner[2]; k<=left_corner[2]+side_size; ++k){
        openvdb::Coord xyz(i, j, k);
        accessor.setValue(xyz, 1.0);
      }
    }
  }
}

// from https://academysoftwarefoundation.github.io/openvdb/codeExamples.html
void makeSphere(openvdb::FloatGrid& grid, float radius, const openvdb::Vec3f& c)
{
    using ValueT = typename openvdb::FloatGrid::ValueType;
    const ValueT outside = grid.background();
    const ValueT inside = -outside;
    int padding = int(openvdb::math::RoundUp(openvdb::math::Abs(outside)));
    int dim = int(radius + padding);

    typename openvdb::FloatGrid::Accessor accessor = grid.getAccessor();
    openvdb::Coord ijk;
    int &i = ijk[0], &j = ijk[1], &k = ijk[2];
    for (i = c[0] - dim; i < c[0] + dim; ++i) {
        const float x2 = openvdb::math::Pow2(i - c[0]);
        for (j = c[1] - dim; j < c[1] + dim; ++j) {
            const float x2y2 = openvdb::math::Pow2(j - c[1]) + x2;
            for (k = c[2] - dim; k < c[2] + dim; ++k) {
                // The distance from the sphere surface in voxels
                const float dist = openvdb::math::Sqrt(x2y2
                    + openvdb::math::Pow2(k - c[2])) - radius;
                ValueT val = ValueT(dist);
                if (val < inside || outside < val) continue;
                accessor.setValue(ijk, val);
            }
        }
    }
    openvdb::tools::signedFloodFill(grid.tree());
}