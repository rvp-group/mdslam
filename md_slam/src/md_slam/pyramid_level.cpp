#include "pyramid_level.h"
#include "utils.h"
#include <srrg_system_utils/char_array.h>

namespace md_slam {
  using namespace std; // erivaffanculo
  using namespace srrg2_core;

  //! generates a scaled pyramid from this one. Scale should be an int
  void MDPyramidLevel::writeToCharArray(char*& dest, size_t& size) const {
    srrg2_core::writeToCharArray(dest, size, rows());
    srrg2_core::writeToCharArray(dest, size, cols());
    srrg2_core::writeToCharArray(dest, size, min_depth);
    srrg2_core::writeToCharArray(dest, size, max_depth);
    srrg2_core::writeToCharArray(dest, size, camera_matrix);
    srrg2_core::writeToCharArray(dest, size, sensor_offset);
    srrg2_core::writeToCharArray(dest, size, camera_type);
    for (int i = 0; i < 3; ++i)
      srrg2_core::writeToCharArray(dest, size, thresholds[i]);
    for (int i = 0; i < 3; ++i)
      srrg2_core::writeToCharArray(dest, size, policies[i]);
    srrg2_core::writeToCharArray(dest, size, mask_grow_radius);
    for (size_t r = 0; r < rows(); ++r) {
      for (size_t c = 0; c < cols(); ++c) {
        const MDPyramidMatrixEntry& entry = matrix.at(r, c);
        float v                           = 0;
        if (entry.masked()) {
          srrg2_core::writeToCharArray(dest, size, v);
          continue;
        }
        srrg2_core::writeToCharArray(dest, size, entry.depth());
        srrg2_core::writeToCharArray(dest, size, entry.intensity());
        for (int i = 0; i < 3; ++i) {
          srrg2_core::writeToCharArray(dest, size, entry.value[2 + i]);
        }
      }
    }
  }

  //! reads a pyramid from a byte array
  void MDPyramidLevel::readFromCharArray(const char*& src, size_t& size) {
    size_t rows_, cols_;
    srrg2_core::readFromCharArray(rows_, src, size);
    srrg2_core::readFromCharArray(cols_, src, size);
    resize(rows_, cols_);
    srrg2_core::readFromCharArray(min_depth, src, size);
    srrg2_core::readFromCharArray(max_depth, src, size);
    srrg2_core::readFromCharArray(camera_matrix, src, size);
    srrg2_core::readFromCharArray(sensor_offset, src, size);
    srrg2_core::readFromCharArray(camera_type, src, size);
    for (int i = 0; i < 3; ++i)
      srrg2_core::readFromCharArray(thresholds[i], src, size);
    for (int i = 0; i < 3; ++i)
      srrg2_core::readFromCharArray(policies[i], src, size);
    srrg2_core::readFromCharArray(mask_grow_radius, src, size);
    for (size_t r = 0; r < rows(); ++r) {
      for (size_t c = 0; c < cols(); ++c) {
        MDPyramidMatrixEntry& entry = matrix.at(r, c);
        float v;
        srrg2_core::readFromCharArray(v, src, size);
        if (v == 0) {
          entry.setMasked(true);
          continue;
        }
        entry.setMasked(false);
        entry.setDepth(v);
        srrg2_core::readFromCharArray(v, src, size);
        entry.setIntensity(v);
        for (int i = 0; i < 3; ++i) {
          srrg2_core::readFromCharArray(entry.value[2 + i], src, size);
        }
      }
    }
    updateDerivatives();
  }

  void MDPyramidLevel::write(std::ostream& os) const {
    static constexpr size_t b_size = 1024 * 1024 * 50; // 50 MB of buffer
    char* buffer                   = new char[b_size];
    size_t size                    = b_size;
    char* buffer_end               = buffer;
    writeToCharArray(buffer_end, size);
    size_t real_size = buffer_end - buffer;
    os.write(buffer, real_size);
    delete[] buffer;
  }

  bool MDPyramidLevel::read(std::istream& is) {
    static constexpr size_t b_size = 1024 * 1024 * 50; // 50 MB of buffer
    char* buffer                   = new char[b_size];
    is.read(buffer, b_size);
    size_t real_size       = is.gcount();
    const char* buffer_end = buffer;
    this->readFromCharArray(buffer_end, real_size);
    delete[] buffer;
    return true;
  }

  MDPyramidLevel::MDPyramidLevel(size_t rows_, size_t cols_) {
    resize(rows_, cols_);
    sensor_offset = Eigen::Isometry3f::Identity();
  }

  void MDPyramidLevel::resize(const size_t& rows_, const size_t& cols_) {
    matrix.resize(rows_, cols_);
  }

  static inline Vector3f lift(const float f) {
    return Vector3f(f, f, f);
  }

  void MDPyramidLevel::getIntensity(ImageFloat& intensity) const {
    intensity.resize(rows(), cols());
    intensity.fill(0);
    for (size_t k = 0; k < matrix.size(); ++k)
      intensity.at(k) = matrix.at(k).intensity();
  }

  void MDPyramidLevel::getDepth(ImageFloat& depth) const {
    depth.resize(rows(), cols());
    depth.fill(0);
    for (size_t k = 0; k < matrix.size(); ++k)
      depth.at(k) = matrix.at(k).depth();
  }

  void MDPyramidLevel::getNormals(ImageVector3f& normals) const {
    normals.resize(rows(), cols());
    for (size_t k = 0; k < matrix.size(); ++k)
      normals.at(k) = matrix.at(k).normal();
  }

  void MDPyramidLevel::toTiledImage(ImageVector3f& canvas) {
    // collage,
    //           value cloud dx dy
    // intensity
    // depth
    // normals

    canvas.resize(rows() * 3, cols() /* *3*/);
    canvas.fill(Vector3f::Zero());
    int masked        = 0;
    int non_masked    = 0;
    float depth_scale = 1. / (max_depth - min_depth);
    for (size_t r = 0; r < rows(); ++r) {
      for (size_t c = 0; c < cols(); ++c) {
        const MDPyramidMatrixEntry& entry = matrix.at(r, c);
        if (entry.masked()) {
          ++masked;
          canvas.at(r, c) = Vector3f(0, 0, 1);
          continue;
        }
        ++non_masked;
        // intensity
        canvas.at(r, c) = lift(entry.intensity());
        // depth
        canvas.at(r + rows(), c) = lift(depth_scale * (entry.depth() - min_depth));
        // normals
        canvas.at(r + 2 * rows(), c) = entry.normal();
      }
    }
  }

  void MDPyramidLevel::growMask() {
    const int& radius = mask_grow_radius;
    std::vector<int> ball_offsets;
    int r2 = pow(radius, 2);
    for (int r = -radius; r < radius + 1; ++r) {
      for (int c = -radius; c < radius + 1; ++c) {
        int idx = r * cols() + c;
        if ((r * r + c * c) <= r2) {
          ball_offsets.push_back(idx);
        }
      }
    }
    ImageUInt8 mask(rows(), cols());
    for (size_t i = 0; i < mask.data().size(); ++i)
      mask.data()[i] = matrix.data()[i].masked();
    for (size_t i = 0; i < mask.data().size(); ++i) {
      if (!mask.data()[i])
        continue;
      for (auto offset : ball_offsets) {
        int target = offset + i;
        if (target < 0 || target >= (int) matrix.data().size())
          continue;
        matrix.data()[target].setMasked(true);
      }
    }
  }

  void MDPyramidLevel::toCloud(MDMatrixCloud& target) const {
    target.resize(rows(), cols());
    PointNormalIntensity3f p;
    p.setZero();
    p.status = POINT_STATUS::Invalid;
    target.fill(p);
    const float ifx = 1. / camera_matrix(0, 0);
    const float ify = 1. / camera_matrix(1, 1);
    const float cx  = camera_matrix(0, 2);
    const float cy  = camera_matrix(1, 2);

    Matrix3f inv_K = camera_matrix.inverse();
    for (size_t r_idx = 0; r_idx < rows(); ++r_idx) {
      const MDPyramidMatrixEntry* src = matrix.rowPtr(r_idx);
      PointNormalIntensity3f* dest    = target.rowPtr(r_idx);
      for (size_t c_idx = 0; c_idx < cols(); ++c_idx, ++src, ++dest) {
        float w = src->depth();
        if (src->masked() || w < min_depth || w > max_depth)
          continue;
#ifdef _MD_ENABLE_SUPERRES_
        const float& r = src->r;
        const float& c = src->c;
#else
        const float r = r_idx;
        const float c = c_idx;
#endif
        dest->status = POINT_STATUS::Valid;
        switch (camera_type) {
          case Pinhole: {
            Vector3f p          = inv_K * Vector3f(c * w, r * w, w);
            dest->coordinates() = p;
          } break;
          case Spherical: {
            float azimuth          = ifx * (c - cx);
            float elevation        = ify * (r - cy);
            float s0               = sin(azimuth);
            float c0               = cos(azimuth);
            float s1               = sin(elevation);
            float c1               = cos(elevation);
            dest->coordinates()(0) = c0 * c1 * w;
            dest->coordinates()(1) = s0 * c1 * w;
            dest->coordinates()(2) = s1 * w;
          } break;
          default:;
        }
        dest->intensity() = src->intensity();
        dest->normal()    = src->normal();
      }
    }
    target.transformInPlace<TRANSFORM_CLASS::Isometry>(sensor_offset);
  }

  void MDPyramidLevel::fromCloud(MDMatrixCloud& src_cloud) {
    MDPyramidMatrixEntry zero_entry;
    zero_entry.setDepth(max_depth + 1);
    matrix.fill(zero_entry);

    Isometry3f inv_sensor_offset = sensor_offset.inverse();
    Vector3f polar_point;
    Vector3f coordinates;
    Vector3f camera_point = Vector3f::Zero();
    const float& fx       = camera_matrix(0, 0);
    const float& fy       = camera_matrix(1, 1);
    const float& cx       = camera_matrix(0, 2);
    const float& cy       = camera_matrix(1, 2);
    float w               = 0;
    for (const auto& src : src_cloud) {
      if (src.status != POINT_STATUS::Valid)
        continue;
      coordinates    = inv_sensor_offset * src.coordinates();
      const float& x = coordinates.x();
      const float& y = coordinates.y();
      const float& z = coordinates.z();
      switch (camera_type) {
        case Pinhole: {
          w = coordinates(2);
          if (w < min_depth || w > max_depth)
            continue;
          camera_point = camera_matrix * coordinates;
          camera_point.block<2, 1>(0, 0) *= 1. / w;
        } break;
        case Spherical: {
          w = coordinates.norm();
          if (w < min_depth || w > max_depth)
            continue;
          polar_point.x()  = atan2(y, x);
          polar_point.y()  = atan2(coordinates.z(), sqrt(x * x + y * y));
          polar_point.z()  = z;
          camera_point.x() = fx * polar_point.x() + cx;
          camera_point.y() = fy * polar_point.y() + cy;
          camera_point.z() = w;
        } break;
        default:;
      }
      int c = cvRound(camera_point.x());
      int r = cvRound(camera_point.y());
      if (!matrix.inside(r, c))
        continue;
      MDPyramidMatrixEntry& entry = matrix.at(r, c);
      if (w < entry.depth()) {
        entry.setIntensity(src.intensity());
        entry.setDepth(w);
        entry.setNormal(inv_sensor_offset.linear() * src.normal());
#ifdef _MD_ENABLE_SUPERRES_
        entry.c = camera_point.x();
        entry.r = camera_point.y();
#endif
        entry.setMasked(false);
      }
    }
    growMask();
    updateDerivatives();
  }

  template <typename Matrix_>
  void applyPolicy(MDPyramidMatrixEntry& entry,
                   Matrix_&& m,
                   FilterPolicy policy,
                   float squared_threshold) {
    if (entry.masked())
      return;

    float n = m.squaredNorm();
    if (n < squared_threshold)
      return;

    switch (policy) {
      case Suppress:
        entry.setMasked(1);
        break;
      case Clamp:
        m *= sqrt(squared_threshold / n);
        break;
      default:;
    }
  }

  void MDPyramidLevel::updateDerivatives() {
    // these are for the normalization
    const float i2 = pow(thresholds[Intensity], 2);
    const float d2 = pow(thresholds[Depth], 2);
    const float n2 = pow(thresholds[Normal], 2);

    const size_t& rows = matrix.rows();
    const size_t& cols = matrix.cols();
    // we start from 1st row
    for (size_t r = 1; r < rows - 1; ++r) {
      // fetch the row vectors
      // in the iteration below we start from the 1st column
      // so we increment the pointers by 1

      const MDPyramidMatrixEntry* entry_ptr_r0 = matrix.rowPtr(r - 1) + 1;
      MDPyramidMatrixEntry* entry_ptr          = matrix.rowPtr(r) + 1;
      const MDPyramidMatrixEntry* entry_ptr_r1 = matrix.rowPtr(r + 1) + 1;

      for (size_t c = 1; c < cols - 1; ++c, ++entry_ptr_r0, ++entry_ptr, ++entry_ptr_r1) {
        const Vector5f& v_r0 = entry_ptr_r0->value;
        const Vector5f& v_r1 = entry_ptr_r1->value;
        const Vector5f& v_c0 = (entry_ptr - 1)->value;
        const Vector5f& v_c1 = (entry_ptr + 1)->value;

        Matrix5_2f& derivatives = entry_ptr->derivatives;

        derivatives.col(1) = .5 * v_r1 - .5 * v_r0;
        derivatives.col(0) = .5 * v_c1 - .5 * v_c0;

        // here we ignore, clamp or suppress
        // the derivatives according to the selected policy
        applyPolicy(*entry_ptr, derivatives.row(0), policies[Intensity], i2);
        applyPolicy(*entry_ptr, derivatives.row(1), policies[Depth], d2);
        applyPolicy(*entry_ptr, derivatives.block<3, 2>(2, 0), policies[Normal], n2);
      }
    }
  }

  void MDPyramidLevel::scaleTo(MDPyramidLevel& dest_, int scale) const {
    assert(rows() % scale == 0 && "MDPyramidLevel::scaleTo | rows not multiple of scale");
    assert(cols() % scale == 0 && "MDPyramidLevel::scaleTo | cols not multiple of scale");

    float inv_scale = 1. / scale;

    // copy shit to scaled level
    dest_.sensor_offset = sensor_offset;
    dest_.camera_type   = camera_type;
    dest_.resize(rows() / scale, cols() / scale);
    dest_.camera_matrix = camera_matrix;
    dest_.camera_matrix.block<2, 3>(0, 0) *= inv_scale;
    dest_.min_depth = min_depth;
    dest_.max_depth = max_depth;
    memcpy(dest_.thresholds, thresholds, sizeof(thresholds));
    memcpy(dest_.policies, policies, sizeof(policies));

    MDPyramidMatrixEntry null_entry;
    null_entry.setMasked(false);
    dest_.matrix.fill(null_entry);
    Matrix_<int> counters(dest_.rows(), dest_.cols());
    counters.fill(0);
    for (size_t r = 0; r < rows(); ++r) {
      int dr = r / scale;
      for (size_t c = 0; c < cols(); ++c) {
        const auto& src = matrix.at(r, c);
        int dc          = c / scale;
        auto& dest      = dest_.matrix.at(dr, dc);
        dest.setMasked(dest.masked() | src.masked());
        if (dest.masked())
          continue;
#ifdef _MD_ENABLE_SUPERRES_
        dest.r += src.r * inv_scale;
        dest.c += src.c * inv_scale;
#endif
        auto& ctr = counters.at(dr, dc);
        dest.value += src.value;
        ++ctr;
      }
    }
    for (size_t r = 0; r < dest_.rows(); ++r) {
      for (size_t c = 0; c < dest_.cols(); ++c) {
        auto& entry = dest_.matrix.at(r, c);
        auto& value = entry.value;
        auto& ctr   = counters.at(r, c);
        if (entry.masked())
          continue;
        assert(ctr && "MDPyramidLevel::scaleTo | counters are 0");
        float inv_ctr = 1. / ctr;
        value *= inv_ctr;
#ifdef _MD_ENABLE_SUPERRES_
        entry.r *= inv_ctr;
        entry.c *= inv_ctr;
#endif
        value.block<3, 1>(2, 0).normalize();
      }
    }
    dest_.updateDerivatives();
  }

} // namespace md_slam
