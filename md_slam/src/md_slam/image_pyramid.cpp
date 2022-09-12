#include "image_pyramid.h"
#include "utils.h"
#include <srrg_system_utils/char_array.h>

namespace md_slam {
  using namespace std; // erivaffanculo
  using namespace srrg2_core;

  void MDImagePyramid::writeToCharArray(char*& dest, size_t& size) const {
    // serialize the timestamp
    srrg2_core::writeToCharArray(dest, size, _timestamp);
    // serialize the number of levels
    size_t num_levels = _levels.size();
    srrg2_core::writeToCharArray(dest, size, num_levels);
    for (size_t i = 0; i < num_levels; ++i) {
      srrg2_core::writeToCharArray(dest, size, _relative_scales[i]);
    }
    if (!num_levels)
      return;
    _levels[0]->writeToCharArray(dest, size);
  }
  //! reads a pyramid from a byte array
  void MDImagePyramid::readFromCharArray(const char*& src, size_t& size) {
    srrg2_core::readFromCharArray(_timestamp, src, size);
    size_t num_levels;
    srrg2_core::readFromCharArray(num_levels, src, size);
    if (!num_levels) {
      _levels.clear();
      _relative_scales.clear();
      return;
    }
    resize(num_levels);
    for (size_t i = 0; i < num_levels; ++i) {
      srrg2_core::readFromCharArray(_relative_scales[i], src, size);
    }

    // read the first level
    _levels[0].reset(new MDPyramidLevel(0, 0));
    _levels[0]->readFromCharArray(src, size);
    for (size_t i = 1; i < _levels.size(); ++i) {
      _levels[i].reset(new MDPyramidLevel(0, 0));
      _levels[0]->scaleTo(*_levels[i], _relative_scales[i]);
    }
  }

  void MDImagePyramid::write(std::ostream& os) const {
    static constexpr size_t b_size = 1024 * 1024 * 50; // 50 MB of buffer
    char* buffer                   = new char[b_size];
    size_t size                    = b_size;
    char* buffer_end               = buffer;
    this->writeToCharArray(buffer_end, size);
    size_t real_size = buffer_end - buffer;
    os.write(buffer, real_size);
    delete[] buffer;
  }

  bool MDImagePyramid::read(std::istream& is) {
    static constexpr size_t b_size = 1024 * 1024 * 50; // 50 MB of buffer
    char* buffer                   = new char[b_size];
    is.read(buffer, b_size);
    size_t real_size       = is.gcount();
    const char* buffer_end = buffer;
    this->readFromCharArray(buffer_end, real_size);
    delete[] buffer;
    return true;
  }
} // namespace md_slam
