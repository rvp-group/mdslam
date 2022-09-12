#pragma once

namespace srrg2_core {
  constexpr size_t pcl_alignment=16;

  constexpr size_t roundup(size_t num, size_t multiple) {
    const size_t mod = num % multiple;
    return mod == 0 ? num : num + multiple - mod;
  }

  template <typename FieldPackType_, int i>
  static constexpr size_t field_offset_ =
    roundup(sizeof(typename FieldPackType_::template TypeAt<i>), pcl_alignment)+field_offset_<typename FieldPackType_::FieldPackRestType, i-1>;

  template <typename FieldPackType_>
  static constexpr size_t field_offset_<FieldPackType_, 0> = 0;

}
