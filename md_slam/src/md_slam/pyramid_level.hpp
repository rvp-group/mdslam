namespace md_slam {

  bool MDPyramidLevel::getSubPixel(Vector5f& value,
                                   Matrix5_2f& derivative,
                                   const Vector2f& image_point) const {
    float c = image_point.x();
    float r = image_point.y();
    int r0  = (int) r;
    int c0  = (int) c;
    if (!matrix.inside(r0, c0))
      return false;

    int r1 = r0 + 1;
    int c1 = c0 + 1;
    if (!matrix.inside(r1, c1))
      return false;

    const MDPyramidMatrixEntry& p00 = matrix(r0, c0);
    const MDPyramidMatrixEntry& p01 = matrix(r0, c1);
    const MDPyramidMatrixEntry& p10 = matrix(r1, c0);
    const MDPyramidMatrixEntry& p11 = matrix(r1, c1);
    if (p00.masked() || p01.masked() || p10.masked() || p11.masked())
      return false;

    const float dr  = r - (float) r0;
    const float dc  = c - (float) c0;
    const float dr1 = 1.f - dr;
    const float dc1 = 1.f - dc;

    value = (p00.value * dc1 + p01.value * dc) * dr1 + (p10.value * dc1 + p11.value * dc) * dr;

    derivative = (p00.derivatives * dc1 + p01.derivatives * dc) * dr1 +
                 (p10.derivatives * dc1 + p11.derivatives * dc) * dr;

    return true;
  }

} // namespace md_slam
