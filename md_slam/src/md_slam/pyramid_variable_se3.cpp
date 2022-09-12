#include "pyramid_variable_se3.h"

namespace md_slam {
  using namespace std;
  using namespace srrg2_solver;
  void MDVariableSE3::serialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context) {
    VariableSE3QuaternionRight::serialize(odata, context);
    ObjectData* reference_data = new ObjectData;
    _pyramid.serialize(*reference_data, context);
    odata.setDouble("timestamp", pyramid()->timestamp());
    odata.setField("pyramid", reference_data);
  }

  void MDVariableSE3::deserialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context) {
    VariableSE3QuaternionRight::deserialize(odata, context);
    _timestamp                 = odata.getDouble("timestamp");
    ObjectData* reference_data = dynamic_cast<ObjectData*>(odata.getField("pyramid"));
    if (!reference_data) {
      throw std::runtime_error("MDVariableSE3::deserialize | no zero level");
    }
    _pyramid.deserialize(*reference_data, context);
  }
} // namespace md_slam
