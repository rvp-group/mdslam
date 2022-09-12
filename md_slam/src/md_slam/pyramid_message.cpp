#include "pyramid_message.h"

namespace md_slam {
  using namespace std; // erivaffanculo
  using namespace srrg2_core;

  void MDImagePyramidMessage::serialize(srrg2_core::ObjectData& odata,
                                        srrg2_core::IdContext& context) {
    BaseSensorMessage::serialize(odata, context);
    ObjectData* reference_data = new ObjectData;
    _pyramid.serialize(*reference_data, context);
    odata.setField("pyramid", reference_data);
  }

  void MDImagePyramidMessage::deserialize(srrg2_core::ObjectData& odata,
                                          srrg2_core::IdContext& context) {
    BaseSensorMessage::deserialize(odata, context);
    ObjectData* reference_data = dynamic_cast<ObjectData*>(odata.getField("pyramid"));
    if (!reference_data) {
      throw std::runtime_error("MDImagePyramidMessage::deserialize|no zero level");
    }
    _pyramid.deserialize(*reference_data, context);
  }
} // namespace md_slam
