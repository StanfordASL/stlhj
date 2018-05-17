void add_sq(
    beacls::FloatVec& lane,
    beacls::FloatVec xs,
    FLOAT_TYPE sq_offset,
    FLOAT_TYPE length,
    FLOAT_TYPE dim,
    const size_t numel){

    if (dim == 0){
      std::transform(xs.cbegin(), xs.cend(), lane.begin(),
          [lane_offset, length](const auto &xs_i) {
          return 1- std::pow(((xs_i - sq_offset)/length),2); });
    }
    else if (dim == 1) {
    beacls::FloatVec lane_temp;
    lane_temp.assign(numel, 0.);
    std::transform(xs.cbegin(), xs.cend(), lane_temp.begin(),
        [lane_offset, length](const auto &xs_i) {
        return 1- std::pow(((xs_i - sq_offset)/length),2); });

    std::transform(lane_temp.cbegin(), lane_temp.cend(), lane.begin(), lane.begin(),
        [](const auto &lane_temp_i, const auto &lane_i) {
        return std::min(lane_temp_i,lane_i); });
    }

}
