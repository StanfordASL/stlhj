void alpha_road(
    beacls::FloatVec& alpha,
    beacls::FloatVec xs,
    FLOAT_TYPE dim,
    const size_t numel,
    FLOAT_TYPE alpha_offset,
    FLOAT_TYPE theta_offset,
    FLOAT_TYPE vehicle_width,
    FLOAT_TYPE length){

    if (dim == 0){
      std::transform(xs.cbegin(), xs.cend(), alpha.begin(),
          [alpha_offset, length, vehicle_width](const auto &xs_i) {
          return 1- std::pow(((xs_i - alpha_offset)/(length-vehicle_width)),2); });
    }
    else if (dim == 2) {
    std::transform(xs.cbegin(), xs.cend(), alpha.begin(), alpha.begin(),
        [theta_offset, length](const auto &xs_i, const auto &alpha_i) {
        return alpha_i - std::pow(((xs_i- theta_offset)/(M_PI)),2); });
    }

}
