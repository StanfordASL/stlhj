void beta_sq(
    beacls::FloatVec& beta,
    beacls::FloatVec xs,
    FLOAT_TYPE dim,
    const size_t numel,
    FLOAT_TYPE beta_offset,
    FLOAT_TYPE length){

    if (dim == 0){
      std::transform(xs.cbegin(), xs.cend(), beta.begin(),
          [beta_offset, length](const auto &xs_i) {
          return 1- std::pow(((xs_i - beta_offset)/length),2); });
    }
    else if (dim == 1) {
    beacls::FloatVec beta_temp;
    beta_temp.assign(numel, 0.);
    std::transform(xs.cbegin(), xs.cend(), beta_temp.begin(),
        [beta_offset, length](const auto &xs_i) {
        return 1- std::pow(((xs_i - beta_offset)/length),2); });

    std::transform(beta_temp.cbegin(), beta_temp.cend(), beta.begin(), beta.begin(),
        [](const auto &beta_temp_i, const auto &beta_i) {
        return std::min(beta_temp_i,beta_i); });

    std::transform(beta.cbegin(), beta.cend(), beta.begin(),
        std::negate<FLOAT_TYPE>());
    }

}
