void beta_fun(
    beacls::FloatVec& beta,
    beacls::FloatVec xs,
    FLOAT_TYPE beta_offset){
      // beta = (x0-beta_offset)^2 + (x1-beta_offset)^2
      std::transform(xs.cbegin(), xs.cend(), beta.begin(), beta.begin(),
          [beta_offset](const auto &xs_i, const auto &beta_i) {
          return beta_i - std::pow((xs_i - beta_offset), 2); });

}
