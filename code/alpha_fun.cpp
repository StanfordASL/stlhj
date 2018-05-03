void alpha_fun(
    beacls::FloatVec& alpha,
    beacls::FloatVec xs,
    FLOAT_TYPE alpha_offset){

      std::transform(xs.cbegin(), xs.cend(), alpha.begin(),
          [alpha_offset](const auto &xs_i) {
          return xs_i - alpha_offset; });

}
