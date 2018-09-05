void add_sq(
  beacls::FloatVec& lane,
  beacls::FloatVec xs,
  FLOAT_TYPE xg,
  FLOAT_TYPE yg,
  FLOAT_TYPE x0,
  FLOAT_TYPE y0,
  size_t dim){

  FLOAT_TYPE a = 3.;
  FLOAT_TYPE b = (a-2.)/(std::pow(x0-xg,2)+std::pow(y0-yg,2));

    if (dim == 0){
      //reset the default values
      std::fill(lane.begin(),lane.end(),0.);
      // for (int i = 0; i<lane.size(); ++i){
      //   if (lane[i] <= -10.){
      //     lane[i]=0.;
      //   }

      std::transform(xs.cbegin(), xs.cend(), lane.begin(), lane.begin(),
      [a, b, xg](const auto &xs_i, const auto &lane_i) {
        return lane_i + a - b*std::pow((xs_i - xg),2); });

      } else if (dim == 1){

        std::transform(xs.cbegin(), xs.cend(), lane.begin(), lane.begin(),
        [b, yg](const auto &xs_i, const auto &lane_i) {
          return lane_i - b*std::pow((xs_i - yg),2); });


        }
      }
