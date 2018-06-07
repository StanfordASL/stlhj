void add_lane(
  beacls::FloatVec& lane,
  beacls::FloatVec xs,
  FLOAT_TYPE lane_offset,
  FLOAT_TYPE lane_width,
  FLOAT_TYPE theta_offset,
  FLOAT_TYPE vehicle_width,
  size_t dim,
  beacls::FloatVec vrange){


    if (dim == 0 || dim == 1){
      //reset the default values
        std::fill(lane.begin(),lane.end(),0.);
      // for (int i = 0; i<lane.size(); ++i){
      //   if (lane[i] <= -10.){
      //     lane[i]=0.;
      //   }


      std::transform(xs.cbegin(), xs.cend(), lane.begin(),
      [lane_offset, lane_width, vehicle_width](const auto &xs_i) {
        return 1- std::pow(((xs_i - lane_offset)/((lane_width-vehicle_width)/2.)),2); });

      } else if (dim == 2) {

        std::transform(xs.cbegin(), xs.cend(), lane.begin(), lane.begin(),
        [theta_offset](const auto &xs_i, const auto &lane_i) {
          return lane_i - 5.*std::pow((theta_diff(xs_i,theta_offset)/(M_PI)),2); });

        }

        else if (dim == 3) {
            for (int i = 0; i <= lane.size()-1; ++i) {
              if (xs[i]>vrange[1]){
                lane[i] = -1.;
              }
            }
        }
      }
