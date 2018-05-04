void add_lane(
  beacls::FloatVec& alpha_temp,
  beacls::FloatVec alpha,
  FLOAT_TYPE lane_width,
  FLOAT_TYPE theta_offset,
  FLOAT_TYPE vehicle_width,
  beacls::FloatVec xs){


    if (dim == 0 || dim == 1){
      std::transform(xs.cbegin(), xs.cend(), lane.begin(),
      [alpha_offset, length, vehicle_width](const auto &xs_i) {
        return 1- std::pow(((xs_i - alpha_offset)/((lane_width/2.)-vehicle_width)),2); });
      }
      else if (dim == 2) {
        std::transform(xs.cbegin(), xs.cend(), lane.begin(), lane.begin(),
        [theta_offset, length](const auto &xs_i, const auto &lane_i) {
          return lane_i - std::pow(((xs_i- theta_offset)/(M_PI)),2); });
        }
      }
    }
