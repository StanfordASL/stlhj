
void add_sq(
  beacls::FloatVec& lane,
  beacls::FloatVec xs,
  FLOAT_TYPE lane_offset,
  FLOAT_TYPE lane_width,
  FLOAT_TYPE vehicle_width,
  FLOAT_TYPE theta_min,
  FLOAT_TYPE theta_max,
  const size_t numel,
  size_t dim){


    if (dim == 0){
      //reset the default values
      for (int i = 0; i<lane.size(); ++i){
        if (lane[i] <= -10.){
          lane[i]=0.;
        }
      }

      std::transform(xs.cbegin(), xs.cend(), lane.begin(),
      [lane_offset, lane_width, vehicle_width](const auto &xs_i) {
        return 1- std::pow(((xs_i - lane_offset)/((lane_width-vehicle_width)/2.)),2); });

      } else if (dim == 1) {
        beacls::FloatVec lane_temp;
        lane_temp.assign(lane.size(), 0.);
        std::transform(xs.cbegin(), xs.cend(), lane_temp.begin(),
        [lane_offset, lane_width, vehicle_width](const auto &xs_i) {
          return 1- std::pow(((xs_i - lane_offset)/((lane_width-vehicle_width)/2.)),2); });

          //    printf("%u\n",lane_temp.size());
          //    printf("%u\n",lane.size());
          //
          //
          std::transform(lane_temp.cbegin(), lane_temp.cend(), lane.begin(), lane.begin(),
          [](const auto &lane_temp_i, const auto &lane_i) {
            return std::max(lane_temp_i,lane_i); });

          } else if (dim == 2) {

              FLOAT_TYPE min_theta_diff;
              for (int i=0;i<lane.size();i++){
                if (xs[i]<theta_min || xs[i]>theta_max){
                  min_theta_diff = std::min(theta_diff(xs[i],theta_min),theta_diff(xs[i],theta_max));
                  lane[i] = lane[i] - 5.*std::pow(min_theta_diff/M_PI,2);

                }
              }
            }
          }
