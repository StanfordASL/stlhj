void add_lane_plain(
  beacls::FloatVec& lane,
  beacls::FloatVec xs,
  size_t dim,
  beacls::FloatVec vrange,
  beacls::FloatVec y2range){


    if (dim == 0 || dim == 1){
      //reset the default values
      std::fill(lane.begin(),lane.end(),2.5);
      // for (int i = 0; i<lane.size(); ++i){
      //   if (lane[i] <= -10.){
      //     lane[i]=0.;
      //   }


      // std::transform(xs.cbegin(), xs.cend(), lane.begin(),
      // [lane_offset, lane_width, vehicle_width](const auto &xs_i) {
      //   return 1- std::pow(((xs_i - lane_offset)/((lane_width-vehicle_width)/2.)),2); });

    }
     // else if (dim == 3) {
     //          for (int i = 0; i <= lane.size()-1; ++i) {
     //            if (xs[i]<vrange[0] || xs[i]>vrange[1]){
     //              lane[i] = -10.;
     //            }
     //          }
     //      }
     //      else if (dim == 4) {
     //        for (int i = 0; i <= lane.size()-1; ++i) {
     //          if (xs[i]<y2range[0] || xs[i]>y2range[1]){
     //            lane[i] = -10.;
     //          }
     //        }
     //      }
  }
