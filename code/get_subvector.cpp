void get_subvector(
  beacls::FloatVec& alpha_temp,
  beacls::FloatVec alpha,
  const std::vector<size_t> shape,
  const beacls::FloatVec range,
  beacls::FloatVec gmin,
  beacls::FloatVec gmax,
  const size_t dim){

  FLOAT_TYPE x_unit = shape[0]/(gmax[0]-gmin[0]);
  FLOAT_TYPE y_unit = shape[1]/(gmax[1]-gmin[1]);
  beacls::IntegerVec x_index{int(x_unit*range[0]+0.5),int(x_unit*range[1]+0.5)};
  beacls::IntegerVec y_index{int(y_unit*range[2]+0.5),int(y_unit*range[3]+0.5)};

    if (dim == 0 ){

      alpha_temp = alpha

    }





    }| dim == 1){
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
