void update_lane(
  beacls::FloatVec alpha_temp,
  beacls::FloatVec& alpha,
  const std::vector<size_t> shape,
  beacls::FloatVec range,
  beacls::FloatVec gmin,
  beacls::FloatVec gmax,
  const size_t dim){

    FLOAT_TYPE x_unit = (shape[0]-1)/(gmax[0]-gmin[0]);
    FLOAT_TYPE y_unit = (shape[1]-1)/(gmax[1]-gmin[1]);
    long unsigned int x1 = (long unsigned int)(x_unit*(range[0]-gmin[0])+0.5);
    long unsigned int x2 = (long unsigned int)(x_unit*(range[1]-gmin[0])+0.5);
    beacls::IntegerVec x_index{x1,x2};
    long unsigned int y1 = (long unsigned int)(y_unit*(range[2]-gmin[1])+0.5);
    long unsigned int y2 = (long unsigned int)(y_unit*(range[3]-gmin[1])+0.5);
    beacls::IntegerVec y_index{y1,y2};

    int x_size, y_size, y, z;
    unsigned long int n_dist = 0;
    beacls::IntegerVec n;
    x_size = shape[0];
    y_size = shape[1];

    for (z = 0; z <= shape[2]-1; ++z) {
      for (y = y_index[0]; y <= y_index[1]; ++y){
        n = {z*y_size*x_size + y*x_size + x_index[0], z*y_size*x_size + y*x_size + x_index[1]};
        std::copy(alpha_temp.begin()+n_dist,alpha_temp.begin()+n_dist+n[1]-n[0]+1,alpha.begin()+n[0]);
        n_dist = n_dist + n[1] - n[0] + 1;

      }
    }
  }
