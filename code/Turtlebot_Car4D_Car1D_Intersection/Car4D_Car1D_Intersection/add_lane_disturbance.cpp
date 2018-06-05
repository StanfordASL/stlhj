void add_lane_disturbance(
  beacls::FloatVec& lane,
  const std::vector<size_t> shape,
  beacls::FloatVec range,
  beacls::FloatVec gmin,
  beacls::FloatVec gmax,
  FLOAT_TYPE vehicle_width,
  size_t dim){

    FLOAT_TYPE x_unit = (static_cast<float>(shape[0])-1)/(gmax[0]-gmin[0]);
    FLOAT_TYPE y_unit = (static_cast<float>(shape[1])-1)/(gmax[1]-gmin[1]);
    long unsigned int x1 = (long unsigned int)(x_unit*(range[0]-gmin[0])+0.5);
    long unsigned int x2 = (long unsigned int)(x_unit*(range[1]-gmin[0])+0.5);
    beacls::IntegerVec x_index{x1,x2};
    long unsigned int y1 = (long unsigned int)(y_unit*(range[2]-gmin[1])+0.5);
    long unsigned int y2 = (long unsigned int)(y_unit*(range[3]-gmin[1])+0.5);
    beacls::IntegerVec y_index{y1,y2};

    long unsigned int x_size, y_size, z_size, a_size, b_size, y, z, a, b, b2y, yz_unit;

    FLOAT_TYPE y_add = (y_unit*vehicle_width);
    // printf("y_unit=%f\n",y_unit);
    // printf("y_add=%f\n",y_add);
    FLOAT_TYPE x_add = (x_unit*vehicle_width);
    // printf("x_unit=%f\n",x_unit);
    // printf("x_add=%f\n",x_add);

    beacls::IntegerVec n;
    x_size = x2-x1+1;
    // printf("x_size=%lu\n",x_size);
    y_size = y2-y1+1;
    z_size = shape[2];
    a_size = shape[3];
    b_size = shape[4];
    FLOAT_TYPE by_unit = static_cast<float>(b_size)/y_size;
    beacls::IntegerVec b2y_index;
    beacls::IntegerVec y_addvec{0,0};
    beacls::IntegerVec x_addvec{(long unsigned int)((static_cast<float>(x_size)-1.)/2.-x_add/2.-0.5),(long unsigned int)((static_cast<float>(x_size)-1.)/2.+x_add/2.+0.5)};
    // printf("x_addvec=[%lu,%lu]\n",x_addvec[0],x_addvec[1]);

    for (b = 0; b < b_size; ++b) {
      b2y = (long unsigned int)(b/by_unit+0.5);
      b2y_index.push_back(b2y);
    }

    for (b = 0; b < b_size; ++b) {
      for (a = 0; a < a_size; ++a) {
        for(z = 0; z < z_size; ++z) {
          y_addvec[0] = (long unsigned int)(b2y_index[b]-y_add/2.-0.5);
          y_addvec[1] = (long unsigned int)(b2y_index[b]+y_add/2.+0.5);

          if (y_addvec[0]<gmin[1]){
            y_addvec[0] == gmin[1];
          }
          if (y_addvec[1]>gmax[1]){
            y_addvec[1] == gmax[1];
          }
          for (y = y_addvec[0]; y <= y_addvec[1]; ++y){
            n = {b*a_size*z_size*y_size*x_size + a*z_size*y_size*x_size + z*y_size*x_size + y*x_size + x_addvec[0], b*a_size*z_size*y_size*x_size + a*z_size*y_size*x_size + z*y_size*x_size + y*x_size + x_addvec[1]};
            std::fill(lane.begin()+n[0],lane.begin()+n[1],-2.);
          }
        }
      }
    }
  }
