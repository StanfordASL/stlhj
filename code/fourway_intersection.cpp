void fourway_intersection(
  beacls::FloatVec& lane,
  levelset::HJI_Grid* g,
  beacls::FloatVec gmin,
  beacls::FloatVec gmax)
  {

    FLOAT_TYPE _offset = 0.;
    FLOAT_TYPE theta_offset = M_PI/2;
    FLOAT_TYPE vehicle_width = 0.5;
    FLOAT_TYPE length = 2.;

    const size_t numel = g->get_numel();
    const size_t num_dim = g->get_num_of_dimensions();
    const std::vector<size_t> shape = g->get_shape();

    lane.assign(numel, -20.);
    beacls::FloatVec lane_temp;
    std::vector<beacls::FloatVec> xs_temp;

    for (size_t dim = 0; dim < num_dim; ++dim) {

      if (dim == 0 || dim ==2){
        const beacls::FloatVec &xs = g->get_xs(dim);
        const beacls::FloatVec range{0.,4.,-12.-4.}; //{xmin,xmax,ymin,ymax}
        get_subvector(lane_temp,lane,shape,range,gmin,gmax,dim);
        get_subvector(xs_temp,xs,shape,range,gmin,gmax,dim);
        add_lane(lane_temp,xs_temp);
        update_lane(lane_temp,lane);

      }



    }
  }
