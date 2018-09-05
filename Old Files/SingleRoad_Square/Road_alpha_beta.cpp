#include "alpha_road.cpp"
#include "beta_road.cpp"
void Road_alpha_beta(
  beacls::FloatVec& alpha,
  beacls::FloatVec& beta,
  levelset::HJI_Grid* g)
  {

   FLOAT_TYPE alpha_offset = 0.;
   FLOAT_TYPE beta_offset = 0.;
   FLOAT_TYPE theta_offset = M_PI/2;
   FLOAT_TYPE vehicle_width = 0.5;
   FLOAT_TYPE length = 2.;

   const size_t numel = g->get_numel();
   const size_t num_dim = g->get_num_of_dimensions();
   alpha.assign(numel, 0.);
   beta.assign(numel, 0.);

   for (size_t dim = 0; dim < num_dim; ++dim) {
     const beacls::FloatVec &xs = g->get_xs(dim);

     if (dim == 0 || dim == 2){
        alpha_road(alpha, xs, dim, numel, alpha_offset, theta_offset, vehicle_width, length);
        beta_road(beta, xs, dim, numel, beta_offset, length);
     }
   }
  }
