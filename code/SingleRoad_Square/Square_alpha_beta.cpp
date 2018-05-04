#include "alpha_sq.cpp"
#include "beta_sq.cpp"
void Square_alpha_beta(
  beacls::FloatVec& alpha,
  beacls::FloatVec& beta,
  levelset::HJI_Grid* g)
  {

    FLOAT_TYPE alpha_offset = 0.;
    FLOAT_TYPE beta_offset = 0.;
    FLOAT_TYPE length = 4.;

    const size_t numel = g->get_numel();
    const size_t num_dim = g->get_num_of_dimensions();

    alpha.assign(numel, 0.);
    beta.assign(numel, 0.);

    for (size_t dim = 0; dim < num_dim; ++dim) {
      const beacls::FloatVec &xs = g->get_xs(dim);

      if (dim == 0 || dim == 1){
        alpha_sq(alpha, xs, dim, numel, alpha_offset, length);
        beta_sq(beta, xs, dim, numel, beta_offset, length);
      }
    }
  }
