FLOAT_TYPE theta_diff(
  FLOAT_TYPE a,
  FLOAT_TYPE b)
  {
  FLOAT_TYPE mod = std::fmod(std::abs(a-b),(2.*M_PI));
  if (mod>M_PI){
    mod = 2.*M_PI - mod;
  }
  return mod;
}
