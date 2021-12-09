#include <algorithm>
#include <array>

namespace sun {

template <typename T, std::size_t N> class DiscreteTimeFilter {
private:
  /* data */
  T a, b;

  std::array<T, N> u_kminus1_, u_k_, yk_;

  DiscreteTimeFilter();

  void update_input(const std::array<T, N> &input) {
    u_kminus1_ = u_k_;
    u_k_ = input;
  }

  void update_input(const T *input) {
    u_kminus1_ = u_k_;
    memcpy(u_k_.data(), input, N * sizeof(T));
  }

  void update_output() {
    for (int i = 0; i < N; i++) {
      yk_[i] = b * (u_k_[i] + u_kminus1_[i]) - a * yk_[i];
    }
  }

public:
  DiscreteTimeFilter(T cut_time_over_Ts) {
    T alpha = 2 * cut_time_over_Ts;
    b = T(1) / (T(1) + alpha);
    a = (T(1) - alpha) / (T(1) + alpha);
  }

  ~DiscreteTimeFilter() = default;

  void apply_filter(const std::array<T, N> &input) {
    update_input(input);
    update_output();
  }

  void apply_filter(const T *input) {
    update_input(input);
    update_output();
  }

  void set_output(const std::array<T, N> &out) {
    u_kminus1_ = out;
    u_k_ = out;
    yk_ = out;
  }

  void set_output(const T *out) {
    memcpy(u_kminus1_.data(), out, N * sizeof(T));
    memcpy(u_k_.data(), out, N * sizeof(T));
    memcpy(yk_.data(), out, N * sizeof(T));
  }

  const std::array<T, N> &get_output() { return yk_; }

  const T *get_output_ptr() { return yk_.data(); }
};

} // namespace sun
