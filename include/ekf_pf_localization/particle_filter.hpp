#pragma once

#include "ekf_pf_localization/common.hpp"

namespace ekf_pf_localization
{

struct Particle
{
  State state;
  double weight;
};

class ParticleFilter
{
public:
  ParticleFilter(int num_particles)
  : num_particles_(num_particles)
  {
    particles_.resize(num_particles_);
  }

  void initialize(const State & initial_state, double stddev)
  {
    for (auto & p : particles_)
    {
      p.state.x = initial_state.x + sampleNormal(0.0, stddev);
      p.state.y = initial_state.y + sampleNormal(0.0, stddev);
      p.state.theta = initial_state.theta + sampleNormal(0.0, stddev);
      p.weight = 1.0 / num_particles_;
    }
  }

  // ----------------------------
  // Prediction
  // ----------------------------
  void predict(const Control & u, double dt, double noise_std)
  {
    for (auto & p : particles_)
    {
      Control noisy_u = u;
      noisy_u.v += sampleNormal(0.0, noise_std);
      noisy_u.omega += sampleNormal(0.0, noise_std);

      p.state = propagateState(p.state, noisy_u, dt);
      p.state.theta = normalizeAngle(p.state.theta);
    }
  }

  // ----------------------------
  // Measurement Update
  // z = [x, y]
  // ----------------------------
  void updateWeights(const Eigen::Vector2d & z, double meas_std)
  {
    double sum_weights = 0.0;

    for (auto & p : particles_)
    {
      double dx = z(0) - p.state.x;
      double dy = z(1) - p.state.y;
      double error = dx * dx + dy * dy;

      p.weight = std::exp(-error / (2 * meas_std * meas_std));
      sum_weights += p.weight;
    }

    // Normalize
    for (auto & p : particles_)
      p.weight /= sum_weights;
  }

  // ----------------------------
  // Resampling
  // ----------------------------
  void resample()
  {
    std::vector<Particle> new_particles;
    new_particles.resize(num_particles_);

    std::vector<double> cumulative(num_particles_);
    cumulative[0] = particles_[0].weight;
    for (int i = 1; i < num_particles_; i++)
      cumulative[i] = cumulative[i - 1] + particles_[i].weight;

    for (int i = 0; i < num_particles_; i++)
    {
      double r = ((double) rand()) / RAND_MAX;
      for (int j = 0; j < num_particles_; j++)
      {
        if (r <= cumulative[j])
        {
          new_particles[i] = particles_[j];
          new_particles[i].weight = 1.0 / num_particles_;
          break;
        }
      }
    }

    particles_ = new_particles;
  }

  // ----------------------------
  // State Estimate
  // ----------------------------
  State estimate() const
  {
    State est{0, 0, 0};
    for (const auto & p : particles_)
    {
      est.x += p.state.x * p.weight;
      est.y += p.state.y * p.weight;
      est.theta += p.state.theta * p.weight;
    }
    est.theta = normalizeAngle(est.theta);
    return est;
  }

  const std::vector<Particle> & particles() const
  {
    return particles_;
  }

private:
  int num_particles_;
  std::vector<Particle> particles_;
};

}  // namespace ekf_pf_localization
