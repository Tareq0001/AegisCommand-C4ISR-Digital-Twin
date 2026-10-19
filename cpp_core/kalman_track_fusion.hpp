/**
 * AegisCommand C4ISR — C++17 Unscented Kalman Filter (UKF) Ballistic Track Fusion
 * Standard: NATO STANAG 4586 / High-G Maneuvering Hypersonic & Ballistic Tracking
 * Architecture: Scaled Unscented Transform (Van der Merwe / Julier-Uhlmann)
 */

#ifndef AEGIS_KALMAN_TRACK_FUSION_HPP
#define AEGIS_KALMAN_TRACK_FUSION_HPP

#include <array>
#include <vector>
#include <cmath>
#include <cstdint>
#include <string>

namespace aegis::c4isr {

constexpr size_t STATE_DIM = 6;      // [x, y, z, vx, vy, vz]
constexpr size_t MEAS_DIM = 4;       // [range, azimuth, elevation, doppler]
constexpr size_t SIGMA_COUNT = 2 * STATE_DIM + 1; // 13 sigma points

struct RadarMeasurement {
    double timestamp;     // Seconds
    double range;         // Slant range in meters
    double azimuth;       // Radians [0 .. 2*PI)
    double elevation;     // Radians [-PI/2 .. +PI/2]
    double dopplerRadial; // Radial velocity in m/s (negative closing)
    std::array<double, MEAS_DIM> noiseVariance; // [sigma_r^2, sigma_az^2, sigma_el^2, sigma_dop^2]
};

struct TrackEstimate {
    uint32_t trackId;
    double timestamp;
    std::array<double, STATE_DIM> state;             // [x, y, z, vx, vy, vz]
    std::array<std::array<double, STATE_DIM>, STATE_DIM> covariance; // 6x6 P
    double trackQuality;                             // [1.0 .. 15.0]
    bool confirmed;
};

class UnscentedKalmanFilter {
public:
    UnscentedKalmanFilter();
    ~UnscentedKalmanFilter() = default;

    void initialize(const std::array<double, STATE_DIM>& initialState,
                    const std::array<std::array<double, STATE_DIM>, STATE_DIM>& initialCov);

    void predict(double dt);
    void update(const RadarMeasurement& meas);

    [[nodiscard]] const std::array<double, STATE_DIM>& getState() const { return m_state; }
    [[nodiscard]] const std::array<std::array<double, STATE_DIM>, STATE_DIM>& getCovariance() const { return m_covP; }

    // Multi-sensor Covariance Intersection (CI) fusion
    static TrackEstimate fuseTracksCovarianceIntersection(
        const TrackEstimate& trackA,
        const TrackEstimate& trackB,
        double omegaWeight = 0.5);

private:
    std::array<double, STATE_DIM> m_state{};
    std::array<std::array<double, STATE_DIM>, STATE_DIM> m_covP{};

    // Scaled Unscented Transform Parameters
    double m_alpha{1e-3};
    double m_beta{2.0};
    double m_kappa{0.0};
    double m_lambda{0.0};

    std::array<double, SIGMA_COUNT> m_weightsMean{};
    std::array<double, SIGMA_COUNT> m_weightsCov{};

    void computeSigmaPoints(std::array<std::array<double, STATE_DIM>, SIGMA_COUNT>& sigmaPoints);
    static std::array<double, STATE_DIM> propagateKinematics(const std::array<double, STATE_DIM>& x, double dt);
    static std::array<double, MEAS_DIM> measurementFunction(const std::array<double, STATE_DIM>& x);
};

} // namespace aegis::c4isr

#endif // AEGIS_KALMAN_TRACK_FUSION_HPP
