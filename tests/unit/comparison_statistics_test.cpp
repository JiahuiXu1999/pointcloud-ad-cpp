#include "comparison_statistics.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <string_view>
#include <vector>

namespace {

using pointcloud_ad::comparison::CoverageField;
using pointcloud_ad::comparison::CoverageReason;
using pointcloud_ad::comparison::CoverageSample;
using pointcloud_ad::comparison::CoverageSummary;
using pointcloud_ad::comparison::DeviationField;
using pointcloud_ad::comparison::DeviationReason;
using pointcloud_ad::comparison::DeviationSample;
using pointcloud_ad::comparison::DeviationStatistics;
using pointcloud_ad::comparison::summarize_coverage;
using pointcloud_ad::comparison::summarize_deviation;

bool expect(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}

bool near(double actual, double expected, double tolerance = 1.0e-9) {
  return std::abs(actual - expected) <= tolerance;
}

} // namespace

int main() {
  bool passed = true;

  // Deviation statistics over a known sample set: 1, 2, 3, 4 plus two invalid reasons.
  {
    std::vector<DeviationSample> samples;
    for (double value : {1.0, 2.0, 3.0, 4.0}) {
      DeviationSample sample;
      sample.signed_mm = value;
      sample.reason = DeviationReason::valid;
      samples.push_back(sample);
    }
    {
      DeviationSample sample;
      sample.reason = DeviationReason::no_neighbor;
      samples.push_back(sample);
    }
    {
      DeviationSample sample;
      sample.reason = DeviationReason::input_invalid;
      samples.push_back(sample);
    }
    DeviationField field(std::move(samples), 4U);
    const DeviationStatistics statistics = summarize_deviation(field);

    passed &= expect(statistics.valid_count == 4U, "deviation valid count must be 4");
    passed &= expect(near(statistics.mean_signed_mm, 2.5), "mean signed deviation must be 2.5");
    passed &= expect(near(statistics.rms_mm, std::sqrt(7.5)), "RMS deviation must be sqrt(7.5)");
    passed &= expect(near(statistics.max_abs_mm, 4.0), "max absolute deviation must be 4.0");
    passed &= expect(near(statistics.p95_abs_mm, 4.0), "p95 absolute deviation must be 4.0");
    passed &= expect(statistics.no_neighbor == 1U, "no_neighbor count must be 1");
    passed &= expect(statistics.input_invalid == 1U, "input_invalid count must be 1");
    passed &= expect(statistics.normal_missing == 0U && statistics.normal_mismatch == 0U &&
                         statistics.reference_boundary == 0U,
                     "unused deviation reasons must count zero");
  }

  // All-invalid deviation field must report zero statistics.
  {
    std::vector<DeviationSample> samples;
    for (int index = 0; index < 3; ++index) {
      DeviationSample sample;
      sample.reason = DeviationReason::normal_missing;
      samples.push_back(sample);
    }
    DeviationField field(std::move(samples), 0U);
    const DeviationStatistics statistics = summarize_deviation(field);
    passed &= expect(statistics.valid_count == 0U && statistics.normal_missing == 3U &&
                         near(statistics.mean_signed_mm, 0.0) && near(statistics.rms_mm, 0.0),
                     "all-invalid deviation field must report zero statistics");
  }

  // Coverage summary over a known sample set.
  {
    std::vector<CoverageSample> samples;
    for (int index = 0; index < 3; ++index) {
      CoverageSample sample;
      sample.reason = CoverageReason::covered;
      samples.push_back(sample);
    }
    for (int index = 0; index < 2; ++index) {
      CoverageSample sample;
      sample.reason = CoverageReason::no_neighbor;
      samples.push_back(sample);
    }
    {
      CoverageSample sample;
      sample.reason = CoverageReason::scan_boundary;
      samples.push_back(sample);
    }
    {
      CoverageSample sample;
      sample.reason = CoverageReason::input_invalid;
      samples.push_back(sample);
    }
    CoverageField field(std::move(samples), 3U, 6U);
    const CoverageSummary summary = summarize_coverage(field);

    passed &= expect(summary.covered_count == 3U, "covered count must be 3");
    passed &= expect(summary.valid_count == 6U, "valid reference count must be 6");
    passed &= expect(near(summary.coverage_ratio, 0.5), "coverage ratio must be 0.5");
    passed &= expect(summary.no_neighbor == 2U, "no_neighbor count must be 2");
    passed &= expect(summary.scan_boundary == 1U, "scan_boundary count must be 1");
    passed &= expect(summary.input_invalid == 1U, "input_invalid count must be 1");
  }

  return passed ? 0 : 1;
}
