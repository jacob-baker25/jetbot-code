/* Runtime-only admission policy for Karto sequential scan matches. */
#ifndef KARTO_SDK__SEQUENTIAL_MATCH_GUARD_H_
#define KARTO_SDK__SEQUENTIAL_MATCH_GUARD_H_

#include <cmath>

namespace karto
{

enum class SequentialMatchDecisionReason
{
  Disabled,
  Accepted,
  Nonfinite,
  LowResponse,
  TranslationLimit,
  RotationLimit
};

struct SequentialMatchGuardConfig
{
  double minimum_response;
  double maximum_translation;
  double maximum_rotation;
};

struct SequentialMatchDecision
{
  bool enabled;
  bool accepted;
  SequentialMatchDecisionReason reason;
  double translation_correction;
  double rotation_correction;
};

inline const char * SequentialMatchDecisionReasonName(
  SequentialMatchDecisionReason reason)
{
  switch (reason) {
    case SequentialMatchDecisionReason::Disabled:
      return "disabled";
    case SequentialMatchDecisionReason::Accepted:
      return "accepted";
    case SequentialMatchDecisionReason::Nonfinite:
      return "nonfinite";
    case SequentialMatchDecisionReason::LowResponse:
      return "low_response";
    case SequentialMatchDecisionReason::TranslationLimit:
      return "translation_limit";
    case SequentialMatchDecisionReason::RotationLimit:
      return "rotation_limit";
  }
  return "unknown";
}

inline SequentialMatchDecision EvaluateSequentialMatch(
  const SequentialMatchGuardConfig & config,
  double response,
  double search_x,
  double search_y,
  double search_heading,
  double best_x,
  double best_y,
  double best_heading)
{
  const bool enabled = config.minimum_response > 0.0 ||
    config.maximum_translation > 0.0 || config.maximum_rotation > 0.0;
  SequentialMatchDecision decision = {
    enabled, true, SequentialMatchDecisionReason::Disabled, 0.0, 0.0};

  if (!enabled) {
    return decision;
  }

  if (!std::isfinite(response) || !std::isfinite(search_x) ||
    !std::isfinite(search_y) || !std::isfinite(search_heading) ||
    !std::isfinite(best_x) || !std::isfinite(best_y) ||
    !std::isfinite(best_heading))
  {
    decision.accepted = false;
    decision.reason = SequentialMatchDecisionReason::Nonfinite;
    return decision;
  }

  decision.translation_correction = std::hypot(best_x - search_x, best_y - search_y);
  decision.rotation_correction = std::abs(std::atan2(
      std::sin(best_heading - search_heading),
      std::cos(best_heading - search_heading)));

  if (config.minimum_response > 0.0 && response < config.minimum_response) {
    decision.accepted = false;
    decision.reason = SequentialMatchDecisionReason::LowResponse;
  } else if (config.maximum_translation > 0.0 &&
    decision.translation_correction > config.maximum_translation)
  {
    decision.accepted = false;
    decision.reason = SequentialMatchDecisionReason::TranslationLimit;
  } else if (config.maximum_rotation > 0.0 &&
    decision.rotation_correction > config.maximum_rotation)
  {
    decision.accepted = false;
    decision.reason = SequentialMatchDecisionReason::RotationLimit;
  } else {
    decision.reason = SequentialMatchDecisionReason::Accepted;
  }
  return decision;
}

}  // namespace karto

#endif  // KARTO_SDK__SEQUENTIAL_MATCH_GUARD_H_
