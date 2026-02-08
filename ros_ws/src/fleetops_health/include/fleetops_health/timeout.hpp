#pragma once

inline bool is_timed_out(double age_sec, double timeout_sec)
{
  return age_sec > timeout_sec;
}