#ifndef NMEA_H_
#define NMEA_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct
{
    bool rmc_valid;
    bool gns_valid;
    int gga_sats;
    int gns_sats;
} nmea_parse_state_t;

typedef struct
{
    int min_fix_quality;
    int min_satellites;
    bool enable_gns_support;
} nmea_parse_config_t;

typedef struct
{
    uint32_t nmea_total;
    uint32_t gga_total;
    uint32_t rmc_total;
    uint32_t gns_total;
    uint32_t unknown_total;
    uint32_t malformed_total;
    uint32_t checksum_fail_total;
    uint32_t oversize_total;
    uint32_t gga_reject_total;
    uint32_t gns_reject_total;
    uint32_t rmc_invalid_status_total;
    uint32_t speed_parse_fail_total;
    uint32_t coord_parse_fail_total;
    uint32_t valid_fix_total;
    uint32_t msgq_timeout_total;
    uint32_t signal_lost_total;
} nmea_obs_stats_t;

void nmea_reset_parse_state(nmea_parse_state_t* state);
void nmea_reset_obs_stats(nmea_obs_stats_t* obs);
void nmea_get_default_config(nmea_parse_config_t* config);

double nmea_knots_to_kmh(double knots);
double nmea_knots_to_ms(double knots);

bool nmea_process_sentence(const char* nmea,
                           nmea_parse_state_t* state,
                           const nmea_parse_config_t* config,
                           nmea_obs_stats_t* obs,
                           int* sat_out,
                           char* time_out,
                           size_t time_out_len,
                           double* speed_knots_out,
                           double* lat_out,
                           double* lon_out);

void nmea_build_observability_summary(const nmea_obs_stats_t* obs,
                                      int uptime_ms,
                                      int first_fix_ms,
                                      int captured_frames,
                                      int uart_overflow_delta,
                                      int uart_line_drop_delta,
                                      int log_drop_delta,
                                      bool session_success,
                                      char* out,
                                      size_t out_len);

#endif /* NMEA_H_ */
