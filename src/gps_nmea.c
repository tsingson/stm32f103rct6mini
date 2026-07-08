#include "gps_nmea.h"

#include <ctype.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NMEA_LOCAL_BUF_SIZE 128
#define NMEA_FIELD_MAX      16
#define NMEA_DEFAULT_MIN_FIX_QUALITY 1
#define NMEA_DEFAULT_MIN_SATELLITES  1

static void normalize_config(const nmea_parse_config_t* in, nmea_parse_config_t* out)
{
    out->min_fix_quality = NMEA_DEFAULT_MIN_FIX_QUALITY;
    out->min_satellites = NMEA_DEFAULT_MIN_SATELLITES;
    out->enable_gns_support = true;

    if (in == NULL)
    {
        return;
    }

    if (in->min_fix_quality > 0)
    {
        out->min_fix_quality = in->min_fix_quality;
    }
    if (in->min_satellites > 0)
    {
        out->min_satellites = in->min_satellites;
    }
    out->enable_gns_support = in->enable_gns_support;
}

static int current_valid_sats(const nmea_parse_state_t* state)
{
    return (state->gga_sats > state->gns_sats) ? state->gga_sats : state->gns_sats;
}

static bool has_fused_fix(const nmea_parse_state_t* state, const nmea_parse_config_t* config)
{
    return (state->rmc_valid || state->gns_valid) &&
        (current_valid_sats(state) >= config->min_satellites);
}

void nmea_reset_parse_state(nmea_parse_state_t* state)
{
    if (state == NULL)
    {
        return;
    }

    state->rmc_valid = false;
    state->gns_valid = false;
    state->gga_sats = 0;
    state->gns_sats = 0;
}

void nmea_reset_obs_stats(nmea_obs_stats_t* obs)
{
    if (obs == NULL)
    {
        return;
    }

    (void)memset(obs, 0, sizeof(*obs));
}

void nmea_get_default_config(nmea_parse_config_t* config)
{
    if (config == NULL)
    {
        return;
    }

    config->min_fix_quality = NMEA_DEFAULT_MIN_FIX_QUALITY;
    config->min_satellites = NMEA_DEFAULT_MIN_SATELLITES;
    config->enable_gns_support = true;
}

double nmea_knots_to_kmh(double knots)
{
    return knots * 1.852;
}

double nmea_knots_to_ms(double knots)
{
    return knots * 0.5144444444444445;
}

static bool parse_nmea_coord(const char* coord, const char* dir, double* out)
{
    char* endptr = NULL;
    double raw;
    int degrees;
    double minutes;
    char hemi;

    if (coord == NULL || dir == NULL || out == NULL || coord[0] == '\0' || dir[0] == '\0')
    {
        return false;
    }

    raw = strtod(coord, &endptr);
    if (endptr == coord || !isfinite(raw) || raw < 0.0)
    {
        return false;
    }

    degrees = (int)(raw / 100.0);
    minutes = raw - (degrees * 100.0);
    if (minutes < 0.0 || minutes >= 60.0)
    {
        return false;
    }

    *out = (double)degrees + (minutes / 60.0);
    hemi = (char)toupper((unsigned char)dir[0]);
    if (hemi == 'S' || hemi == 'W')
    {
        *out = -*out;
    }
    else if (hemi != 'N' && hemi != 'E')
    {
        return false;
    }

    return true;
}

static bool parse_int_field(const char* text, int* out)
{
    char* endptr = NULL;
    long v;

    if (text == NULL || out == NULL || text[0] == '\0')
    {
        return false;
    }

    v = strtol(text, &endptr, 10);
    if (endptr == text || (*endptr != '\0' && *endptr != '*'))
    {
        return false;
    }
    if (v < INT_MIN || v > INT_MAX)
    {
        return false;
    }

    *out = (int)v;
    return true;
}

static bool parse_double_field(const char* text, double* out)
{
    char* endptr = NULL;
    double v;

    if (text == NULL || out == NULL || text[0] == '\0')
    {
        return false;
    }

    v = strtod(text, &endptr);
    if (endptr == text || !isfinite(v) || v < 0.0)
    {
        return false;
    }

    *out = v;
    return true;
}

static int hex_char_to_int(char c)
{
    if (c >= '0' && c <= '9')
    {
        return c - '0';
    }
    c = (char)toupper((unsigned char)c);
    if (c >= 'A' && c <= 'F')
    {
        return 10 + (c - 'A');
    }
    return -1;
}

static bool validate_and_strip_checksum(char* line)
{
    char* star = strchr(line, '*');
    uint8_t calc = 0;
    int hi;
    int lo;

    if (line[0] == '$')
    {
        line++;
    }

    if (star == NULL)
    {
        return true;
    }

    if (star[1] == '\0' || star[2] == '\0' || star[3] != '\0')
    {
        return false;
    }

    hi = hex_char_to_int(star[1]);
    lo = hex_char_to_int(star[2]);
    if (hi < 0 || lo < 0)
    {
        return false;
    }

    for (char* p = line; p < star; p++)
    {
        calc ^= (uint8_t)(*p);
    }

    *star = '\0';
    return calc == (uint8_t)((hi << 4) | lo);
}

static int split_fields_preserve_empty(char* line, char* fields[], int max_fields)
{
    int count = 0;
    char* cur = line;

    if (max_fields <= 0)
    {
        return 0;
    }

    fields[count++] = cur;
    while (*cur != '\0')
    {
        if (*cur == ',')
        {
            *cur = '\0';
            if (count >= max_fields)
            {
                return count;
            }
            fields[count++] = cur + 1;
        }
        cur++;
    }

    return count;
}

static bool sentence_suffix_is(const char* sentence_id, const char* type3)
{
    size_t len;

    if (sentence_id == NULL || type3 == NULL)
    {
        return false;
    }

    if (sentence_id[0] == '$')
    {
        sentence_id++;
    }

    len = strlen(sentence_id);
    if (len < 3U)
    {
        return false;
    }

    return (toupper((unsigned char)sentence_id[len - 3]) == (unsigned char)type3[0] &&
        toupper((unsigned char)sentence_id[len - 2]) == (unsigned char)type3[1] &&
        toupper((unsigned char)sentence_id[len - 1]) == (unsigned char)type3[2]);
}

bool nmea_process_sentence(const char* nmea,
                           nmea_parse_state_t* state,
                           const nmea_parse_config_t* config,
                           nmea_obs_stats_t* obs,
                           int* sat_out,
                           char* time_out,
                           size_t time_out_len,
                           double* speed_knots_out,
                           double* lat_out,
                           double* lon_out)
{
    char nmea_copy[NMEA_LOCAL_BUF_SIZE];
    char* fields[NMEA_FIELD_MAX] = {NULL};
    int token_count = 0;
    size_t nmea_len;
    nmea_parse_config_t cfg;

    if (nmea == NULL || state == NULL || obs == NULL || sat_out == NULL ||
        time_out == NULL || time_out_len < 7U || speed_knots_out == NULL ||
        lat_out == NULL || lon_out == NULL)
    {
        return false;
    }

    normalize_config(config, &cfg);
    obs->nmea_total++;

    nmea_len = strnlen(nmea, sizeof(nmea_copy) + 1U);
    if (nmea_len == 0U || nmea_len >= sizeof(nmea_copy))
    {
        obs->oversize_total++;
        obs->malformed_total++;
        return false;
    }

    (void)memcpy(nmea_copy, nmea, nmea_len + 1U);

    /* Trim optional line endings to support generic caller inputs. */
    while (nmea_len > 0U && (nmea_copy[nmea_len - 1U] == '\r' || nmea_copy[nmea_len - 1U] == '\n'))
    {
        nmea_copy[nmea_len - 1U] = '\0';
        nmea_len--;
    }
    if (nmea_len == 0U)
    {
        obs->malformed_total++;
        return false;
    }

    if (!validate_and_strip_checksum(nmea_copy))
    {
        obs->checksum_fail_total++;
        obs->malformed_total++;
        return false;
    }

    token_count = split_fields_preserve_empty(nmea_copy, fields, NMEA_FIELD_MAX);
    if (token_count <= 0 || fields[0] == NULL || fields[0][0] == '\0')
    {
        obs->malformed_total++;
        return false;
    }

    if (token_count < 2)
    {
        obs->malformed_total++;
        return false;
    }

    if (sentence_suffix_is(fields[0], "GGA"))
    {
        obs->gga_total++;
        if (token_count >= 8 && fields[6] != NULL && fields[7] != NULL)
        {
            int quality = 0;
            int sats = 0;

            if (parse_int_field(fields[6], &quality) && parse_int_field(fields[7], &sats) &&
                quality >= cfg.min_fix_quality && sats >= cfg.min_satellites)
            {
                state->gga_sats = sats;
                *sat_out = sats;
            }
            else
            {
                obs->gga_reject_total++;
                state->gga_sats = 0;
                *sat_out = 0;
            }
        }
        else
        {
            obs->malformed_total++;
        }

        if (has_fused_fix(state, &cfg))
        {
            *sat_out = current_valid_sats(state);
            obs->valid_fix_total++;
            return true;
        }
        return false;
    }

    if (sentence_suffix_is(fields[0], "RMC"))
    {
        obs->rmc_total++;
        if (token_count >= 7 && fields[1] != NULL && fields[2] != NULL &&
            fields[3] != NULL && fields[4] != NULL && fields[5] != NULL && fields[6] != NULL)
        {
            double speed_knots = 0.0;

            if (toupper((unsigned char)fields[2][0]) == 'A')
            {
                state->rmc_valid = true;
                if (strnlen(fields[1], 16) >= 6U)
                {
                    (void)snprintf(time_out, time_out_len, "%.6s", fields[1]);
                }
                else
                {
                    (void)snprintf(time_out, time_out_len, "000000");
                }

                if (!parse_nmea_coord(fields[3], fields[4], lat_out) ||
                    !parse_nmea_coord(fields[5], fields[6], lon_out))
                {
                    obs->coord_parse_fail_total++;
                    state->rmc_valid = false;
                }

                if (state->rmc_valid && token_count >= 8 && fields[7] != NULL &&
                    parse_double_field(fields[7], &speed_knots))
                {
                    *speed_knots_out = speed_knots;
                }
                else if (state->rmc_valid && token_count >= 8 && fields[7] != NULL &&
                    fields[7][0] != '\0')
                {
                    obs->speed_parse_fail_total++;
                }
            }
            else
            {
                obs->rmc_invalid_status_total++;
                state->rmc_valid = false;
            }
        }
        else
        {
            obs->malformed_total++;
        }

        if (has_fused_fix(state, &cfg))
        {
            *sat_out = current_valid_sats(state);
            obs->valid_fix_total++;
            return true;
        }
        return false;
    }

    if (cfg.enable_gns_support && sentence_suffix_is(fields[0], "GNS"))
    {
        obs->gns_total++;
        if (token_count >= 8 && fields[1] != NULL && fields[2] != NULL && fields[3] != NULL &&
            fields[4] != NULL && fields[5] != NULL && fields[6] != NULL && fields[7] != NULL)
        {
            int sats = 0;
            char mode = (char)toupper((unsigned char)fields[6][0]);
            bool mode_valid = (mode != '\0' && mode != 'N');

            if (mode_valid && parse_int_field(fields[7], &sats) && sats >= cfg.min_satellites &&
                parse_nmea_coord(fields[2], fields[3], lat_out) &&
                parse_nmea_coord(fields[4], fields[5], lon_out))
            {
                state->gns_valid = true;
                state->gns_sats = sats;
                *sat_out = sats;
                if (strnlen(fields[1], 16) >= 6U)
                {
                    (void)snprintf(time_out, time_out_len, "%.6s", fields[1]);
                }
            }
            else
            {
                if (!mode_valid || sats < cfg.min_satellites)
                {
                    obs->gns_reject_total++;
                }
                else
                {
                    obs->coord_parse_fail_total++;
                }
                state->gns_valid = false;
                state->gns_sats = 0;
            }
        }
        else
        {
            obs->malformed_total++;
        }

        if (has_fused_fix(state, &cfg))
        {
            *sat_out = current_valid_sats(state);
            obs->valid_fix_total++;
            return true;
        }
        return false;
    }

    obs->unknown_total++;
    return false;
}

void nmea_build_observability_summary(const nmea_obs_stats_t* obs,
                                      int uptime_ms,
                                      int first_fix_ms,
                                      int captured_frames,
                                      int uart_overflow_delta,
                                      int uart_line_drop_delta,
                                      int log_drop_delta,
                                      bool session_success,
                                      char* out,
                                      size_t out_len)
{
    if (obs == NULL || out == NULL || out_len == 0U)
    {
        return;
    }

    (void)snprintf(
        out,
        out_len,
        "[OBS] %s up=%dms fix=%dms frames=%d "
        "NMEA=%u(GGA=%u,RMC=%u,GNS=%u,UNK=%u) err=%u(chk=%u,ovf=%u,GGA=%u,GNS=%u,RMC=%u,spd=%u,coord=%u) "
        "to=%u lost=%u uart_ovf=%d uart_drop=%d log_drop=%d",
        session_success ? "ok" : "fail",
        uptime_ms,
        first_fix_ms,
        captured_frames,
        obs->nmea_total,
        obs->gga_total,
        obs->rmc_total,
        obs->gns_total,
        obs->unknown_total,
        obs->malformed_total,
        obs->checksum_fail_total,
        obs->oversize_total,
        obs->gga_reject_total,
        obs->gns_reject_total,
        obs->rmc_invalid_status_total,
        obs->speed_parse_fail_total,
        obs->coord_parse_fail_total,
        obs->msgq_timeout_total,
        obs->signal_lost_total,
        uart_overflow_delta,
        uart_line_drop_delta,
        log_drop_delta);
}
