/**
 * action_keys.c - UDP/HTTP action dispatcher for joystick2crsf
 *
 * Shared config parser and dispatcher for channel-driven UDP/HTTP actions.
 */

#define _POSIX_C_SOURCE 200809L
#define _GNU_SOURCE
#include "action_keys.h"

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

#define ACTION_DEBOUNCE_MS 500

static void trim(char *s)
{
    if (!s) {
        return;
    }
    char *start = s;
    while (*start && isspace((unsigned char)*start)) {
        start++;
    }
    char *end = start + strlen(start);
    while (end > start && isspace((unsigned char)end[-1])) {
        end--;
    }
    size_t len = (size_t)(end - start);
    if (start != s) {
        memmove(s, start, len);
    }
    s[len] = '\0';
}

static int parse_host_port(const char *spec, char **host_out, char **port_out)
{
    if (!spec) {
        return -1;
    }
    char *dup = strdup(spec);
    if (!dup) {
        return -1;
    }

    char *host = dup;
    char *port = NULL;

    if (dup[0] == '[') {
        char *closing = strchr(dup, ']');
        if (!closing || closing[1] != ':' || !closing[2]) {
            free(dup);
            return -1;
        }
        *closing = '\0';
        host = dup + 1;
        port = closing + 2;
    } else {
        char *colon = strrchr(dup, ':');
        if (!colon || !colon[1]) {
            free(dup);
            return -1;
        }
        *colon = '\0';
        host = dup;
        port = colon + 1;
    }

    char *host_copy = strdup(host);
    char *port_copy = strdup(port);
    free(dup);

    if (!host_copy || !port_copy) {
        free(host_copy);
        free(port_copy);
        return -1;
    }

    *host_out = host_copy;
    *port_out = port_copy;
    return 0;
}

static int open_udp_target(const char *target, struct sockaddr_storage *addr, socklen_t *addrlen)
{
    char *host = NULL;
    char *port = NULL;
    if (parse_host_port(target, &host, &port) < 0) {
        fprintf(stderr, "Invalid UDP target '%s'\n", target);
        return -1;
    }

    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;

    struct addrinfo *res = NULL;
    int rc = getaddrinfo(host, port, &hints, &res);
    free(host);
    free(port);
    if (rc != 0) {
        fprintf(stderr, "UDP getaddrinfo: %s\n", gai_strerror(rc));
        return -1;
    }

    int fd = -1;
    for (struct addrinfo *ai = res; ai; ai = ai->ai_next) {
        fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd >= 0) {
            int flags = fcntl(fd, F_GETFL, 0);
            if (flags >= 0 && !(flags & O_NONBLOCK)) {
                fcntl(fd, F_SETFL, flags | O_NONBLOCK);
            }
            memcpy(addr, ai->ai_addr, ai->ai_addrlen);
            *addrlen = (socklen_t)ai->ai_addrlen;
            break;
        }
    }
    freeaddrinfo(res);
    if (fd < 0) {
        perror("udp socket");
    }
    return fd;
}

static void free_binding(action_binding_t *b)
{
    if (!b) {
        return;
    }
    if (b->udp_fd >= 0) {
        close(b->udp_fd);
    }
    memset(b, 0, sizeof(*b));
    b->udp_fd = -1;
}

static int64_t timespec_diff_ms(const struct timespec *start, const struct timespec *end)
{
    int64_t sec = (int64_t)end->tv_sec - (int64_t)start->tv_sec;
    int64_t nsec = (int64_t)end->tv_nsec - (int64_t)start->tv_nsec;
    if (nsec < 0) {
        sec -= 1;
        nsec += 1000000000L;
    }
    return sec * 1000 + nsec / 1000000L;
}

static int debounced(action_binding_t *b)
{
    if (!b) {
        return 0;
    }
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (b->last_dispatch.tv_sec == 0 && b->last_dispatch.tv_nsec == 0) {
        b->last_dispatch = now;
        return 0;
    }
    if (timespec_diff_ms(&b->last_dispatch, &now) < ACTION_DEBOUNCE_MS) {
        return 1;
    }
    b->last_dispatch = now;
    return 0;
}

void action_keys_bindings_init(const action_keys_config_t *cfg,
                               action_binding_t bindings[ACTION_MAX])
{
    if (!cfg || !bindings) {
        return;
    }
    for (size_t i = 0; i < ACTION_MAX; i++) {
        memset(&bindings[i], 0, sizeof(action_binding_t));
        bindings[i].udp_fd = -1;
        bindings[i].last_dispatch = (struct timespec){0, 0};
    }
    for (size_t i = 0; i < cfg->action_count && i < ACTION_MAX; i++) {
        bindings[i].spec = cfg->actions[i];
    }
}

void action_keys_free_bindings(action_binding_t bindings[ACTION_MAX])
{
    if (!bindings) {
        return;
    }
    for (size_t i = 0; i < ACTION_MAX; i++) {
        free_binding(&bindings[i]);
    }
}

static int init_udp(action_binding_t *b)
{
    if (!b) {
        return -1;
    }
    if (b->udp_fd >= 0) {
        return 0;
    }
    const char *dest = b->spec.destination;
    if (!strncasecmp(dest, "udp://", 6)) {
        dest += 6;
    }
    b->udp_fd = open_udp_target(dest, &b->udp_addr, &b->udp_addrlen);
    return b->udp_fd >= 0 ? 0 : -1;
}

static int send_udp(action_binding_t *b)
{
    if (!b) {
        return -1;
    }
    if (init_udp(b) < 0) {
        return -1;
    }
    ssize_t n = sendto(b->udp_fd, b->spec.body, b->spec.body_len, MSG_NOSIGNAL,
                       (struct sockaddr *)&b->udp_addr, b->udp_addrlen);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return -1;
        }
        close(b->udp_fd);
        b->udp_fd = -1;
        return -1;
    }
    return 0;
}

static int parse_http_url(const char *url, char **host_out, char **port_out, char **path_out)
{
    const char *prefix = "http://";
    size_t plen = strlen(prefix);
    if (strncmp(url, prefix, plen) != 0) {
        return -1;
    }
    const char *host_start = url + plen;
    const char *path_start = strchr(host_start, '/');
    if (!path_start) {
        path_start = url + strlen(url);
    }
    size_t host_len = (size_t)(path_start - host_start);
    if (host_len == 0) {
        return -1;
    }
    const char *port_sep = NULL;
    const char *host_end = NULL;
    if (host_start[0] == '[') {
        const char *closing = memchr(host_start, ']', host_len);
        if (!closing || closing == host_start + 1) {
            return -1;
        }
        host_end = closing + 1;
        if (host_end < path_start && *host_end == ':') {
            port_sep = host_end;
        }
    } else {
        port_sep = memchr(host_start, ':', host_len);
        host_end = port_sep ? port_sep : path_start;
    }
    if (!host_end) {
        host_end = path_start;
    }
    size_t host_only_len = (size_t)(host_end - host_start);
    if (host_only_len == 0) {
        return -1;
    }
    char *host = NULL;
    if (host_start[0] == '[') {
        if (host_only_len <= 2) {
            return -1;
        }
        host = strndup(host_start + 1, host_only_len - 2);
    } else {
        host = strndup(host_start, host_only_len);
    }
    char *port = NULL;
    if (port_sep && port_sep < path_start) {
        size_t port_len = (size_t)(path_start - port_sep - 1);
        if (port_len == 0) {
            free(host);
            return -1;
        }
        port = strndup(port_sep + 1, port_len);
    } else {
        port = strdup("80");
    }
    const char *path = (*path_start) ? path_start : "/";
    char *path_copy = strdup(*path ? path : "/");
    if (!host || !port || !path_copy) {
        free(host);
        free(port);
        free(path_copy);
        return -1;
    }
    *host_out = host;
    *port_out = port;
    *path_out = path_copy;
    return 0;
}

static int send_http(action_binding_t *b)
{
    if (!b) {
        return -1;
    }
    char *host = NULL;
    char *port = NULL;
    char *path = NULL;
    if (parse_http_url(b->spec.destination, &host, &port, &path) < 0) {
        return -1;
    }

    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    struct addrinfo *res = NULL;
    int rc = getaddrinfo(host, port, &hints, &res);
    if (rc != 0 || !res) {
        free(host);
        free(port);
        free(path);
        if (res) {
            freeaddrinfo(res);
        }
        return -1;
    }

    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    struct addrinfo *ai_connected = NULL;
    int fd = -1;
    for (struct addrinfo *ai = res; ai; ai = ai->ai_next) {
        fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0) {
            continue;
        }

        int flags = fcntl(fd, F_GETFL, 0);
        if (flags >= 0 && !(flags & O_NONBLOCK)) {
            fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        }

        int rc_conn = connect(fd, ai->ai_addr, ai->ai_addrlen);
        if (rc_conn < 0 && errno == EINPROGRESS) {
            struct pollfd pfd;
            memset(&pfd, 0, sizeof(pfd));
            pfd.fd = fd;
            pfd.events = POLLOUT;
            int poll_rc = poll(&pfd, 1, b->spec.timeout_ms);
            if (poll_rc > 0 && (pfd.revents & (POLLOUT | POLLERR | POLLHUP))) {
                int soerr = 0;
                socklen_t slen = sizeof(soerr);
                if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &slen) == 0 && soerr == 0) {
                    rc_conn = 0;
                } else {
                    errno = soerr;
                    rc_conn = -1;
                }
            } else {
                rc_conn = -1;
                if (poll_rc == 0) {
                    errno = ETIMEDOUT;
                }
            }
        }

        if (rc_conn == 0) {
            ai_connected = ai;
            break;
        }
        close(fd);
        fd = -1;
    }
    freeaddrinfo(res);
    if (fd < 0) {
        free(host);
        free(port);
        free(path);
        return -1;
    }

    char req[2048];
    const char *method = (b->spec.method == ACTION_HTTP_POST) ? "POST" : "GET";
    int written = snprintf(req, sizeof(req),
                           "%s %s HTTP/1.0\r\nHost: %s\r\nUser-Agent: action_keys\r\n",
                           method, path, host);
    if (written < 0 || (size_t)written >= sizeof(req)) {
        close(fd);
        free(host);
        free(port);
        free(path);
        return -1;
    }
    size_t off = (size_t)written;
    for (size_t i = 0; i < b->spec.header_count && off < sizeof(req); i++) {
        int n = snprintf(req + off, sizeof(req) - off, "%s\r\n", b->spec.headers[i]);
        if (n < 0 || (size_t)n >= sizeof(req) - off) {
            close(fd);
            free(host);
            free(port);
            free(path);
            return -1;
        }
        off += (size_t)n;
    }
    if (b->spec.method == ACTION_HTTP_POST) {
        int n = snprintf(req + off, sizeof(req) - off,
                         "Content-Length: %zu\r\nContent-Type: application/json\r\n",
                         b->spec.body_len);
        if (n < 0 || (size_t)n >= sizeof(req) - off) {
            close(fd);
            free(host);
            free(port);
            free(path);
            return -1;
        }
        off += (size_t)n;
    }
    if (off + 2 >= sizeof(req)) {
        close(fd);
        free(host);
        free(port);
        free(path);
        return -1;
    }
    req[off++] = '\r';
    req[off++] = '\n';

    int send_timeout = b->spec.timeout_ms;
    if (ai_connected && ai_connected->ai_next) {
        /* If we iterated, subtract the elapsed time from the original timeout. */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        int64_t elapsed = timespec_diff_ms(&start, &now);
        if (elapsed > 0 && elapsed < b->spec.timeout_ms) {
            send_timeout = (int)(b->spec.timeout_ms - elapsed);
        } else if (elapsed >= b->spec.timeout_ms) {
            send_timeout = 0;
        }
    }

    struct pollfd pfd = { .fd = fd, .events = POLLOUT };
    int poll_rc = poll(&pfd, 1, send_timeout);
    if (poll_rc <= 0 || !(pfd.revents & (POLLOUT | POLLERR | POLLHUP))) {
        close(fd);
        free(host);
        free(port);
        free(path);
        return -1;
    }

    ssize_t n = send(fd, req, off, MSG_NOSIGNAL);
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        n = 0;
    }
    if (n < 0) {
        close(fd);
        free(host);
        free(port);
        free(path);
        return -1;
    }
    size_t sent = (size_t)n;
    while (sent < off) {
        poll_rc = poll(&pfd, 1, send_timeout);
        if (poll_rc <= 0 || !(pfd.revents & POLLOUT)) {
            close(fd);
            free(host);
            free(port);
            free(path);
            return -1;
        }
        n = send(fd, req + sent, off - sent, MSG_NOSIGNAL);
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            continue;
        }
        if (n < 0) {
            close(fd);
            free(host);
            free(port);
            free(path);
            return -1;
        }
        sent += (size_t)n;
    }

    if (b->spec.method == ACTION_HTTP_POST && b->spec.body_len > 0) {
        sent = 0;
        while (sent < b->spec.body_len) {
            poll_rc = poll(&pfd, 1, send_timeout);
            if (poll_rc <= 0 || !(pfd.revents & POLLOUT)) {
                close(fd);
                free(host);
                free(port);
                free(path);
                return -1;
            }
            n = send(fd, b->spec.body + sent, b->spec.body_len - sent, MSG_NOSIGNAL);
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                continue;
            }
            if (n < 0) {
                close(fd);
                free(host);
                free(port);
                free(path);
                return -1;
            }
            sent += (size_t)n;
        }
    }

    close(fd);
    free(host);
    free(port);
    free(path);
    return 0;
}

static int dispatch(action_binding_t *b)
{
    if (!b) {
        return -1;
    }
    if (debounced(b)) {
        return 0;
    }
    if (b->spec.transport == ACTION_TRANSPORT_UDP) {
        return send_udp(b);
    }
    return send_http(b);
}

static void maybe_log(const action_keys_config_t *cfg,
                      const action_spec_t *spec,
                      const char *event,
                      int rc)
{
    if (!cfg || !cfg->verbose) {
        return;
    }
    if (spec) {
        const char *transport = (spec->transport == ACTION_TRANSPORT_HTTP) ? "http" : "udp";
        fprintf(stderr, "%s %s -> %s (%s)\n",
                event, transport, spec->destination, rc == 0 ? "ok" : "fail");
    } else {
        fprintf(stderr, "%s\n", event);
    }
}

void action_keys_config_defaults(action_keys_config_t *cfg)
{
    cfg->http_timeout_ms = ACTION_HTTP_TIMEOUT_MS_DEFAULT;
    cfg->verbose = 1;
    cfg->action_count = 0;
    memset(cfg->actions, 0, sizeof(cfg->actions));
    for (size_t i = 0; i < ACTION_MAX; i++) {
        cfg->actions[i].channel = -1;
        cfg->actions[i].edge = ACTION_EDGE_HIGH;
        cfg->actions[i].press = ACTION_PRESS_ANY;
    }
}

static int parse_action(const action_keys_config_t *cfg, const char *val, action_spec_t *out,
                        const char *path, int lineno)
{
    if (!cfg || !val || !out) {
        return -1;
    }
    action_spec_t tmp;
    memset(&tmp, 0, sizeof(tmp));
    tmp.channel = -1;
    tmp.edge = ACTION_EDGE_HIGH;
    tmp.press = ACTION_PRESS_ANY;
    tmp.transport = ACTION_TRANSPORT_UDP;
    tmp.method = ACTION_HTTP_GET;
    tmp.timeout_ms = cfg->http_timeout_ms;

    char *dup = strdup(val);
    if (!dup) {
        fprintf(stderr, "%s:%d: out of memory parsing action\n", path, lineno);
        return -1;
    }
    char *cursor = dup;
    while (cursor && *cursor) {
        while (*cursor == ',' || isspace((unsigned char)*cursor)) {
            cursor++;
        }
        if (!*cursor) {
            break;
        }
        char *key = cursor;
        char *eq = strchr(key, '=');
        if (!eq || !eq[1]) {
            fprintf(stderr, "%s:%d: action token '%s' missing '='\n", path, lineno, key);
            free(dup);
            return -1;
        }
        *eq = '\0';
        char *val = eq + 1;
        char *next = val;
        int is_body = (!strcasecmp(key, "body") || !strcasecmp(key, "payload"));
        if (!is_body) {
            char *comma = strchr(val, ',');
            if (comma) {
                *comma = '\0';
                cursor = comma + 1;
            } else {
                cursor = val + strlen(val);
            }
        } else {
            cursor = val + strlen(val);
        }
        trim(key);
        trim(val);
        if (!strcasecmp(key, "channel")) {
            int ch = atoi(val);
            if (ch < 1 || ch > 16) {
                fprintf(stderr, "%s:%d: channel must be 1-16\n", path, lineno);
                free(dup);
                return -1;
            }
            tmp.channel = ch - 1;
        } else if (!strcasecmp(key, "edge")) {
            if (!strcasecmp(val, "high")) {
                tmp.edge = ACTION_EDGE_HIGH;
            } else if (!strcasecmp(val, "low")) {
                tmp.edge = ACTION_EDGE_LOW;
            } else {
                fprintf(stderr, "%s:%d: edge must be high or low\n", path, lineno);
                free(dup);
                return -1;
            }
        } else if (!strcasecmp(key, "press")) {
            if (!strcasecmp(val, "short")) {
                tmp.press = ACTION_PRESS_SHORT;
            } else if (!strcasecmp(val, "long")) {
                tmp.press = ACTION_PRESS_LONG;
            } else if (!strcasecmp(val, "any")) {
                tmp.press = ACTION_PRESS_ANY;
            } else {
                fprintf(stderr, "%s:%d: press must be short, long, or any\n", path, lineno);
                free(dup);
                return -1;
            }
        } else if (!strcasecmp(key, "transport")) {
            if (!strcasecmp(val, "udp")) {
                tmp.transport = ACTION_TRANSPORT_UDP;
            } else if (!strcasecmp(val, "http")) {
                tmp.transport = ACTION_TRANSPORT_HTTP;
            } else {
                fprintf(stderr, "%s:%d: transport must be udp or http\n", path, lineno);
                free(dup);
                return -1;
            }
        } else if (!strcasecmp(key, "method")) {
            if (!strcasecmp(val, "get")) {
                tmp.method = ACTION_HTTP_GET;
            } else if (!strcasecmp(val, "post")) {
                tmp.method = ACTION_HTTP_POST;
            } else {
                fprintf(stderr, "%s:%d: method must be GET or POST\n", path, lineno);
                free(dup);
                return -1;
            }
        } else if (!strcasecmp(key, "url") || !strcasecmp(key, "destination") ||
                   !strcasecmp(key, "dest")) {
            if (strlen(val) >= ACTION_MAX_DEST_LEN) {
                fprintf(stderr, "%s:%d: destination too long (max %d)\n",
                        path, lineno, ACTION_MAX_DEST_LEN - 1);
                free(dup);
                return -1;
            }
            snprintf(tmp.destination, sizeof(tmp.destination), "%s", val);
        } else if (!strcasecmp(key, "body") || !strcasecmp(key, "payload")) {
            size_t len = strlen(val);
            if (len >= ACTION_MAX_BODY_LEN) {
                fprintf(stderr, "%s:%d: body too long (max %d)\n",
                        path, lineno, ACTION_MAX_BODY_LEN - 1);
                free(dup);
                return -1;
            }
            snprintf(tmp.body, sizeof(tmp.body), "%s", val);
            tmp.body_len = len;
            break; /* body consumes the rest of the line */
        } else if (!strcasecmp(key, "header")) {
            if (tmp.header_count >= ACTION_MAX_HEADERS) {
                fprintf(stderr, "%s:%d: too many headers (max %d)\n",
                        path, lineno, ACTION_MAX_HEADERS);
                free(dup);
                return -1;
            }
            if (strlen(val) >= ACTION_MAX_HEADER_LEN) {
                fprintf(stderr, "%s:%d: header too long (max %d)\n",
                        path, lineno, ACTION_MAX_HEADER_LEN - 1);
                free(dup);
                return -1;
            }
            snprintf(tmp.headers[tmp.header_count], sizeof(tmp.headers[0]), "%s", val);
            tmp.header_count++;
        } else if (!strcasecmp(key, "timeout_ms")) {
            tmp.timeout_ms = atoi(val);
            if (tmp.timeout_ms <= 0) {
                tmp.timeout_ms = ACTION_HTTP_TIMEOUT_MS_DEFAULT;
            }
        } else {
            fprintf(stderr, "%s:%d: unknown action field '%s'\n", path, lineno, key);
            free(dup);
            return -1;
        }
    }
    free(dup);

    if (tmp.channel < 0 || tmp.channel >= 16) {
        fprintf(stderr, "%s:%d: action missing valid channel\n", path, lineno);
        return -1;
    }
    if (tmp.destination[0] == '\0') {
        fprintf(stderr, "%s:%d: action missing destination\n", path, lineno);
        return -1;
    }
    if (tmp.transport == ACTION_TRANSPORT_HTTP &&
        tmp.method == ACTION_HTTP_GET && tmp.body_len > 0) {
        tmp.method = ACTION_HTTP_POST;
    }
    *out = tmp;
    return 0;
}

int action_keys_config_load(action_keys_config_t *cfg, const char *path)
{
    FILE *fp = fopen(path, "r");
    if (!fp) {
        fprintf(stderr, "Failed to open config %s: %s\n", path, strerror(errno));
        return -1;
    }
    char line[1024];
    int lineno = 0;
    while (fgets(line, sizeof(line), fp)) {
        lineno++;
        char *hash = strchr(line, '#');
        if (hash) {
            *hash = '\0';
        }
        trim(line);
        if (!line[0]) {
            continue;
        }
        char *eq = strchr(line, '=');
        if (!eq) {
            fprintf(stderr, "%s:%d: ignoring line without '='\n", path, lineno);
            continue;
        }
        *eq = '\0';
        char *key = line;
        char *val = eq + 1;
        trim(key);
        trim(val);

        if (!strcasecmp(key, "http_timeout_ms")) {
            cfg->http_timeout_ms = atoi(val);
            if (cfg->http_timeout_ms <= 0) {
                cfg->http_timeout_ms = ACTION_HTTP_TIMEOUT_MS_DEFAULT;
            }
        } else if (!strcasecmp(key, "verbose")) {
            cfg->verbose = atoi(val) ? 1 : 0;
        } else if (!strncasecmp(key, "action_", 7)) {
            if (cfg->action_count >= ACTION_MAX) {
                fprintf(stderr, "%s:%d: maximum of %d actions reached; ignoring\n",
                        path, lineno, ACTION_MAX);
                continue;
            }
            if (parse_action(cfg, val, &cfg->actions[cfg->action_count], path, lineno) == 0) {
                cfg->action_count++;
            }
        } else {
            /* Ignore unrelated keys so the action config can live inside joystick2crsf.conf. */
        }
    }
    fclose(fp);
    return 0;
}

void action_keys_handle_press(const action_keys_config_t *cfg,
                              action_binding_t bindings[ACTION_MAX],
                              int channel_index,
                              action_edge_t edge,
                              action_press_t press)
{
    for (size_t i = 0; i < cfg->action_count && i < ACTION_MAX; i++) {
        const action_spec_t *a = &cfg->actions[i];
        if (a->channel != channel_index) {
            continue;
        }
        if (a->edge != edge) {
            continue;
        }
        if (a->press != ACTION_PRESS_ANY && a->press != press) {
            continue;
        }
        int rc = dispatch(&bindings[i]);
        maybe_log(cfg, a, "action", rc);
    }
}

void action_keys_build_watchlist(const action_keys_config_t *cfg,
                                 int watch_high[16],
                                 int watch_low[16])
{
    if (!cfg) {
        return;
    }
    for (int i = 0; i < 16; i++) {
        watch_high[i] = 0;
        watch_low[i] = 0;
    }
    for (size_t i = 0; i < cfg->action_count && i < ACTION_MAX; i++) {
        const action_spec_t *a = &cfg->actions[i];
        if (a->channel < 0 || a->channel >= 16) {
            continue;
        }
        if (a->edge == ACTION_EDGE_HIGH) {
            watch_high[a->channel] = 1;
        } else {
            watch_low[a->channel] = 1;
        }
    }
}
