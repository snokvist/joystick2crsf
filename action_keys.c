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
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <stdarg.h>
#include <sys/syscall.h>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

void action_log_verbose(const char *fmt, ...)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    long tid = syscall(SYS_gettid);

    char buf[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    fprintf(stderr, "[%ld.%06ld] [TID:%ld] %s\n",
            (long)ts.tv_sec, (long)(ts.tv_nsec / 1000), tid, buf);
}

static int set_nonblock(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        return -1;
    }
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        return -1;
    }
    return 0;
}

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

static int resolve_ip_target(const char *host, const char *port, struct sockaddr_storage *addr, socklen_t *addrlen)
{
    action_log_verbose("resolve_ip_target: resolving %s:%s", host, port);
    if (!host || !port || !addr || !addrlen) {
        return -1;
    }

    int port_num = atoi(port);
    if (port_num <= 0 || port_num > 65535) {
        return -1;
    }

    struct sockaddr_in *sin = (struct sockaddr_in *)addr;
    struct sockaddr_in6 *sin6 = (struct sockaddr_in6 *)addr;

    memset(addr, 0, sizeof(*addr));

    /* Try IPv4 */
    if (inet_pton(AF_INET, host, &sin->sin_addr) == 1) {
        sin->sin_family = AF_INET;
        sin->sin_port = htons((uint16_t)port_num);
        *addrlen = sizeof(struct sockaddr_in);
        return 0;
    }

    /* Try IPv6 */
    if (inet_pton(AF_INET6, host, &sin6->sin6_addr) == 1) {
        sin6->sin6_family = AF_INET6;
        sin6->sin6_port = htons((uint16_t)port_num);
        *addrlen = sizeof(struct sockaddr_in6);
        return 0;
    }

    return -1;
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

    int rc = resolve_ip_target(host, port, addr, addrlen);
    if (rc != 0) {
        fprintf(stderr, "Invalid IP address or port: %s:%s (must be direct IP)\n", host, port);
        free(host);
        free(port);
        return -1;
    }

    /* We need to determine family to create socket, which is inside addr */
    int family = ((struct sockaddr *)addr)->sa_family;
    int fd = socket(family, SOCK_DGRAM, 0);

    free(host);
    free(port);

    if (fd < 0) {
        perror("udp socket");
    } else {
        set_nonblock(fd);
    }
    return fd;
}

static int init_udp(action_worker_t *worker, int index, const char *destination)
{
    if (!worker || index < 0 || index >= ACTION_MAX) {
        return -1;
    }
    if (worker->sockets[index] >= 0) {
        return 0;
    }
    const char *dest = destination;
    if (!strncasecmp(dest, "udp://", 6)) {
        dest += 6;
    }
    worker->sockets[index] = open_udp_target(dest, &worker->socket_addrs[index], &worker->socket_addr_lens[index]);
    return worker->sockets[index] >= 0 ? 0 : -1;
}

static int send_udp(action_worker_t *worker, int index, const action_spec_t *spec)
{
    if (!worker || !spec) {
        return -1;
    }
    if (init_udp(worker, index, spec->destination) < 0) {
        return -1;
    }
    ssize_t n = sendto(worker->sockets[index], spec->body, spec->body_len, MSG_NOSIGNAL,
                       (struct sockaddr *)&worker->socket_addrs[index], worker->socket_addr_lens[index]);
    return (n < 0) ? -1 : 0;
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

static int send_http(const action_spec_t *spec)
{
    if (!spec) {
        return -1;
    }
    char *host = NULL;
    char *port = NULL;
    char *path = NULL;
    if (parse_http_url(spec->destination, &host, &port, &path) < 0) {
        return -1;
    }

    struct sockaddr_storage addr;
    socklen_t addrlen = 0;
    int rc = resolve_ip_target(host, port, &addr, &addrlen);
    if (rc != 0) {
        free(host);
        free(port);
        free(path);
        return -1;
    }

    int family = ((struct sockaddr *)&addr)->sa_family;
    int fd = socket(family, SOCK_STREAM, 0);
    if (fd >= 0) {
        struct timeval tv;
        tv.tv_sec = spec->timeout_ms / 1000;
        tv.tv_usec = (spec->timeout_ms % 1000) * 1000;
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        if (connect(fd, (struct sockaddr *)&addr, addrlen) != 0) {
            /* If blocking connect fails, or if we want strict non-blocking we might need more logic.
               But usually SO_SNDTIMEO handles the timeout. */
            close(fd);
            fd = -1;
        }
    }
    if (fd < 0) {
        free(host);
        free(port);
        free(path);
        return -1;
    }

    char req[2048];
    const char *method = (spec->method == ACTION_HTTP_POST) ? "POST" : "GET";
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
    for (size_t i = 0; i < spec->header_count && off < sizeof(req); i++) {
        int n = snprintf(req + off, sizeof(req) - off, "%s\r\n", spec->headers[i]);
        if (n < 0 || (size_t)n >= sizeof(req) - off) {
            close(fd);
            free(host);
            free(port);
            free(path);
            return -1;
        }
        off += (size_t)n;
    }
    if (spec->method == ACTION_HTTP_POST) {
        int n = snprintf(req + off, sizeof(req) - off,
                         "Content-Length: %zu\r\nContent-Type: application/json\r\n",
                         spec->body_len);
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

    ssize_t n = send(fd, req, off, MSG_NOSIGNAL);
    if (n < 0) {
        close(fd);
        free(host);
        free(port);
        free(path);
        return -1;
    }
    if (spec->method == ACTION_HTTP_POST && spec->body_len > 0) {
        ssize_t nb = send(fd, spec->body, spec->body_len, MSG_NOSIGNAL);
        if (nb < 0) {
            close(fd);
            free(host);
            free(port);
            free(path);
            return -1;
        }
    }

    char discard[512];
    while (recv(fd, discard, sizeof(discard), 0) > 0) {
    }
    close(fd);
    free(host);
    free(port);
    free(path);
    return 0;
}

/* maybe_log removed to prevent main thread blocking on stderr */

/* Worker Thread Implementation */

static void *action_worker_thread(void *arg)
{
    action_worker_t *w = (action_worker_t *)arg;

    /* Set worker priority. If <= 0, use SCHED_OTHER (non-RT).
       Otherwise try SCHED_FIFO with the requested priority. */
    struct sched_param param;
    if (w->priority > 0) {
        param.sched_priority = w->priority;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
            /* Fallback to normal if RT fails */
            param.sched_priority = 0;
            pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
        }
    } else {
        param.sched_priority = 0;
        pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
    }

    while (1) {
        action_queue_item_t item;
        int has_item = 0;

        /* Logging inside lock removed to prevent blocking main thread */
        pthread_mutex_lock(&w->mutex);

        while (w->running && w->head == w->tail) {
            pthread_cond_wait(&w->cond, &w->mutex);
        }
        if (!w->running) {
            pthread_mutex_unlock(&w->mutex);
            break;
        }

        item = w->queue[w->head];
        w->head = (w->head + 1) % ACTION_QUEUE_SIZE;
        has_item = 1;
        pthread_mutex_unlock(&w->mutex);

        action_log_verbose("action (executing) %s -> %s",
                           (item.spec.transport == ACTION_TRANSPORT_UDP) ? "udp" : "http",
                           item.spec.destination);

        if (has_item) {
            int rc = -1;
            /* action_log_verbose("worker: processing start"); */
            if (item.spec.transport == ACTION_TRANSPORT_UDP) {
                rc = send_udp(w, item.index, &item.spec);
            } else {
                rc = send_http(&item.spec);
            }
            action_log_verbose("worker: processing done (rc=%d)", rc);
        }
    }
    return NULL;
}

void action_keys_worker_init(action_worker_t *worker, int priority)
{
    if (!worker) return;
    memset(worker, 0, sizeof(*worker));
    worker->priority = priority;

    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
    pthread_mutex_init(&worker->mutex, &attr);
    pthread_mutexattr_destroy(&attr);

    pthread_cond_init(&worker->cond, NULL);
    worker->running = 1;
    for (int i = 0; i < ACTION_MAX; i++) {
        worker->sockets[i] = -1;
    }

    if (pthread_create(&worker->thread, NULL, action_worker_thread, worker) != 0) {
        perror("pthread_create");
        worker->running = 0;
    }
}

void action_keys_worker_stop(action_worker_t *worker)
{
    if (!worker) return;

    pthread_mutex_lock(&worker->mutex);
    worker->running = 0;
    pthread_cond_broadcast(&worker->cond);
    pthread_mutex_unlock(&worker->mutex);

    if (worker->thread) {
        pthread_join(worker->thread, NULL);
        worker->thread = 0;
    }

    pthread_mutex_destroy(&worker->mutex);
    pthread_cond_destroy(&worker->cond);

    for (int i = 0; i < ACTION_MAX; i++) {
        if (worker->sockets[i] >= 0) {
            close(worker->sockets[i]);
            worker->sockets[i] = -1;
        }
    }
}

static void enqueue_action(action_worker_t *worker, int index, const action_spec_t *spec)
{
    if (!worker || !worker->running) return;

    /* Logging removed to prevent blocking main thread */
    pthread_mutex_lock(&worker->mutex);

    int next_tail = (worker->tail + 1) % ACTION_QUEUE_SIZE;
    if (next_tail != worker->head) {
        worker->queue[worker->tail].index = index;
        worker->queue[worker->tail].spec = *spec;
        worker->tail = next_tail;
        pthread_cond_signal(&worker->cond);
    } else {
        /* Dropping action; do not log to stderr to avoid blocking the main thread */
    }
    pthread_mutex_unlock(&worker->mutex);
}

void action_keys_config_defaults(action_keys_config_t *cfg)
{
    cfg->http_timeout_ms = ACTION_HTTP_TIMEOUT_MS_DEFAULT;
    cfg->debounce_ms = ACTION_DEBOUNCE_MS_DEFAULT;
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
        } else if (!strcasecmp(key, "action_debounce_ms")) {
            cfg->debounce_ms = atoi(val);
            if (cfg->debounce_ms < 0) {
                cfg->debounce_ms = 0;
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

static int64_t timespec_diff_ms_local(const struct timespec *start, const struct timespec *end)
{
    int64_t sec = (int64_t)end->tv_sec - (int64_t)start->tv_sec;
    int64_t nsec = (int64_t)end->tv_nsec - (int64_t)start->tv_nsec;
    if (nsec < 0) {
        sec -= 1;
        nsec += 1000000000L;
    }
    return sec * 1000 + nsec / 1000000L;
}

void action_keys_handle_press(const action_keys_config_t *cfg,
                              action_worker_t *worker,
                              action_state_t *state,
                              int channel_index,
                              action_edge_t edge,
                              action_press_t press,
                              const struct timespec *now)
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

        int64_t diff = timespec_diff_ms_local(&state->last_dispatch, now);
        if (diff >= cfg->debounce_ms) {
            enqueue_action(worker, (int)i, a);
            state->last_dispatch = *now;
            state->pending_idx = -1;
        } else {
            state->pending_idx = (int)i;
        }
    }
}

void action_keys_process_pending(const action_keys_config_t *cfg,
                                 action_worker_t *worker,
                                 action_state_t *state,
                                 const struct timespec *now)
{
    if (state->pending_idx < 0) {
        return;
    }
    if (state->pending_idx >= ACTION_MAX || (size_t)state->pending_idx >= cfg->action_count) {
        state->pending_idx = -1;
        return;
    }

    int64_t diff = timespec_diff_ms_local(&state->last_dispatch, now);
    if (diff >= cfg->debounce_ms) {
        enqueue_action(worker, state->pending_idx, &cfg->actions[state->pending_idx]);
        state->last_dispatch = *now;
        state->pending_idx = -1;
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
