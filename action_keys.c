/**
 * action_keys.c - simple hotkey-driven UDP/HTTP action dispatcher
 *
 * Reads actions from a config file and fires them when matching keyboard keys
 * (stdin raw mode) are pressed.
 */

#define _POSIX_C_SOURCE 200809L
#define _GNU_SOURCE
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <netdb.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

#define ACTION_MAX            32
#define ACTION_MAX_BODY_LEN   512
#define ACTION_MAX_HEADER_LEN 128
#define ACTION_MAX_HEADERS    8
#define ACTION_MAX_DEST_LEN   256
#define ACTION_HTTP_TIMEOUT_MS_DEFAULT 1500
#define DEFAULT_CONF "/etc/action_keys.conf"

typedef enum {
    ACTION_TRANSPORT_UDP = 0,
    ACTION_TRANSPORT_HTTP
} action_transport_t;

typedef enum {
    ACTION_HTTP_GET = 0,
    ACTION_HTTP_POST
} action_http_method_t;

typedef enum {
    ACTION_KEY_NONE = 0,
    ACTION_KEY_CHAR,
    ACTION_KEY_UP,
    ACTION_KEY_DOWN,
    ACTION_KEY_LEFT,
    ACTION_KEY_RIGHT,
    ACTION_KEY_ENTER,
    ACTION_KEY_SPACE
} action_keycode_t;

typedef struct {
    action_keycode_t key_code;     /* special keys or printable char */
    char key_char;                 /* valid when key_code == ACTION_KEY_CHAR */
    action_transport_t transport;
    action_http_method_t method;
    char destination[ACTION_MAX_DEST_LEN];
    char body[ACTION_MAX_BODY_LEN];
    size_t body_len;
    char headers[ACTION_MAX_HEADERS][ACTION_MAX_HEADER_LEN];
    size_t header_count;
    int timeout_ms;
} action_t;

typedef struct {
    action_t spec;
    int udp_fd;
    struct sockaddr_storage udp_addr;
    socklen_t udp_addrlen;
} action_binding_t;

typedef struct {
    int http_timeout_ms;
    int verbose;
    char input_device[256];
    size_t action_count;
    action_t actions[ACTION_MAX];
} config_t;

static volatile int g_run = 1;

static void on_sigint(int sig){ (void)sig; g_run = 0; }

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
    const char *port_sep = memchr(host_start, ':', host_len);
    const char *host_end = port_sep ? port_sep : path_start;
    size_t host_only_len = (size_t)(host_end - host_start);
    char *host = strndup(host_start, host_only_len);
    char *port = NULL;
    if (port_sep && port_sep < path_start) {
        size_t port_len = (size_t)(path_start - port_sep - 1);
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

    int fd = -1;
    for (struct addrinfo *ai = res; ai; ai = ai->ai_next) {
        fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0) {
            continue;
        }
        struct timeval tv;
        tv.tv_sec = b->spec.timeout_ms / 1000;
        tv.tv_usec = (b->spec.timeout_ms % 1000) * 1000;
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        if (connect(fd, ai->ai_addr, ai->ai_addrlen) == 0) {
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

    ssize_t n = send(fd, req, off, MSG_NOSIGNAL);
    if (n < 0) {
        close(fd);
        free(host);
        free(port);
        free(path);
        return -1;
    }
    if (b->spec.method == ACTION_HTTP_POST && b->spec.body_len > 0) {
        ssize_t nb = send(fd, b->spec.body, b->spec.body_len, MSG_NOSIGNAL);
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

static int dispatch(action_binding_t *b)
{
    if (!b) {
        return -1;
    }
    if (b->spec.transport == ACTION_TRANSPORT_UDP) {
        return send_udp(b);
    }
    return send_http(b);
}

static void maybe_log(const config_t *cfg, const action_t *spec, const char *event, int rc)
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

static void config_defaults(config_t *cfg)
{
    cfg->http_timeout_ms = ACTION_HTTP_TIMEOUT_MS_DEFAULT;
    cfg->verbose = 1;
    snprintf(cfg->input_device, sizeof(cfg->input_device), "%s", "/dev/input/joystick2crsf-keys");
    cfg->action_count = 0;
    memset(cfg->actions, 0, sizeof(cfg->actions));
    for (size_t i = 0; i < ACTION_MAX; i++) {
        cfg->actions[i].key_code = ACTION_KEY_NONE;
        cfg->actions[i].key_char = 0;
    }
}

static int parse_action(const config_t *cfg, const char *val, action_t *out,
                        const char *path, int lineno)
{
    if (!cfg || !val || !out) {
        return -1;
    }
    action_t tmp;
    memset(&tmp, 0, sizeof(tmp));
    tmp.key_code = ACTION_KEY_NONE;
    tmp.key_char = 0;
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
        if (!strcasecmp(key, "key")) {
            if (!*val) {
                fprintf(stderr, "%s:%d: action key cannot be empty\n", path, lineno);
                free(dup);
                return -1;
            }
            if (!strcasecmp(val, "up")) {
                tmp.key_code = ACTION_KEY_UP;
            } else if (!strcasecmp(val, "down")) {
                tmp.key_code = ACTION_KEY_DOWN;
            } else if (!strcasecmp(val, "left")) {
                tmp.key_code = ACTION_KEY_LEFT;
            } else if (!strcasecmp(val, "right")) {
                tmp.key_code = ACTION_KEY_RIGHT;
            } else if (!strcasecmp(val, "enter") || !strcasecmp(val, "return")) {
                tmp.key_code = ACTION_KEY_ENTER;
            } else if (!strcasecmp(val, "space") || !strcasecmp(val, "spacebar")) {
                tmp.key_code = ACTION_KEY_SPACE;
            } else {
                tmp.key_code = ACTION_KEY_CHAR;
                tmp.key_char = val[0];
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

static int config_load(config_t *cfg, const char *path)
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
        } else if (!strcasecmp(key, "input_device")) {
            snprintf(cfg->input_device, sizeof(cfg->input_device), "%s", val);
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
            fprintf(stderr, "%s:%d: unknown key '%s'\n", path, lineno, key);
        }
    }
    fclose(fp);
    return 0;
}

static int set_stdin_raw(int *old_flags, struct termios *old_term)
{
    if (!old_flags || !old_term) {
        return -1;
    }
    int fd = fileno(stdin);
    *old_flags = fcntl(fd, F_GETFL, 0);
    if (*old_flags < 0) {
        return -1;
    }
    if (fcntl(fd, F_SETFL, *old_flags | O_NONBLOCK) < 0) {
        return -1;
    }
    if (tcgetattr(fd, old_term) < 0) {
        return -1;
    }
    struct termios raw = *old_term;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &raw) < 0) {
        return -1;
    }
    return 0;
}

static void restore_stdin(int old_flags, const struct termios *old_term)
{
    int fd = fileno(stdin);
    if (old_flags >= 0) {
        fcntl(fd, F_SETFL, old_flags);
    }
    if (old_term) {
        tcsetattr(fd, TCSANOW, old_term);
    }
}

static void handle_keycode(action_keycode_t code, char ch,
                           const config_t *cfg, action_binding_t bindings[ACTION_MAX])
{
    if (cfg && cfg->verbose) {
        const char *name = NULL;
        char msg[32];
        switch (code) {
        case ACTION_KEY_UP: name = "key up"; break;
        case ACTION_KEY_DOWN: name = "key down"; break;
        case ACTION_KEY_LEFT: name = "key left"; break;
        case ACTION_KEY_RIGHT: name = "key right"; break;
        case ACTION_KEY_ENTER: name = "key enter"; break;
        case ACTION_KEY_SPACE: name = "key space"; break;
        default:
            if (code == ACTION_KEY_CHAR) {
                snprintf(msg, sizeof(msg), "key '%c'", ch ? ch : '?');
                name = msg;
            }
            break;
        }
        if (name) {
            maybe_log(cfg, NULL, name, 0);
        }
    }
    for (size_t i = 0; i < cfg->action_count && i < ACTION_MAX; i++) {
        const action_t *a = &cfg->actions[i];
        if (a->key_code == ACTION_KEY_NONE) {
            continue;
        }
        if (a->key_code == ACTION_KEY_CHAR && code == ACTION_KEY_CHAR && a->key_char == ch) {
            int rc = dispatch(&bindings[i]);
            maybe_log(cfg, a, "action", rc);
        } else if (a->key_code == code && code != ACTION_KEY_CHAR) {
            int rc = dispatch(&bindings[i]);
            maybe_log(cfg, a, "action", rc);
        }
    }
}

static int map_evdev_key(int code, action_keycode_t *out_code, char *out_char)
{
    if (!out_code || !out_char) {
        return -1;
    }
    *out_code = ACTION_KEY_NONE;
    *out_char = 0;
    switch (code) {
    case KEY_UP: *out_code = ACTION_KEY_UP; return 0;
    case KEY_DOWN: *out_code = ACTION_KEY_DOWN; return 0;
    case KEY_LEFT: *out_code = ACTION_KEY_LEFT; return 0;
    case KEY_RIGHT: *out_code = ACTION_KEY_RIGHT; return 0;
    case KEY_ENTER:
    case KEY_KPENTER: *out_code = ACTION_KEY_ENTER; return 0;
    case KEY_SPACE: *out_code = ACTION_KEY_SPACE; return 0;
    case KEY_1: *out_code = ACTION_KEY_CHAR; *out_char = '1'; return 0;
    case KEY_2: *out_code = ACTION_KEY_CHAR; *out_char = '2'; return 0;
    case KEY_3: *out_code = ACTION_KEY_CHAR; *out_char = '3'; return 0;
    case KEY_4: *out_code = ACTION_KEY_CHAR; *out_char = '4'; return 0;
    case KEY_5: *out_code = ACTION_KEY_CHAR; *out_char = '5'; return 0;
    case KEY_6: *out_code = ACTION_KEY_CHAR; *out_char = '6'; return 0;
    case KEY_7: *out_code = ACTION_KEY_CHAR; *out_char = '7'; return 0;
    case KEY_8: *out_code = ACTION_KEY_CHAR; *out_char = '8'; return 0;
    case KEY_9: *out_code = ACTION_KEY_CHAR; *out_char = '9'; return 0;
    case KEY_0: *out_code = ACTION_KEY_CHAR; *out_char = '0'; return 0;
    case KEY_A: *out_code = ACTION_KEY_CHAR; *out_char = 'a'; return 0;
    case KEY_B: *out_code = ACTION_KEY_CHAR; *out_char = 'b'; return 0;
    case KEY_C: *out_code = ACTION_KEY_CHAR; *out_char = 'c'; return 0;
    case KEY_D: *out_code = ACTION_KEY_CHAR; *out_char = 'd'; return 0;
    case KEY_E: *out_code = ACTION_KEY_CHAR; *out_char = 'e'; return 0;
    case KEY_F: *out_code = ACTION_KEY_CHAR; *out_char = 'f'; return 0;
    case KEY_G: *out_code = ACTION_KEY_CHAR; *out_char = 'g'; return 0;
    case KEY_H: *out_code = ACTION_KEY_CHAR; *out_char = 'h'; return 0;
    case KEY_I: *out_code = ACTION_KEY_CHAR; *out_char = 'i'; return 0;
    case KEY_J: *out_code = ACTION_KEY_CHAR; *out_char = 'j'; return 0;
    case KEY_K: *out_code = ACTION_KEY_CHAR; *out_char = 'k'; return 0;
    case KEY_L: *out_code = ACTION_KEY_CHAR; *out_char = 'l'; return 0;
    case KEY_M: *out_code = ACTION_KEY_CHAR; *out_char = 'm'; return 0;
    case KEY_N: *out_code = ACTION_KEY_CHAR; *out_char = 'n'; return 0;
    case KEY_O: *out_code = ACTION_KEY_CHAR; *out_char = 'o'; return 0;
    case KEY_P: *out_code = ACTION_KEY_CHAR; *out_char = 'p'; return 0;
    case KEY_Q: *out_code = ACTION_KEY_CHAR; *out_char = 'q'; return 0;
    case KEY_R: *out_code = ACTION_KEY_CHAR; *out_char = 'r'; return 0;
    case KEY_S: *out_code = ACTION_KEY_CHAR; *out_char = 's'; return 0;
    case KEY_T: *out_code = ACTION_KEY_CHAR; *out_char = 't'; return 0;
    case KEY_U: *out_code = ACTION_KEY_CHAR; *out_char = 'u'; return 0;
    case KEY_V: *out_code = ACTION_KEY_CHAR; *out_char = 'v'; return 0;
    case KEY_W: *out_code = ACTION_KEY_CHAR; *out_char = 'w'; return 0;
    case KEY_X: *out_code = ACTION_KEY_CHAR; *out_char = 'x'; return 0;
    case KEY_Y: *out_code = ACTION_KEY_CHAR; *out_char = 'y'; return 0;
    case KEY_Z: *out_code = ACTION_KEY_CHAR; *out_char = 'z'; return 0;
    default: return -1;
    }
}

int main(int argc, char **argv)
{
    const char *conf_path = DEFAULT_CONF;
    if (argc > 2) {
        fprintf(stderr, "Usage: %s [config_path]\n", argv[0]);
        return 1;
    }
    if (argc == 2) {
        conf_path = argv[1];
    }

    signal(SIGINT, on_sigint);

    config_t cfg;
    config_defaults(&cfg);
    if (config_load(&cfg, conf_path) < 0) {
        return 1;
    }
    if (cfg.action_count == 0) {
        fprintf(stderr, "No actions configured; exiting.\n");
        return 1;
    }

    action_binding_t bindings[ACTION_MAX];
    memset(bindings, 0, sizeof(bindings));
    for (size_t i = 0; i < cfg.action_count && i < ACTION_MAX; i++) {
        bindings[i].spec = cfg.actions[i];
        bindings[i].udp_fd = -1;
    }

    int input_fd = -1;
    if (cfg.input_device[0]) {
        input_fd = open(cfg.input_device, O_RDONLY | O_NONBLOCK);
        if (input_fd < 0) {
            fprintf(stderr, "Failed to open input device %s: %s\n",
                    cfg.input_device, strerror(errno));
        } else if (cfg.verbose) {
            fprintf(stderr, "Listening for keys on %s\n", cfg.input_device);
        }
    }

    int old_flags = -1;
    struct termios old_term;
    if (set_stdin_raw(&old_flags, &old_term) < 0) {
        fprintf(stderr, "Warning: failed to set stdin raw mode; keyboard triggers disabled.\n");
    } else {
        fprintf(stderr, "Press configured keys to fire actions; Ctrl+C to exit.\n");
    }

    while (g_run) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fileno(stdin), &rfds);
    int max_fd = fileno(stdin);
    if (input_fd >= 0) {
        FD_SET(input_fd, &rfds);
        if (input_fd > max_fd) {
            max_fd = input_fd;
        }
    }
    struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };
    int rc = select(max_fd + 1, &rfds, NULL, NULL, &tv);
        if (rc > 0 && FD_ISSET(fileno(stdin), &rfds)) {
            char buf[32];
            ssize_t n = read(fileno(stdin), buf, sizeof(buf));
            if (n > 0) {
                ssize_t i = 0;
                while (i < n) {
                    unsigned char c = (unsigned char)buf[i++];
                    if (c == 0x1B && i + 1 < n && buf[i] == '[') {
                        unsigned char code = (unsigned char)buf[i + 1];
                        if (code == 'A') {
                            handle_keycode(ACTION_KEY_UP, 0, &cfg, bindings);
                            maybe_log(&cfg, NULL, "key up", 0);
                        } else if (code == 'B') {
                            handle_keycode(ACTION_KEY_DOWN, 0, &cfg, bindings);
                            maybe_log(&cfg, NULL, "key down", 0);
                        } else if (code == 'C') {
                            handle_keycode(ACTION_KEY_RIGHT, 0, &cfg, bindings);
                            maybe_log(&cfg, NULL, "key right", 0);
                        } else if (code == 'D') {
                            handle_keycode(ACTION_KEY_LEFT, 0, &cfg, bindings);
                            maybe_log(&cfg, NULL, "key left", 0);
                        }
                        i += 2;
                        continue;
                    }
                    if (c == '\r' || c == '\n') {
                        handle_keycode(ACTION_KEY_ENTER, 0, &cfg, bindings);
                        maybe_log(&cfg, NULL, "key enter", 0);
                    } else if (c == ' ') {
                        handle_keycode(ACTION_KEY_SPACE, 0, &cfg, bindings);
                        maybe_log(&cfg, NULL, "key space", 0);
                    } else {
                        handle_keycode(ACTION_KEY_CHAR, (char)c, &cfg, bindings);
                        char msg[32];
                        snprintf(msg, sizeof(msg), "key '%c'", c);
                        maybe_log(&cfg, NULL, msg, 0);
                    }
                }
            }
        }
        if (rc > 0 && input_fd >= 0 && FD_ISSET(input_fd, &rfds)) {
            struct input_event ev[16];
            ssize_t n = read(input_fd, ev, sizeof(ev));
            if (n > 0) {
                size_t count = (size_t)n / sizeof(struct input_event);
                for (size_t i = 0; i < count; i++) {
                    if (ev[i].type != EV_KEY) {
                        continue;
                    }
                    if (ev[i].value != 1 && ev[i].value != 2) { /* press or repeat */
                        continue;
                    }
                    action_keycode_t code;
                    char ch;
                    if (map_evdev_key(ev[i].code, &code, &ch) == 0) {
                        handle_keycode(code, ch, &cfg, bindings);
                    }
                }
            }
        }
    }

    restore_stdin(old_flags, &old_term);
    if (input_fd >= 0) {
        close(input_fd);
    }
    for (size_t i = 0; i < ACTION_MAX; i++) {
        free_binding(&bindings[i]);
    }
    return 0;
}
