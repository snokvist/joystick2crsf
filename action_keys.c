/**
 * action_keys.c - simple hotkey-driven UDP/HTTP action dispatcher
 *
 * Reads actions from a config file and fires them when matching keyboard keys
 * (stdin raw mode) are pressed.
 */

#define _POSIX_C_SOURCE 200809L
#define _GNU_SOURCE
#include <ctype.h>
#include <curl/curl.h>
#include <errno.h>
#include <fcntl.h>
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
    CURL *curl;
    struct curl_slist *curl_headers;
} action_binding_t;

typedef struct {
    int http_timeout_ms;
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

static size_t curl_discard_cb(void *ptr, size_t size, size_t nmemb, void *userdata)
{
    (void)ptr;
    (void)userdata;
    return size * nmemb;
}

static void free_binding(action_binding_t *b)
{
    if (!b) {
        return;
    }
    if (b->udp_fd >= 0) {
        close(b->udp_fd);
    }
    if (b->curl) {
        curl_easy_cleanup(b->curl);
    }
    if (b->curl_headers) {
        curl_slist_free_all(b->curl_headers);
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

static int init_http(action_binding_t *b)
{
    if (!b) {
        return -1;
    }
    if (b->curl) {
        return 0;
    }
    CURL *curl = curl_easy_init();
    if (!curl) {
        return -1;
    }
    curl_easy_setopt(curl, CURLOPT_URL, b->spec.destination);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, (long)b->spec.timeout_ms);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_discard_cb);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "action_keys");
    if (b->spec.method == ACTION_HTTP_POST) {
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, b->spec.body);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)b->spec.body_len);
    } else {
        curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
    }
    struct curl_slist *hdrs = NULL;
    for (size_t i = 0; i < b->spec.header_count; i++) {
        hdrs = curl_slist_append(hdrs, b->spec.headers[i]);
    }
    if (hdrs) {
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, hdrs);
    }
    b->curl_headers = hdrs;
    b->curl = curl;
    return 0;
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

static int send_http(action_binding_t *b)
{
    if (!b) {
        return -1;
    }
    if (init_http(b) < 0) {
        return -1;
    }
    CURLcode rc = curl_easy_perform(b->curl);
    if (rc != CURLE_OK) {
        return -1;
    }
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

static void config_defaults(config_t *cfg)
{
    cfg->http_timeout_ms = ACTION_HTTP_TIMEOUT_MS_DEFAULT;
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
    char *save = NULL;
    for (char *tok = strtok_r(dup, ",", &save); tok; tok = strtok_r(NULL, ",", &save)) {
        char *eq = strchr(tok, '=');
        if (!eq || !eq[1]) {
            fprintf(stderr, "%s:%d: action token '%s' missing '='\n", path, lineno, tok);
            free(dup);
            return -1;
        }
        *eq = '\0';
        char *k = tok;
        char *v = eq + 1;
        trim(k);
        trim(v);
        if (!strcasecmp(k, "key")) {
            if (!*v) {
                fprintf(stderr, "%s:%d: action key cannot be empty\n", path, lineno);
                free(dup);
                return -1;
            }
            if (!strcasecmp(v, "up")) {
                tmp.key_code = ACTION_KEY_UP;
            } else if (!strcasecmp(v, "down")) {
                tmp.key_code = ACTION_KEY_DOWN;
            } else if (!strcasecmp(v, "left")) {
                tmp.key_code = ACTION_KEY_LEFT;
            } else if (!strcasecmp(v, "right")) {
                tmp.key_code = ACTION_KEY_RIGHT;
            } else if (!strcasecmp(v, "enter") || !strcasecmp(v, "return")) {
                tmp.key_code = ACTION_KEY_ENTER;
            } else if (!strcasecmp(v, "space") || !strcasecmp(v, "spacebar")) {
                tmp.key_code = ACTION_KEY_SPACE;
            } else {
                tmp.key_code = ACTION_KEY_CHAR;
                tmp.key_char = v[0];
            }
        } else if (!strcasecmp(k, "transport")) {
            if (!strcasecmp(v, "udp")) {
                tmp.transport = ACTION_TRANSPORT_UDP;
            } else if (!strcasecmp(v, "http")) {
                tmp.transport = ACTION_TRANSPORT_HTTP;
            } else {
                fprintf(stderr, "%s:%d: transport must be udp or http\n", path, lineno);
                free(dup);
                return -1;
            }
        } else if (!strcasecmp(k, "method")) {
            if (!strcasecmp(v, "get")) {
                tmp.method = ACTION_HTTP_GET;
            } else if (!strcasecmp(v, "post")) {
                tmp.method = ACTION_HTTP_POST;
            } else {
                fprintf(stderr, "%s:%d: method must be GET or POST\n", path, lineno);
                free(dup);
                return -1;
            }
        } else if (!strcasecmp(k, "url") || !strcasecmp(k, "destination") ||
                   !strcasecmp(k, "dest")) {
            if (strlen(v) >= ACTION_MAX_DEST_LEN) {
                fprintf(stderr, "%s:%d: destination too long (max %d)\n",
                        path, lineno, ACTION_MAX_DEST_LEN - 1);
                free(dup);
                return -1;
            }
            snprintf(tmp.destination, sizeof(tmp.destination), "%s", v);
        } else if (!strcasecmp(k, "body") || !strcasecmp(k, "payload")) {
            size_t len = strlen(v);
            if (len >= ACTION_MAX_BODY_LEN) {
                fprintf(stderr, "%s:%d: body too long (max %d)\n",
                        path, lineno, ACTION_MAX_BODY_LEN - 1);
                free(dup);
                return -1;
            }
            snprintf(tmp.body, sizeof(tmp.body), "%s", v);
            tmp.body_len = len;
        } else if (!strcasecmp(k, "header")) {
            if (tmp.header_count >= ACTION_MAX_HEADERS) {
                fprintf(stderr, "%s:%d: too many headers (max %d)\n",
                        path, lineno, ACTION_MAX_HEADERS);
                free(dup);
                return -1;
            }
            if (strlen(v) >= ACTION_MAX_HEADER_LEN) {
                fprintf(stderr, "%s:%d: header too long (max %d)\n",
                        path, lineno, ACTION_MAX_HEADER_LEN - 1);
                free(dup);
                return -1;
            }
            snprintf(tmp.headers[tmp.header_count], sizeof(tmp.headers[0]), "%s", v);
            tmp.header_count++;
        } else if (!strcasecmp(k, "timeout_ms")) {
            tmp.timeout_ms = atoi(v);
            if (tmp.timeout_ms <= 0) {
                tmp.timeout_ms = ACTION_HTTP_TIMEOUT_MS_DEFAULT;
            }
        } else {
            fprintf(stderr, "%s:%d: unknown action field '%s'\n", path, lineno, k);
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
    for (size_t i = 0; i < cfg->action_count && i < ACTION_MAX; i++) {
        const action_t *a = &cfg->actions[i];
        if (a->key_code == ACTION_KEY_NONE) {
            continue;
        }
        if (a->key_code == ACTION_KEY_CHAR && code == ACTION_KEY_CHAR && a->key_char == ch) {
            dispatch(&bindings[i]);
        } else if (a->key_code == code && code != ACTION_KEY_CHAR) {
            dispatch(&bindings[i]);
        }
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

    if (curl_global_init(CURL_GLOBAL_DEFAULT) != 0) {
        fprintf(stderr, "Failed to initialize curl\n");
        return 1;
    }

    action_binding_t bindings[ACTION_MAX];
    memset(bindings, 0, sizeof(bindings));
    for (size_t i = 0; i < cfg.action_count && i < ACTION_MAX; i++) {
        bindings[i].spec = cfg.actions[i];
        bindings[i].udp_fd = -1;
        bindings[i].curl = NULL;
        bindings[i].curl_headers = NULL;
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
        struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };
        int rc = select(fileno(stdin) + 1, &rfds, NULL, NULL, &tv);
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
                        } else if (code == 'B') {
                            handle_keycode(ACTION_KEY_DOWN, 0, &cfg, bindings);
                        } else if (code == 'C') {
                            handle_keycode(ACTION_KEY_RIGHT, 0, &cfg, bindings);
                        } else if (code == 'D') {
                            handle_keycode(ACTION_KEY_LEFT, 0, &cfg, bindings);
                        }
                        i += 2;
                        continue;
                    }
                    if (c == '\r' || c == '\n') {
                        handle_keycode(ACTION_KEY_ENTER, 0, &cfg, bindings);
                    } else if (c == ' ') {
                        handle_keycode(ACTION_KEY_SPACE, 0, &cfg, bindings);
                    } else {
                        handle_keycode(ACTION_KEY_CHAR, (char)c, &cfg, bindings);
                    }
                }
            }
        }
    }

    restore_stdin(old_flags, &old_term);
    for (size_t i = 0; i < ACTION_MAX; i++) {
        free_binding(&bindings[i]);
    }
    curl_global_cleanup();
    return 0;
}
