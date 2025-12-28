#ifndef ACTION_KEYS_H
#define ACTION_KEYS_H

#include <stddef.h>
#include <sys/socket.h>
#include <time.h>
#include <pthread.h>

#define ACTION_MAX                     32
#define ACTION_DEBOUNCE_MS             500
#define ACTION_MAX_BODY_LEN            512
#define ACTION_MAX_HEADER_LEN          128
#define ACTION_MAX_HEADERS             8
#define ACTION_MAX_DEST_LEN            256
#define ACTION_HTTP_TIMEOUT_MS_DEFAULT 1500
#define ACTION_KEYS_DEFAULT_CONF       "/etc/joystick2crsf.conf"
#define ACTION_QUEUE_SIZE              64

typedef enum {
    ACTION_TRANSPORT_UDP = 0,
    ACTION_TRANSPORT_HTTP
} action_transport_t;

typedef enum {
    ACTION_HTTP_GET = 0,
    ACTION_HTTP_POST
} action_http_method_t;

typedef enum {
    ACTION_EDGE_HIGH = 0,
    ACTION_EDGE_LOW
} action_edge_t;

typedef enum {
    ACTION_PRESS_ANY = 0,
    ACTION_PRESS_SHORT,
    ACTION_PRESS_LONG
} action_press_t;

typedef struct {
    int channel; /* zero-based channel index */
    action_edge_t edge;
    action_press_t press;
    action_transport_t transport;
    action_http_method_t method;
    char destination[ACTION_MAX_DEST_LEN];
    char body[ACTION_MAX_BODY_LEN];
    size_t body_len;
    char headers[ACTION_MAX_HEADERS][ACTION_MAX_HEADER_LEN];
    size_t header_count;
    int timeout_ms;
} action_spec_t;

typedef struct {
    action_spec_t spec;
    int index; /* To map to cached socket */
} action_queue_item_t;

typedef struct {
    pthread_t thread;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    int running;

    action_queue_item_t queue[ACTION_QUEUE_SIZE];
    int head;
    int tail;

    /* Cached UDP sockets: index corresponds to config->actions index */
    int sockets[ACTION_MAX];
    struct sockaddr_storage socket_addrs[ACTION_MAX];
    socklen_t socket_addr_lens[ACTION_MAX];
} action_worker_t;

typedef struct {
    struct timespec last_dispatch;
    int pending_idx;
} action_state_t;

typedef struct {
    int http_timeout_ms;
    int verbose;
    size_t action_count;
    action_spec_t actions[ACTION_MAX];
} action_keys_config_t;

void action_keys_config_defaults(action_keys_config_t *cfg);
int action_keys_config_load(action_keys_config_t *cfg, const char *path);

void action_keys_worker_init(action_worker_t *worker);
void action_keys_worker_stop(action_worker_t *worker);

void action_keys_handle_press(const action_keys_config_t *cfg,
                              action_worker_t *worker,
                              action_state_t *state,
                              int channel_index,
                              action_edge_t edge,
                              action_press_t press,
                              const struct timespec *now);

void action_keys_process_pending(const action_keys_config_t *cfg,
                                 action_worker_t *worker,
                                 action_state_t *state,
                                 const struct timespec *now);

void action_keys_build_watchlist(const action_keys_config_t *cfg,
                                 int watch_high[16],
                                 int watch_low[16]);

#endif
