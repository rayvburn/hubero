#define HUBERO_LOG_ENABLE 1
#define HUBERO_LOG(fmt, ...) do { if (HUBERO_LOG_ENABLE) printf("%s " fmt, "[HuBeRo]", ##__VA_ARGS__); } while (0)
