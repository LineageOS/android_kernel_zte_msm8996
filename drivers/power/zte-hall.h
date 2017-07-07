#ifdef CONFIG_ZTE_HALL_AVAILABLE

struct hall_pwrkey {
    struct input_dev *hall_pwr;
};

typedef enum
{
    HALL_STATE_NULL,
    HALL_STATE_OPEN,
    HALL_STATE_CLOSE,
} HALL_STATE;
#endif
