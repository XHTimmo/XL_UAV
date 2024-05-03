
/* Attributes State Machine */
enum
{
    IDX_SVC,
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,

    HRS_IDX_NB,
};

typedef struct
{
    uint8_t *messige;
    uint8_t en;
} BLU_Messige;

extern BLU_Messige blu_messige;

void BLU_init(void);