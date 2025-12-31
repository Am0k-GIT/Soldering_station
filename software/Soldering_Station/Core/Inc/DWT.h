// DWT tics in one microsecond
// for 168MHz: 168 000 000 / 1 000 000 = 168
// for 216MHz: 216 000 000 / 1 000 000 = 216

#define    DWT_CYCCNT    *(volatile uint32_t*)0xE0001004
#define    DWT_CONTROL   *(volatile uint32_t*)0xE0001000
#define    SCB_DEMCR     *(volatile uint32_t*)0xE000EDFC

uint32_t DWT_Get_Current_Tick ()
{
    return DWT_CYCCNT;
}

uint32_t DWT_Elapsed_Tick (uint32_t t0)
{
    if (DWT_CYCCNT > t0)
        return DWT_CYCCNT - t0;

    return (uint32_t) ((((uint64_t) 0x100000000) + DWT_CYCCNT) - t0);
}

void DWT_Init()
{
	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;// разрешаем использовать DWT
	DWT_CONTROL|= DWT_CTRL_CYCCNTENA_Msk; // включаем счётчик
}
