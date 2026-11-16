/*
 * Host simulation runner.
 *
 * Usage:  bms_sim [scenario.csv] [trace.csv]
 *
 * With one argument the trace is written to stdout; with two it is written to
 * the second file so the CMake `simulate` target can build plots without shell
 * redirection.
 *
 * The scenario file contains one row per setpoint change:
 *
 *   t_ms,current_ma,cell_min_mv,cell_max_mv,cell_max_idx,temp_dc,bus_mv,charger,clear
 *
 *  - t_ms          absolute simulation time of the setpoint change
 *  - current_ma    pack current, positive = charge
 *  - cell_min_mv   voltage applied to every cell
 *  - cell_max_mv   voltage applied to cell_max_idx (use -1 for "all equal")
 *  - temp_dc       cell and board temperature in 0.1 degC
 *  - bus_mv        bus voltage, 0 = tied to the pack voltage
 *  - charger       1 = charger present
 *  - clear         1 = VCU requests a DTC clear (rising edge)
 *
 * A CSV trace is written to stdout so that tools/plot_svg can render it:
 *
 *   time_ms,state,soc_permille,current_ma,cell0..cell4,temp0_dc,
 *   bal_mask,bal_inhibit,ccl_ma,dcl_ma,neg,pre,pos,fault_bits
 */
#include "app/app.h"
#include "hal/hal_sim.h"
#include "services/can_matrix.h"

#include <stdio.h>
#include <string.h>

#define SIM_MAX_ROWS 128U
#define SIM_SAMPLE_MS 100U
#define SIM_VCU_PERIOD_MS 100U
#define SIM_CELL_COUNT 16U

typedef struct
{
    uint32_t t_ms;
    int32_t current_ma;
    uint16_t cell_min_mv;
    uint16_t cell_max_mv;
    int cell_max_idx;
    int16_t temp_dc;
    uint16_t bus_mv;
    int charger;
    int clear;
} SimRow_t;

static SimRow_t s_rows[SIM_MAX_ROWS];
static uint32_t s_row_count;

static void ApplyRow(const SimRow_t *row)
{
    for (uint8_t i = 0U; i < SIM_CELL_COUNT; ++i)
    {
        HalSim_SetCellMv(i, row->cell_min_mv);
    }
    if ((row->cell_max_idx >= 0) && (row->cell_max_idx < (int)SIM_CELL_COUNT))
    {
        HalSim_SetCellMv((uint8_t)row->cell_max_idx, row->cell_max_mv);
    }

    for (uint8_t i = 0U; i < 5U; ++i)
    {
        HalSim_SetCellTempDeciC(i, row->temp_dc);
    }
    HalSim_SetBoardTempDc(row->temp_dc);
    HalSim_SetPackCurrentMa(row->current_ma);

    uint32_t pack_mv = (uint32_t)row->cell_min_mv * SIM_CELL_COUNT;
    if ((row->cell_max_idx >= 0) && (row->cell_max_mv > row->cell_min_mv))
    {
        pack_mv += (uint32_t)(row->cell_max_mv - row->cell_min_mv);
    }
    HalSim_SetPackVoltageMv((uint16_t)pack_mv);
    HalSim_SetBusVoltageMv((row->bus_mv != 0U) ? row->bus_mv : (uint16_t)pack_mv);
    HalSim_SetChargerPresent(row->charger != 0);
}

static void InjectVcuCommand(const SimRow_t *row, uint8_t alive)
{
    CanVcuCommand_t command;
    (void)memset(&command, 0, sizeof(command));
    command.contactor_req = 1U; /* close request */
    command.balancing_enable = true;
    command.sleep_req = false;
    command.fault_clear_req = (row->clear != 0);
    command.alive_counter = (uint8_t)(alive & 0x0FU);

    uint8_t data[CAN_DLC];
    CanMatrix_PackVcuCommand(data, &command);
    HalSim_CanInject(CAN_ID_VCU_COMMAND, data, CAN_DLC);
}

static void PrintHeader(FILE *out)
{
    (void)fprintf(out, "time_ms,state,soc_permille,current_ma,cell0_mv,cell1_mv,cell2_mv,"
                       "cell3_mv,cell4_mv,temp0_dc,bal_mask,bal_inhibit,ccl_ma,dcl_ma,"
                       "neg,pre,pos,fault_bits\n");
}

static void PrintSample(FILE *out, uint32_t now_ms)
{
    AppSnapshot_t snap;
    App_GetSnapshot(&snap);

    (void)fprintf(out, "%lu,%s,%ld,%ld,%u,%u,%u,%u,%u,%d,%u,%u,%ld,%ld,%d,%d,%d,%lu\n",
                  (unsigned long)now_ms,
                  (snap.state_name != NULL) ? snap.state_name : "UNKNOWN",
                  (long)snap.soc_permille,
                  (long)snap.pack_current_ma,
                  snap.cell_mv[0], snap.cell_mv[1], snap.cell_mv[2], snap.cell_mv[3], snap.cell_mv[4],
                  (int)snap.cell_temp_dc[0],
                  snap.bal_mask,
                  snap.bal_inhibit,
                  (long)snap.ccl_ma,
                  (long)snap.dcl_ma,
                  snap.contactor_neg ? 1 : 0,
                  snap.contactor_pre ? 1 : 0,
                  snap.contactor_pos ? 1 : 0,
                  (unsigned long)snap.fault_bits);
}

static bool ParseScenario(const char *path)
{
    FILE *file = fopen(path, "r");
    if (file == NULL)
    {
        (void)fprintf(stderr, "bms_sim: cannot open scenario '%s'\n", path);
        return false;
    }

    char line[256];
    s_row_count = 0U;
    while ((s_row_count < SIM_MAX_ROWS) && (fgets(line, (int)sizeof(line), file) != NULL))
    {
        if ((line[0] == '#') || (line[0] == '\n') || (line[0] == '\r') || (line[0] == '\0'))
        {
            continue;
        }

        unsigned int t_ms = 0U;
        int current_ma = 0;
        unsigned int cell_min = 0U;
        unsigned int cell_max = 0U;
        int cell_idx = -1;
        int temp_dc = 250;
        unsigned int bus_mv = 0U;
        int charger = 0;
        int clear = 0;

        if (sscanf(line, "%u,%d,%u,%u,%d,%d,%u,%d,%d",
                   &t_ms, &current_ma, &cell_min, &cell_max, &cell_idx,
                   &temp_dc, &bus_mv, &charger, &clear) != 9)
        {
            (void)fprintf(stderr, "bms_sim: malformed scenario line: %s", line);
            (void)fclose(file);
            return false;
        }

        SimRow_t *row = &s_rows[s_row_count++];
        row->t_ms = t_ms;
        row->current_ma = current_ma;
        row->cell_min_mv = (uint16_t)cell_min;
        row->cell_max_mv = (uint16_t)cell_max;
        row->cell_max_idx = (cell_max >= cell_min) ? cell_idx : -1;
        row->temp_dc = (int16_t)temp_dc;
        row->bus_mv = (uint16_t)bus_mv;
        row->charger = charger;
        row->clear = clear;
    }

    (void)fclose(file);
    return s_row_count > 0U;
}

static bool LoadDefaultScenario(void)
{
    s_rows[0] = (SimRow_t){0U, 20000, 3450U, 3480U, 4, 250, 0U, 1, 0};
    s_rows[1] = (SimRow_t){60000U, 20000, 3450U, 3470U, 4, 250, 0U, 1, 0};
    s_rows[2] = (SimRow_t){90000U, 20000, 3450U, 3460U, 4, 250, 0U, 1, 0};
    s_rows[3] = (SimRow_t){120000U, 20000, 3450U, 3455U, 4, 260, 0U, 1, 0};
    s_row_count = 4U;
    return true;
}

int main(int argc, char **argv)
{
    if (argc > 1)
    {
        if (!ParseScenario(argv[1]))
        {
            return 1;
        }
    }
    else if (!LoadDefaultScenario())
    {
        return 1;
    }

    HalSim_Init();
    ApplyRow(&s_rows[0]);

    App_Init();

    FILE *out = stdout;
    if ((argc > 2) && (argv[2] != NULL))
    {
        out = fopen(argv[2], "w");
        if (out == NULL)
        {
            (void)fprintf(stderr, "bms_sim: cannot write '%s'\n", argv[2]);
            return 1;
        }
    }

    PrintHeader(out);

    uint32_t next_row = 1U;
    const SimRow_t *row = &s_rows[0];
    uint8_t alive = 0U;
    uint32_t last_ms = s_rows[s_row_count - 1U].t_ms;

    for (uint32_t now = 0U; now <= last_ms; ++now)
    {
        if (now > 0U)
        {
            HalSim_AdvanceMs(1U);
        }

        while ((next_row < s_row_count) && (s_rows[next_row].t_ms <= now))
        {
            row = &s_rows[next_row];
            ApplyRow(row);
            next_row++;
        }

        if ((now % SIM_VCU_PERIOD_MS) == 0U)
        {
            InjectVcuCommand(row, alive);
            alive = (uint8_t)((alive + 1U) & 0x0FU);
        }

        App_Step(now);

        if ((now % SIM_SAMPLE_MS) == 0U)
        {
            PrintSample(out, now);
        }
    }

    if (out != stdout)
    {
        (void)fclose(out);
    }
    return 0;
}
