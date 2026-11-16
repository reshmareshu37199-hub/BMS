/*
 * plot_svg - tiny, dependency-free CSV -> SVG line plotter.
 *
 * Usage:
 *   bms_plot <trace.csv> <out.svg> <column> [column ...]
 *
 * The first CSV column must be time_ms.  Every requested column is drawn as
 * one line with a shared linear Y axis.  The tool uses only fixed-size static
 * buffers (no malloc) so that it builds and runs anywhere the firmware does.
 */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PLOT_MAX_SERIES 8
#define PLOT_MAX_SAMPLES 20000
#define PLOT_MAX_COLUMNS 40
#define PLOT_NAME_LEN 40

#define PLOT_WIDTH 920.0
#define PLOT_HEIGHT 440.0
#define PLOT_LEFT 78.0
#define PLOT_RIGHT 24.0
#define PLOT_TOP 46.0
#define PLOT_BOTTOM 62.0

static char s_header[PLOT_MAX_COLUMNS][PLOT_NAME_LEN];
static int s_column_count;

static double s_x[PLOT_MAX_SAMPLES];
static double s_y[PLOT_MAX_SERIES][PLOT_MAX_SAMPLES];
static char s_series_name[PLOT_MAX_SERIES][PLOT_NAME_LEN];
static int s_series_count;
static int s_sample_count;

static const char *const s_colours[PLOT_MAX_SERIES] = {
    "#1f77b4", "#d62728", "#2ca02c", "#9467bd",
    "#ff7f0e", "#8c564b", "#17becf", "#e377c2",
};

static void Trim(char *text)
{
    size_t length = strlen(text);
    while ((length > 0U) &&
           ((text[length - 1U] == '\n') || (text[length - 1U] == '\r') ||
            (text[length - 1U] == ' ') || (text[length - 1U] == '\t')))
    {
        text[length - 1U] = '\0';
        length--;
    }
}

/* Split `line` on commas into s_header[]. */
static void ParseHeader(char *line)
{
    s_column_count = 0;
    char *cursor = line;
    while ((s_column_count < PLOT_MAX_COLUMNS) && (cursor != NULL))
    {
        char *comma = strchr(cursor, ',');
        if (comma != NULL)
        {
            *comma = '\0';
        }
        (void)snprintf(s_header[s_column_count], PLOT_NAME_LEN, "%s", cursor);
        Trim(s_header[s_column_count]);
        s_column_count++;
        cursor = (comma != NULL) ? (comma + 1) : NULL;
    }
}

static int FindColumn(const char *name)
{
    for (int i = 0; i < s_column_count; ++i)
    {
        if (strcmp(s_header[i], name) == 0)
        {
            return i;
        }
    }
    return -1;
}

static bool ParseRow(char *line, int sample)
{
    int column = 0;
    char *cursor = line;
    bool have_time = false;
    double time_ms = 0.0;

    while ((column < s_column_count) && (cursor != NULL))
    {
        char *comma = strchr(cursor, ',');
        if (comma != NULL)
        {
            *comma = '\0';
        }

        if (column == 0)
        {
            char *end = NULL;
            time_ms = strtod(cursor, &end);
            have_time = (end != cursor);
        }
        else
        {
            for (int s = 0; s < s_series_count; ++s)
            {
                if (FindColumn(s_series_name[s]) == column)
                {
                    char *end = NULL;
                    double value = strtod(cursor, &end);
                    s_y[s][sample] = (end != cursor) ? value : 0.0;
                }
            }
        }

        column++;
        cursor = (comma != NULL) ? (comma + 1) : NULL;
    }

    if (!have_time)
    {
        return false;
    }
    s_x[sample] = time_ms;
    return true;
}

static void FormatValue(char *buffer, size_t size, double value)
{
    (void)snprintf(buffer, size, "%.4g", value);
}

int main(int argc, char **argv)
{
    if (argc < 4)
    {
        (void)fprintf(stderr, "usage: bms_plot <trace.csv> <out.svg> <column> [column...]\n");
        return 2;
    }

    s_series_count = argc - 3;
    if (s_series_count > PLOT_MAX_SERIES)
    {
        (void)fprintf(stderr, "bms_plot: at most %d series\n", PLOT_MAX_SERIES);
        return 2;
    }

    FILE *input = fopen(argv[1], "r");
    if (input == NULL)
    {
        (void)fprintf(stderr, "bms_plot: cannot open '%s'\n", argv[1]);
        return 1;
    }

    char line[1024];
    if (fgets(line, (int)sizeof(line), input) == NULL)
    {
        (void)fprintf(stderr, "bms_plot: '%s' is empty\n", argv[1]);
        (void)fclose(input);
        return 1;
    }
    ParseHeader(line);

    if (FindColumn("time_ms") != 0)
    {
        (void)fprintf(stderr, "bms_plot: first column must be time_ms\n");
        (void)fclose(input);
        return 1;
    }

    for (int s = 0; s < s_series_count; ++s)
    {
        (void)snprintf(s_series_name[s], PLOT_NAME_LEN, "%s", argv[3 + s]);
        if (FindColumn(s_series_name[s]) < 0)
        {
            (void)fprintf(stderr, "bms_plot: no column '%s'\n", s_series_name[s]);
            (void)fclose(input);
            return 1;
        }
    }

    while ((s_sample_count < PLOT_MAX_SAMPLES) &&
           (fgets(line, (int)sizeof(line), input) != NULL))
    {
        if ((line[0] == '\0') || (line[0] == '\n') || (line[0] == '#'))
        {
            continue;
        }
        if (ParseRow(line, s_sample_count))
        {
            s_sample_count++;
        }
    }
    (void)fclose(input);

    if (s_sample_count < 2)
    {
        (void)fprintf(stderr, "bms_plot: not enough samples in '%s'\n", argv[1]);
        return 1;
    }

    double xmin = s_x[0];
    double xmax = s_x[s_sample_count - 1];
    double ymin = s_y[0][0];
    double ymax = ymin;
    for (int s = 0; s < s_series_count; ++s)
    {
        for (int i = 0; i < s_sample_count; ++i)
        {
            double value = s_y[s][i];
            if (value < ymin)
            {
                ymin = value;
            }
            if (value > ymax)
            {
                ymax = value;
            }
        }
    }
    if (xmax <= xmin)
    {
        xmax = xmin + 1.0;
    }
    if ((ymax - ymin) < 1e-9)
    {
        ymax = ymin + 1.0;
    }

    const double plot_w = PLOT_WIDTH - PLOT_LEFT - PLOT_RIGHT;
    const double plot_h = PLOT_HEIGHT - PLOT_TOP - PLOT_BOTTOM;

    FILE *output = fopen(argv[2], "w");
    if (output == NULL)
    {
        (void)fprintf(stderr, "bms_plot: cannot write '%s'\n", argv[2]);
        return 1;
    }

    (void)fprintf(output,
                  "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                  "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%.0f\" height=\"%.0f\" "
                  "viewBox=\"0 0 %.0f %.0f\">\n",
                  PLOT_WIDTH, PLOT_HEIGHT, PLOT_WIDTH, PLOT_HEIGHT);
    (void)fprintf(output,
                  "<rect width=\"100%%\" height=\"100%%\" fill=\"#ffffff\"/>\n"
                  "<text x=\"%g\" y=\"24\" font-family=\"Segoe UI,Arial,sans-serif\" "
                  "font-size=\"15\" font-weight=\"bold\" fill=\"#222\">BMS simulation trace</text>\n",
                  PLOT_LEFT);

    /* Grid and Y labels. */
    for (int i = 0; i <= 5; ++i)
    {
        double fraction = (double)i / 5.0;
        double y = PLOT_TOP + (plot_h * fraction);
        double value = ymax - ((ymax - ymin) * fraction);
        char label[32];
        FormatValue(label, sizeof(label), value);
        (void)fprintf(output,
                      "<line x1=\"%g\" y1=\"%g\" x2=\"%g\" y2=\"%g\" stroke=\"#e6e6e6\" stroke-width=\"1\"/>\n",
                      PLOT_LEFT, y, PLOT_LEFT + plot_w, y);
        (void)fprintf(output,
                      "<text x=\"%g\" y=\"%g\" text-anchor=\"end\" font-family=\"Segoe UI,Arial,sans-serif\" "
                      "font-size=\"11\" fill=\"#555\">%s</text>\n",
                      PLOT_LEFT - 8.0, y + 4.0, label);
    }

    /* X grid and time labels (seconds). */
    for (int i = 0; i <= 5; ++i)
    {
        double fraction = (double)i / 5.0;
        double x = PLOT_LEFT + (plot_w * fraction);
        double value = (xmin + ((xmax - xmin) * fraction)) / 1000.0;
        char label[32];
        (void)snprintf(label, sizeof(label), "%.4g", value);
        (void)fprintf(output,
                      "<line x1=\"%g\" y1=\"%g\" x2=\"%g\" y2=\"%g\" stroke=\"#e6e6e6\" stroke-width=\"1\"/>\n",
                      x, PLOT_TOP, x, PLOT_TOP + plot_h);
        (void)fprintf(output,
                      "<text x=\"%g\" y=\"%g\" text-anchor=\"middle\" font-family=\"Segoe UI,Arial,sans-serif\" "
                      "font-size=\"11\" fill=\"#555\">%s</text>\n",
                      x, PLOT_TOP + plot_h + 18.0, label);
    }

    /* Axes. */
    (void)fprintf(output,
                  "<rect x=\"%g\" y=\"%g\" width=\"%g\" height=\"%g\" fill=\"none\" stroke=\"#888\"/>\n",
                  PLOT_LEFT, PLOT_TOP, plot_w, plot_h);
    (void)fprintf(output,
                  "<text x=\"%g\" y=\"%g\" text-anchor=\"middle\" font-family=\"Segoe UI,Arial,sans-serif\" "
                  "font-size=\"12\" fill=\"#333\">time (s)</text>\n",
                  PLOT_LEFT + (plot_w / 2.0), PLOT_HEIGHT - 16.0);

    /* Data. */
    for (int s = 0; s < s_series_count; ++s)
    {
        (void)fprintf(output,
                      "<polyline fill=\"none\" stroke=\"%s\" stroke-width=\"1.6\" "
                      "stroke-linejoin=\"round\" points=\"",
                      s_colours[s]);
        for (int i = 0; i < s_sample_count; ++i)
        {
            double x = PLOT_LEFT + (((s_x[i] - xmin) / (xmax - xmin)) * plot_w);
            double y = PLOT_TOP + (((ymax - s_y[s][i]) / (ymax - ymin)) * plot_h);
            (void)fprintf(output, "%.2f,%.2f ", x, y);
        }
        (void)fprintf(output, "\"/>\n");
    }

    /* Legend. */
    for (int s = 0; s < s_series_count; ++s)
    {
        double x = PLOT_LEFT + 12.0 + ((double)(s % 3) * 210.0);
        double y = PLOT_TOP + 14.0 + ((double)(s / 3) * 18.0);
        (void)fprintf(output,
                      "<line x1=\"%g\" y1=\"%g\" x2=\"%g\" y2=\"%g\" stroke=\"%s\" stroke-width=\"2.5\"/>\n",
                      x, y, x + 22.0, y, s_colours[s]);
        (void)fprintf(output,
                      "<text x=\"%g\" y=\"%g\" font-family=\"Segoe UI,Arial,sans-serif\" "
                      "font-size=\"11\" fill=\"#333\">%s</text>\n",
                      x + 28.0, y + 4.0, s_series_name[s]);
    }

    (void)fprintf(output, "</svg>\n");
    if (fclose(output) != 0)
    {
        (void)fprintf(stderr, "bms_plot: failed to finalise '%s'\n", argv[2]);
        return 1;
    }

    (void)printf("wrote %s (%d samples, %d series)\n", argv[2], s_sample_count, s_series_count);
    return 0;
}
