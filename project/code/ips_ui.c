#include "ips_ui.h"
#include "laser.h"
#include "ring.h"

/* 在指定图像行画一条间隔点标记线，用来提示搜索行或参考行。 */
static void IPS_Draw_Row_Marker(uint16 x, uint16 y, uint8 row, uint16 color)
{
    uint8 i;

    if (row >= SEARCH_IMAGE_H) {
        return;
    }

    for (i = 0; i < SEARCH_IMAGE_W; i += 2) {
        ips200_draw_point(x + i, y + row, color);
    }
}

/* 显示摄像头灰度图，并叠加左右控制线、中线、参考行和目标点。 */
void IPS_Show_Image_And_Line(uint16 x, uint16 y)
{
    uint8 r;
    int16 p;

    ips200_show_gray_image(x, y, &mt9v03x_image[0][0],
                           SEARCH_IMAGE_W, SEARCH_IMAGE_H,
                           SEARCH_IMAGE_W, SEARCH_IMAGE_H,
                           white_min_point);

    IPS_Draw_Row_Marker(x, y, STOP_ROW, RGB565_CYAN);
    IPS_Draw_Row_Marker(x, y, controlReferenceLine, RGB565_YELLOW);

    if (current_step != 0) {
        IPS_Draw_Row_Marker(x, y, (uint8)((controlReferenceLine + SEARCH_IMAGE_H - 1) >> 1), RGB565_MAGENTA);
    }

    for (r = STOP_ROW; r < SEARCH_IMAGE_H; r++) {
        p = left_control_line[r];
        if (p >= 0 && p < SEARCH_IMAGE_W) {
            ips200_draw_point(x + p, y + r, RGB565_BLUE);
        }

        p = right_control_line[r];
        if (p >= 0 && p < SEARCH_IMAGE_W) {
            ips200_draw_point(x + p, y + r, RGB565_BLUE);
        }

        p = mid_line[r];
        if (p >= 0 && p < SEARCH_IMAGE_W) {
            ips200_draw_point(x + p, y + r, RGB565_RED);
        }
    }

    if (tar_flag &&
        tar_x > 1 && tar_x < SEARCH_IMAGE_W - 2 &&
        tar_y > 1 && tar_y < SEARCH_IMAGE_H - 2) {
        p = tar_x;
        r = tar_y;
        ips200_draw_point(x + p, y + r, RGB565_MAGENTA);
        ips200_draw_point(x + p - 1, y + r, RGB565_MAGENTA);
        ips200_draw_point(x + p + 1, y + r, RGB565_MAGENTA);
        ips200_draw_point(x + p, y + r - 1, RGB565_MAGENTA);
        ips200_draw_point(x + p, y + r + 1, RGB565_MAGENTA);
    }
}
