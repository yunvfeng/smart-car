#ifndef _IPS_UI_H_
#define _IPS_UI_H_

#include "zf_common_typedef.h"
#include "zf_device_ips200.h"
#include "ips_ui.h"
#include "image.h"
#include "laser.h"
#include "ring.h"

/* 显示灰度图，并叠加边线、中线、参考行和目标点。 */
void IPS_Show_Image_And_Line(uint16 x, uint16 y);

#endif
