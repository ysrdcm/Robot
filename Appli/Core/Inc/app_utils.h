#ifndef __APP_UTILS_H
#define __APP_UTILS_H

#ifndef MIN
#define MIN(a, b)               ((a) < (b) ? (a) : (b))
#endif

#define ALIGN_VALUE(v, a)       (((v) + (a) - 1) & ~((a) - 1))

#endif
