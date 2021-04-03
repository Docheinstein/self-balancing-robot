#ifndef STD_H
#define STD_H

#include <stdbool.h>
#include <stdlib.h>

#define BOOL_TO_STRING(x) ((x) ? "true" : "false")
#define ABS(x) ((x) > 0 ? (x) : (-(x)))

#define RAD_TO_DEG(rad) ((rad) * 180.0 / 3.14159265358979323846)
#define DEG_TO_RAD(deg) ((deg) * 3.14159265358979323846 / 180.0)

bool streq(const char *s1, const char *s2);
bool strneq(const char *s1, const char *s2, size_t len);
bool strstarts(const char *s1, const char *s2);

float mapf(float x, float in_min, float in_max, float out_min, float out_max);

#endif /* STD_H*/
