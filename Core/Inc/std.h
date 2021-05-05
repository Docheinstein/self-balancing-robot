#ifndef STD_H
#define STD_H

#include <stdbool.h>
#include <stdlib.h>

#define BOOL_TO_STR(x) ((x) ? "true" : "false")
#define ABS(x) ((x) > 0 ? (x) : (-(x)))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

#define RAD_TO_DEG(rad) ((rad) * 180.0 / 3.14159265358979323846)
#define DEG_TO_RAD(deg) ((deg) * 3.14159265358979323846 / 180.0)

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define SIGNF(x) ((x) > 0 ? 1.0f : -1.0f)

/* Returns true if s1 == s2 */
bool streq(const char *s1, const char *s2);

/* Returns true if s1 != s2 */
bool strneq(const char *s1, const char *s2, size_t len);

/* Returns true if s1 starts with s2. */
bool strstarts(const char *s1, const char *s2);

/* Maps x from [in_min, in_max] to [out_min, out_max] */
float mapf(float x, float in_min, float in_max, float out_min, float out_max);

/* Returns max if x > max, min if x < min, x otherwise */
float rangef(float x, float min, float max);

/* Returns max if x > max, min if x < min, x otherwise */
int range(int x, int min, int max);

#endif /* STD_H*/
