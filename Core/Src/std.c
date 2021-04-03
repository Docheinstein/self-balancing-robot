#include "std.h"
#include <string.h>

bool streq(const char *s1, const char *s2)
{
	return strcmp(s1, s2) == 0;
}

bool strneq(const char *s1, const char *s2, size_t len)
{
	return strncmp(s1, s2, len) == 0;
}

/* Returns true if s1 starts with s2. */
bool strstarts(const char *s1, const char *s2)
{
	return strneq(s1, s2, strlen(s2));
}

/* Maps x from [in_min, in_max] to [out_min, out_max] */
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
