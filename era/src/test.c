#include <math.h>
#include <limits.h>

#if defined(HPVM)
#include "hpvm.h"
#include "hetero.h"
#endif

#if false
static unsigned int _rev (unsigned int v) {
  unsigned int r = v;
  int s = sizeof(v) * CHAR_BIT - 1;

  for (v >>= 1; v; v >>= 1)
  {
    r <<= 1;
    r |= v & 1;
    s--;
  }
  r <<= s;

  return r;
}


__attribute__ ((noinline)) void testFunction(float* array, size_t array_sz, float* array1, size_t array1_sz) {
	void* Section = __hetero_section_begin();
	void* T = __hetero_task_begin(2, array, array_sz, array1, array1_sz, 2, array, array_sz, array1, array1_sz);

	__hpvm__hint(DEVICE);

	int interleaver_pattern[48] = {  0 , 3, 6, 9,12,15,18,21,
                                       24,27,30,33,36,39,42,45,
                                       1 , 4, 7,10,13,16,19,22,
                                       25,28,31,34,37,40,43,46,
                                       2 , 5, 8,11,14,17,20,23,
                                       26,29,32,35,38,41,44,47};

	for(int ii = 0; ii < 48; ii++) {
		int i = _rev(ii);
		array[ii] = array1[i];
	}



	__hetero_task_end(T);
	__hetero_section_end(Section);
}

__attribute__ ((noinline)) void testFunctionWrapper(float* array, size_t array_sz, float* array1, size_t array1_sz) {
	void* Section = __hetero_section_begin();
	void* T = __hetero_task_begin(2, array, array_sz, array1, array1_sz, 2, array, array_sz, array1, array1_sz);

	testFunction(array, array_sz, array1, array1_sz);

	__hetero_task_end(T);
	__hetero_section_end(Section);
}

int main() {
	float array[48];
	size_t array_sz = 48*sizeof(float);
	float array1[48];
	size_t array1_sz = 48*sizeof(float);

	void* DAG = __hetero_launch(testFunctionWrapper, 2, array, array_sz, array1, array1_sz, 
			2, array, array_sz, array1, array1_sz);
	__hetero_wait(DAG);

	return 0;
}
#endif
