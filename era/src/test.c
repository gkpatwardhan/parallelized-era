#include <math.h>

#if defined(HPVM)
#include "hpvm.h"
#include "hetero.h"
#endif

#if false
__attribute__ ((noinline)) void testFunction(float* array, size_t array_sz) {
	void* Section = __hetero_section_begin();
	void* T = __hetero_task_begin(1, array, array_sz, 1, array, array_sz);

	__hpvm__hint(DEVICE);

	int interleaver_pattern[48] = {  0 , 3, 6, 9,12,15,18,21,
                                       24,27,30,33,36,39,42,45,
                                       1 , 4, 7,10,13,16,19,22,
                                       25,28,31,34,37,40,43,46,
                                       2 , 5, 8,11,14,17,20,23,
                                       26,29,32,35,38,41,44,47};

	for(int ii = 0; ii < 48; ii++) {
		array[ii] = (float)(interleaver_pattern[ii]);
	}


	__hetero_task_end(T);
	__hetero_section_end(Section);
}

__attribute__ ((noinline)) void testFunctionWrapper(float* array, size_t array_sz) {
	void* Section = __hetero_section_begin();
	void* T = __hetero_task_begin(1, array, array_sz, 1, array, array_sz);

	testFunction(array, array_sz);

	__hetero_task_end(T);
	__hetero_section_end(Section);
}

int main() {
	float array[48];
	size_t array_sz = 48*sizeof(float);

	void* DAG = __hetero_launch(testFunctionWrapper, 1, array, array_sz, 1, array, array_sz);
	__hetero_wait(DAG);

	return 0;
}
#endif
