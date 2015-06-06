#pragma once
#ifdef OPENMP
    #include "omp.h"
#endif

namespace openmp{

const int NUM_THREADS = 8;
// set scheduler cpu affinity, number of omp threads, init parallel eigen3
inline void setNumThreads(int n, bool out = false){
#ifdef OPENMP
#if __unix__
	cpu_set_t my_set;
	CPU_ZERO(&my_set);
	for(int i = 0; i < n; ++i)
		CPU_SET(i, &my_set);
	sched_setaffinity(0, sizeof(cpu_set_t), &my_set);
#endif

    omp_set_num_threads(n);
	Eigen::initParallel();

	if(out)
	{
		std::cout << " Number of processors available: " << omp_get_num_procs()
			<< " MAX number of OpenMP threads: "   << omp_get_max_threads() << std::endl;
	}
#endif
}

} // openmp::
