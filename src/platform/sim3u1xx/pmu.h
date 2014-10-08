// SIM3 power/sleep support

#ifndef __PMU_H__
#define __PMU_H__

void sim3_pmu_sleep( int seconds );
void sim3_pmu_pm9( int seconds );
void sim3_pmu_reboot( void );

#endif
