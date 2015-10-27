/* According to
 *   http://help.robotc.net/WebHelpVEX/index.htm#Resources/topics/General_C_Programming/Data_Types.htm
 * long is 64 bit, while float is only 32 bit, so we are going to keep time in microseconds (10^-6 sec)
 */

int hf_sensor_cnt;
int hf_sensor_id[5];
long hf_sensor_val[5]; // latest sensor value (after the change)
long hf_sensor_t[5];   // timestamp when sensor was observed to change the value (microseconds)
long hf_sensor_nxt[5]; // (nSysTime) when sensor is expected to change its value (microseconds)

/* Need to start this task with kHighPriority:
 * http://help.robotc.net/WebHelpVEX/index.htm#Resources/topics/VEX_Cortex/ROBOTC/Task_Control/startTask.htm
 */
task sensor_task()
{
	while(1)
	{
		int i,j,n;
		long curCnt[5];
		short curVal[5];
		for( i=0 ; i<hf_sensor_cnt ; i++ )
		{
			curVal[i] = sensorValue[hf_sensor_id[i]];
			curCnt[i] = -1;
		}
		hogCPU(); // obtain exclusive lock on CPU by this task
		long nSysCurr = nSysTime;
		while(nSysCurr == nSysTime); // spin until timer ticks
		nSysCurr = nSysTime;
		for( n=0 ; n ; n++ )
		{
			if( nSysCurr == nSysTime ) // still in the same time tick
			{
				for( i=0 ; i<hf_sensor_cnt ; i++ )
				{
					short s = sensorValue[hf_sensor_id[i]];
					if( curVal[i] != s ) // has sensor changed?
					{
						curVal[i] = s; // remember new value
						curCnt[i] = n; // remember sub-tick count
					}
				}
			}
			else // nSysTime changed
			{
				int min_dt = 10*1000;
				for(i=0;i<hf_sensor_cnt;i++)
				{
					if( curCnt[i] >= 0 ) // there was sensor change during prev tick cycle
					{
						hf_sensor_val[i] = curVal[i];
						hf_sensor_t[i] = nSysCurr * 1000 + (curCnt[i]*1000)/n;
						curCnt[i] = -2; // this sensor has been read in this measurement cycle
					}
					else if( curCnt[i] == -1 ) // for sensors that still haven't changed
					{
						long dt = hf_sensor_nxt[i] - nSysTime * 1000; // when would it change?
						if( dt>0 && dt < min_dt)
						{
							min_dt = dt; // remember closest future expected change time
						}
					}
				}
				if( min_dt > 3000 ) // if it is more than 3 msec away - yield to other tasks
				{
					break;
				}
				nSysCurr = nSysTime; // otherwise monitor next clock cycle
				n = 0;
			}
		}
		releaseCPU(); // release exclusive lock on CPU by this task
		delay(1); // yield at least one 1 msec cycle to other tasks
	}
}
