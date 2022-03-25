# Analysis
This documentation contains the analysis of the tasks' performance. For explanations on their implementations, please read [this file](README.md).

# Table of contents
1. [Time Analysis of each task](#time_analysis)
2. [Critical Instant Analysis](#critical)
4. [Total CPU utilisation](#CPU_utilisation)
5. [Safe Access and Synchronisation](#safety_features)
6. [Analysis of inter-task blocking dependencies](#intertask_blocking)





# Time Analysis of each task <a name="time_analysis"></a>

- Initiation: A new iteration of a task
- Initiation interval (τ): time between initiations of a particular task
- Deadline: time by which the task must be complete – often assumed to be the same as τ
- Execution time (T): CPU time needed to complete a task if it is not interrupted
- Latency, Ln: time actually taken to complete a task

To measure the execution time for a single task, we have to disable all the other tasks. Next, we run each task separately for 32 iterations and find the average. This is to average out measurement inaccuracies.

|             Task             | Initiation Interval, τi (ms) | Execution Time, Ti (us) (32 iterations) | Execution Time, Ti (us) (per iteration) | τn/τi | (τn/τi)*Ti (s) |                 Commit ID                |
|:----------------------------:|:----------------------------:|:---------------------------------------:|:---------------------------------------:|:-----:|:--------------:|:----------------------------------------:|
| scanKeysTask                 |                           20 |                                    3051 |                                   95.34 |     5 |    0.000476719 | 4e62d4fa369764adce4eb34d112c3d424d3daae7 |
| displayUpdateTask            |                          100 |                                  555572 |                                17361.63 |     1 |    0.017361625 | c2020273debcac73a500bc6ae7fe348c4ae6c57c |
| decodeTask                   |                         25.2 |                                    2852 |                                  89.125 |  3.97 |    0.000353671 | 8901783b784156c23fd8db63d74868925d00c72c |
| CAN_TX_Task                  |                           60 |                                      79 |                                    2.47 |  1.67 |    4.11458E-06 | 4852abb2121396965b59a2d9735347eb99a36166 |
| generateCurrentStepArrayTask |                           20 |                                   26000 |                                   812.5 |     5 |      0.0040625 | 9a9f857e22da221b61651f16c45e3e618d74ba2f |

| Lowest priority task, initiation interval, tn (ms) | 100   |
|----------------------------------------------------|-------|
| Latency, Ln (sum of (tn/ti)*Ti) (ms)               | 22.26 |

Since latency, Ln (22.26ms) is lesser than the longest initiation task, tn (100ms), all tasks are executed within the interval of the longest task. 

# Critical Instant Analysis <a name="critical"></a>
A critical instant for a task is defined to be an instant at which a request for that task will have the largest response time. In other words, it considers the worst-case scenario where every task i is initiated at the same time. 

Critical analysis for rate monotonic scheduling:
Assumptions: 
1. Single CPU
2. Tasks have fixed execution times
3. Fixied initiation interval for each task, which is also its deadline
4. No dependencies, no switching overheads
5. Fixed task priority

We consider the latency of the lowest-priority task (highest initiation interval, τn) and compare that to τn. For the schedule to work, the latency has to be less than τn. 

-- How we adapted the code --

1. scanKeysTask
- Worst case occurs when all keys are pressed 
- implemented by initialising prevQuartetStates with zeros (keys are pressed) so that when the function runs and finds the keys are released (not 0), it generates a load of release messages. Disable the update of prevQuartetStates to make the same thing happen every time.

2. displayUpdateTask
- Worst case scenario is in receiver mode (rx_or_tx == 1) as that will have the most items printed on the display

3. decodeTask
- Only for 1 key at a time

4. CAN_TX_Task
- Worst case depends on scanKeysTask

5. generateCurrentStepArrayTask
- Worst case scenario is when 5 keys are pressed at once. 
- Set localRxTxMultipliedArray[i] = 1 to get the longest execution time 


# Total CPU utilisation <a name="CPU_utilisation"></a>
Utilisation refers to the portion of time that the CPU is busy. It refers to a computer's usage of processing resources, or the amount of work handled by a CPU. 

Based on the assumptions of rate-monotonic scheduling, if we were to then allocate the shortest initiation interval as the highest priority, we will obtain the most optimal CPU utilisation. 

# Safe Access and Synchronisation <a name="safety_features"></a>
Synchronisation is the technique to overcome the problem of concurrent access to shared data which can result in data inconsistency. Concurrent data access is dangerous as it may result in the interrupt or scheduler swapping threads. As a result, the code will not function as expected. 

In order to have safe access and synchronisation, our code utilises mutexes, semaphores, queues, atomic memory access which are designed to support concurrent systems

### Identification of shared data structures
#### Shared Variables
- knob2Rotation and knob3Rotation values are modified in scanKeysTask thread (by means of an update of the class object), and accessed in the sampleISR interrupt ond displayUpdateTask thread. These global variables are accessed and stored using atomic load and atomic store respectively. 
- globalRxTxCounter is used in the polyphony stage to track the total number of notes that are are being played, and are incremented and decremented in decodeTask thread. this is for future work where additional threads may likely have to access this important shared counter.
- as part of polyphony implementation, the currentStepSizeArr[] replaces the currentStepSize variable. these elements within the currentStepSizeArr[] are updated in the generateCurrentStepArrayTask thread and accessed in the sampleISR interrupt. The updating and loading is done through use of atomic store.

#### Shared Arrays
- keyArray is updated in the ScanKeysTask and used in the DisplayUpdateTask threads, and a mutex is used to protect this array. 
- Global arrays such as globalRxTxMultipliedArray are updated by decodeTask thread and accessed in generateCurrentStepArrayTask thread. Similarly, a mutex is used to protect it.

# Analysis of inter-task blocking dependencies <a name="intertask_blocking"></a>
insert text here 
