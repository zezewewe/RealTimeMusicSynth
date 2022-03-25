# Analysis
This documentation contains the analysis of the tasks' performance. For explanations on their implementations, please read [this file](README.md).

# Table of contents
1. [Time Analysis of each task](#time_analysis)
2. [Critical Instant Analysis](#critical)
4. [Total CPU utilisation](#CPU_utilisation)
5. [Safe Access and Synchronisation](#safety_features)
6. [Analysis of inter-task blocking dependencies](#intertask_blocking)





# Time Analysis of each task <a name="time_analysis"></a>
|             Task             | Initiation Interval, ti (ms) | Execution Time, Ti (us) (32 iterations) | Execution Time, Ti (us) (per iteration) | tn/ti | (tn/ti)*Ti (s) |                 Commit ID                |
|:----------------------------:|:----------------------------:|:---------------------------------------:|:---------------------------------------:|:-----:|:--------------:|:----------------------------------------:|
| scanKeysTask                 |                           20 |                                    3051 |                                   95.34 |     5 |    0.000476719 | 4e62d4fa369764adce4eb34d112c3d424d3daae7 |
| displayUpdateTask            |                          100 |                                  555572 |                                17361.63 |     1 |    0.017361625 | c2020273debcac73a500bc6ae7fe348c4ae6c57c |
| decodeTask                   |                         25.2 |                                    2852 |                                  89.125 |  3.97 |    0.000353671 | 8901783b784156c23fd8db63d74868925d00c72c |
| CAN_TX_Task                  |                           60 |                                      79 |                                    2.47 |  1.67 |    4.11458E-06 | 4852abb2121396965b59a2d9735347eb99a36166 |
| generateCurrentStepArrayTask |                           20 |                                   26000 |                                   812.5 |     5 |      0.0040625 | 9a9f857e22da221b61651f16c45e3e618d74ba2f |

# Critical Instant Analysis <a name="critical"></a>
insert text here 

# Total CPU utilisation <a name="CPU_utilisation"></a>
insert text here 

# Safe Access and Synchronisation <a name="safety_features"></a>
insert text here 

# Analysis of inter-task blocking dependencies <a name="intertask_blocking"></a>
insert text here 
