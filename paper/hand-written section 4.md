Often, builders and hobbyists only discover how controlbale their rocket is after they build and test it. This paper aims to characterize the effcts of parameters like thrust, nozzle moment arm, pitch inertia, and loop delay on controlability so design desicions can be better informed through the quntification of build tradeoffs before building. Section 4 provides this map of tradeoffs.
The population consists of 2400 rocket designs sampled across a latin hypercube sweeping our parameters of interest. For example, the designs' maximum TVC gimbal ranged from 2.0–15.0°, median 8.6°. I used ten physics factors in addition to 6 DOF physics detailed in Fig. 1.
The parameter map is shown in Fig. 3. The shown axes are pitch inertia and loop delay. Qualitatively, failures tend to cluster at low inertia. In this experiment, failure means a peak RMSE of more than 20 degrees.
The graph shows that a randomly chosen failing design has lower pitch inertia than a randomly chosen surviving design 93.7% of the time. Additionally, it shows that the median failing design sits at the 5.4th percentile of the population inertia distribution. The median Iyy of failures is 0.19 times the population median. The distribution of these failures is not a cutoff, but rather a more gradual trend. Interestingly, disturbance angular acceleration scales as 1/Iyy while control authority is capped by max gimbal, so no gain choice buys it back.
Here are the unregularized, standardized predictors, bootstrap 95% CIs:
- log(1/Iyy) **+2.37** [1.94, 3.14]
- log(T·L) **+1.77** [1.30, 2.45]
- log(τ) **+2.90** [2.24, 3.92]
- all three positive, all CIs exclude zero
- keff as a legitimate *grouping*: raw-log ratio T·L/(1/Iyy) = **1.18** where keff would require 1.00
- dropping T·L **costs** accuracy: CV AUC **0.9829 → 0.9687**
In prior research, I thought the significant value was inertia, not authority because L2 regularization penalised a manually-added intercept column. However, refitting unregularized reversed it.