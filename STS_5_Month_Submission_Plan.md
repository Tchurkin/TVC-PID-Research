# Five-Month STS Execution Plan

## Goal
Use the next 5 months to turn the project from a promising control experiment into a submission that makes one narrow, defensible claim: there exists a measurable regime boundary where delay-driven prediction error overtakes aerodynamic-driven prediction error in your TVC rocket, and that boundary can be identified with simulation plus flight data.

## What Has To Be True For Finalist-Level Work
- The question is narrow and original.
- The methods are clean enough that another person could repeat them.
- The data volume is high enough to support a real conclusion, not a one-off observation.
- The analysis includes uncertainty, not just pretty plots.
- The final paper shows one central figure that answers the research question immediately.

## Month 1: Lock The System And Build The Measurement Pipeline

### Week 1
- Freeze the exact rocket configuration you will use for the project.
- Record mass, CG, inertia estimate, TVC linkage geometry, nozzle offset, servo model, controller gains, and firmware version.
- Create a hardware log sheet and keep it unchanged for every later launch.

### Week 2
- Bench test your logging chain.
- Verify you can record, with synchronized timestamps:
- IMU attitude or raw gyro/accel
- actuator command
- actuator measured angle if available
- battery voltage
- any mode/state flags from the controller
- Run repeated bench disturbances and confirm timestamp drift is acceptably small.

### Week 3
- Measure actuator latency on the bench using commanded step inputs.
- Estimate sensor/filter lag by injecting known motion and comparing filtered output to raw or reference timing.
- Build a small calibration data folder with at least 10 repeated trials for each estimate.

### Week 4
- Replace placeholder constants in [PID_Rocket_Sweep.m](PID_Rocket_Sweep.m) with measured values.
- Confirm the code exports stable CSV outputs for your baseline case.
- Define your exact stability criteria now and do not change them later unless documented.

## Month 2: Make The Simulation Credible Before Flights

### Week 5
- Run parameter sweeps across actuator delay, sensor delay, aero damping, and thrust variation.
- Identify which combinations produce clearly stable, borderline, and unstable behavior.
- Use this to design safe but informative flight conditions.

### Week 6
- Add at least one controlled disturbance family to the simulation so regime changes are not based on a single initial condition.
- Suggested options:
- initial angular-rate perturbation
- small launch rail exit angle offset
- lateral wind impulse term

### Week 7
- Run repeated Monte Carlo style simulations at each regime bin.
- Save the fraction unstable, mean residual, and confidence intervals by regime.
- Decide which three regime centers you will actually target in flight.

### Week 8
- Build your first draft figures from simulation alone.
- You should have:
- one regime-comparison figure
- one matched delay-vs-aero contribution figure
- one transition-boundary figure
- If the story is still muddled here, tighten the question before launching.

## Month 3: Fly The First Half Of The Campaign

### Week 9
- Conduct 2 to 3 flights in the low-rate regime.
- Immediately review data quality the same day.
- Reject and repeat any flight with bad timestamps or incomplete actuator logging.

### Week 10
- Conduct 2 to 3 flights in the mid-rate regime.
- Keep all non-test variables as fixed as possible.
- Start a flight-by-flight comparison notebook or markdown log.

### Week 11
- Conduct 2 to 3 flights in the high-rate regime.
- Do not chase dramatic flights; prioritize clean data and repeatability.
- Mark every flight as usable, partially usable, or unusable.

### Week 12
- Pause launches and compare the first half of the campaign against simulation.
- Check whether the measured divergence pattern looks like your simulated boundary story.
- If not, revise the model now, before the second half of launches.

## Month 4: Finish Flights And Lock Analysis

### Week 13
- Finish the remaining launches so you end with about 15 usable flights.
- Make sure each regime has replicates, not just one successful run.

### Week 14
- Clean and align all flight logs.
- Produce one master table with all flight metadata and signal paths.
- Compute per-flight residual metrics using the exact same code path every time.

### Week 15
- Compare three models against each flight:
- simplified baseline model
- aero-only corrected model
- delay-only corrected model
- combined corrected model
- For each flight, determine which correction most reduced error.

### Week 16
- Estimate the transition boundary from actual flight-supported evidence.
- Use confidence intervals or bootstrap resampling.
- Decide on the one sentence claim you can honestly defend from the data.

## Month 5: Write For Submission, Not For Yourself

### Week 17
- Draft the paper structure:
- problem
- why current simulation assumptions fail
- experimental method
- regime boundary result
- engineering implication

### Week 18
- Build final figures and remove anything that does not directly support the main claim.
- Your best figure should show where delay minus aero contribution crosses regime significance.

### Week 19
- Write the abstract, then rewrite it after the figures are final.
- Cut every claim that is not directly supported by measured data.

### Week 20
- Give the paper to 2 to 3 technically strong readers.
- Ask them only three questions:
- What is the exact claim?
- What evidence supports it?
- What part still feels weak or unconvincing?

### Final 7-10 Days Before Submission
- Finalize the research paper, activity description, and any supplementary materials.
- Prepare short explanations for novelty, difficulty, and your specific contribution.
- Make sure your code, figures, and log tables are archived and reproducible.

## Minimum Experimental Standard You Should Aim For
- 15 usable launches, not 15 attempts.
- At least 4 to 5 usable launches per regime bin.
- Bench-derived actuator and sensor delay estimates.
- One frozen controller/gains configuration for the whole campaign.
- One analysis pipeline used identically for every flight.
- Confidence intervals on the main boundary claim.

## What Would Most Improve Your Odds
- Clean actuator position logging.
- A convincing matched comparison between aero-only and delay-only model improvements.
- A central figure that immediately shows when simplified simulation stops being predictive.
- Clear evidence that the result matters for real control design decisions, not just for this one rocket.

## What To Avoid
- Changing gains halfway through and treating the data as one study.
- Using only simulation to make the final claim.
- Making the paper about general rocket control instead of your exact boundary-identification question.
- Including too many weak figures instead of a few strong ones.