##### Clarification Regarding Entropy
The video mentions that entropy will decrease after the measurement update (sense) step and that entropy will increase after movement step (move).

In general, entropy represents the amount of uncertainty in a system. Since the measurement update step decreases uncertainty, entropy will decrease. The movement step increases uncertainty, so entropy will increase after this step.

Let's look at our current example where the robot could be at one of five different positions. The maximum uncertainty occurs when all positions have equal probabilities [0.2,0.2,0.2,0.2,0.2]

Following the formula Entropy=Σ(−p×log(p)), we get −5×(.2)×log(0.2)=0.699.

Taking a measurement will decrease uncertainty and entropy. Let's say after taking a measurement, the probabilities become [0.05,0.05,0.05,0.8,0.05]. Now we have a more certain guess as to where the robot is located and our entropy has decreased to 0.338.
