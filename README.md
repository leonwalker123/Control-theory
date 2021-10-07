# Control-theory
# some simulation code files for classic papers in control theory
# simulation code files for paper "Xu Jin, Wassim M.Haddad, Tansel Yucelen. An adaptive control architecture for mitigating sensor and actuator attacks in cyber-physical systems,2017,IEEE Transactions on Automatic Control"
# key words: CPS, false data injection attack, adaptive framework
# Instructions:
1. Put all ".m" files under the same matlab workspace. 
2. The "main.m" file consists of four cases(case1-case4), each time you run "main.m" you need to make sure that you enable only one case.
case1: nominal case, namely no attack + standard state feedback control, you will get stable system states and control inputs.
case2: sensor&actuator attacks + standard state feedback control, you will get divergent results.
case3: sensor&actuator attacks + corrective controller(with sgn function), you will get convergent system states, while there exists some shattering phenomena resulting from the discontinous term sgn in control function.
case4: sensor&actuator attacks + corrective controller(with tanh function), with properly chosen control parameters, you can get convergent system states and smooth control inputs.

For more information, please visit the academic paper noted above.
