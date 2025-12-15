# 12-14-EOD-

fetch->decode->rename->dispatch is working (according to chat's interpretation of the testbench). made some changes to improve/fix writeback, commit, and recovery for most/all of the modules. Have not integrated execute, writeback, or commit yet. I think the next steps are asking chat to verify the performance of execute, writeback, and commit in their own separate tb's before integrating into the processor. 

I've included the entire project and processor + tb. make sure to set the hierarchies correctly (see email) and ensure the fetch module uses the icache_sim.sv in simulation. Also, make sure to add the attached processor_instructions.mem file inside your project folder->.sim->sim_1->behav->xsim. This will be what your fetch reads from in sim. 
