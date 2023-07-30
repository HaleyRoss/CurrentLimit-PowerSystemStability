using PowerSimulationsDynamics
using PowerSystems
using PowerFlows
using Logging
using Sundials
using PlotlyJS
using DataFrames
using OrdinaryDiffEq
PSY = PowerSystems # Define shorthand names for frequently referenced packages
PSD = PowerSimulationsDynamics

####### Establish Functions ########

# Establish function to change the load from constant power to constant impedance - compute new parameters
function _compute_total_load_parameters(load::PSY.StandardLoad)
    # Constant Power Data
    constant_active_power = PSY.get_constant_active_power(load)
    constant_reactive_power = PSY.get_constant_reactive_power(load)
    max_constant_active_power = PSY.get_max_constant_active_power(load)
    max_constant_reactive_power = PSY.get_max_constant_reactive_power(load)
    # Constant Current Data
    current_active_power = PSY.get_current_active_power(load)
    current_reactive_power = PSY.get_current_reactive_power(load)
    max_current_active_power = PSY.get_max_current_active_power(load)
    max_current_reactive_power = PSY.get_max_current_reactive_power(load)
    # Constant Admittance Data
    impedance_active_power = PSY.get_impedance_active_power(load)
    impedance_reactive_power = PSY.get_impedance_reactive_power(load)
    max_impedance_active_power = PSY.get_max_impedance_active_power(load)
    max_impedance_reactive_power = PSY.get_max_impedance_reactive_power(load)
    # Total Load Calculations
    active_power = constant_active_power + current_active_power + impedance_active_power
    reactive_power =
        constant_reactive_power + current_reactive_power + impedance_reactive_power
    max_active_power =
        max_constant_active_power + max_current_active_power + max_impedance_active_power
    max_reactive_power =
        max_constant_reactive_power +
        max_current_reactive_power +
        max_impedance_reactive_power
    return active_power, reactive_power, max_active_power, max_reactive_power
end
# Establish function to change the load from constant power to constant impedance - apply new parameters
function transform_load_to_constant_impedance(load::PSY.StandardLoad)
    # Total Load Calculations
    active_power, reactive_power, max_active_power, max_reactive_power =
        _compute_total_load_parameters(load)
    # Set Impedance Power
    PSY.set_impedance_active_power!(load, active_power)
    PSY.set_impedance_reactive_power!(load, reactive_power)
    PSY.set_max_impedance_active_power!(load, 2+max_active_power)
    PSY.set_max_impedance_reactive_power!(load, 2+max_reactive_power)
    # Set everything else to zero
    PSY.set_constant_active_power!(load, 0.0)
    PSY.set_constant_reactive_power!(load, 0.0)
    PSY.set_max_constant_active_power!(load, 0.0)
    PSY.set_max_constant_reactive_power!(load, 0.0)
    PSY.set_current_active_power!(load, 0.0)
    PSY.set_current_reactive_power!(load, 0.0)
    PSY.set_max_current_active_power!(load, 0.0)
    PSY.set_max_current_reactive_power!(load, 0.0)
    return
end

# Establish function to control load set points - change P and Q here if you want to change constant impedance load values pre-disturbance, but you'll have to rebalance the powerflow by changing the generator setpoints
function set_load_real_and_reactive_power(load::PSY.StandardLoad)
    PSY.set_impedance_active_power!(load,0.42) # Set the load active power 
    PSY.set_impedance_reactive_power!(load,0.05) # Set the load reactive power
    return
end

####### Setup Simulation ########

# Define filepath to saved system
file_dir = "Yourpathnamehere/14BusNew" 

# Define the system
sys = System(joinpath(file_dir, "14bus.raw"), joinpath(file_dir, "dyn_data.dyr"))

# Note GFM = grid forming inverter, GFL = grid following inverter
# Define case to be tested: choose from: 1 = GFM Only, 2= GFL Only, 3 = GFM Bus 01 GFL Move, or 4 = GFM Move GFL Bus 03
testcase = 4

# Define type of current limiting to be used # 1.0 instantaeous limiter, 2.0 magnitude limiter, 0.0 none
# For GFM
GFM_limtype = 1.0
# For GFL
GFL_limtype = 1.0

# Define list of generators to test. Uncomment the gens you want to use so that only one list is active at a time. 
#gens = ["generator-1-1", "generator-2-1", "generator-3-1", "generator-6-1", "generator-8-1"] # All gens included
#gens = ["generator-2-1", "generator-3-1", "generator-6-1", "generator-8-1"] # Gen list for moving GFL (note GFL can't be on Bus 01 becuase it's slack and sets system frequency)
#gens = ["generator-1-1", "generator-2-1", "generator-6-1", "generator-8-1"] # Gen list for moving GFM when GFL on bus 3
gens = ["generator-6-1"] # Only test 1 gen (note if you use this, you might get an error on line 661, this is normal, and all tineseries plots should be displayed)

# Define current limits to test, only one currentlim_values should be uncommented at a time.
iter=-.01 # Set decrement amount 
#currentlim_values = 1.4:iter:0.0 # Test range of limits decreasin by iter each time
currentlim_values= [1.35] # Test a single limit value (iter not used)

# Define loads to test, only one loads should be uncommented at a time.
loads = ["load21"] # Test a single load (note if you use this, you might get an error on line 661, this is normal, and all tineseries plots should be displayed)
#loads = ["load21", "load31", "load41","load51", "load61", "load91","load101", "load111", "load121", "load131","load141"] # Test all loads

####### Run Simulation ########

converge_fail = [] # Establish empty set to hold breakpoint results
fail_eig = [] # Establish empty set to hold eigenvalue results after breakpoint
for gen in gens # Establish for loop to rotate generators
for step in 0.1:0.1:0.11 # You can use this to try different load increases, but this is setup to only test 0.1 for now.
for load in loads #Establish loop to rotate through which load has the disturbance applied
    for currentlim_value in currentlim_values # Establish loop for testing different current limits
        
        # Call the system 
        sys = System(joinpath(file_dir, "14bus.raw"), joinpath(file_dir, "dyn_data.dyr"))
       
        # Transform the loads of the system set control P and Q setpoints using functions defined above
        for l in get_components(PSY.StandardLoad, sys)
            transform_load_to_constant_impedance(l)
            set_load_real_and_reactive_power(l)
        end
        
        # Set generator (PV) bus voltages 
        for b in ["BUS 01", "BUS 02", "BUS 03", "BUS 06", "BUS 08"]
            set_magnitude!(get_component(Bus, sys, b),1.0)
        end
        
        # Set generator bases, ratings, and real power setpoints 
        for p in ["generator-2-1", "generator-3-1","generator-6-1","generator-8-1","generator-1-1"]
            set_base_power!(get_component(ThermalStandard, sys, p),100.0)
            set_base_power!(get_component(DynamicGenerator, sys, p),100.0)
            set_rating!(get_component(ThermalStandard, sys, p), 1.0)
            set_active_power!(get_component(ThermalStandard, sys, p), 0.9556)
            set_active_power_limits!(get_component(ThermalStandard, sys, p), (min=0,max=1.0))
        end

        # Transform, unify, and tune generator dynamic models 
        dyn_gen = get_component(DynamicGenerator, sys, "generator-3-1") # Choose reference generator for dynamic models
        for generator in ["generator-1-1", "generator-2-1", "generator-6-1", "generator-8-1", "generator-3-1"] # Apply changes to all generators
            replace_gov = SteamTurbineGov1(R=0.05,T1=0.2,valve_position_limits=(0.5,1.01),T2=1.25,T3=4.2,D_T=0.39,DB_h=0.01,DB_l=-0.01,T_rate=47.0) # Define new turbine governor model
            replace_pss =STAB1(KT=2.7,T=7.5,T1T3=0.4,T3=1.5,T2T4=0.005,T4=4.0,H_lim=0.15) # Define new power system stabalizer model
            new_name = generator # Grab the name of the generator to alter
            new_ω_ref = get_ω_ref(dyn_gen) # Grab dynamic model data from reference generator
            new_machine = get_machine(dyn_gen) 
            new_shaft = get_shaft(dyn_gen) 
            new_avr = get_avr(dyn_gen) 
            new_prime_mover = replace_gov
            new_pss = replace_pss
            new_base_power = get_base_power(dyn_gen)
            remove_component!(sys, get_component(DynamicGenerator, sys, generator)) # Remove dynamic generator to update
            dyn_inj = PSY.DynamicGenerator(name = new_name, ω_ref = new_ω_ref, machine = new_machine, 
            shaft = new_shaft, avr = new_avr, prime_mover = new_prime_mover, pss = new_pss, base_power = new_base_power) # Define the dynamic generator to replace the removed one
            add_component!(sys, dyn_inj, get_component(ThermalStandard, sys, generator)) # Add the dynamic generator back to the system with the old name (finishing tranforming and unifying)
            # Begin tuning by setting parameters below
            # Machine Parameters
            set_Td0_p!(PSY.get_base_machine(new_machine),9.0) 
            set_Td0_pp!(PSY.get_base_machine(new_machine),0.05) 
            set_Tq0_p!(PSY.get_base_machine(new_machine),0.5) 
            set_Tq0_pp!(PSY.get_base_machine(new_machine),0.031) 
            set_Xd!(PSY.get_base_machine(new_machine),1.51) 
            set_Xq!(PSY.get_base_machine(new_machine),1.18) 
            set_Xd_p!(PSY.get_base_machine(new_machine),0.29) 
            set_Xq_p!(PSY.get_base_machine(new_machine),0.6) 
            set_Xd_pp!(PSY.get_base_machine(new_machine),0.138) 
            set_Xl!(PSY.get_base_machine(new_machine),0.2) 
            set_R!(PSY.get_base_machine(new_machine),0.0015) 
            set_Se!(PSY.get_base_machine(new_machine),(0.01,1.0))
            # Shaft Parameters
            set_H!(new_shaft, 5.0) 
            set_D!(new_shaft, 0.98) 
            #AVR Parameters
            set_Tr!(new_avr,0.001) 
            set_Tb!(new_avr,0.01) 
            set_Tc!(new_avr,0.01) 
            set_Ka!(new_avr,400.0) 
            set_Ta!(new_avr,0.02) 
            set_Va_lim!(new_avr, (-15.0,15.0)) 
            set_Te!(new_avr,0.8) 
            set_Kf!(new_avr,0.03) 
            set_Tf!(new_avr,1.0) 
            set_Kc!(new_avr,0.2) 
            set_Kd!(new_avr,0.38) 
            set_Ke!(new_avr,1.0) 
            set_E_sat!(new_avr, (2.14,3.18))
            set_Se!(new_avr, (0.03,0.1))
            set_Vr_lim!(new_avr, (-6.6,7.3)) 
            set_V_ref!(new_avr, 1.0)
        end

        if testcase == 1 || testcase == 3 || testcase == 4
            if testcase == 1 || testcase == 4 # Use this block to rotate GFM inverter though desired busses
                # Get existing parameters from thermal generator
                thermal_gen = get_component(ThermalStandard, sys, gen)
                therm_active_power = get_active_power(thermal_gen)
                therm_reactive_power = get_reactive_power(thermal_gen)
                therm_rating = get_rating(thermal_gen)
                therm_base = get_base_power(thermal_gen)
                GFMname = "GFM $(gen)"
            elseif testcase == 3 # Use this block to fix GFM at bus 1
                thermal_gen = get_component(ThermalStandard, sys, "generator-1-1")
                therm_active_power = get_active_power(thermal_gen)
                therm_reactive_power = get_reactive_power(thermal_gen)
                therm_rating = get_rating(thermal_gen)
                therm_base = get_base_power(thermal_gen)
                GFMname = "GFM generator-1-1"
            end

            # Revove thermal generator of interest in prep to add GFM inverter
            remove_component!(sys, get_dynamic_injector(thermal_gen))
            remove_component!(sys, thermal_gen)

            # Define the GFM renewabe generators fixed source
            PV_Gen = RenewableFix(
                name = GFMname,
                bus =  get_bus(thermal_gen), 
                available = true,
                prime_mover = PrimeMovers.PVe,
                active_power = therm_active_power,
                reactive_power = therm_reactive_power,
                rating = therm_rating,
                power_factor = .95,
                base_power = therm_base,
            )
            add_component!(sys, PV_Gen)

            # Define the GFM renewable generator's inverter
            inverter = DynamicInverter(
                name = get_name(PV_Gen),
                ω_ref = 1.0, # ω_ref,
                converter = AverageConverter(rated_voltage = 69.0, rated_current = 100.0),
                outer_control = OuterControl(
                    VirtualInertia(Ta = 2.0, kd = 400.0, kω = 20.0), #Ta=VSM Inertia Constant, kd=VSM damping constant, kω=frequency droop gain 
                    ReactivePowerDroop(kq = 0.2, ωf = 1000.0), #kq=frequency droop gain, ωf=filter frequency cutoff0
                ),
                inner_control = VoltageModeControl(
                    kpv = 0.59,     #Voltage controller proportional gain
                    kiv = 736.0,    #Voltage controller integral gain
                    kffv = 1.0,     #Binary variable enabling the voltage feed-forward in output of current controllers (turning on affects state of PI controller)
                    rv = 0.0,      #Virtual resistance in pu
                    lv = 0.2,       #Virtual inductance in pu
                    kpc = 1.27,     #Current controller proportional gain
                    kic = 14.3,     #Current controller integral gain
                    kffi = 0.0,     #Binary variable enabling the current feed-forward in output of current controllers (turing on fails to converge)
                    ωad = 50.0,     #Active damping low pass filter cut-off frequency
                    kad = 0.2,      # Active damping gain
                    I_lim_type = GFM_limtype, #1.0 instantaeous limiter, 2.0 magnitude limiter, 0.0 none
                    I_max = currentlim_value, # Current limit value
                ),
                dc_source = FixedDCSource(voltage = 600.0),
                freq_estimator = KauraPLL(
                    ω_lp = 500.0,    #Cut-off frequency for LowPass filter of PLL filter.
                    kp_pll = 0.084,  #PLL proportional gain
                    ki_pll = 4.69,   #PLL integral gain
                ),
                filter = LCLFilter(lf = 0.08, rf = 0.003, cf = 0.074, lg = 0.2, rg = 0.01),
            )
            add_component!(sys, inverter, PV_Gen)
        end

    if testcase == 2 || testcase == 3 || testcase == 4
        if testcase ==  4  # Use this block if GFL will be fixed at Bus 03
            # Revove thermal generator of interest in prep to add  GFL inverter
            replace_gen = get_component(ThermalStandard, sys, "generator-3-1")
            replace_active_power = get_active_power(replace_gen)
            replace_reactive_power = get_reactive_power(replace_gen)
            replace_rating = get_rating(replace_gen)
            replace_base = get_base_power(replace_gen)
            remove_component!(sys, get_dynamic_injector(replace_gen))
            remove_component!(sys, replace_gen) 
            GFLname = "GFL generator-3-1"
        elseif testcase == 2 || testcase == 3 # Use this block if GFL is moving (note GFL can't be on the souce bus, Bus 01)
            replace_gen = get_component(ThermalStandard, sys, gen)
            replace_active_power = get_active_power(replace_gen)
            replace_reactive_power = get_reactive_power(replace_gen)
            replace_rating = get_rating(replace_gen)
            replace_base = get_base_power(replace_gen)
            remove_component!(sys, get_dynamic_injector(replace_gen))
            remove_component!(sys, replace_gen)
            GFLname = "GFL $(gen)"
        end

        # Define the renewabe generators fixed source
        GFL_Gen = GenericBattery(name= GFLname,
                    available = true, 
                    bus = get_bus(replace_gen), 
                    prime_mover = PrimeMovers.BA, 
                    initial_energy = 1.0, 
                    state_of_charge_limits = (0.01, 1.0),
                    rating = replace_rating,
                    active_power = replace_active_power,
                    input_active_power_limits = (0.01, 1.0),
                    output_active_power_limits = (0.01,1.0),
                    efficiency = (in=1.0,out=1.0),
                    reactive_power = replace_reactive_power,
                    reactive_power_limits = (-1.0,1.0),
                    base_power = replace_base)


        add_component!(sys, GFL_Gen)

        # Define the renewable generator's inverter
        GFL_inverter = DynamicInverter(
            name = get_name(GFL_Gen),
            ω_ref = 1.0, # ω_ref,
            converter = AverageConverter(rated_voltage = 13.8, rated_current = 100.0),
            outer_control = OuterControl(
                ActivePowerPI( Kp_p = 2.83, Ki_p = 0.59, ωz = 0.153, P_ref = 1.0),
                ReactivePowerPI( Kp_q = 2.9, Ki_q = 1.1, ωf = 1.8, V_ref = 1.0, Q_ref = 1.0)),
            inner_control = CurrentModeControl(
                kpc = 0.736,     # Current controller proportional gain
                kic = 0.111,    # Current controller integral gain
                kffv = 1.0,     # Binary variable enabling the voltage feed-forward in output of current controllers
                I_lim_type = GFL_limtype, # 1.0 instantaeous limiter, 2.0 magnitude limiter, 0.0 none
                I_max = currentlim_value, # Current limit value
            ),
            dc_source = FixedDCSource(voltage = 600.0),
            freq_estimator = KauraPLL(
                ω_lp = 1000.0, #Cut-off frequency for LowPass filter of PLL filter.
                kp_pll = 1.6,  #PLL proportional gain
                ki_pll = 8.2,   #PLL integral gain
            ),
            filter = LCLFilter(lf = 0.08, rf = 0.006, cf = 0.0074, lg = 0.09, rg = 0.01),
        )
        add_component!(sys, GFL_inverter, GFL_Gen)
    end

        df2 = run_powerflow(sys) # Run the powerflow
        bus_res2 = df2["bus_results"] # Define powerflow results
        display(bus_res2) # Print powerlfow results (can be commented out)
       
        # Define the simulation
        sim = Simulation(
            ResidualModel, # Type of model used
            sys,         # Call the system
            file_dir,       # Set path for the simulation output
            (0.0, 40.0), # Set time span to simulate (seconds)
            LoadChange(5.0, get_component(StandardLoad, sys, load), :P_ref_power, step); # Define the pertubation (increase in real power on a load)
            console_level = Logging.Info,
            all_lines_dynamic = false, # Toggle transmission line dynamics (false is default)
        )

        # Run Small Signal Analysis
        sm = small_signal_analysis(sim)

        # Show Participation Factor Summary
        pf_summary = summary_participation_factors(sm)
        show(pf_summary, allrows=true)

        # Show Eigenvalue Summary
        eig_summary = summary_eigenvalues(sm)
        show(eig_summary, allrows = true)
        show(last(eig_summary, 6))
        
        # Plot static network eigenvalue analysis results
        p = plot(scatter(
            x = real.(sm.eigenvalues), 
            y = imag.(sm.eigenvalues), 
            name = "Static Network",  mode="markers"),
            Layout(
            xaxis_title = "Real Axis",
            yaxis_title = "Imaginary Axis",
            title = "Static Network, Current Lim: $(currentlim_value), Load: $(load), Step: $(step)",
            ))
        display(p)

        # Execute the simulation
        execute!(sim, IDA(); abstol = 1e-8) 

        # Define the results
        results = read_results(sim)

        # Catch build or converge errors
        try
            if testcase == 1 || testcase == 4 # Use this if GFM if moving
                get_real_current_series(results, "GFM $(gen)") 
            elseif testcase == 2 || testcase == 3 # Use this if GFL is moving
                get_real_current_series(results, "GFL $(gen)") 
            end
        catch err
            if isa(err, MethodError)
                push!(converge_fail,(gen, load, step, currentlim_value, "build/converge_error"))
                break
            else
                continue
            end
        end

        # Catch unstable initial conditions
        if testcase == 1 || testcase == 4 # Use this if GFM if moving
            sim_time, Ir_cnv =  get_state_series(results, ("GFM $(gen)", :ir_cnv))
            sim_time, Ii_cnv =  get_state_series(results, ("GFM $(gen)", :ii_cnv))
        elseif testcase == 2 || testcase == 3 # Use this if GFL is moving
            sim_time, Ir_cnv =  get_state_series(results, ("GFL $(gen)", :ir_cnv))
            sim_time, Ii_cnv =  get_state_series(results, ("GFL $(gen)", :ii_cnv))
        end
        if all(isapprox.(Ir_cnv[1:5], Ir_cnv[6:10], rtol= 1e-3)) == false # Can adjust or loosen tolerance as needed
            push!(converge_fail,(gen, load, step, currentlim_value, "no_steady_state_real"))
            break
        end
        if all(isapprox.(Ii_cnv[1:5], Ii_cnv[6:10], rtol= 1e-3)) == false
            push!(converge_fail,(gen, load, step, currentlim_value, "no_steady_state_imag"))
            break
        end

        # Gather results for real and reactive current
        if testcase == 1 || testcase == 4 # Use this if GFM if moving
            sim_time, cu =get_real_current_series(results, "GFM $(gen)") 
        elseif testcase == 2 || testcase == 3 # Use this if GFL is moving   
            sim_time, cu =get_real_current_series(results, "GFL $(gen)")
        end
        real_current = typeof(sim_time)[] # Setup holding matrix for real current traces
        reactive_current = typeof(sim_time)[] # Setup holding matrix for reactive current traces
        gen_name0 = [] # Setup holding matrix for generator names
        for m in get_name.(get_components(DynamicInverter, sys)) # Cycle through list of generators
            sim_time, i_real = get_real_current_series(results, m) # Get real current results
            sim_time, i_react = get_imaginary_current_series(results, m) # Get reactive current results
            real_current = push!(real_current, i_real) # Push real results to holding matrix
            reactive_current = push!(reactive_current, i_react) # Push reactive results to holding matrix
            gen_name0 = push!(gen_name0, m) # Push names to holding matrix
        end
        # Setup traces for plotting real current
        real_current= reduce(hcat, real_current) # Change shape of holding martix
        nr_i_real = size(real_current, 2) # Get size of matrix to iterate through (# generators)
        real_current_trc = Vector{GenericTrace}(undef, nr_i_real) # Setup trace matrix
        for n in 1:nr_i_real
            real_current_trc[n]=scatter(x=sim_time, y=real_current[:,n], mode="lines", name=gen_name0[n]) # Build trace for each generator
        end
        # Plot real current traces
        p1 = plot(real_current_trc) # Plot
        relayout!(p1, xaxis_title = "Time [sec]", yaxis_title = "Real Current [pu]") # Set new axis names
        # Setup traces for plotting reactive current
        reactive_current = reduce(hcat, reactive_current)
        nr_i_imag = size(reactive_current, 2)
        imag_current_trc = Vector{GenericTrace}(undef,nr_i_imag)
        for n in 1:nr_i_imag
            imag_current_trc[n]=scatter(x=sim_time,y=reactive_current[:,n], mode="lines", name=gen_name0[n])
        end
        #P lot imaginary current traces
        p2 = plot(imag_current_trc)
        relayout!(p2,xaxis_title = "Time [sec]", yaxis_title = "Reactive Current [pu]")
        # Plot tiles of real and reactive current on same chart
        p3=[p1;p2]
        relayout!(p3, showlegend=true, title="Current Out Inverters, I Lim: $(currentlim_value), Load: $(load), Step: $(step)") # Assign title 
        display(p3) 

        # Plot real vs reactive current (phase space plot)
        real_react_i = Vector{GenericTrace}(undef,nr_i_imag)
        for n in 1:nr_i_imag
            real_react_i[n]=scatter(x=real_current[:,n],y=reactive_current[:,n], mode="lines", name=gen_name0[n])
        end
        p14=plot(real_react_i)
        relayout!(p14, showlegend = true, xaxis_title = "Real Current [pu]", yaxis_title = "Reactive Current [pu]", title = "Real vs Reactive Current of IBR,  I Lim: $(currentlim_value), Load: $(load), Step: $(step)")
        display(p14)

        # Gather bus voltage results
        sim_time, v_b_1=get_voltage_magnitude_series(results, 1)
        bus_voltages = typeof(sim_time)[]
        bus_names = []
        for b in 1:14
            sim_time, v_b=get_voltage_magnitude_series(results, b)
            bus_name =  "BUS $(lpad(b,2,"0"))"
            bus_voltage = push!(bus_voltages, v_b)
            bus_names = push!(bus_names, bus_name)
        end
        # Setup Bus Voltage traces
        bus_voltages = reduce(hcat, bus_voltages)
        nr_b_trc = size(bus_voltages,2)
        bus_volt_trc = Vector{GenericTrace}(undef, nr_b_trc)
        for b in 1:nr_b_trc
            bus_volt_trc[b]=scatter(x=sim_time, y=bus_voltages[:,b], mode="lines", name=bus_names[b])
        end
        #Plot Bus Voltages
        p4=plot(bus_volt_trc)
        relayout!(p4, xaxis_title = "Time [sec]", yaxis_title = "Voltage Magnitude [pu]",
        title = "Bus Voltages, Current Lim: $(currentlim_value), Load: $(load), Step: $(step)")
        display(p4)

        # Gather generator speed results
        if testcase == 1 || testcase == 4 # Use this if GFM if moving
            sim_time, w = get_state_series(results, ("GFM $(gen)", :ω_oc)) # Use for GFM Moving
        elseif testcase == 3 # Use for GFM fixed on Bus 03 
            sim_time, w = get_state_series(results, ("GFM generator-1-1", :ω_oc)) 
        elseif testcase == 2 # Use for GFL (only), moving
            sim_time, w = get_state_series(results, ("generator-1-1", :ω)) 
        end
        gen_spd = typeof(sim_time)[]
        gen_name = []
        for n in get_name.(get_components(ThermalStandard, sys))
            sim_time, w_g = get_state_series(results, (n, :ω))
            gen_spd = push!(gen_spd, w_g)
            gen_name = push!(gen_name, n)
        end
        for m in get_name.(get_components(RenewableFix, sys))
            sim_time, w_g = get_state_series(results, (m, :ω_oc))
            gen_spd = push!(gen_spd, w_g)
            gen_name = push!(gen_name, m)
        end
        # Turn gen speed collection of vectors into a matrix
        gen_spd = reduce(hcat, gen_spd)
        # Get number of columns to iterate over from generator speed matrix
        nr_spd_trc = size(gen_spd,2)
        # Setup and empty set of generic traces to hold speed traces in
        spd_traces=Vector{GenericTrace}(undef, nr_spd_trc)
        for n in 1:nr_spd_trc
            spd_traces[n]=scatter(x=sim_time, y=gen_spd[:,n],mode="lines", name=gen_name[n])
        end
        # Plot Speed Traces
        p5=plot(spd_traces)
            relayout!(p5, xaxis_title = "Time [sec]", yaxis_title = "Speed [pu]",
            title = "Generator Speed, Current Lim: $(currentlim_value), Load: $(load), Step: $(step)")
            display(p5)

        # Get real and reactive power output results
        if testcase == 1 || testcase == 4 # Use this if GFM if moving
            sim_time, w = get_activepower_series(results,"GFM $(gen)")
        elseif testcase == 2 || testcase == 3 # Use this if GFL is moving
            sim_time, w = get_activepower_series(results,"GFL $(gen)") 
        end
         gen_pwr = typeof(sim_time)[]
         gen_repwr = typeof(sim_time)[]
         gen_name1 = []
         for n in get_name.(get_components(ThermalStandard, sys))
             sim_time, w_p = get_activepower_series(results, n)
             sim_time, w_p2 = get_reactivepower_series(results, n)
             gen_pwr = push!(gen_pwr, w_p)
             gen_repwr = push!(gen_repwr, w_p2)
             gen_name1 = push!(gen_name1, n)
         end
         for n in get_name.(get_components(DynamicInverter, sys))
             sim_time, w_p = get_activepower_series(results, n)
             sim_time, w_p2 = get_reactivepower_series(results, n)
             gen_pwr = push!(gen_pwr, w_p)
             gen_repwr = push!(gen_repwr, w_p2)
             gen_name1 = push!(gen_name1, n)
         end
         # Setup Real Power Traces
         gen_pwr = reduce(hcat, gen_pwr)
         nr_apwr_trc = size(gen_pwr,2)
         act_pwr_traces=Vector{GenericTrace}(undef, nr_apwr_trc)
         for n in 1:nr_apwr_trc
            act_pwr_traces[n]=scatter(x=sim_time, y=gen_pwr[:,n], mode="lines", name=gen_name1[n])
         end
         #Plot Real Power
         p6=plot(act_pwr_traces)
         relayout!(p6, xaxis_title = "Time [sec]", yaxis_title = "Real Power [pu]",)
        #Setup Reactive Power Traces
        gen_repwr = reduce(hcat, gen_repwr)
        nr_repwr_trc = size(gen_repwr,2)
        re_pwr_traces=Vector{GenericTrace}(undef,nr_repwr_trc)
        for n in 1:nr_repwr_trc
           re_pwr_traces[n]=scatter(x=sim_time, y=gen_repwr[:,n], mode="lines",name=gen_name1[n])
        end
        #Plot Reactive power
        p7=plot(re_pwr_traces)
        relayout!(p7, xaxis_title = "Time [sec]", yaxis_title = "Reactive Power [pu]",)
        # Plot tiled power plots
         p8=[p6;p7]
         relayout!(p8, showlegend=true, title="Power Output Gens, Current Lim: $(currentlim_value), Load: $(load), Step: $(step)")
         display(p8)

         # Gather and states of inverter PI controller
         if testcase == 1 || testcase == 4 # Use this if GFM if moving
            sim_time, st_pv_d=get_state_series(results, ("GFM $(gen)", :γd_ic)) 
         elseif testcase == 2 || testcase == 3 # Use this if GFL is moving
            sim_time, st_pv_d=get_state_series(results, ("GFL $(gen)", :γd_ic)) 
         end
         gen_state_d = typeof(sim_time)[]
         gen_state_q = typeof(sim_time)[]
         gen_name3 = []
         for m in get_name.(get_components(DynamicInverter, sys))
            sim_time, w_d = get_state_series(results, (m, :γd_ic))
            sim_time, w_q = get_state_series(results, (m, :γq_ic))
            gen_state_d = push!(gen_state_d, w_d)
            gen_state_q = push!(gen_state_q, w_q)
            gen_name3 = push!(gen_name3, m)
        end
        # Setup traces for d-axis states
        gen_state_d = reduce(hcat, gen_state_d)
        nr_d_trc = size(gen_state_d, 2)
        d_st_traces = Vector{GenericTrace}(undef,nr_d_trc)
        for n in 1:nr_d_trc
            d_st_traces[n]=scatter(x=sim_time,y=gen_state_d[:,n], mode="lines", name=gen_name3[n])
        end
        # Plot d-axis states
        p9 = plot(d_st_traces)
        relayout!(p9, xaxis_title = "Time [sec]", yaxis_title = "D - Axis Integrator State")
        # Setup Traces for Q-axis states
        gen_state_q = reduce(hcat, gen_state_q)
        nr_q_trc = size(gen_state_q, 2)
        q_st_traces = Vector{GenericTrace}(undef,nr_q_trc)
        for n in 1:nr_q_trc
            q_st_traces[n]=scatter(x=sim_time,y=gen_state_q[:,n], mode="lines", name=gen_name3[n])
        end
        # Plot Q-Axis states
        p10 = plot(q_st_traces)
        relayout!(p10,xaxis_title = "Time [sec]", yaxis_title = "Q - Axis Integrator State")
        # Plot tiles state series of inverters PI controllers
         p11=[p9;p10]
         relayout!(p11, title = "D & Q Axis St of PI Cntrlr, I Lim: $(currentlim_value), Load: $(load), Step: $(step)",)
             display(p11)  

         # Gather reactive voltage out of inverters
        if testcase == 1 || testcase == 4 # Use this if GFM if moving
            sim_time, st_pv_d=get_state_series(results, ("GFM $(gen)", :vq_pll))
        elseif testcase == 2 || testcase == 3 # Use this if GFL is moving
            sim_time, stv_pv_d=get_state_series(results, ("GFL $(gen)", :vq_pll))
        end
         volt_state_q = typeof(sim_time)[]
         gen_name4 = []
         for m in get_name.(get_components(DynamicInverter, sys))
            sim_time, v_q = get_state_series(results, (m, :vq_pll))
            gen_state_q = push!(volt_state_q, v_q)
            gen_name3 = push!(gen_name4, m)
        end
        # Setup traces for Q-axis votage
        volt_state_q = reduce(hcat, volt_state_q)
        nr_qv_trc = size(volt_state_q, 2)
        qv_st_traces = Vector{GenericTrace}(undef,nr_qv_trc)
        for n in 1:nr_q_trc
            qv_st_traces[n]=scatter(x=sim_time,y=volt_state_q[:,n], mode="lines", name=gen_name4[n])
        end
        # Plot Q-axis voltage
        p13 = plot(qv_st_traces)
        relayout!(p13, showlegend=true, xaxis_title = "Time [sec]", yaxis_title = "Reactive voltage [pu]", title = "Q Axis Voltage, I Lim: $(currentlim_value), Load: $(load), Step: $(step)",) 
             display(p13)  

        # Check for large signal stability based on current at the inverter filter
        if testcase == 1 || testcase == 4 # Use this if GFM if moving
            sim_time, Ir_filter =  get_state_series(results, ("GFM $(gen)", :ir_filter)) # Use these two rows if GFM is moving
            sim_time, Ii_filter =  get_state_series(results, ("GFM $(gen)", :ii_filter))
        elseif testcase == 2 || testcase == 3 # Use this if GFL is moving
            sim_time, Ir_filter =  get_state_series(results, ("GFL $(gen)", :ir_filter)) 
            sim_time, Ii_filter =  get_state_series(results, ("GFL $(gen)", :ii_filter))
        end
        length = size(Ir_filter)[1]
        display(length)
        if isapprox(maximum(Ir_filter[length-35:length]), minimum(Ir_filter[length-35:length]), rtol= 0.01) == false
            push!(converge_fail,(gen, load, step, currentlim_value, "unstable_real")) 
            push!(fail_eig,(gen, last(eig_summary,3)))
            break
        end
        if isapprox(maximum(Ii_filter[length-40:length]), minimum(Ii_filter[length-40:length]), rtol= 0.05) == false
            push!(converge_fail,(gen, load, step, currentlim_value, "unstable_imag"))
            push!(fail_eig,(gen, last(eig_summary,3)))
            break
        end 

        # Check for small signal stability
        if maximum(eig_summary[:,"Real Part"]) > 0
            push!(converge_fail,(gen, load, step, currentlim_value,"small_signal"))
            push!(fail_eig,(gen, last(eig_summary,3)))
            break
        end 
    end
end
end
end  
unstable_results=DataFrame(converge_fail) # Take results and convert to data frame for plotting
if testcase == 1 || testcase == 4 # Use this if GFM if moving
    rename!(unstable_results, [:GFM_Generator, :Load_Stepped, :Step_Change, :Current_Limit, :Report_Source]) 
    p12=plot(unstable_results, x=:Load_Stepped, y=:GFM_Generator, color=:Current_Limit, text=:Report_Source, marker=attr(size=10, sizemode="area"), mode="markers", Layout(xaxis_type = "category", xaxis_categoryarray = ["load21", "load31", "load41", "load51", "load61", "load91", "load101", "load111", "load121", "load131", "load141"], xaxis_categoryorder = "array", yaxis_type = "category", yaxis_categoryorder = "category descending"))
elseif testcase == 2 || testcase == 3 # Use this if GFL is moving
    rename!(unstable_results, [:GFL_Generator, :Load_Stepped, :Step_Change, :Current_Limit, :Report_Source]) 
    p12=plot(unstable_results, x=:Load_Stepped, y=:GFL_Generator, color=:Current_Limit, text=:Report_Source, marker=attr(size=10, sizemode="area"), mode="markers", Layout(xaxis_type = "category", xaxis_categoryarray = ["load21", "load31", "load41", "load51", "load61", "load91", "load101", "load111", "load121", "load131", "load141"], xaxis_categoryorder = "array", yaxis_type = "category", yaxis_categoryorder = "category descending"))
end
show(unstable_results, allrows=true)
display(p12)
show(fail_eig)