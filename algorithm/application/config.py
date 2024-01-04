## Configuration file for a specific main script/system, including the desired setup/parameters into the main script.
# @author  Andre Maia Pereira
# @date    2021-01-27

## IMPORT MODULES
from __future__ import absolute_import
from __future__ import print_function

# Import LLR class modules
import logging
import os
import sys

## SCRIPT FUNCTIONS

class Base:
    """Configuration for the Local Level Routing System"""

    def __init__(self,
                 optParser):

        # Get options
        self.addOptions(optParser)

        self.args = []
        raw_options = []
        nextIsValue2Add = False
        for arg in sys.argv[1:]:
            if nextIsValue2Add is True:
                self.args.append(arg)
                nextIsValue2Add = False
            elif "=" in arg:
                arg_key, arg_val = arg.split("=")
                if not optParser.has_option(arg_key):
                    self.args.append(arg_key)
                    self.args.append(arg_val)
                else:
                    raw_options.append(arg_key)
                    raw_options.append(arg_val)
            else:
                if "--" in arg or "-" in arg:
                    if not optParser.has_option(arg):
                        self.args.append(arg)
                        nextIsValue2Add = True
                    else:
                        raw_options.append(arg)
                else:
                    raw_options.append(arg)
        
        if len(self.args) > 0:
            print("Arguments", self.args, "not in the list of options, it will pass to SUMO. Make sure SUMO use them.")


        self.options, _ = optParser.parse_args(raw_options)

        # Scenario Name
        self.CONFIG_NAME = self.options.config_name
        self.SECONDARY_CONFIG_NAME = self.CONFIG_NAME.split(".")[1]

        # Object name
        self.name = self.CONFIG_NAME

        # Definition of standard values
        self.STEP_LENGTH = round(float(self.options.step_length), 1)
        self.BEGIN_T = round(float(self.options.begin), 1)
        self.END_T = round(float(self.options.end), 1)
        self.WARM_UP_T = round(float(self.options.warm_up_t), 1)
        self.NET_ATTRIB_FILES = self.options.path_net_files

        # Simulation interface Name
        self.SIMUL_INTERFACE = self.options.simul_interf
        self.RUN_MODE = self.options.run_mode

        # Logging
        try:
            logging.basicConfig(
                filename=self.options.path_outputs.replace("Necessary Outputs/", "Unnecessary Outputs/") + 'PYTHONlog.' +
                         self.CONFIG_NAME + ".log", level=logging.INFO, format='%(asctime)s %(message)s')
        except FileNotFoundError:
            logging.basicConfig(
                filename=os.path.abspath("../" + self.options.path_outputs.replace("./Necessary Outputs/",
                                                                                   "Unnecessary Outputs/"))
                         + 'PYTHONlog.' +
                         self.CONFIG_NAME + ".log", level=logging.INFO, format='%(asctime)s %(message)s')

        self.logger = logging.getLogger(__name__)
        self.logger.info('Start Simulation')

    def defSUMOparams(self,
                      sumoBinary,
                      additional_files,
                      route_files):
        """Define the stream of parameters to input into SUMO"""

        SUMOparams = [sumoBinary, "-c", self.options.simul_filename,
                      "--net", self.options.net_filename,
                      "--route-files", route_files,
                      "--additional-files", additional_files] \
                     + (lambda bool_NOoutputs: ["--error-log",
                                                self.options.path_outputs.replace("Necessary Outputs",
                                                                                  "Unnecessary Outputs") + "SUMOlog."
                                                + self.CONFIG_NAME + ".txt",
                                                # "--vehroute-output", self.options.path_outputs + "vehroute."
                                                # + self.CONFIG_NAME + ".xml",
                                                # "--tripinfo-output", self.options.path_outputs + "tripinfo."
                                                # + self.CONFIG_NAME + ".xml",
                                                # "--queue-output", self.options.path_outputs + "queue."
                                                # + self.CONFIG_NAME + ".xml",
                                                # "--queue-output.period", self.options.interval_measurements
                                                ]
        if not bool_NOoutputs else [])(self.options.no_outputs) \
                     + ["--begin", self.options.begin,
                        "--end", self.options.end,
                        "--step-length", self.options.step_length] \
                     + (lambda reac_t: ["--default.action-step-length", reac_t]
        if reac_t != self.options.step_length else [])(str(self.options.std_reaction_t))

        return SUMOparams

    def addOptions(self,
                   optParser):
        """Add more optParser options into the main script optParser"""

        # Configuration setup name
        optParser.add_option("--config-name",
                             default="",
                             action="store", type="string",
                             help="Name of the configuration/scenario setup")

        # Simulation interface (if a simulation)
        optParser.add_option("--simul-interf", action="store", type="string", default="traci",
                             help="Define simulation interface to choose, if SUMO either <traci> or <libsumo>. "
                                  "In case it is not a simulation use <none>")
        optParser.add_option("--run-mode", action="store", type="string", default="testing",
                             help="Define the run mode (testing activate asserts), either release or testing."
                                  "In case it is not a simulation use <none>")

        # Print what is happening
        optParser.add_option("--verbose", action="store_true", default=False,
                             help="Print out what is happening in the code")

        # Inputs for Generating Network Attribute Plots
        optParser.add_option("--sel-edges",
                             default="",
                             action="store", type="string",
                             help="network id of edges to be marked in the plot. Use comma as separator")
        optParser.add_option("--sel-lanes",
                             default="",
                             action="store", type="string",
                             help="network id of lanes to be marked in the plot. Use comma as separator")
        optParser.add_option("--sel-jcts",
                             default="",
                             action="store", type="string",
                             help="network id of junctions to be marked in the plot. Use comma as separator")
        optParser.add_option("--excluding-types", action="store", default="ship,rail_fast,rail_electric,"
                                                                          "rail,rail_urban,tram,pedestrian,bicycle",
                             help="Define type of vehicles that the lanes which only allow these vehicles will not be "
                                  "considered. Use comma as separator")
        optParser.add_option("--net-xlim",
                             default="",
                             action="store", type="string",
                             help="Coordinates of the x axis limits containing the modelled area in the network, "
                                  "use comma for min-max values")
        optParser.add_option("--net-ylim",
                             default="",
                             action="store", type="string",
                             help="Coordinates of the y axis limits containing the modelled area in the network, "
                                  "use comma for min-max values")
        optParser.add_option("--net-xlim-zoomed",
                             default="",
                             action="store", type="string",
                             help="Coordinates of the x axis  limits containing zoomed areas in the network, "
                                  "use comma for min-max values and semi colon for each zoomed area")
        optParser.add_option("--net-ylim-zoomed",
                             default="",
                             action="store", type="string",
                             help="Coordinates of the y axis limits containing zoomed areas in the network, "
                                  "use comma for min-max values and semi colon for each zoomed area")

        # Specific Input Filenames
        optParser.add_option("--tlights-filename",
                             default="",
                             action="store", type="string",
                             help="SUMO signalized junctions traffic light plans file."
                                  "If not given, it uses data from --net_filename")
        optParser.add_option("--turndefs-filename",
                             default="",
                             action="store", type="string",
                             help="SUMO edge turn definitions file")
        optParser.add_option("--flows-filename",
                             default="",
                             action="store", type="string",
                             help="SUMO edge flows file")
        optParser.add_option("--trips-filename",
                             default="",
                             action="store", type="string",
                             help="SUMO route and trips file")
        optParser.add_option("--detectors-filename",
                             default="",
                             action="store", type="string",
                             help="SUMO detectors file")
        optParser.add_option("--detectors-data-filename",
                             default="",
                             action="store", type="string",
                             help="SUMO detectors data file")
        optParser.add_option("--vtypes-filename",
                             default="",
                             action="store", type="string",
                             help="SUMO vehicle types file. If not given, it uses data from --flows_filename")

        ## SUMO INPUTS

        # SUMO Generate Output Files Option
        optParser.add_option("--no-outputs",
                             default=False,
                             action="store_true",
                             help="call to not generate any SUMO output")
        
        # SUMO GUI Option
        optParser.add_option("--nogui", action="store_true", default=False, help="run the commandline version of sumo")
        
        
        # SUMO Simulation Paths Inputs and Outputs
        optParser.add_option("--simul-filename",
                             default="",
                             action="store", type="string",
                             help="SUMO simulation file")
        optParser.add_option("--net-filename",
                             default="",
                             action="store", type="string",
                             help="SUMO network file")
        optParser.add_option("--fixed-route-files",
                             default="",
                             action="store", type="string",
                             help="SUMO fixed (same for all scenarios) vehicle route files, use comma as separator")
        optParser.add_option("--dyn-route-files",
                             default="",
                             action="store", type="string",
                             help="SUMO dynamic (appending sce. name) vehicle route files, use comma as separator")
        optParser.add_option("--fixed-additional-files",
                             default="",
                             action="store", type="string",
                             help="SUMO fixed (same for all scenarios) additional files, use comma as separator")
        optParser.add_option("--dyn-additional-files",
                             default="",
                             action="store", type="string",
                             help="SUMO dynamic (appending sce. name) additional files, use comma as separator")
        optParser.add_option("--path-inputs",
                             default="./Simulation Inputs/",
                             action="store", type="string",
                             help="path of SUMO input files")
        optParser.add_option("--path-outputs",
                             default="./Simulation Outputs/Necessary Outputs/",
                             action="store", type="string",
                             help="path of SUMO output files")
        optParser.add_option("--path-net-files",
                             default="./Scenario Generated Files/",
                             action="store", type="string",
                             help="path of SUMO output files")
        optParser.add_option("--path-det-data",
                             default="./Simulation Inputs/Detectors Data/",
                             action="store", type="string",
                             help="path of SUMO lane detectors' and outputs files")

        # SUMO Simulation Time Interval
        optParser.add_option("--begin", default="", action="store", type="string",
                             help="begin simulation time step or script starting time")
        optParser.add_option("--end", default="", action="store", type="string",
                             help="End step (if simulation)")
        optParser.add_option("--step-length", default="1", action="store", type="string",
                             help="time step length in seconds")
        optParser.add_option("--warm-up-t", default=1800, action="store", type="int",
                             help="warm up time for the travel time accuracy report (if simulation)")
        optParser.add_option("--interval-measurements", default=0, action="store", type="float",
                             help="Interval of measurement for code statistics. Use zero for the whole simulation")

        # SUMO Parameters
        optParser.add_option("--std-reaction-t", default=1, action="store", type="float",
                             help="standard reaction time (it can be different per vehicle type).")