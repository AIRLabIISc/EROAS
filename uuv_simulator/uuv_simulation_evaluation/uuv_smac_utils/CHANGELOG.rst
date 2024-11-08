^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uuv_smac_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2018-08-01)
------------------

0.4.3 (2018-07-31)
------------------
* FIX Missing KPIS error
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Rerun criteria using calculated cost standard deviation
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Calculation of max. time for motion primitive configuration
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.4.2 (2018-07-30)
------------------
* CHANGE Adapt code for Python 2 and 3 compatibility
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Type error while reading cost dict output
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Storing constraint information to file
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Return task file name instead of index for failed tasks vector
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Test of the standard deviation amongst all scenarios to determine if a simulation has to be repeated
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.4.1 (2018-07-27)
------------------
* CHANGE Make input parameter map optional in opt. configuration class
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.4.0 (2018-07-25)
------------------
* ADD Option to add cost function and constraints from file
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD All task parameters to the output report generated after SMAC run
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.3.4 (2018-07-23)
------------------
* RM Two MPS curve based motions
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.3.3 (2018-07-23)
------------------
* RM UUV_TERMINATE file before starting an evaluation of SMAC results
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.3.2 (2018-07-23)
------------------
* SET Max. number of processes for SMAC evaluation to 1
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Failed path variable
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.3.1 (2018-07-19)
------------------
* FIX Log message for cases where a task fails
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.3.0 (2018-07-17)
------------------
* ADD sync_smac_files script to CMakeLists
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Script to synchronize SMAC output files in output directory
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* SET Matplotlib backend as first command
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Force change matplotlib backend
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Set PDF backend when DISPLAY is not set
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Testing for display for matplotlib backend setting
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.2.8 (2018-07-16)
------------------
* ADD Test for DISPLAY variable for Docker runs
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.2.7 (2018-07-16)
------------------
* ADD Test for DISPLAY variable for use in Docker containers
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.2.6 (2018-07-16)
------------------
* ADD Variable for filename
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Flag to turn off plots
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* RM Old legacy code
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Best results folder to results folder
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.2.5 (2018-07-11)
------------------
* ADD Generation of script to remove SMAC configuration files
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.2.4 (2018-07-11)
------------------
* FIX Missing function for stopping all ROS related processes
  Signed-off-by: Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.2.3 (2018-07-11)
------------------
* ADD Send SIGTERM to the simulation process pool
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Signal handler for SIGINT and SIGTERM
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhães

0.2.2 (2018-07-09)
------------------
* FIX Add configured initial position in task configuration
Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Option of starting motion before all motion primitives
Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.2.1 (2018-07-07)
------------------

0.2.0 (2018-07-06)
------------------
* UPDATE SMAC wrapper for multi-task scenarios
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* UPDATE Plots for multi-task structure
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Utility functions for running simulation pools
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Simulation pool for multi-task scenarios
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Option file parser for SMAC scenarios
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes

0.1.5 (2018-06-06)
------------------
* ADD Circle motion primitive
  Signed-off-by: Musa Morena Marcusso Manhaes (CR/AEI) <musa.marcusso@de.bosch.com>
* FIX Evaluate script for multiple simulations
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Use simulation pool in SMAC utils package
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Use data parsers to access recorded data
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD SMAC utils package (simulation pool and input parser)
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Time offset to SMAC results evaluation script
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Source of error vector tags
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Typo in log message
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Set stdout for SMAC process to default
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Package versions
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Templates for scripts to run the optimization
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Create a smac_results file if it doesn't exist
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* CHANGE Package version
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* MAINT Change package version
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX Verbose mode tag
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* FIX The path to the SMAC run subfolder
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD Option to find the subfolder in the SMAC folder
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD New option to restore state from previous run
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* ADD SMAC utils package
  Signed-off-by: Musa Morena Marcusso Manhaes <Musa.Marcusso@de.bosch.com>
* Contributors: Musa Morena Marcusso Manhaes
