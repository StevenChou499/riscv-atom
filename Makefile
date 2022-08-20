#================================================================#
#      ____  _________ _______    __         __                  #
#     / __ \/  _/ ___// ____/ |  / /  ____ _/ /_____  ____ ___   #
#    / /_/ // / \__ \/ /    | | / /  / __ `/ __/ __ \/ __ `__ \  #
#   / _, _// / ___/ / /___  | |/ /  / /_/ / /_/ /_/ / / / / / /  #
#  /_/ |_/___//____/\____/  |___/   \__,_/\__/\____/_/ /_/ /_/   #
#                                                                #
#================================================================#
#
# README:
#
# 	This is a self documenting Makefile, see 'help' target for 
#	more info. Whenever adding a new taget document it a comment 
# 	that stats with a hash(#) symbol folllowed by a tilde(~) 
# 	symbol. Anything after # followed by ~ will be displayed 
#	whenever "make help: is called.
#================================================================#

# Bash color codes
COLOR_RED 		= \033[0;31m
COLOR_GREEN 	= \033[0;32m
COLOR_YELLOW 	= \033[0;33m
COLOR_NC 		= \033[0m

Target ?= atombones
#======================================================================
# Recepies
#======================================================================
default: sim elfdump scripts lib
	@echo "\n$(COLOR_GREEN)----------------------------------------------"
	@echo "Build Succesful!"
	@echo "----------------------------------------------$(COLOR_NC)"
	@echo " 1). atomsim [$(Target)]"
	@echo " 2). elfdump tool"
	@echo " 3). scripts"
	@echo " 4). software libraries"

all : default doxy-pdf
	@echo " 5). doxygen-docs in latex, html & pdf "


#======== Help ========
# See : https://swcarpentry.github.io/make-novice/08-self-doc/index.html

#~	help		:	Show help message

.PHONY : help
help : Makefile
	@echo "*** RISC-V Atom root Makefile ***"
	@echo "Usage:"
	@echo "	$$ make target=<atomsim/hydrogensoc> [TARGET]"
	@echo ""
	@echo "TARGETs:"
	@egrep -h '\s##\s' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\t\033[36m%-20s\033[0m %s\n", $$1, $$2}'
	@echo ""


# ======== AtomSim ========
.PHONY : sim
sim:                       			## Build atomsim for specified target [default: atombones]
	@echo "$(COLOR_GREEN)>> Building Atomsim...$(COLOR_NC)"
	make -C atomsim Target=$(Target) DEBUG=1


.PHONY: clean-sim
clean-sim:							## Clean atomsim build files
	@echo "$(COLOR_GREEN)>> Cleaning atomsim [$(atomsim)/build/*] ...$(COLOR_NC)"
	make -C atomsim clean


# ======== SCAR ========
.PHONY: scar     			
scar: $(bin_dir)/$(sim_executable)	## Verify target using scar
	@echo "\n$(COLOR_GREEN)>> Running SCAR... $(COLOR_NC)"
	make -C test/scar/


.PHONY: clean-scar
clean-scar:							## Clean scar directory
	@echo "$(COLOR_GREEN)>> Cleaning libs [test/scar/work/*] ...$(COLOR_NC)"
	make -C test/scar/ clean


# ======== ElfDump ========
.PHONY: elfdump
elfdump:							## Build elfdump utility
	@echo "$(COLOR_GREEN)>> Building elfdump...$(COLOR_NC)"
	make -C tools/elfdump/


.PHONY: clean-elfdump
clean-elfdump:					    ## Clean elfdump directory
	@echo "$(COLOR_GREEN)>> Cleaning elfdump directory [tools/elfdump/bin/*] ...$(COLOR_NC)"
	make -C tools/elfdump clean


# ======== SW libs ========
.PHONY: lib
lib:								## compile software libraries
	@echo "$(COLOR_GREEN)>> Compiling software libraries ...$(COLOR_NC)"
	make -C sw/lib/ Target=$(Target)


.PHONY: clean-lib
clean-lib:							## Clean software libs
	@echo "$(COLOR_GREEN)>> Cleaning libs [$(doxygen_doc_dir)/*] ...$(COLOR_NC)"
	make -C sw/lib/ clean


# ======== Documentation ========
.PHONY: doxy
doxy:								## Generate atomsim C++ source documentation
	@echo "$(COLOR_GREEN)>> Generating Doxygen C++ documentation [latex & html]...$(COLOR_NC)"
	make -C sim/docs/


.PHONY: doxy-pdf
doxy-pdf: doxy						## Generate atomsim C++ source documentation (pdf)
	@echo "$(COLOR_GREEN)>> Generating Doxygen C++ documentation [pdf]...$(COLOR_NC)"
	make -C sim/docs/ pdf


.PHONY: clean-doc
clean-doc:							## Clean generated docs
	@echo "$(COLOR_GREEN)>> Cleaning docs [$(doxygen_doc_dir)/*] ...$(COLOR_NC)"
	make -C sim/docs/ clean


# ======== clean ========
.PHONY: clean						
clean: clean-sim					## Alias for clean-sim


.PHONY: clean-all					
clean-all: clean-sim clean-doc clean-lib clean-scar 	## Clean all build files