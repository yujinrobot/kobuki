include $(shell rospack find mk)/cmake.mk

maps:
	wget http://files.yujinrobot.com/maps/lobby.pgm -O resources/maps/lobby.pgm
	wget http://files.yujinrobot.com/maps/lab.pgm -O resources/maps/lab.pgm

unison:
	@if [ -f ~/.unison/kobuki.prf ]; then \
                echo ""; \
                echo "Unison profile for kobuki already exists, to overwrite, run:"; \
                echo ""; \
                echo "     make unison-update"; \
                echo ""; \
        else \
                echo ""; \
                echo "Copying unison profile for kobuki to ~/.unison:"; \
                echo ""; \
                mkdir -p ~/.unison; \
                cp resources/unison/kobuki.prf ~/.unison/kobuki.prf; \
        fi

unison-update:
	@echo ""
	@echo "Updating the kobuki profile with the most recent version."
	@echo ""
	cp resources/unison/kobuki.prf ~/.unison/kobuki.prf
	@echo ""

.PHONY: unison unison-update
